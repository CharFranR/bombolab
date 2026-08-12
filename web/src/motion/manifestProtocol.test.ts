/**
 * AUDITORÍA BOMBOLAB — test de regresión del contrato de protocolo v2 (web ↔ firmware).
 *
 * Bug raíz de la cadena física: el web enviaba SAMPLE sin haber enviado MANIFEST,
 * y el firmware v2 rechaza el primer SAMPLE con ERR BAD_STATE
 * (gate `manifest_ready` — protocol_v2.cpp handle_sample).
 *
 * FIX MÍNIMO (una capa, solo web): `sendManifestHeader` envía `MANIFEST <count> <durationUs>`
 * como línea individual tras el handshake y antes de los SAMPLEs (como el CLI serial-test).
 * El firmware no responde a MANIFEST; el error, si ocurre, aparece en la primera ventana ACK.
 *
 * Estructura:
 *  - FakeV2Firmware: port fiel de protocol_v2.cpp (HELLO/MANIFEST/SAMPLE/EXECUTE,
 *    gate manifest_ready, ACK cada 24 recibidos si el ring tiene espacio, ring 48
 *    con ERR RING_FULL si se desborda) + log del orden de procesamiento.
 *  - FakeSerialPort: superficie WebSerial usada por serial.ts.
 *  - Test 1: flujo COMPLETO de producción (handshake → MANIFEST → SAMPLE → ACK →
 *    sin ERR/RING_FULL → estado final). FALLABA antes del fix con ERR BAD_STATE.
 *  - Test 2: contrato del firmware — SAMPLE sin MANIFEST es rechazado (evidencia
 *    del fallo original, ahora como contrato pasante).
 *  - Test 3: mismo flujo con 49 SAMPLEs — verifica que el límite del ring (48) no
 *    genera RING_FULL durante el upload por ventanas (el web se detiene a los 48
 *    sin ACK, tal como el firmware espera).
 */
import { describe, expect, it } from 'vitest';
import { handshakeV2, uploadManifest, sendManifestHeader, continueManifestUpload, awaitSendSerial, readSerialLines, type UploadResult } from '../serial';
import { buildManifest, sliceLines, V2_CHUNK_MAX } from './manifest';
import type { PlanSample } from './planTimeline';

export const RING_SIZE = 48;

// ─── Fake V2 firmware (semántica de protocol_v2.cpp + executor básico) ───
export class FakeV2Firmware {
  private output: string[] = [];
  /** Log del orden en que se procesaron los comandos (head + args). */
  log: string[] = [];
  state: 'IDLE' | 'RECEIVING' | 'RUNNING' = 'IDLE';
  manifestSeen = false;
  received = 0;
  stored = 0; // ocupación del ring (sin consumo: nunca baja durante el upload)

  /** Inyecta líneas residuales como si el firmware las hubiera emitido antes
   *  (p. ej. respuestas OK/ERR de frames legacy previos al handshake). */
  injectResidue(lines: string[]): void {
    for (const l of lines) this.output.push(l);
  }

  /** Modo "firmware viejo sin v2": rechaza HELLO con ERR sin token (parser legacy). */
  legacyOnly = false;

  /** Modo "firmware v2 en mal estado": rechaza HELLO con ERR BAD_STATE. */
  rejectHello = false;

  /** Modo "firmware mudo": recibe SAMPLEs pero NO emite ACK en las ventanas
   *  (simula el firmware colgado/en estado raro del bug 2026-08-10). */
  silentAck = false;

  /** Retardo (ms) para el ACK de ventana: simula un transporte lento (p. ej.
   *  WebSerial de Firefox) que entrega el ACK MUCHO después de emitido. */
  lateAckDelayMs = 0;

  /** Modo streaming (RUNNING real): tras EXECUTE consume una muestra cada
   *  consumeMs y emite T-line + ACK por muestra (como v2_executor_tick), y
   *  DONE al consumir todas las declaradas. El store replica v2_executor_store:
   *  ring lleno → ERR BAD_STATE (el abort que reproduce el bug 2026-08-12). */
  streaming = false;
  consumeMs = 10;
  declared = 0;
  consumed = 0; // muestras consumidas en RUNNING (T-lines emitidas)
  private ringJoints: number[][] = [];
  private consumeTimer: ReturnType<typeof setTimeout> | null = null;

  private startConsuming(): void {
    this.consumeTimer = setTimeout(() => {
      this.consumeTimer = null;
      if (this.state !== 'RUNNING' || !this.streaming) return;
      if (this.stored > 0) {
        this.stored -= 1;
        this.consumed += 1;
        const joints = this.ringJoints.shift() ?? [];
        this.output.push(`T ${this.consumed * 10000} ${joints.join(' ')}`);
        this.output.push(`ACK ${RING_SIZE - this.stored}`);
        if (this.consumed >= this.declared && this.stored === 0) {
          this.output.push('DONE');
          this.state = 'IDLE';
          this.manifestSeen = false;
          return;
        }
      }
      this.startConsuming();
    }, this.consumeMs);
  }

  private stopConsuming(): void {
    if (this.consumeTimer) {
      clearTimeout(this.consumeTimer);
      this.consumeTimer = null;
    }
  }

  feed(bytes: Uint8Array): void {
    const text = new TextDecoder().decode(bytes);
    for (const raw of text.split('\n')) {
      const line = raw.replace(/\r$/, '').trim();
      if (!line) continue;
      if (this.legacyOnly) {
        // parser legacy: responde OK a frames de 6 ints, ERR a lo demás
        const ok = line.split(',').length === 6 && line.split(',').every((v) => Number.isFinite(Number(v)));
        this.output.push(ok ? 'OK' : 'ERR');
        continue;
      }
      this.process(line);
    }
  }

  private process(line: string): void {
    const [head, ...rest] = line.split(' ');
    if (head === 'HELLO') {
      this.log.push(line);
      if (this.rejectHello) {
        this.output.push('ERR BAD_STATE');
        return;
      }
      if (this.state !== 'IDLE') return this.error('BAD_STATE');
      this.state = 'RECEIVING';
      this.output.push('HELLO 2 OK CHUNK_MAX 24');
      return;
    }
    if (head === 'MANIFEST') {
      this.log.push(line);
      if (this.state === 'IDLE') return this.error('BAD_STATE');
      this.manifestSeen = true;
      this.received = 0;
      this.declared = Number(rest[0] ?? 0);
      return; // el firmware NO responde a MANIFEST
    }
    if (head === 'SAMPLE') {
      this.log.push(`SAMPLE#${this.received}`);
      if (this.state === 'IDLE' || !this.manifestSeen) return this.error('BAD_STATE');
      const joints = rest.slice(0, 6).map(Number);
      if (joints.some((j) => j < 500 || j > 2400)) return this.error('OUT_OF_RANGE');
      this.received += 1;
      if (this.streaming && this.state === 'RUNNING') {
        // v2_executor_store: ring lleno → ERR BAD_STATE → discard_to_idle
        if (this.stored >= RING_SIZE) {
          this.log.push('ERR BAD_STATE (ring lleno)');
          this.output.push('ERR BAD_STATE');
          this.manifestSeen = false;
          this.stored = 0;
          this.state = 'IDLE';
          this.stopConsuming();
          return;
        }
        this.stored += 1;
        this.ringJoints.push(joints);
        return;
      }
      if (this.stored >= RING_SIZE) {
        // El store real del firmware devuelve V2_ERR_BAD_STATE cuando el ring
        // (48) está lleno y no hay EXECUTE aún.
        this.log.push('ERR BAD_STATE');
        this.output.push('ERR BAD_STATE');
        this.stored = 0;
        this.manifestSeen = false;
        this.state = 'IDLE';
        return;
      }
      this.stored += 1;
      if (this.streaming) this.ringJoints.push(joints); // se consumirán en RUNNING
      // ACK cada 24 recibidos si el ring tiene espacio (executor no corriendo aquí)
      if (this.received % V2_CHUNK_MAX === 0 && RING_SIZE - this.stored > 0) {
        if (!this.silentAck) {
          const ack = `ACK ${RING_SIZE - this.stored}`;
          this.log.push(ack);
          if (this.lateAckDelayMs > 0) {
            setTimeout(() => this.output.push(ack), this.lateAckDelayMs);
          } else {
            this.output.push(ack);
          }
        } else {
          this.log.push(`ACK ${RING_SIZE - this.stored} (suprimido)`);
        }
      }
      return;
    }
    if (head === 'EXECUTE') {
      this.log.push(line);
      if (this.state === 'IDLE' || !this.manifestSeen) return this.error('BAD_STATE');
      this.state = 'RUNNING';
      if (this.streaming) this.startConsuming();
      return; // el firmware no responde a EXECUTE
    }
    this.error('BAD_LINE');
  }

  private error(token: string): void {
    this.log.push(`ERR ${token}`);
    this.output.push(`ERR ${token}`);
    if (token === 'BAD_STATE' || token === 'RING_FULL') {
      // discard_to_idle en el firmware real
      this.manifestSeen = false;
      this.stored = 0;
      this.state = 'IDLE';
      this.stopConsuming();
    }
  }

  /** Bytes disponibles para el host (vacía la cola). */
  readBytes(): Uint8Array | null {
    if (this.output.length === 0) return null;
    return new TextEncoder().encode(this.output.shift() + '\n');
  }
}

// ─── Fake SerialPort (superficie WebSerial que usa serial.ts) ───
export class FakeSerialPort {
  constructor(
    private firmware: FakeV2Firmware,
    /** Retardo de aterrizaje por write (ms): modela el buffer del OS — el write
     *  resuelve apenas se ENCOLA (el web corre adelante) y los bytes llegan al
     *  firmware después. 0 = instantáneo. */
    private feedDelayMs = 0,
  ) {}

  get writable(): any {
    return {
      getWriter: () => ({
        ready: Promise.resolve(),
        write: async (data: Uint8Array) => {
          if (this.feedDelayMs > 0) {
            setTimeout(() => this.firmware.feed(data), this.feedDelayMs);
          } else {
            this.firmware.feed(data);
          }
        },
        releaseLock: () => {},
      }),
    };
  }

  get readable(): any {
    return {
      getReader: () => ({
        read: async (): Promise<{ value: Uint8Array; done: boolean }> => {
          for (let i = 0; i < 200; i++) {
            const bytes = this.firmware.readBytes();
            if (bytes) return { value: bytes, done: false };
            await new Promise((r) => setTimeout(r, 2));
          }
          return { value: new Uint8Array(0), done: false };
        },
        releaseLock: () => {},
      }),
    };
  }
}

export function fakeSamples(n: number): PlanSample[] {
  const out: PlanSample[] = [];
  for (let i = 0; i < n; i++) {
    out.push({ t: i * 0.05, q_us: [1472, 1472 + i, 1379, 1524, 1163 + i, 1183], count: 1 });
  }
  return out;
}

function uploadLines(port: FakeSerialPort, lines: string[], onSample?: (q: number[]) => void): Promise<UploadResult> {
  return uploadManifest(port as any, sliceLines(lines, V2_CHUNK_MAX), V2_CHUNK_MAX, undefined, onSample);
}

describe('contrato de protocolo v2 web↔firmware', () => {
  it('1 — flujo COMPLETO del web (handshake → MANIFEST → SAMPLE → ACK → sin ERR/RING_FULL → estado final)', async () => {
    const fw = new FakeV2Firmware();
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(30));
    expect(built).not.toBeInstanceOf(Error);
    const lines = (built as Exclude<typeof built, Error>).lines;

    // 1) Handshake
    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    expect(fw.state).toBe('RECEIVING');

    // 2) MANIFEST como línea individual ANTES de los SAMPLEs (función de producción)
    const manifest = built as Exclude<typeof built, Error>;
    sendManifestHeader(port as any, manifest.count, manifest.durationUs);

    // 3) SAMPLEs por ventanas de 24
    const upload = await uploadLines(port, lines);

    // 4) Orden correcto: MANIFEST antes de cualquier SAMPLE
    const firstSample = fw.log.findIndex((l) => l.startsWith('SAMPLE'));
    const manifestIdx = fw.log.findIndex((l) => l.startsWith('MANIFEST'));
    expect(manifestIdx).toBeGreaterThanOrEqual(0);
    expect(firstSample).toBeGreaterThan(manifestIdx);
    expect(fw.log[manifestIdx]).toBe(`MANIFEST ${manifest.count} ${manifest.durationUs}`);

    // 5) ACK esperado en la ventana 24 (el firmware responde ACK con espacio libre)
    expect(fw.log).toContain(`ACK ${RING_SIZE - 24}`);

    // 6) Sin errores → sin RING_FULL, sin rechazos
    expect(fw.log.filter((l) => l.startsWith('ERR'))).toEqual([]);

    // 7) Upload completo y estado final esperado (RECEIVING: listo para EXECUTE)
    expect(upload.error).toBeUndefined();
    expect(upload.sent).toBe(manifest.count);
    expect(fw.state).toBe('RECEIVING');
  }, 10_000);

  it('2 — contrato del firmware: SAMPLE sin MANIFEST es rechazado con ERR BAD_STATE (evidencia del bug original)', async () => {
    const fw = new FakeV2Firmware();
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(30)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    // SIN sendManifestHeader — flujo pre-fix
    const upload = await uploadLines(port, built.lines);

    expect(upload.error).toBe('ERR BAD_STATE');
    expect(upload.sent).toBe(24); // la primera ventana ya recibe el rechazo
    expect(fw.state).toBe('IDLE'); // discard_to_idle
  }, 10_000);

  it('3b — onSample reporta cada SAMPLE enviado en orden (exportar traza en modo manifest)', async () => {
    const fw = new FakeV2Firmware();
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(30)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    sendManifestHeader(port as any, built.count, built.durationUs);

    const received: number[][] = [];
    const upload = await uploadLines(port, built.lines, (q) => received.push(q));

    expect(upload.error).toBeUndefined();
    expect(received).toHaveLength(30);
    expect(received[0]).toEqual(fakeSamples(30)[0].q_us);
    expect(received[29]).toEqual(fakeSamples(30)[29].q_us);
  }, 10_000);

  it('5 — handshakeV2 sobrevive al residuo legacy (OK/ERR sin token) y encuentra el HELLO OK', async () => {
    const fw = new FakeV2Firmware();
    // residuo típico: respuestas de heartbeat/frames legacy previos
    fw.injectResidue(['OK', 'OK', 'ERR', 'OK']);
    const port = new FakeSerialPort(fw);

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
  }, 10_000);

  it('6 — handshakeV2 falla limpiamente si el firmware v2 rechaza HELLO (ERR con token)', async () => {
    const fw = new FakeV2Firmware();
    fw.rejectHello = true;
    const port = new FakeSerialPort(fw);

    const r = await handshakeV2(port as any);
    expect(r).toBeInstanceOf(Error);
    expect((r as Error).message).toContain('ERR BAD_STATE');
  }, 10_000);

  it('11 — upload >48 samples se detiene en el límite del ring (48) sin ERR BAD_STATE', async () => {
    const fw = new FakeV2Firmware();
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(217)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    await sendManifestHeader(port as any, built.count, built.durationUs);

    const upload = await uploadLines(port, built.lines);
    expect(upload.error).toBeUndefined();
    expect(upload.sent).toBe(48); // el ring (2×24) — el resto va tras EXECUTE
    expect(fw.log.filter((l) => l.startsWith('ERR'))).toEqual([]);
  }, 15_000);

  it('10 — readSerialLines respeta el timeout cuando read() nunca resuelve (sin colgar)', async () => {
    // Simula Web Serial sin data: la promesa de read() queda pendiente para siempre.
    const neverPort = {
      readable: {
        getReader: () => ({
          read: () => new Promise(() => {}), // nunca resuelve
          releaseLock: () => {},
        }),
      },
      writable: null,
    };
    const t0 = Date.now();
    const lines = await readSerialLines(neverPort as any, 300);
    const elapsed = Date.now() - t0;
    expect(lines).toEqual([]);
    expect(elapsed).toBeGreaterThanOrEqual(250); // esperó el timeout
    expect(elapsed).toBeLessThan(2000); // pero no se colgó para siempre
  }, 10_000);

  it('9 — handshakeV2 drena un backlog legacy grande (60 OKs) y encuentra el HELLO OK', async () => {
    const fw = new FakeV2Firmware();
    fw.injectResidue(Array.from({ length: 60 }, () => 'OK'));
    const port = new FakeSerialPort(fw);

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
  }, 15_000);

  it('7 — handshakeV2 contra firmware legacy-only (sin v2) devuelve Error, no cuelga', async () => {
    const fw = new FakeV2Firmware();
    fw.legacyOnly = true;
    const port = new FakeSerialPort(fw);

    const r = await handshakeV2(port as any);
    expect(r).toBeInstanceOf(Error);
  }, 15_000);

  it('8 — readSerialLines con stopWhen retorna apenas llega la línea buscada (no espera el batch)', async () => {
    const fw = new FakeV2Firmware();
    fw.injectResidue(['ACK 24']);
    const port = new FakeSerialPort(fw);

    const t0 = Date.now();
    const lines = await readSerialLines(port as any, 3000, (l) => l.startsWith('ACK '));
    const elapsed = Date.now() - t0;

    expect(lines).toEqual(['ACK 24']);
    expect(elapsed).toBeLessThan(2000); // no esperó el timeout completo
  }, 10_000);

  it('3 — 49 SAMPLEs no desbordan el ring (48) durante el upload por ventanas', async () => {
    const fw = new FakeV2Firmware();
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(49)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    sendManifestHeader(port as any, built.count, built.durationUs);

    const upload = await uploadLines(port, built.lines);

    expect(fw.log.filter((l) => l.startsWith('ERR'))).toEqual([]);
    expect(upload.error).toBeUndefined();
    // El web se detiene en la ventana sin ACK (ring lleno a los 48, sin EXECUTE aún):
    // esto es el comportamiento diseñado — NO genera RING_FULL porque no envía más allá.
    expect(fw.stored).toBeLessThanOrEqual(RING_SIZE);
  }, 10_000);

  it('12 — REGRESIÓN: firmware mudo (sin ACK) con ring incompleto → error explícito, NO éxito falso', async () => {
    const fw = new FakeV2Firmware();
    // Simula el bug 2026-08-10: el firmware no confirma la primera ventana
    // (ring a 24 de 288). Antes uploadManifest retornaba {sent:24} SIN error
    // y App.tsx mandaba EXECUTE con el ring incompleto → firmware mudo, sin T-lines.
    fw.silentAck = true;
    // Residuo típico del parser legacy que además cortaba la ventana ACK antes.
    fw.injectResidue(['OK', 'OK', 'OK', 'OK', 'OK', 'OK', 'OK', 'OK']);
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(288)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    sendManifestHeader(port as any, built.count, built.durationUs);

    const upload = await uploadLines(port, built.lines);

    // Sin ACK y con ring incompleto → el upload DEBE fallar (App.tsx no manda
    // EXECUTE). Nunca éxito silencioso con el ring a 24 de 288.
    expect(upload.error).toBeDefined();
    expect(upload.sent).toBeLessThan(built.count);
  }, 15_000);

  it('13 — REGRESIÓN: uploadManifest descarta líneas basura y ESPERA el ACK real de la ventana', async () => {
    const fw = new FakeV2Firmware();
    // Basura legacy ANTES del ACK legítimo: el web debe ignorarla y seguir
    // esperando hasta encontrar "ACK 24" (no cortar por el límite de 8 líneas).
    fw.injectResidue(['OK', 'OK', 'OK', 'OK', 'OK', 'OK', 'OK', 'OK']);
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(30)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    sendManifestHeader(port as any, built.count, built.durationUs);

    const upload = await uploadLines(port, built.lines);

    expect(upload.error).toBeUndefined();
    expect(upload.sent).toBe(built.count); // 30/30 — el ACK real sí llegó
    expect(fw.log).toContain(`ACK ${RING_SIZE - 24}`);
  }, 15_000);

  it('14 — REGRESIÓN 2026-08-12: continueManifestUpload no desborda el ring en RUNNING (ERR BAD_STATE por samples en vuelo)', async () => {
    // Reproduce el abort del manifest real: tras EXECUTE el web manda `free`
    // samples por cada batch de ACKs, pero sus writes aterrizan con retardo
    // (buffer del OS) mientras el firmware sigue consumiendo → el ring (48)
    // se llena → v2_executor_store responde ERR BAD_STATE → manifest muerto.
    // Con el tope de vuelo (consumed - sent) el web nunca supera el ring.
    const fw = new FakeV2Firmware();
    fw.streaming = true;
    fw.consumeMs = 10;
    const port = new FakeSerialPort(fw, 60);
    const built = buildManifest(fakeSamples(120)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    await sendManifestHeader(port as any, built.count, built.durationUs);
    const upload = await uploadManifest(port as any, sliceLines(built.lines, V2_CHUNK_MAX), V2_CHUNK_MAX);
    expect(upload.error).toBeUndefined();
    expect(upload.sent).toBe(RING_SIZE); // ring lleno (48), listo para EXECUTE

    await awaitSendSerial(port as any, new TextEncoder().encode('EXECUTE\n'));

    const remaining = built.lines.slice(upload.sent);
    const res = await continueManifestUpload(port as any, RING_SIZE, remaining);

    // Esperar a que aterricen los últimos writes en vuelo (modelo OS buffer) y
    // el firmware consuma lo que quedó en el ring.
    const deadline = Date.now() + 2000;
    while (fw.consumed + fw.stored < built.count && Date.now() < deadline) {
      await new Promise((r) => setTimeout(r, 25));
    }

    // El fix: sin ERR BAD_STATE, todas las muestras se entregan y se almacenan
    // sin rechazo (el web sale al enviar la última; el firmware termina de
    // consumir después, por eso se valida stored + consumed == declaradas).
    expect(res.error).toBeUndefined();
    expect(res.sent).toBe(remaining.length);
    expect(fw.log.filter((l) => l.startsWith('ERR'))).toEqual([]);
    expect(fw.consumed + fw.stored).toBe(built.count);
  }, 20_000);

  it('15 — ACK tardío (transporte lento): la re-lectura evita el falso "sin ACK"', async () => {
    // Bug 2026-08-12 b: el WebSerial de Firefox entrega el ACK con retardo;
    // la ventana de 1200ms expiraba y el web abortaba con "sin ACK tras 24
    // samples" aunque el firmware SÍ había confirmado. Con 3000ms + re-lectura
    // el ACK tardío se captura sin reenviar nada (sin riesgo de duplicados).
    const fw = new FakeV2Firmware();
    fw.lateAckDelayMs = 3800; // supera la ventana inicial (3000ms): solo la re-lectura lo captura
    const port = new FakeSerialPort(fw);
    const built = buildManifest(fakeSamples(30)) as Exclude<ReturnType<typeof buildManifest>, Error>;

    const chunkMax = await handshakeV2(port as any);
    expect(chunkMax).toBe(24);
    await sendManifestHeader(port as any, built.count, built.durationUs);

    const upload = await uploadLines(port, built.lines);

    expect(upload.error).toBeUndefined();
    expect(upload.sent).toBe(30);
    expect(fw.log).toContain(`ACK ${RING_SIZE - 24}`);
  }, 15_000);
});
