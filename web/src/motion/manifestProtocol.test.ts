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
import { handshakeV2, uploadManifest, sendManifestHeader, readSerialLines, type UploadResult } from '../serial';
import { buildManifest, sliceLines, V2_CHUNK_MAX } from './manifest';
import type { PlanSample } from './planTimeline';

const RING_SIZE = 48;

// ─── Fake V2 firmware (semántica de protocol_v2.cpp + executor básico) ───
class FakeV2Firmware {
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
      return; // el firmware NO responde a MANIFEST
    }
    if (head === 'SAMPLE') {
      this.log.push(`SAMPLE#${this.received}`);
      if (this.state === 'IDLE' || !this.manifestSeen) return this.error('BAD_STATE');
      const joints = rest.slice(0, 6).map(Number);
      if (joints.some((j) => j < 500 || j > 2400)) return this.error('OUT_OF_RANGE');
      this.received += 1;
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
      // ACK cada 24 recibidos si el ring tiene espacio (executor no corriendo aquí)
      if (this.received % V2_CHUNK_MAX === 0 && RING_SIZE - this.stored > 0) {
        const ack = `ACK ${RING_SIZE - this.stored}`;
        this.log.push(ack);
        this.output.push(ack);
      }
      return;
    }
    if (head === 'EXECUTE') {
      this.log.push(line);
      if (this.state === 'IDLE' || !this.manifestSeen) return this.error('BAD_STATE');
      this.state = 'RUNNING';
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
    }
  }

  /** Bytes disponibles para el host (vacía la cola). */
  readBytes(): Uint8Array | null {
    if (this.output.length === 0) return null;
    return new TextEncoder().encode(this.output.shift() + '\n');
  }
}

// ─── Fake SerialPort (superficie WebSerial que usa serial.ts) ───
class FakeSerialPort {
  constructor(private firmware: FakeV2Firmware) {}

  get writable(): any {
    return {
      getWriter: () => ({
        ready: Promise.resolve(),
        write: async (data: Uint8Array) => {
          this.firmware.feed(data);
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

function fakeSamples(n: number): PlanSample[] {
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
});
