// ─── Conversión q → servo ──────────────────────────────────────────────
// Misma lógica que Robot::q_to_servo() en Rust

const DEG = 180 / Math.PI;

// fabri_creator servo offsets (90° en rad) y direcciones
const OFFSETS_DEG = [90, 90, 81, 95, 60];
const DIRECTIONS = [-1, -1, 1, -1, -1];

// ─── Pulse-width (µs) wire format ─────────────────────────────────────────
// The wire now carries servo pulse widths in microseconds (500-2400):
// 1 µs ≈ 0.1° — 10× finer than the old integer-degree protocol, which
// quantized the drawing to ~3.5 mm steps at the arm's lever (1° at 200 mm).
// The firmware auto-detects units per frame (degrees ≤175, µs ≥500).

const US_PER_DEG = (2400 - 544) / 180; // ≈ 10.31 µs/deg (Servo lib mapping)

export function servoDegToUs(deg: number): number {
  return 544 + deg * US_PER_DEG;
}

export function servoUsToDeg(us: number): number {
  return (us - 544) / US_PER_DEG;
}

/** Convierte q (rad) a servo pulse widths en µs FLOAT dentro de [544, 2400]. */
export function qToServoUs(q: number[]): number[] {
  return q.map((qi, i) => {
    const deg = DIRECTIONS[i] * (qi * DEG) + OFFSETS_DEG[i];
    return servoDegToUs(Math.max(5, Math.min(175, deg)));
  });
}

/** Gripper percent (0–100, 100 = closed) → µs FLOAT. */
export function gripperToServoUs(gripperPct: number): number {
  const deg = Math.max(5, Math.min(175, 170 - (gripperPct / 100) * 120));
  return servoDegToUs(deg);
}

// ─── Wire format ────────────────────────────────────────────────────────
// Formato: "a1,a2,a3,a4,a5,g\n" — igual que ArduinoNano

/** Gripper percent (0–100, 100 = closed) → servo degrees FLOAT [50, 170]. */
export function gripperToServo(gripperPct: number): number {
  return Math.max(5, Math.min(175, 170 - (gripperPct / 100) * 120));
}

/** Encode a full 6-value servo frame (5 joints + gripper, degrees). */
export function encodeWire(servoDeg: number[]): Uint8Array {
  const str = `${Math.round(servoDeg[0])},${Math.round(servoDeg[1])},${Math.round(servoDeg[2])},${Math.round(servoDeg[3])},${Math.round(servoDeg[4])},${Math.round(servoDeg[5])}\n`;
  return new TextEncoder().encode(str);
}

export function buildWire(jointsDeg: number[], gripperPct: number): Uint8Array {
  return encodeWire([...jointsDeg, gripperToServo(gripperPct)]);
}

// ─── WebSerial ──────────────────────────────────────────────────────────

export async function requestSerialPort(): Promise<SerialPort> {
  if (!navigator.serial) {
    throw new Error('WebSerial no disponible. Usá Chrome/Edge.');
  }
  return navigator.serial.requestPort();
}

export async function openPort(port: SerialPort): Promise<void> {
  await port.open({ baudRate: 115200 });
}

// ── Capa serial persistente por puerto ─────────────────────────────────
// El patrón getWriter→write→releaseLock por frame pierde datos en Web Serial
// (la escritura queda en vuelo al liberar el lock; las fallas son silenciosas).
// Se mantiene UN writer y UN reader vivos por puerto, y el buffer de lectura
// parcial persiste entre llamadas (una línea cortada a mitad de timeout no se pierde).
const serialWriters = new Map<SerialPort, WritableStreamDefaultWriter<Uint8Array>>();
const serialReaders = new Map<SerialPort, ReadableStreamDefaultReader<Uint8Array>>();
const serialBuffers = new Map<SerialPort, string>();
/** Loop de lectura de fondo por puerto: consume el readable continuamente y
 *  alimenta serialBuffers. readSerialLines solo procesa el buffer — así ningún
 *  timeout deja promesas colgadas ni se pierden bytes. */
const serialReadLoops = new Map<SerialPort, Promise<void>>();

function ensureReadLoop(port: SerialPort): void {
  if (serialReadLoops.has(port) || !port.readable) return;
  const reader = port.readable.getReader();
  serialReaders.set(port, reader);
  const loop = (async () => {
    const decoder = new TextDecoder();
    try {
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        serialBuffers.set(port, (serialBuffers.get(port) ?? '') + decoder.decode(value, { stream: true }));
      }
    } catch {
      /* puerto perdido/cerrado: el loop muere */
    } finally {
      serialReadLoops.delete(port);
      serialReaders.delete(port);
    }
  })();
  serialReadLoops.set(port, loop);
}

export function sendSerial(port: SerialPort, data: Uint8Array): void {
  let writer = serialWriters.get(port);
  if (!writer) {
    const w = port.writable?.getWriter();
    if (!w) return;
    writer = w;
    serialWriters.set(port, writer);
  }
  void writer.ready
    .then(() => writer!.write(data))
    .catch(() => {
      // Escritura fallida: descartar el writer; el próximo envío crea uno nuevo.
      try {
        writer!.releaseLock();
      } catch {
        /* ya liberado */
      }
      serialWriters.delete(port);
    });
}

/** Drena líneas pendientes del firmware (p. ej. ERRs stale de intentos previos). */
export async function drainSerial(port: SerialPort, maxMs = 800): Promise<number> {
  let drained = 0;
  let emptyRuns = 0;
  const deadline = Date.now() + maxMs;
  while (Date.now() < deadline && emptyRuns < 2) {
    const lines = await readSerialLines(port, 200);
    drained += lines.length;
    emptyRuns = lines.length === 0 ? emptyRuns + 1 : 0;
  }
  return drained;
}

/**
 * Escritura ESPERADA (backpressure real): para el protocolo v2 (HELLO/MANIFEST/
 * SAMPLE/EXECUTE) las líneas deben llegar íntegras; un write() sin esperar en
 * ráfagas puede entregar bytes partidos → ERR BAD_LINE en el firmware.
 */
export async function awaitSendSerial(port: SerialPort, data: Uint8Array): Promise<void> {
  let writer = serialWriters.get(port);
  if (!writer) {
    const w = port.writable?.getWriter();
    if (!w) throw new Error('puerto sin writable');
    writer = w;
    serialWriters.set(port, writer);
  }
  try {
    await writer.ready;
    await writer.write(data);
  } catch (e) {
    try {
      writer.releaseLock();
    } catch {
      /* ya liberado */
    }
    serialWriters.delete(port);
    throw e;
  }
}

/** Libera writer/reader/buffer del puerto (al desconectar). */
export function releaseSerial(port: SerialPort): void {
  const w = serialWriters.get(port);
  if (w) {
    try {
      w.releaseLock();
    } catch {
      /* ya liberado */
    }
    serialWriters.delete(port);
  }
  const r = serialReaders.get(port);
  if (r) {
    try {
      r.releaseLock(); // el loop de fondo falla y muere solo
    } catch {
      /* ya liberado */
    }
    serialReaders.delete(port);
    serialReadLoops.delete(port);
  }
  serialBuffers.delete(port);
}


/**
 * Lee líneas del puerto. Con `stopWhen` retorna en cuanto una línea la cumple
 * (sin esperar 8 líneas ni el timeout): crítico para el handshake y las ventanas
 * ACK, donde esperar el batch completo retrasa el EXECUTE varios segundos.
 */
export async function readSerialLines(
  port: SerialPort,
  timeoutMs = 1500,
  stopWhen?: (line: string) => boolean,
): Promise<string[]> {
  if (!port.readable) return [];
  ensureReadLoop(port);
  const lines: string[] = [];
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline && lines.length < 8) {
    const buf = serialBuffers.get(port) ?? '';
    const idx = buf.indexOf('\n');
    if (idx < 0) {
      // El loop de fondo alimenta el buffer; esperar un poco y revisar de nuevo.
      await new Promise((r) => setTimeout(r, 20));
      continue;
    }
    const line = buf.slice(0, idx).replace(/\r$/, '');
    serialBuffers.set(port, buf.slice(idx + 1));
    lines.push(line);
    if (stopWhen && stopWhen(line)) return lines;
  }
  return lines;
}

export async function handshakeV2(port: SerialPort): Promise<number | Error> {
  // El firmware puede tener líneas residuales en el buffer (respuestas "OK"/"ERR"
  // de frames legacy previos: heartbeat/sendQ). Reintentar hasta encontrar el
  // "HELLO 2 OK" real. Un "ERR " con token (formato v2) es un rechazo real del
  // firmware v2; un "OK"/"ERR" sin token es residuo del parser legacy y se ignora.
  // 0) Drenar el backlog de respuestas legacy: el firmware responde "OK" a cada
  // frame del interpolator y el web nunca las lee → se acumulan en el buffer del
  // firmware y tapan el HELLO OK. Leer descartando hasta 2 lecturas vacías.
  let emptyRuns = 0;
  for (let i = 0; i < 12 && emptyRuns < 2; i++) {
    const drained = await readSerialLines(port, 250);
    emptyRuns = drained.length === 0 ? emptyRuns + 1 : 0;
  }

  const seen: string[] = [];
  for (let attempt = 0; attempt < 5; attempt++) {
    await awaitSendSerial(port, new TextEncoder().encode('HELLO 2\n'));
    // stopWhen: retornar apenas aparezca el HELLO OK (o un rechazo v2), sin
    // esperar el batch de 8 líneas — antes esto retrasaba el EXECUTE ~1.5s.
    const lines = await readSerialLines(port, 1500, (l) => l.startsWith('HELLO 2 OK') || l.startsWith('ERR '));
    for (const line of lines) {
      seen.push(line);
      if (line.startsWith('HELLO 2 OK CHUNK_MAX')) {
        const m = /CHUNK_MAX (\d+)/.exec(line);
        if (m) return Number(m[1]);
      }
      if (line.startsWith('ERR ')) {
        return new Error('el firmware rechazó HELLO 2: ' + line);
      }
    }
  }
  const tail = seen.slice(-8).join(' | ');
  return new Error('sin respuesta HELLO 2 (5 intentos' + (tail ? ' — líneas leídas: ' + tail : ' — nada leído') + ')');
}

export interface UploadResult {
  sent: number;
  total: number;
  error?: string;
}

/** Extrae los 6 valores µs de una línea `SAMPLE q1..q6 dt` (null si no es SAMPLE). */
function sampleQ(line: string): number[] | null {
  const parts = line.split(' ');
  if (parts[0] !== 'SAMPLE' || parts.length < 8) return null;
  return parts.slice(1, 7).map(Number);
}

/**
 * Envía el encabezado `MANIFEST <count> <durationUs>` que el firmware v2 exige
 * ANTES del primer SAMPLE (gate `manifest_ready` en protocol_v2.cpp handle_sample).
 * El firmware NO responde a esta línea (verificable en el CLI serial-test, que la
 * envía sin leer respuesta). Si el firmware rechaza el encabezado, el error se
 * superficializa en la primera ventana ACK de `uploadManifest`.
 */
export async function sendManifestHeader(port: SerialPort, count: number, durationUs: number): Promise<void> {
  await awaitSendSerial(port, new TextEncoder().encode(`MANIFEST ${count} ${durationUs}\n`));
}

export async function continueManifestUpload(
  port: SerialPort,
  remainingLines: string[],
  onProgress?: (sent: number, total: number) => void,
  onTelemetry?: (tUs: number, joints: number[]) => void,
  onSample?: (q: number[]) => void,
): Promise<UploadResult> {
  const enc = new TextEncoder();
  const total = remainingLines.length;
  let sent = 0;
  let done = false;
  while (sent < total && !done) {
    const lines = await readSerialLines(port, 3000);
    let free = 0;
    for (const line of lines) {
      if (line.startsWith('T ')) {
        const tm = /^T (\d+) ((\d+ ){5}\d+)$/.exec(line);
        if (tm && onTelemetry) {
          onTelemetry(Number(tm[1]), tm[2].split(' ').map(Number));
        }
        continue;
      }
      if (line.startsWith('ERR ')) {
        return { sent, total, error: line };
      }
      if (line.startsWith('ACK ')) {
        const n = Number(line.slice(4));
        if (Number.isFinite(n)) free = Math.max(free, n);
      }
      if (line === 'DONE') {
        done = true;
      }
    }
    if (free <= 0) {
      continue;
    }
    const toSend = Math.min(free, total - sent);
    for (let i = 0; i < toSend; i++) {
      await awaitSendSerial(port, enc.encode(remainingLines[sent + i] + '\n'));
      const q = sampleQ(remainingLines[sent + i]);
      if (q) onSample?.(q);
    }
    sent += toSend;
    onProgress?.(sent, total);
  }
  return { sent, total };
}

export async function uploadManifest(
  port: SerialPort,
  chunks: string[][],
  chunkMax: number,
  onProgress?: (sent: number, total: number) => void,
  onSample?: (q: number[]) => void,
): Promise<UploadResult> {
  const enc = new TextEncoder();
  let sent = 0;
  const total = chunks.reduce((acc, c) => acc + c.length, 0);
  // El ring del firmware tiene 2×chunkMax (48) slots: subir más antes del EXECUTE
  // desborda el store (V2_ERR_BAD_STATE). El web sube hasta el ring y el resto
  // se transmite tras EXECUTE vía continueManifestUpload.
  const RING_LIMIT = 2 * chunkMax;
  for (const chunk of chunks) {
    for (const line of chunk) {
      if (sent >= RING_LIMIT) return { sent, total };
      await awaitSendSerial(port, enc.encode(line + '\n'));
      const q = sampleQ(line);
      if (q) onSample?.(q);
      sent += 1;
      onProgress?.(sent, total);
    }
    const windowUsed = sent % chunkMax;
    if (windowUsed === 0 && sent < total) {
      // stopWhen: retornar apenas llegue el ACK (o un ERR), sin esperar el batch
      // completo — antes cada ventana sumaba ~1.2s de espera al arranque.
      const lines = await readSerialLines(port, 1200, (l) => l.startsWith('ACK ') || l.startsWith('ERR '));
      const err = lines.find((l) => l.startsWith('ERR '));
      if (err) {
        return { sent, total, error: err };
      }
      const acked = lines.some((l) => l.startsWith('ACK '));
      if (!acked) {
        return { sent, total };
      }
    }
  }
  return { sent, total };
}
