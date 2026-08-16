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

export function sendSerial(port: SerialPort | null, data: Uint8Array): void {
  if (!port) return;
  const writer = port.writable?.getWriter();
  if (!writer) return;
  writer.write(data);
  writer.releaseLock();
}


export async function readSerialLines(
  port: SerialPort,
  timeoutMs = 1500,
  signal?: AbortSignal,
): Promise<string[]> {
  if (!port.readable) return [];
  const reader = port.readable.getReader();
  const decoder = new TextDecoder();
  const lines: string[] = [];
  let buf = '';
  const deadline = Date.now() + timeoutMs;
  let interrupted = false;
  // Interrupting a pending read(): releaseLock() rejects the in-flight
  // read() (spec: whatwg/streams#1168) WITHOUT closing the stream, so a
  // later getReader() can resume on the same port.
  const interrupt = () => {
    try {
      reader.releaseLock();
    } catch {
      // reader already released — nothing to interrupt
    }
  };
  if (signal) {
    if (signal.aborted) interrupted = true;
    else signal.addEventListener('abort', interrupt, { once: true });
  }
  try {
    while (!interrupted && lines.length < 8) {
      const remaining = deadline - Date.now();
      if (remaining <= 0) break;
      const timer = setTimeout(interrupt, remaining);
      try {
        const { value, done } = await reader.read();
        if (done) break;
        buf += decoder.decode(value, { stream: true });
        let idx: number;
        while ((idx = buf.indexOf('\n')) >= 0) {
          const line = buf.slice(0, idx).replace(/\r$/, '');
          buf = buf.slice(idx + 1);
          lines.push(line);
        }
      } catch {
        // read() interrupted by the timeout/abort — return what we have
        interrupted = true;
        break;
      } finally {
        clearTimeout(timer);
      }
    }
  } finally {
    signal?.removeEventListener('abort', interrupt);
    try {
      reader.releaseLock();
    } catch {
      // already released by the timeout/abort interrupt
    }
  }
  return lines;
}

export async function handshakeV2(port: SerialPort): Promise<number | Error> {
  sendSerial(port, new TextEncoder().encode('HELLO 2\n'));
  const lines = await readSerialLines(port, 2000);
  const hello = lines.find((l) => l.startsWith('HELLO 2 OK CHUNK_MAX'));
  if (!hello) {
    return new Error('sin respuesta HELLO 2: ' + (lines.join(' | ') || 'vacío'));
  }
  const m = /CHUNK_MAX (\d+)/.exec(hello);
  if (!m) {
    return new Error('respuesta HELLO malformada: ' + hello);
  }
  return Number(m[1]);
}

export interface UploadResult {
  sent: number;
  total: number;
  error?: string;
}

export async function continueManifestUpload(
  port: SerialPort,
  remainingLines: string[],
  onProgress?: (sent: number, total: number) => void,
  onTelemetry?: (tUs: number, joints: number[]) => void,
  onDone?: () => void,
  signal?: AbortSignal,
  durationUs = 0,
): Promise<UploadResult> {
  const enc = new TextEncoder();
  const total = remainingLines.length;
  let sent = 0;
  let done = false;
  // DONE is emitted only when the firmware consumes the final sample (v2_tick),
  // i.e. up to the full trajectory time after EXECUTE — keep reading until
  // DONE, bounded by the declared duration so a dead firmware gives up.
  const deadline = Date.now() + durationUs / 1000 + 5000;
  while (!done && !signal?.aborted && Date.now() < deadline) {
    const lines = await readSerialLines(port, 3000, signal);
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
        onDone?.();
      }
    }
    if (free <= 0) {
      continue;
    }
    const toSend = Math.min(free, total - sent);
    for (let i = 0; i < toSend; i++) {
      sendSerial(port, enc.encode(remainingLines[sent + i] + '\n'));
    }
    sent += toSend;
    onProgress?.(sent, total);
  }
  if (!done && !signal?.aborted) return { sent, total, error: 'sin DONE del firmware' };
  return { sent, total };
}

export async function uploadManifest(
  port: SerialPort,
  chunks: string[][],
  chunkMax: number,
  count: number,
  durationUs: number,
  onProgress?: (sent: number, total: number) => void,
  signal?: AbortSignal,
): Promise<UploadResult> {
  const enc = new TextEncoder();
  // Firmware preamble (protocol_v2.cpp handle_manifest): count must match
  // the SAMPLE lines and duration the sum of their dt, or the validator
  // rejects the upload with COUNT_MISMATCH / DURATION_MISMATCH.
  sendSerial(port, enc.encode(`MANIFEST ${count} ${durationUs}\n`));
  let sent = 0;
  // Ring capacity (executor.h: V2_RING_SIZE = 2×V2_CHUNK_MAX): the firmware
  // only ACKs a window while the ring has free slots, so once the ring is
  // full the remaining samples stream after EXECUTE (continueManifestUpload).
  let free = chunkMax * 2;
  const total = chunks.reduce((acc, c) => acc + c.length, 0);
  for (const chunk of chunks) {
    for (const line of chunk) {
      sendSerial(port, enc.encode(line + '\n'));
      sent += 1;
      onProgress?.(sent, total);
    }
    const windowUsed = sent % chunkMax;
    if (windowUsed === 0 && sent < total && !signal?.aborted) {
      const lines = await readSerialLines(port, 1200, signal);
      if (signal?.aborted) return { sent, total };
      const err = lines.find((l) => l.startsWith('ERR '));
      if (err) {
        return { sent, total, error: err };
      }
      const acked = lines.some((l) => l.startsWith('ACK '));
      free -= chunkMax;
      if (!acked) {
        if (free > 0) {
          // A window the ring could accept went unconfirmed — upload failed.
          return { sent, total, error: 'ventana sin ACK: el firmware no confirmó la carga' };
        }
        // Ring full: pre-EXECUTE upload ends here; the rest streams after
        // EXECUTE (continueManifestUpload, ACK-paced).
        break;
      }
    }
  }
  // All samples received pre-EXECUTE → close the manifest with END_UPLOAD and
  // require the validator ACK (count/duration checks run here). When the ring
  // filled instead (partial upload), the rest streams after EXECUTE and the
  // validator is never closed — matching the firmware's RECEIVING→EXECUTE path.
  if (!signal?.aborted && sent === total) {
    sendSerial(port, enc.encode('END_UPLOAD\n'));
    const lines = await readSerialLines(port, 3000, signal);
    const err = lines.find((l) => l.startsWith('ERR '));
    if (err) {
      return { sent, total, error: err };
    }
    if (!lines.some((l) => l.startsWith('ACK '))) {
      return { sent, total, error: 'sin ACK de END_UPLOAD' };
    }
  }
  return { sent, total };
}
