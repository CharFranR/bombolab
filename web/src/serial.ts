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

export function sendSerial(port: SerialPort, data: Uint8Array): void {
  const writer = port.writable?.getWriter();
  if (!writer) return;
  writer.write(data);
  writer.releaseLock();
}


export async function readSerialLines(port: SerialPort, timeoutMs = 1500): Promise<string[]> {
  if (!port.readable) return [];
  const reader = port.readable.getReader();
  const decoder = new TextDecoder();
  const lines: string[] = [];
  let buf = '';
  const deadline = Date.now() + timeoutMs;
  try {
    while (Date.now() < deadline && lines.length < 8) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += decoder.decode(value, { stream: true });
      let idx: number;
      while ((idx = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, idx).replace(/\r$/, '');
        buf = buf.slice(idx + 1);
        lines.push(line);
      }
    }
  } finally {
    reader.releaseLock();
  }
  return lines;
}

export async function handshakeV2(port: SerialPort): Promise<number | Error> {
  sendSerial(port, new TextEncoder().encode('HELLO 2\n'));
  const deadline = Date.now() + 5000;
  while (Date.now() < deadline) {
    const lines = await readSerialLines(port, 1500);
    for (const line of lines) {
      if (line.startsWith('HELLO 2 OK CHUNK_MAX')) {
        const m = /CHUNK_MAX (\d+)/.exec(line);
        if (m) {
          return Number(m[1]);
        }
      }
      if (line.startsWith('ERR ')) {
        return new Error('el firmware rechazó HELLO 2: ' + line);
      }
    }
  }
  return new Error('sin respuesta HELLO 2');
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
      sendSerial(port, enc.encode(remainingLines[sent + i] + '\n'));
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
): Promise<UploadResult> {
  const enc = new TextEncoder();
  let sent = 0;
  const total = chunks.reduce((acc, c) => acc + c.length, 0);
  for (const chunk of chunks) {
    for (const line of chunk) {
      sendSerial(port, enc.encode(line + '\n'));
      sent += 1;
      onProgress?.(sent, total);
    }
    const windowUsed = sent % chunkMax;
    if (windowUsed === 0 && sent < total) {
      const lines = await readSerialLines(port, 1200);
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
