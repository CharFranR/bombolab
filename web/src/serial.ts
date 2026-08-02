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
