// ─── Conversión q → servo ──────────────────────────────────────────────
// Misma lógica que Robot::q_to_servo() en Rust

const DEG = 180 / Math.PI;

// fabri_creator servo offsets (90° en rad) y direcciones
const OFFSETS_DEG = [90, 90, 81, 95, 60];
const DIRECTIONS = [-1, -1, 1, -1, -1];

/** Convierte q (rad) a servo angles (grados enteros [5°, 175°]). */
export function qToServoDeg(q: number[]): number[] {
  return q.map((qi, i) => {
    const deg = DIRECTIONS[i] * (qi * DEG) + OFFSETS_DEG[i];
    return Math.round(Math.max(5, Math.min(175, deg)));
  });
}

// ─── Wire format ────────────────────────────────────────────────────────
// Formato: "a1,a2,a3,a4,a5,g\n" — igual que ArduinoNano

/** Gripper percent (0–100, 100 = closed) → servo degrees [50, 170]. */
export function gripperToServo(gripperPct: number): number {
  return Math.round(170 - (gripperPct / 100) * 120);
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
