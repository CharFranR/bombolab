// ─── Conversión q → servo ──────────────────────────────────────────────
// Misma lógica que Robot::q_to_servo() en Rust

const DEG = 180 / Math.PI;

// fabri_creator servo offsets (90° en rad) y direcciones
const OFFSETS_DEG = [90, 90, 81, 95, 60];
const DIRECTIONS = [-1, -1, 1, -1, -1];

/** Convierte q (rad) a servo angles (grados enteros [10°, 170°]). */
export function qToServoDeg(q: number[]): number[] {
  return q.map((qi, i) => {
    const deg = DIRECTIONS[i] * (qi * DEG) + OFFSETS_DEG[i];
    return Math.round(Math.max(10, Math.min(170, deg)));
  });
}

// ─── Wire format ────────────────────────────────────────────────────────
// Formato: "a1,a2,a3,a4,a5,g\n" — igual que ArduinoNano

export function buildWire(jointsDeg: number[], gripperPct: number): Uint8Array {
  // Gripper servo: 170° (abierto) – 50° (cerrado)
  const g = Math.round(170 - (gripperPct / 100) * 120);
  const str = `${jointsDeg[0]},${jointsDeg[1]},${jointsDeg[2]},${jointsDeg[3]},${jointsDeg[4]},${g}\n`;
  return new TextEncoder().encode(str);
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
