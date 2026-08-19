/**
 * Frame-file parsing + Cartesian plotting for the temporary draw-test view.
 *
 * Accepts the two artifacts the app can produce for the physical pipeline:
 *   1. Trace CSV export (Exportar traza): `ts_ms,j1_us,...,j6_us,count`
 *      — the exact µs frames that were sent to the microcontroller.
 *   2. Raw v2 protocol lines: `SAMPLE q1 q2 q3 q4 q5 g dtUs`
 *      — the wire contract itself.
 *
 * Every frame is converted µs → q → FK → tool-tip XY (three.js coords),
 * so the plot shows what the robot was COMMANDED to draw.
 */
import { qUsToQ } from './fkPath';
import { forwardKinematics } from '../wasm';
import type { RobotDef } from '../kinematics/types';
import { TRACE_CSV_HEADER } from './csv';
import { DRAW_PLANE_Z } from './planes';

export interface FramePoint {
  x: number;
  y: number;
  /** True when the TCP sits on the drawing plane (pen touching paper). */
  drawing: boolean;
}

export interface FramePlot {
  drawing: [number, number][];
  travel: [number, number][];
  /**
   * Drawing points split into STROKES (consecutive on-plane runs). The
   * plot must render each stroke as its own polyline: connecting all
   * drawing points in order would draw phantom chords across pen-up
   * gaps (the viewer shows points, so it never has this artifact).
   */
  drawingStrokes: [number, number][][];
  frames: number;
}

/** A classified point in trajectory order. */
export interface TaggedPoint {
  x: number;
  y: number;
  drawing: boolean;
}

/**
 * Split ordered, classified points into drawing strokes. A stroke is a
 * maximal run of consecutive drawing points; it breaks when a travel
 * point intervenes OR when two consecutive drawing points jump further
 * than `gap` mm apart (guards against a missing travel sample between
 * strokes).
 */
export function splitStrokes(points: TaggedPoint[], gap = 4): [number, number][][] {
  const strokes: [number, number][][] = [];
  let current: [number, number][] | null = null;
  let last: [number, number] | null = null;
  for (const p of points) {
    if (!p.drawing) {
      current = null;
      last = null;
      continue;
    }
    const pt: [number, number] = [p.x, p.y];
    if (current === null) {
      current = [pt];
      strokes.push(current);
    } else {
      const d = Math.hypot(pt[0] - last![0], pt[1] - last![1]);
      if (d > gap) {
        current = [pt];
        strokes.push(current);
      } else {
        current.push(pt);
      }
    }
    last = pt;
  }
  return strokes;
}

/** Parse a trace CSV or SAMPLE-line file into (t, q_us) frames. */
export function parseFrameFile(text: string): { t: number; q_us: number[] }[] {
  const trimmed = text.trim();
  if (trimmed.length === 0) throw new Error('archivo vacío');

  const firstLine = trimmed.split('\n')[0].trim();

  if (firstLine === TRACE_CSV_HEADER) {
    return parseTraceCsvFrames(text);
  }
  if (firstLine.startsWith('SAMPLE ')) {
    return parseSampleLines(text);
  }
  throw new Error(
    'formato no reconocido: se espera un trace CSV (ts_ms,j1_us,...) o líneas SAMPLE (SAMPLE q1..q6 dtUs)',
  );
}

function parseTraceCsvFrames(text: string): { t: number; q_us: number[] }[] {
  const out: { t: number; q_us: number[] }[] = [];
  const lines = text.split('\n');
  for (let i = 1; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line) continue;
    const fields = line.split(',');
    if (fields.length !== 8) throw new Error(`fila ${i + 1}: se esperaban 8 campos, hay ${fields.length}`);
    const ts = Number(fields[0]);
    if (!Number.isFinite(ts)) throw new Error(`fila ${i + 1}: ts_ms inválido`);
    const q_us = fields.slice(1, 7).map((f) => Number(f));
    if (q_us.some((v) => !Number.isInteger(v))) {
      throw new Error(`fila ${i + 1}: los valores de joint deben ser enteros (µs)`);
    }
    const count = Number(fields[7]);
    if (!Number.isInteger(count) || count < 1) {
      throw new Error(`fila ${i + 1}: count debe ser un entero positivo`);
    }
    // Expand the dedup count into repeated frames at the same timestamp
    // step (they represent held positions; one plotted point is enough,
    // but keep the weight via repeated entries is wasteful — keep one).
    out.push({ t: ts, q_us });
  }
  return out;
}

function parseSampleLines(text: string): { t: number; q_us: number[] }[] {
  const out: { t: number; q_us: number[] }[] = [];
  const lines = text.split('\n');
  let t = 0;
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].trim();
    if (!line) continue;
    const m = /^SAMPLE ((?:\d+ ){5}\d+) (\d+)$/.exec(line);
    if (!m) throw new Error(`línea ${i + 1}: SAMPLE malformada`);
    const q_us = m[1].split(' ').map(Number);
    const dtUs = Number(m[2]);
    if (q_us.some((v) => v < 500 || v > 2400)) {
      throw new Error(`línea ${i + 1}: joint fuera de rango [500, 2400]`);
    }
    out.push({ t, q_us });
    t += dtUs / 1e6;
  }
  if (out.length === 0) throw new Error('no hay líneas SAMPLE');
  return out;
}

/** Convert frames to tool-tip XY positions via µs → q → FK. */
export function framesToPlot(robot: RobotDef, frames: { q_us: number[] }[], planeZ = DRAW_PLANE_Z): FramePlot {
  const tagged: TaggedPoint[] = [];
  for (const f of frames) {
    const q = qUsToQ(f.q_us);
    const segments = robot.segments.map((seg, i) => ({ ...seg, q: q[i] ?? seg.q }));
    const fk = forwardKinematics(segments, robot.baseTransform, robot.toolTransform);
    // DH translation lives in [3],[7],[11]. Top-down XY view of the drawing
    // plane: horizontal axis = DH x, vertical axis = DH y.
    const x = fk.ee[3];
    const y = fk.ee[7];
    const z = fk.ee[11];
    tagged.push({ x, y, drawing: Math.abs(z - planeZ) < 2.0 });
  }
  const drawing = tagged.filter((p) => p.drawing).map((p) => [p.x, p.y] as [number, number]);
  const travel = tagged.filter((p) => !p.drawing).map((p) => [p.x, p.y] as [number, number]);
  return { drawing, travel, drawingStrokes: splitStrokes(tagged), frames: frames.length };
}
