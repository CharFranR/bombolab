/**
 * G-code → MotionCommandJS parser for the drawing mode.
 *
 * Supports the subset of G-code used by typical 2D pen-plotter / CNC files:
 *   G0/G1 (linear moves), G90/G91 (abs/relative), G21/G20 (mm/in),
 *   G92 (set origin), G4 (dwell), M3/M5 (spindle/pen down/up).
 * G2/G3 (arcs) are rejected with a warning in v1.
 *
 * Pen semantics auto-detect:
 *   - if the file uses M3/M5, those control the pen (Z is ignored);
 *   - otherwise the lowest Z value is the drawing plane (pen down) and any
 *     higher Z is travel (pen up);
 *   - if neither, the pen stays down for every move.
 *
 * X/Y are autofit (scale + center) so the drawing's bounding box fits the
 * robot's drawing area; Z is mapped to planeZ (draw) / travelZ (lift).
 */

import type { MotionCommandJS } from './commands';

export interface GcodeOptions {
  /** Drawing plane Z (mm). Default 80. */
  planeZ?: number;
  /** Travel Z with the pen lifted (mm). Default planeZ + 5. */
  travelZ?: number;
  /** Feed used when the file has no F word (mm/s). Default 40. */
  defaultSpeed?: number;
  /** Upper clamp for feed (mm/s). Default 100. */
  maxSpeed?: number;
  /** Target drawing area for autofit. Defaults to the robot's area. */
  area?: { xMin: number; xMax: number; yMin: number; yMax: number };
  /** Center + scale the drawing to fit `area`. Default true. */
  autofit?: boolean;
}

export interface GcodeBounds {
  min: [number, number];
  max: [number, number];
}

export interface GcodeParseResult {
  commands: MotionCommandJS[];
  warnings: string[];
  bounds: GcodeBounds | null;
  moveCount: number;
}

interface RawMove {
  kind: 'move';
  x: number;
  y: number;
  z: number;
  speed: number;
  /** Spindle/pen state snapshot at this move (M3 = on = drawing). */
  spindle: boolean;
}

interface RawWait {
  kind: 'wait';
  duration: number;
}

type RawEvent = RawMove | RawWait;

export function parseGcode(text: string, opts: GcodeOptions = {}): GcodeParseResult {
  const planeZ = opts.planeZ ?? 80;
  const travelZ = opts.travelZ ?? planeZ + 5;
  const defaultSpeed = opts.defaultSpeed ?? 40;
  const maxSpeed = opts.maxSpeed ?? 100;
  const area = opts.area ?? { xMin: 160, xMax: 240, yMin: -35, yMax: 35 };

  const warnings: string[] = [];

  // ── Pass 1: tokenize into raw events ──────────────────────────────────
  let relative = false;
  let perUnit = 1; // mm per gcode unit (G21=1, G20=25.4)
  let pos: [number, number, number] = [0, 0, 0];
  let speed = defaultSpeed;
  let spindleOn = false;
  let usedSpindle = false;
  let hasZ = false;
  let zMin = Infinity;
  let modalG = 1;

  const rawEvents: RawEvent[] = [];

  const lines = stripComments(text).split('\n');

  for (const rawLine of lines) {
    const line = rawLine.trim();
    if (!line) continue;

    const coords: Partial<Record<'X' | 'Y' | 'Z', number>> = {};
    let sawCoord = false;
    let currentG: number | null = null;
    let hasG = false;
    let hasM3 = false;
    let hasM5 = false;
    let isArc = false;
    let isWait = false;
    let homing = false;
    let waitP: number | null = null;
    let waitS: number | null = null;
    let ended = false;

    for (const m of line.matchAll(/([A-Za-z])\s*(-?[\d.]+)?/g)) {
      const code = m[1].toUpperCase();
      const value = m[2] !== undefined ? parseFloat(m[2]) : NaN;
      switch (code) {
        case 'G': {
          const g = Math.round(value);
          hasG = true;
          currentG = g;
          if (g === 0 || g === 1) modalG = g;
          else if (g === 2 || g === 3) isArc = true;
          else if (g === 4) isWait = true;
          else if (g === 20) perUnit = 25.4;
          else if (g === 21) perUnit = 1;
          else if (g === 90) relative = false;
          else if (g === 91) relative = true;
          else if (g === 28) homing = true;
          break;
        }
        case 'M': {
          const mc = Math.round(value);
          if (mc === 3) {
            usedSpindle = true;
            hasM3 = true;
          } else if (mc === 5) {
            usedSpindle = true;
            hasM5 = true;
          } else if (mc === 2 || mc === 30) {
            ended = true;
          }
          break;
        }
        case 'X':
          coords.X = value * perUnit;
          sawCoord = true;
          break;
        case 'Y':
          coords.Y = value * perUnit;
          sawCoord = true;
          break;
        case 'Z':
          coords.Z = value * perUnit;
          sawCoord = true;
          hasZ = true;
          break;
        case 'F':
          if (!Number.isNaN(value) && value > 0) {
            speed = Math.max(1, Math.min(maxSpeed, (value * perUnit) / 60));
          }
          break;
        case 'P':
          waitP = value;
          break;
        case 'S':
          waitS = value;
          break;
        default:
          break; // N, T, I, J, R, E, ...
      }
    }

    if (ended) {
      warnings.push('M2/M30 (fin de programa) — se ignora el resto del archivo');
      break;
    }
    if (homing) warnings.push(`G28 (homing) ignorado: "${line}"`);
    if (isArc) {
      warnings.push(`G2/G3 (arco) no soportado — línea omitida: "${line}"`);
      continue;
    }
    if (isWait) {
      const duration = waitP !== null ? waitP : waitS !== null ? waitS / 1000 : 0;
      if (duration > 0) rawEvents.push({ kind: 'wait', duration });
      continue;
    }
    // G92: redefine the current position.
    if (currentG === 92 && sawCoord) {
      if (coords.X !== undefined) pos[0] = coords.X;
      if (coords.Y !== undefined) pos[1] = coords.Y;
      if (coords.Z !== undefined) pos[2] = coords.Z;
      continue;
    }
    // Linear move (modal G0/G1) with at least one coordinate word.
    if (hasG ? modalG === 0 || modalG === 1 : sawCoord) {
      if (sawCoord) {
        const nx = coords.X !== undefined ? (relative ? pos[0] + coords.X : coords.X) : pos[0];
        const ny = coords.Y !== undefined ? (relative ? pos[1] + coords.Y : coords.Y) : pos[1];
        const nz = coords.Z !== undefined ? (relative ? pos[2] + coords.Z : coords.Z) : pos[2];
        pos = [nx, ny, nz];
        if (nz < zMin) zMin = nz;
        rawEvents.push({ kind: 'move', x: nx, y: ny, z: nz, speed, spindle: spindleOn });
      }
    }
    // M3/M5 (typically on their own line).
    if (hasM3) spindleOn = true;
    if (hasM5) spindleOn = false;
  }

  // ── Pass 2: decide pen per move, collect drawn bounds ─────────────────
  const mode: 'spindle' | 'z' | 'always' = usedSpindle
    ? 'spindle'
    : hasZ
      ? 'z'
      : 'always';

  const ordered: { pen?: 'down' | 'up'; move?: [number, number, number]; speed?: number; wait?: number }[] = [];
  const boundsPts: [number, number][] = [];
  let currentPen = false;
  let lastPos: [number, number] | null = null;

  for (const ev of rawEvents) {
    if (ev.kind === 'wait') {
      ordered.push({ wait: ev.duration });
      continue;
    }
    const pen = mode === 'spindle' ? ev.spindle : mode === 'z' ? ev.z <= zMin + 0.1 : true;
    if (pen !== currentPen) {
      ordered.push({ pen: pen ? 'down' : 'up' });
      // A stroke starts where the pen went down — include that point so
      // autofit covers the full segment, not just the move targets.
      if (pen && lastPos) boundsPts.push(lastPos);
      currentPen = pen;
    }
    if (pen) boundsPts.push([ev.x, ev.y]);
    lastPos = [ev.x, ev.y];
    ordered.push({ move: [ev.x, ev.y, pen ? planeZ : travelZ], speed: ev.speed });
  }

  // ── Pass 3: autofit + emit MotionCommandJS ────────────────────────────
  const fit = opts.autofit !== false && boundsPts.length > 0
    ? computeAutofit(boundsPts, area)
    : { scale: 1, tx: 0, ty: 0 };

  const commands: MotionCommandJS[] = [];
  let moveCount = 0;
  for (const ev of ordered) {
    if (ev.pen) commands.push(ev.pen === 'down' ? { type: 'penDown' } : { type: 'penUp' });
    if (ev.wait !== undefined) commands.push({ type: 'wait', duration: ev.wait });
    if (ev.move) {
      commands.push({
        type: 'move',
        target: [ev.move[0] * fit.scale + fit.tx, ev.move[1] * fit.scale + fit.ty, ev.move[2]],
        speed: ev.speed ?? defaultSpeed,
      });
      moveCount++;
    }
  }

  let bounds: GcodeBounds | null = null;
  if (boundsPts.length > 0) {
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const [x, y] of boundsPts) {
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;
    }
    bounds = { min: [minX, minY], max: [maxX, maxY] };
  }

  return { commands, warnings, bounds, moveCount };
}

function computeAutofit(
  points: [number, number][],
  area: { xMin: number; xMax: number; yMin: number; yMax: number },
): { scale: number; tx: number; ty: number } {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const [x, y] of points) {
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
  }
  const boxW = maxX - minX;
  const boxH = maxY - minY;
  const areaW = area.xMax - area.xMin;
  const areaH = area.yMax - area.yMin;
  const availW = Math.max(areaW - 10, 1); // 5 mm margin each side
  const availH = Math.max(areaH - 10, 1);
  let scale = 1;
  if (boxW > 1e-6 && boxH > 1e-6) scale = Math.min(availW / boxW, availH / boxH);
  else if (boxW > 1e-6) scale = availW / boxW;
  else if (boxH > 1e-6) scale = availH / boxH;
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;
  const targetCx = (area.xMin + area.xMax) / 2;
  const targetCy = (area.yMin + area.yMax) / 2;
  return { scale, tx: targetCx - cx * scale, ty: targetCy - cy * scale };
}

/** Remove `;` line comments and `(...)` block comments. */
function stripComments(text: string): string {
  let out = '';
  let inParen = false;
  let i = 0;
  const n = text.length;
  while (i < n) {
    const ch = text[i];
    if (inParen) {
      if (ch === ')') inParen = false;
      i++;
      continue;
    }
    if (ch === '(') {
      inParen = true;
      i++;
      continue;
    }
    if (ch === ';') {
      while (i < n && text[i] !== '\n') i++;
      continue;
    }
    out += ch;
    i++;
  }
  return out;
}
