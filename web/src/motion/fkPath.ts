/**
 * Reconstruct the ACTUAL Cartesian path the robot will draw from the
 * IK-resolved servo plan (`PlanSample.q_us`).
 *
 * Each sample's servo pulse widths are converted back to joint angles q
 * and pushed through forward kinematics, so the resulting path reflects
 * the REAL IK resolution — including its discontinuities, quantization
 * and smoothing — not the ideal gcode geometry shown by `drawingPath`.
 */
import { servoUsToDeg } from '../serial';
import { forwardKinematics } from '../wasm';
import type { RobotDef } from '../kinematics/types';
import type { PlanSample } from './planTimeline';
import { DRAW_PLANE_Z } from './planes';

// Servo mapping constants (mirror of serial.ts: OFFSETS_DEG / DIRECTIONS).
const DEG = 180 / Math.PI;
const OFFSETS_DEG = [90, 90, 81, 95, 60];
const DIRECTIONS = [-1, -1, 1, -1, -1];

/** Convert servo pulse widths (µs, first 5 channels) back to joint angles q (rad). */
export function qUsToQ(qUs: number[]): number[] {
  return qUs.slice(0, 5).map((us, i) => {
    const deg = servoUsToDeg(us);
    return (deg - OFFSETS_DEG[i]) / (DIRECTIONS[i] * DEG);
  });
}

/** DH (x, y, z) → three.js (x, z, y) — same mapping as robotToThree/framePose. */
function toThree(p: [number, number, number]): [number, number, number] {
  return [p[0], p[2], p[1]];
}

/** Distance from point p to segment [a, b] (3D). */
function distToSegment(
  p: [number, number, number],
  a: [number, number, number],
  b: [number, number, number],
): number {
  const abx = b[0] - a[0];
  const aby = b[1] - a[1];
  const abz = b[2] - a[2];
  const len2 = abx * abx + aby * aby + abz * abz;
  if (len2 <= 1e-12) {
    return Math.hypot(p[0] - a[0], p[1] - a[1], p[2] - a[2]);
  }
  let t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby + (p[2] - a[2]) * abz) / len2;
  t = Math.max(0, Math.min(1, t));
  const qx = a[0] + t * abx;
  const qy = a[1] + t * aby;
  const qz = a[2] + t * abz;
  return Math.hypot(p[0] - qx, p[1] - qy, p[2] - qz);
}

/**
 * Max deviation (mm) of `path` from the polyline `reference` (same coord
 * space). Every point of `path` is measured against the closest point on
 * the reference polyline. Used to quantify how far the IK-resolved servo
 * path strays from the ideal gcode trace.
 */
export function maxDeviationToPath(
  path: [number, number, number][],
  reference: [number, number, number][],
): number {
  if (path.length === 0 || reference.length === 0) return 0;
  let max = 0;
  for (const p of path) {
    let best = Infinity;
    for (let i = 0; i < reference.length - 1; i++) {
      const d = distToSegment(p, reference[i], reference[i + 1]);
      if (d < best) best = d;
    }
    // Also compare against the last point (degenerate single-point reference).
    if (reference.length === 1) best = distToSegment(p, reference[0], reference[0]);
    if (best > max) max = best;
  }
  return max;
}

/**
 * IK-resolved path split by pen state, in three.js coords.
 * `drawing` = samples while the pen is down (or whose TCP sits on the
 * drawing plane — belt and braces for gcode without explicit penDown).
 * `travel` = everything else (pen up / off-plane).
 *
 * NO sample is dropped: the full commanded trajectory is always
 * represented, so the overlay covers the whole path and the deviation
 * metric is computed over every drawing point.
 */
export interface IkPathSplit {
  drawing: [number, number, number][];
  travel: [number, number, number][];
}

export function ikPathFromSamples(
  robot: RobotDef,
  samples: PlanSample[],
  planeZ = DRAW_PLANE_Z,
): IkPathSplit {
  const drawing: [number, number, number][] = [];
  const travel: [number, number, number][] = [];
  for (const s of samples) {
    const q = qUsToQ(s.q_us);
    const segments = robot.segments.map((seg, i) => ({ ...seg, q: q[i] ?? seg.q }));
    const fk = forwardKinematics(segments, robot.baseTransform, robot.toolTransform);
    const p = toThree([fk.ee[3], fk.ee[7], fk.ee[11]]);
    const onPlane = Math.abs(fk.ee[11] - planeZ) < 2.0;
    if (s.penDown || onPlane) {
      drawing.push(p);
    } else {
      travel.push(p);
    }
  }
  return { drawing, travel };
}
