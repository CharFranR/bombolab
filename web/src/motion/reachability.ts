/**
 * Drawing-workspace reachability for the FABRI Creator drawing mode.
 *
 * Source of truth is the existing constrained drawing IK solver
 * (`solveDrawingPlaneIk`): a point is reachable iff it converges there within
 * tolerance. This module only *gates* what the drawing pipeline feeds into the
 * motion player + IK; it never changes the IK / planner / wasm movement logic.
 *
 * Two uses:
 *  - `validateDrawingCommands`: pre-flight gate of a whole trajectory.
 *  - `isReachablePoint`: O(1) sync check used by the playback loop guard.
 *
 * To make a 3000+ point gcode validate instantly, we first build a polar
 * reachability-band boundary table per drawing Z (sweep angles, binary-search
 * the reachable radius interval with the solver), then look the band up in O(1).
 */
import { initWasm, fabriCreator, solveDrawingPlaneIk } from '../wasm';
import type { RobotDef } from '../kinematics/types';
import type { MotionCommandJS } from './commands';

export const DRAW_PLANE_Z = 80;
export const TRAVEL_PLANE_Z = 85;

export interface DrawingArea {
  xMin: number;
  xMax: number;
  yMin: number;
  yMax: number;
}

export interface ReachResult {
  ok: boolean;
  checked: number;
  failures: [number, number, number][];
}

const DEG = Math.PI / 180;
const TWO_PI = 2 * Math.PI;
const MAX_RADIUS = 400;
const COARSE_STEP = 16;
const ANGLE_COUNT = 72; // 5° resolution — smooth boundary, cheap
const REACH_TOLERANCE = 10; // mm, matches App.tsx drawingActive threshold
const ZERO_Q: number[] = [0, 0, 0, 0, 0];

interface Band {
  z: number;
  inner: Float64Array;
  outer: Float64Array;
}
const bands: Band[] = [];

/** The band cache is built once per session from the default robot model
 *  (`fabriCreator`). If the model or calibration changes mid-session the
 *  tables become stale; call this after such a change to rebuild them. */
export function resetReachabilityCache(): void {
  bands.length = 0;
}

function nextTick(): Promise<void> {
  return new Promise((res) => setTimeout(res, 0));
}

function robotForOracle(): RobotDef {
  return fabriCreator();
}

// ─── Solver oracle ─────────────────────────────────────────────────────────

function reaches(robot: RobotDef, p: [number, number, number]): boolean {
  const res = solveDrawingPlaneIk(robot, p, ZERO_Q);
  return res.converged && res.error < REACH_TOLERANCE;
}

function reachesAt(robot: RobotDef, th: number, z: number, r: number): boolean {
  if (r > MAX_RADIUS) return false;
  const p: [number, number, number] = [r * Math.cos(th), r * Math.sin(th), z];
  return reaches(robot, p);
}

// ─── Boundary band construction (one-time per Z) ───────────────────────────

async function ensureBand(z: number): Promise<Band> {
  const existing = bands.find((b) => Math.abs(b.z - z) <= 1.0);
  if (existing) return existing;
  const robot = robotForOracle();
  const b: Band = {
    z,
    inner: new Float64Array(ANGLE_COUNT),
    outer: new Float64Array(ANGLE_COUNT),
  };
  for (let a = 0; a < ANGLE_COUNT; a++) {
    const th = (a / ANGLE_COUNT) * TWO_PI;
    const [ri, ro] = findBand(robot, th, z);
    b.inner[a] = ri;
    b.outer[a] = ro;
    if (a % 12 === 0) await nextTick();
  }
  bands.push(b);
  return b;
}

function refineEdge(
  robot: RobotDef,
  th: number,
  z: number,
  lo: number,
  hi: number,
  findingInner: boolean,
): number {
  for (let i = 0; i < 10; i++) {
    const mid = (lo + hi) / 2;
    if (reachesAt(robot, th, z, mid)) {
      if (findingInner) hi = mid;
      else lo = mid;
    } else if (findingInner) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return findingInner ? hi : lo;
}

function findBand(robot: RobotDef, th: number, z: number): [number, number] {
  // Coarse scan — reachable interval is contiguous, so stop once we pass it.
  let inBand = false;
  let first = -1;
  let last = -1;
  for (let r = 0; r <= MAX_RADIUS; r += COARSE_STEP) {
    const ok = reachesAt(robot, th, z, r);
    if (ok) {
      if (first < 0) first = r;
      last = r;
      inBand = true;
    } else if (inBand) {
      break; // past outer edge
    }
  }
  if (first < 0) return [0, 0];
  const ri = refineEdge(robot, th, z, Math.max(0, first - COARSE_STEP), first, true);
  const ro = refineEdge(
    robot,
    th,
    z,
    last,
    Math.min(MAX_RADIUS, last + COARSE_STEP),
    false,
  );
  return [ri, ro];
}

function bandAt(b: Band, x: number, y: number): { rIn: number; rOut: number } {
  const th = Math.atan2(y, x);
  const a = (((th < 0 ? th + TWO_PI : th) / TWO_PI) * ANGLE_COUNT) % ANGLE_COUNT;
  const i0 = Math.floor(a);
  const i1 = (i0 + 1) % ANGLE_COUNT;
  const f = a - i0;
  return {
    rIn: b.inner[i0] * (1 - f) + b.inner[i1] * f,
    rOut: b.outer[i0] * (1 - f) + b.outer[i1] * f,
  };
}

function bandForZ(z: number): Band | undefined {
  return bands.find((b) => Math.abs(b.z - z) <= 2.0);
}

// ─── Public API ────────────────────────────────────────────────────────────

/** True iff point is inside the reachable drawing band (table-backed). */
export function isReachablePoint(p: [number, number, number]): boolean {
  const b = bandForZ(p[2]);
  if (b) {
    const { rIn, rOut } = bandAt(b, p[0], p[1]);
    const r = Math.hypot(p[0], p[1]);
    return r >= rIn && r <= rOut;
  }
  // Tables not built yet (shouldn't happen during playback, only pre-flight):
  // fall back to a synchronous solver probe.
  return reaches(robotForOracle(), p);
}

export interface ValidateOptions {
  sampleStep?: number;
  tolerance?: number;
  maxFailures?: number;
  onProgress?: (done: number, total: number) => void;
}

/** Pre-flight: validate every segment of a command list is within reach. */
export async function validateDrawingCommands(
  cmds: MotionCommandJS[],
  opts: ValidateOptions = {},
): Promise<ReachResult> {
  await initWasm();
  const step = opts?.sampleStep ?? 4;
  const tol = opts?.tolerance ?? REACH_TOLERANCE;
  const maxFail = opts?.maxFailures ?? 20;

  if (bands.length === 0) await ensureBand(DRAW_PLANE_Z);

  const points: [number, number, number][] = [];
  let prev: [number, number, number] | null = null;
  for (const c of cmds) {
    if (c.type === 'move') {
      if (prev) sampleSegment(prev, c.target, step, points);
      else points.push(c.target);
      prev = c.target;
    }
  }
  if (points.length === 0) return { ok: true, checked: 0, failures: [] };

  const zs = [...new Set(points.map((p) => Math.round(p[2])))];
  for (const z of zs) {
    if (!bandForZ(z)) await ensureBand(z);
  }

  const failures: [number, number, number][] = [];
  for (let i = 0; i < points.length; i++) {
    if (!isReachablePoint(points[i])) {
      if (failures.length < maxFail) failures.push(points[i]);
    }
    if ((i + 1) % 200 === 0) {
      opts?.onProgress?.(i + 1, points.length);
      await nextTick();
    }
  }
  opts?.onProgress?.(points.length, points.length);
  return { ok: failures.length === 0, checked: points.length, failures };
}

function sampleSegment(
  a: [number, number, number],
  b: [number, number, number],
  step: number,
  out: [number, number, number][],
): void {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const dz = b[2] - a[2];
  const len = Math.hypot(dx, dy, dz);
  if (len === 0) {
    out.push(b);
    return;
  }
  const n = Math.max(1, Math.ceil(len / step));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    out.push([a[0] + dx * t, a[1] + dy * t, a[2] + dz * t]);
  }
}

/** Largest axis-aligned rectangle inscribed in the reachable drawing band.
 *  Validated to be safe at BOTH the drawing plane (pen down) and the travel
 *  plane (pen up) so travel moves are never flagged either. */
export async function safeDrawingArea(
  z: number = DRAW_PLANE_Z,
  travelZ: number = TRAVEL_PLANE_Z,
): Promise<DrawingArea> {
  await initWasm();
  await ensureBand(z);
  await ensureBand(travelZ);
  const b = bandForZ(z)!;
  const bt = bandForZ(travelZ)!;
  let best: DrawingArea | null = null;
  for (let cx = 170; cx <= 235; cx += 5) {
    for (let cy = -15; cy <= 15; cy += 5) {
      for (let hx = 20; hx <= 45; hx += 5) {
        for (let hy = 15; hy <= 40; hy += 5) {
          const a = { xMin: cx - hx, xMax: cx + hx, yMin: cy - hy, yMax: cy + hy };
          // must fit at pen-down AND pen-up heights
          if (rectFits(b, a) && rectFits(bt, a)) {
            if (
              !best ||
              (a.xMax - a.xMin) * (a.yMax - a.yMin) >
                (best.xMax - best.xMin) * (best.yMax - best.yMin)
            )
              best = a;
          }
        }
      }
    }
  }
  return best ?? { xMin: 160, xMax: 240, yMin: -35, yMax: 35 };
}

function rectFits(b: Band, a: DrawingArea): boolean {
  const step = 4;
  const xs = range(a.xMin, a.xMax, step);
  const ys = range(a.yMin, a.yMax, step);
  for (const x of xs) if (!pt(b, x, a.yMin) || !pt(b, x, a.yMax)) return false;
  for (const y of ys) if (!pt(b, a.xMin, y) || !pt(b, a.xMax, y)) return false;
  return true;
}

function pt(b: Band, x: number, y: number): boolean {
  const { rIn, rOut } = bandAt(b, x, y);
  const r = Math.hypot(x, y);
  return r >= rIn && r <= rOut;
}

function range(lo: number, hi: number, step: number): number[] {
  const out: number[] = [];
  for (let v = lo; v <= hi + 1e-9; v += step) out.push(v);
  if (Math.abs(out[out.length - 1] - hi) > 1e-9) out.push(hi);
  return out;
}
