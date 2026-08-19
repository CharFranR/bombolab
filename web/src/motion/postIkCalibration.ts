/**
 * Cartesian compensation applied after the nominal IK target is authored and
 * before any reachability, singularity, IK, playback, or manifest work.
 *
 * The calibration file describes an affine model of the measured Cartesian
 * position: measured = A * commanded + b.  Therefore a desired measured point
 * is converted back to the commanded point with A⁻¹ * (desired - b).
 * `speed` in MotionCommandJS is mm/s; the velocity coefficient in the JSON was
 * fitted against G-code feed rates in mm/min, so this module converts it with
 * `speed * 60` before evaluating the factor.
 */
import type { MotionCommandJS } from './commands';

export interface PostIkCalibrationPoint {
  center_x: number;
  center_y: number;
  a11: number;
  a12: number;
  a21: number;
  a22: number;
  b_x: number;
  b_y: number;
}

export interface PostIkCalibration {
  version: 1;
  backlash_offset_x_mm: number;
  k_velocity_factor: number;
  calibration_points: PostIkCalibrationPoint[];
}

export interface AffineMatrix2D {
  a11: number;
  a12: number;
  a21: number;
  a22: number;
}

export type CartesianPoint = [number, number, number];

const CALIBRATION_URL = '/post-ik-calibration.json';
export const X_MIN = 210;
export const X_MAX = 310;
export const Y_MIN = -70;
export const Y_MAX = -20;
const EXACT_CENTER_EPSILON = 1e-9;
const IDW_POWER = 2;
const MAX_NEIGHBORS = 4;

function isFiniteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}

function requireFiniteNumber(value: unknown, name: string): number {
  if (!isFiniteNumber(value)) throw new Error(`post-IK calibration: ${name} must be finite`);
  return value;
}

function matrixOf(point: PostIkCalibrationPoint): AffineMatrix2D {
  return { a11: point.a11, a12: point.a12, a21: point.a21, a22: point.a22 };
}

export function determinant2x2(matrix: AffineMatrix2D): number {
  return matrix.a11 * matrix.a22 - matrix.a12 * matrix.a21;
}

/** Invert an affine 2×2 linear part, failing closed for singular/reflected data. */
export function invertAffineMatrix(matrix: AffineMatrix2D): AffineMatrix2D {
  const determinant = determinant2x2(matrix);
  if (!(determinant > 0) || !Number.isFinite(determinant)) {
    throw new Error(`post-IK calibration: matrix determinant must be positive (got ${determinant})`);
  }
  return {
    a11: matrix.a22 / determinant,
    a12: -matrix.a12 / determinant,
    a21: -matrix.a21 / determinant,
    a22: matrix.a11 / determinant,
  };
}

/** Alias kept descriptive for callers that use the mathematical terminology. */
export const inverseAffineMatrix = invertAffineMatrix;

function validatePoint(value: unknown, index: number): PostIkCalibrationPoint {
  if (!value || typeof value !== 'object') {
    throw new Error(`post-IK calibration: calibration_points[${index}] must be an object`);
  }
  const point = value as Record<string, unknown>;
  const names = ['center_x', 'center_y', 'a11', 'a12', 'a21', 'a22', 'b_x', 'b_y'] as const;
  const numbers = Object.fromEntries(
    names.map((name) => [name, requireFiniteNumber(point[name], `calibration_points[${index}].${name}`)]),
  ) as Record<(typeof names)[number], number>;
  const matrix = matrixOf(numbers as PostIkCalibrationPoint);
  if (!(determinant2x2(matrix) > 0)) {
    throw new Error(`post-IK calibration: calibration_points[${index}] has a non-positive determinant`);
  }
  return numbers as PostIkCalibrationPoint;
}

/** Parse and validate untrusted JSON before allowing it into the trajectory. */
export function validatePostIkCalibration(value: unknown): PostIkCalibration {
  if (!value || typeof value !== 'object') {
    throw new Error('post-IK calibration: document must be an object');
  }
  const data = value as Record<string, unknown>;
  if (data.version !== 1) {
    throw new Error(`post-IK calibration: unsupported version ${String(data.version)}`);
  }
  const backlash = requireFiniteNumber(data.backlash_offset_x_mm, 'backlash_offset_x_mm');
  const kVelocity = requireFiniteNumber(data.k_velocity_factor, 'k_velocity_factor');
  if (backlash < 0) throw new Error('post-IK calibration: backlash_offset_x_mm must be non-negative');
  if (!Array.isArray(data.calibration_points) || data.calibration_points.length === 0) {
    throw new Error('post-IK calibration: calibration_points must not be empty');
  }
  const points = data.calibration_points.map(validatePoint);
  return {
    version: 1,
    backlash_offset_x_mm: backlash,
    k_velocity_factor: kVelocity,
    calibration_points: points,
  };
}

export async function loadPostIkCalibration(
  url = CALIBRATION_URL,
  fetchImpl: typeof fetch = fetch,
): Promise<PostIkCalibration> {
  const response = await fetchImpl(url);
  if (!response.ok) throw new Error(`post-IK calibration: HTTP ${response.status}`);
  return validatePostIkCalibration(await response.json());
}

function applyAffine(matrix: AffineMatrix2D, bX: number, bY: number, point: [number, number]): [number, number] {
  return [
    matrix.a11 * point[0] + matrix.a12 * point[1] + bX,
    matrix.a21 * point[0] + matrix.a22 * point[1] + bY,
  ];
}

function averagePoints(points: PostIkCalibrationPoint[], weights?: number[]): PostIkCalibrationPoint {
  const ws = weights ?? points.map(() => 1);
  const total = ws.reduce((sum, weight) => sum + weight, 0);
  const average = (name: keyof PostIkCalibrationPoint): number =>
    points.reduce((sum, point, index) => sum + point[name] * ws[index], 0) / total;
  return {
    center_x: average('center_x'),
    center_y: average('center_y'),
    a11: average('a11'),
    a12: average('a12'),
    a21: average('a21'),
    a22: average('a22'),
    b_x: average('b_x'),
    b_y: average('b_y'),
  };
}

export function isWithinCalibrationRegion(x: number, y: number): boolean {
  return isFiniteNumber(x) && isFiniteNumber(y)
    && x >= X_MIN && x <= X_MAX && y >= Y_MIN && y <= Y_MAX;
}

function correctedXY(target: CartesianPoint, calibration: PostIkCalibration): [number, number] {
  const candidates = calibration.calibration_points
    .map((point) => ({
      point,
      distanceSquared: (target[0] - point.center_x) ** 2 + (target[1] - point.center_y) ** 2,
    }))
    .sort((a, b) => a.distanceSquared - b.distanceSquared)
    .slice(0, MAX_NEIGHBORS);

  // Zero-distance duplicates are one local calibration sample, not a reason
  // to pick whichever duplicate happened to occur first in the JSON.
  const exact = candidates.filter(({ distanceSquared }) => distanceSquared <= EXACT_CENTER_EPSILON);
  const local = exact.length > 0
    ? averagePoints(exact.map(({ point }) => point))
    : averagePoints(
        candidates.map(({ point }) => point),
        candidates.map(({ distanceSquared }) => 1 / distanceSquared ** (IDW_POWER / 2)),
      );
  const inverse = invertAffineMatrix(matrixOf(local));
  // The calibration models measured = A·commanded + b. Solving for the
  // commanded point that lands on the desired measured target:
  //   commanded = A⁻¹·(target − b)
  // The previous code applied A⁻¹ only to `target` and then subtracted b
  // untransformed (A⁻¹·target − b), which is wrong whenever A differs from
  // the identity. For the recentralized convention b = center − A·center this
  // is equivalent to center + A⁻¹·(target − center): the center stays fixed.
  return applyAffine(inverse, 0, 0, [target[0] - local.b_x, target[1] - local.b_y]);
}

export function velocityCompensationFactor(speedMmPerSecond: number, calibration: PostIkCalibration): number {
  requireFiniteNumber(speedMmPerSecond, 'speed');
  // The JSON k was fitted with F in mm/min, while MotionCommandJS stores mm/s.
  return 1 + calibration.k_velocity_factor * (speedMmPerSecond * 60 - 1200);
}

export interface CompensateMotionOptions {
  /** Current TCP, used as the beginning of the first stretched segment. */
  start?: CartesianPoint;
}

/** Compensate every move while preserving penUp, penDown and wait verbatim. */
export function compensateMotionCommands(
  commands: MotionCommandJS[],
  calibration: PostIkCalibration,
  options: CompensateMotionOptions = {},
): MotionCommandJS[] {
  let segmentStart: CartesianPoint | null = options.start ? [...options.start] : null;
  let previousMoveInRegion: boolean | null = options.start
    ? isWithinCalibrationRegion(options.start[0], options.start[1])
    : null;
  let previousDirectionX: -1 | 0 | 1 = 0;
  const out: MotionCommandJS[] = [];

  for (const command of commands) {
    if (command.type !== 'move') {
      // Semantic commands do not move the TCP and must not reset backlash history.
      out.push({ ...command });
      continue;
    }

    const moveInRegion = isWithinCalibrationRegion(command.target[0], command.target[1]);
    if (!moveInRegion || previousMoveInRegion === false) {
      // The calibration is only valid inside its explicit workspace region.
      // A segment crossing the boundary is left entirely uncalibrated so
      // affine, velocity, and backlash corrections cannot leak across it.
      previousDirectionX = 0;
      previousMoveInRegion = moveInRegion;
      segmentStart = [...command.target];
      out.push({ type: 'move', target: [...command.target], speed: command.speed });
      continue;
    }

    const corrected: CartesianPoint = [...correctedXY(command.target, calibration), command.target[2]];
    const factor = velocityCompensationFactor(command.speed, calibration);
    const start = segmentStart ?? corrected;
    const stretched: CartesianPoint = [
      start[0] + (corrected[0] - start[0]) * factor,
      start[1] + (corrected[1] - start[1]) * factor,
      start[2] + (corrected[2] - start[2]) * factor,
    ];
    const dx = stretched[0] - start[0];
    const directionX: -1 | 0 | 1 = dx > EXACT_CENTER_EPSILON ? 1 : dx < -EXACT_CENTER_EPSILON ? -1 : 0;
    if (directionX !== 0 && previousDirectionX !== 0 && directionX !== previousDirectionX) {
      // Backlash convention: on reversal, shift X in the NEW travel direction.
      // Pen and wait commands intentionally do not alter this direction state.
      stretched[0] += directionX * calibration.backlash_offset_x_mm;
    }
    if (directionX !== 0) previousDirectionX = directionX;
    previousMoveInRegion = true;
    segmentStart = stretched;
    out.push({ type: 'move', target: stretched, speed: command.speed });
  }
  return out;
}

/** Public point-level API used by tests and diagnostics. */
export function compensateTarget(target: CartesianPoint, calibration: PostIkCalibration): CartesianPoint {
  if (!isWithinCalibrationRegion(target[0], target[1])) return [...target];
  const [x, y] = correctedXY(target, calibration);
  return [x, y, target[2]];
}
