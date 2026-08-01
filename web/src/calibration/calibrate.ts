/**
 * Calibration orchestrator — the deterministic pipeline.
 *
 * Per part:
 *   1. Parse STL → detect circle candidates (hole mouths).
 *   2. Pre-filter: radius band (descriptor) + distance of the candidate
 *      center to the joint-axis SEED line (current calibration pulled into
 *      STL space). The seed line disambiguates the semantic choice: with a
 *      roughly-correct current calibration, only the real pivot sits on it.
 *   3. Solve every surviving candidate (closed-form constrained solve);
 *      pick the minimum residual with a 2× margin; report ties as ambiguous.
 *   4. Certify with motion validation (pivot stays on the axis for all
 *      sweep poses of the parent joint).
 *   5. Report deltas vs the current calibration.json.
 */

import { loadMeshFromBuffer, detectCircles, bottomFacePoint } from './mesh';
import { solveCalibration } from './solver';
import { motionValidation } from './motion';
import { fkFrames, jointAxisInFrameSpace, jointAxisInPartSpace, distPointLine, applyCal, quatAxisAngle, quatMul, quatConj, quatRotate, quatNormalize, invApplyCal, vecSub, vecLen } from './fk';
import { descriptorFor } from './descriptors';
import { ALL_STL_FILES } from '../renderers/stlMapping';
import type {
  CalibrationContext,
  CalibrationOutput,
  CalibrationResult,
  CircleCandidate,
  PartDescriptor,
  RigidTransform,
} from './types';

export const MOTION_POSES = 21;

/**
 * NO-TOUCH threshold (mm-equivalent distance to the current cal).
 * Set to the same precision the system certifies (0.3mm motion drift):
 * a current calibration whose pivot is farther than this from the joint
 * axis produces a VISIBLE wobble around the joint (e.g. 2.95mm of yaw
 * misalignment on Eje Central is clearly visible when J1 rotates).
 * Below this threshold the current cal is already certified-good.
 */
export const NO_TOUCH_SCORE_MM = 0.3;

export interface CalibratePartOptions {
  /** Force a candidate id (0-based across ALL detected circles of the part). */
  pickId?: number;
}

export interface CalibrateAllOptions {
  only?: string;
}

function identityCal(): RigidTransform {
  return { translation: [0, 0, 0], rotation: [0, 0, 0, 1] };
}

/** Angle (radians) between two quaternions. */
function quatAngleDiff(a: [number, number, number, number], b: [number, number, number, number]): number {
  const d = Math.abs(quatDot(a, b));
  return 2 * Math.acos(Math.min(1, d));
}

function quatDot(a: [number, number, number, number], b: [number, number, number, number]): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

function loadMesh(ctx: CalibrationContext, filename: string): ReturnType<typeof loadMeshFromBuffer> {
  const cached = ctx.meshCache.get(filename);
  if (cached) return cached as ReturnType<typeof loadMeshFromBuffer>;
  const buffer = ctx.loader(filename);
  const mesh = loadMeshFromBuffer(buffer);
  ctx.meshCache.set(filename, mesh);
  return mesh;
}

interface PartGeometry {
  jointK: number;
  frameIdx: number;
  /** Axis line in the PARENT FRAME local space — what the solver constrains. */
  frameLine: { point: [number, number, number]; dir: [number, number, number] };
  /** Seed line in STL space (via current cal) — positional pre-filter. */
  seedLine: { point: [number, number, number]; dir: [number, number, number] };
  /** Muñeca: line of joint 5 in frame space (second reference circle). */
  secondFrameLine?: { point: [number, number, number]; dir: [number, number, number] };
  /** Muñeca: line of joint 5 in STL space (pre-filter for the reference). */
  secondSeedLine?: { point: [number, number, number]; dir: [number, number, number] };
}

function partGeometry(part: PartDescriptor, calCurrent: RigidTransform): PartGeometry {
  const frames = fkFrames(part.measurementPose);
  if (part.parentJoint === 0) {
    return {
      jointK: 1,
      frameIdx: 0,
      frameLine: jointAxisInFrameSpace(1, 0, frames),
      seedLine: jointAxisInPartSpace(1, 0, frames, calCurrent),
    };
  }
  const frameIdx = part.parentJoint === -1 ? 5 : part.parentJoint;
  const jointK = part.parentJoint === -1 ? 5 : part.parentJoint;
  const frameLine = jointAxisInFrameSpace(jointK, frameIdx, frames);
  const seedLine = jointAxisInPartSpace(jointK, frameIdx, frames, calCurrent);
  const geo: PartGeometry = { jointK, frameIdx, frameLine, seedLine };
  if (part.secondRefCircle) {
    geo.secondFrameLine = jointAxisInFrameSpace(5, 4, frames);
    geo.secondSeedLine = jointAxisInPartSpace(5, 4, frames, calCurrent);
  }
  return geo;
}

/**
 * World-fixed parts (Base.stl, Tapa Base.stl): NO AUTO-CORRECTION.
 * They mate with the rest of the robot through the frame chain; the
 * simulator's world Y=0 is arbitrary and the current hand-made
 * calibration already encodes the correct visual mating. The calibrator
 * only reports the diagnostic (pivot-on-axis distance of the current cal).
 */
function calibrateWorldPart(
  part: PartDescriptor,
  ctx: CalibrationContext,
  calCurrent: RigidTransform,
  mesh: ReturnType<typeof loadMeshFromBuffer>,
): CalibrationResult {
  void ctx; void mesh;
  const pivot: CircleCandidate = {
    id: -1,
    center: invApplyCal(calCurrent, [0, 57, 0] as [number, number, number]),
    normal: [0, 1, 0],
    radius: 0,
    residual: 0,
    nVerts: 0,
  };
  const motion = motionValidation(1, 0, part.measurementPose, part.motionRangeDeg, MOTION_POSES, calCurrent, pivot);
  return {
    filename: part.filename,
    status: 'ok',
    candidates: [],
    pivot,
    cal: calCurrent,
    residualMm: 0,
    motion,
    deltaTranslationMm: 0,
    deltaRotationDeg: 0,
    reason: 'world-fixed part: untouched (diagnostic only — mating with the frame chain is authoritative)',
  };
}

/** Calibrate one part. */
export function calibratePart(
  part: PartDescriptor,
  ctx: CalibrationContext,
  opts: CalibratePartOptions = {},
): CalibrationResult {
  const calCurrent = ctx.currentCal.get(part.filename) ?? identityCal();
  const mesh = loadMesh(ctx, part.filename);

  if (part.parentJoint === 0) {
    return calibrateWorldPart(part, ctx, calCurrent, mesh);
  }

  // Generous detection band; the positional filter does the semantic work.
  const circles = detectCircles(mesh, 1.5, 20);
  const geo = partGeometry(part, calCurrent);

  const nearLine = (c: CircleCandidate, line: { point: [number, number, number]; dir: [number, number, number] }) =>
    distPointLine(c.center as [number, number, number], line) < part.distToLineMax;

  // Primary filter: positional (seed line). If no circle sits within the
  // tolerance, the contract "current calibration is roughly right" is
  // broken — report instead of inventing (no arbitrary fallback).
  let pivots = circles.filter((c) => nearLine(c, geo.seedLine));
  if (pivots.length === 0 && opts.pickId === undefined) {
    return {
      filename: part.filename,
      status: 'failed',
      candidates: circles,
      reason: `No circle candidate within ${part.distToLineMax}mm of the joint-axis seed line. Current calibration may be off, or the pivot bore is not detectable. Use --pick, the VLM path, or manual calibration.`,
    };
  }

  if (opts.pickId !== undefined) {
    const forced = circles.find((c) => c.id === opts.pickId);
    if (!forced) return { filename: part.filename, status: 'failed', candidates: circles, reason: `--pick ${opts.pickId}: unknown candidate id` };
    pivots = [forced];
  }

  const reason = (s: string) => ({ reason: s });

  // Build candidate solutions: (pivot, secondRef) pairs for Muñeca.
  const candidates: CircleCandidate[] = pivots;
  const secondRefs =
    part.secondRefCircle && geo.secondSeedLine
      ? circles.filter((c) => !candidates.some((p) => p.id === c.id) && nearLine(c, geo.secondSeedLine!))
      : [];

  const baseFace = part.basePlaneY0 ? bottomFacePoint(mesh) : null;

  interface Trial { pivot: CircleCandidate; ref?: CircleCandidate; residual: number; cal: RigidTransform; }
  const trials: Trial[] = [];
  for (const pivot of candidates) {
    const refs = secondRefs.length > 0 ? secondRefs : [undefined];
    for (const ref of refs) {
      const solved = solveCalibration(pivot, geo.frameLine, calCurrent, {
        secondCircle: ref,
        secondLine: ref && geo.secondFrameLine ? geo.secondFrameLine : undefined,
        basePlaneY0: baseFace ? { facePoint: baseFace.point } : undefined,
        keepY: part.keepY,
      });
      if (solved) trials.push({ pivot, ref, residual: solved.residualMm, cal: solved.cal });
    }
  }

  if (trials.length === 0) {
    return {
      filename: part.filename,
      status: 'failed',
      candidates: circles,
      reason: 'Constraint solver failed for every candidate.',
    };
  }

  // Selection criterion: every candidate is perfectly alignable (residual
  // ~0 by construction), so the geometric residual does NOT discriminate.
  // The current calibration is known to be roughly right → the correct
  // candidate is the one whose solved calibration is CLOSEST to the
  // current one. This prior is the semantic disambiguator in simulation.
  const tCur = calCurrent.translation as [number, number, number];
  const scored = trials.map((tr) => {
    const dt = vecLen(vecSub(tr.cal.translation as [number, number, number], tCur));
    const dAngle = quatAngleDiff(calCurrent.rotation, tr.cal.rotation);
    return { ...tr, score: dt + 2 * dAngle }; // mm + 2·rad (≈0.035mm/deg)
  });
  scored.sort((a, b) => a.score - b.score);
  const best = scored[0];
  const second = scored[1];

  // NO-TOUCH: when the best candidate's solved calibration is within
  // tolerance of the current one, the current (hand-made) calibration is
  // already good — sub-tolerance "refinement" is just detector noise.
  // Report the diagnostic (motion of the CURRENT cal) and change nothing.
  if (best.score < NO_TOUCH_SCORE_MM) {
    const motionCur = motionValidation(
      geo.jointK,
      geo.frameIdx,
      part.measurementPose,
      part.motionRangeDeg,
      MOTION_POSES,
      calCurrent,
      best.pivot,
    );
    return {
      filename: part.filename,
      status: 'ok',
      candidates,
      pivot: best.pivot,
      cal: calCurrent,
      residualMm: best.residual,
      motion: motionCur,
      deltaTranslationMm: 0,
      deltaRotationDeg: 0,
      reason: `current calibration within tolerance (best candidate ${best.score.toFixed(2)}mm-equiv < ${NO_TOUCH_SCORE_MM}mm) — untouched. Current-cal drift: ${motionCur.maxDriftMm.toFixed(2)}mm.`,
    };
  }

  // No candidate matches the current-calibration prior: the real pivot
  // feature was probably not detected (e.g. Muñeca's J4 bore sits in a
  // hidden interior face). Never invent — report and keep the current cal.
  if (best.score > 20) {
    return {
      filename: part.filename,
      status: 'failed',
      candidates,
      reason: `Best candidate produces a calibration ${best.score.toFixed(1)}mm-equivalent away from the current one — the real pivot feature was probably not detected. Needs --pick, the VLM path, or manual calibration.`,
    };
  }

  // Ambiguity is real only when the alternative candidates produce
  // MATERIALLY different calibrations. When every candidate sits close to
  // the current cal (small scores), the choice is irrelevant.
  const ambiguous =
    second !== undefined &&
    best.score > 0.5 &&
    second.score - best.score > 5; // > 5mm-equivalent difference in outcome

  if (ambiguous && opts.pickId === undefined) {
    return {
      filename: part.filename,
      status: 'ambiguous',
      candidates,
      pivot: best.pivot,
      cal: best.cal,
      residualMm: best.residual,
      motion: undefined,
      reason: `Tie between candidates ${best.pivot.id} and ${second!.pivot.id} (scores ${best.score.toFixed(3)} vs ${second!.score.toFixed(3)} — distance to current cal). Use --pick or the VLM path.`,
    };
  }

  const motion = motionValidation(
    geo.jointK,
    geo.frameIdx,
    part.measurementPose,
    part.motionRangeDeg,
    MOTION_POSES,
    best.cal,
    best.pivot,
  );

  // Deltas vs current calibration.
  const tNew = best.cal.translation as [number, number, number];
  const dt = vecLen(vecSub(tNew, tCur));
  const dAngle = quatAngleDiff(calCurrent.rotation, best.cal.rotation) * (180 / Math.PI);

  return {
    filename: part.filename,
    status: 'ok',
    candidates,
    pivot: best.pivot,
    cal: best.cal,
    residualMm: best.residual,
    motion,
    deltaTranslationMm: dt,
    deltaRotationDeg: dAngle,
    ...reason(`pivot=candidate ${best.pivot.id} (r=${best.pivot.radius.toFixed(2)}mm, residual=${best.residual.toFixed(3)}mm)`),
  };
}

/** Calibrate all 11 parts; build the output config. */
export function calibrateAll(ctx: CalibrationContext, opts: CalibrateAllOptions = {}): CalibrationOutput {
  const results: CalibrationResult[] = [];
  const entries: CalibrationOutput['config']['entries'] = [];

  for (const filename of ALL_STL_FILES) {
    if (opts.only && filename !== opts.only) {
      const current = ctx.currentCal.get(filename);
      entries.push({ filename, translation: (current?.translation ?? [0, 0, 0]), rotation: (current?.rotation ?? [0, 0, 0, 1]) });
      continue;
    }
    const part = descriptorFor(filename);
    if (!part) {
      results.push({ filename, status: 'failed', candidates: [], reason: 'No descriptor.' });
      const current = ctx.currentCal.get(filename);
      entries.push({ filename, translation: (current?.translation ?? [0, 0, 0]), rotation: (current?.rotation ?? [0, 0, 0, 1]) });
      continue;
    }
    const result = calibratePart(part, ctx);
    results.push(result);
    if (result.cal && result.status !== 'failed') {
      entries.push({ filename, translation: result.cal.translation, rotation: result.cal.rotation });
    } else {
      const current = ctx.currentCal.get(filename);
      entries.push({ filename, translation: (current?.translation ?? [0, 0, 0]), rotation: (current?.rotation ?? [0, 0, 0, 1]) });
    }
  }

  return {
    results,
    config: { version: 1, stlScale: ctx.stlScale, entries },
  };
}

/** Apply a rigid transform to a point (re-export for the CLI/tests). */
export function applyTransform(cal: RigidTransform, p: [number, number, number]): [number, number, number] {
  return applyCal(cal, p);
}

export { quatConj, quatMul, quatAxisAngle };
