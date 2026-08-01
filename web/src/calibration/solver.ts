/**
 * Constraint solver: given a pivot circle candidate and the joint-axis
 * line(s) in the PARENT FRAME's local space, solve the rigid calibration
 * transform (STL → parent frame local space) that mounts the part
 * correctly.
 *
 * The solver works in FRAME space — the space where `cal` lives in the
 * renderer pipeline (`world = F × jaw × scale × cal`). The constraint is
 * simply `cal · pivot ∈ axisLine` (pivot stays on the joint axis). This is
 * independent of the current calibration: the seed line in STL space (via
 * the current cal) is ONLY a pre-filter for candidate selection.
 *
 * - Orientation: minimal rotation mapping circle normal → joint axis,
 *   with the spin about the axis either preserved from the current
 *   calibration (default) or fixed by a second reference circle (Muñeca).
 * - Translation: least-squares projection of the pivot center onto the
 *   axis line(s), preserving the along-axis slide from the current
 *   calibration when the axis alone leaves it free.
 * - Extra hard constraints per descriptor: Base bottom face on Y=0,
 *   Tapa Base keeps its current height.
 */

import {
  applyCal,
  distPointLine,
  quatAxisAngle,
  quatFromAxisAngle,
  quatMul,
  quatNormalize,
  quatRotate,
  vecAdd,
  vecCross,
  vecDot,
  vecNorm,
  vecScale,
  vecSub,
  type Quat,
} from './fk';
import type { AxisLine, CircleCandidate, RigidTransform } from './types';
import { solve3x3 } from './mesh';

export interface SolveOptions {
  /** Second reference circle (Muñeca): its normal must align with secondLine. */
  secondCircle?: CircleCandidate;
  secondLine?: AxisLine;
  /** Base.stl: bottom face point (STL space) must land on Y=0 (frame space). */
  basePlaneY0?: { facePoint: [number, number, number] };
  /** Tapa Base.stl: keep current Y translation (frame space). */
  keepY?: boolean;
}

export interface SolveResult {
  cal: RigidTransform;
  /** Residual distances (mm) of pivot center(s) to the constraint line(s). */
  residualMm: number;
}

/** Minimal rotation mapping a → b (unit vectors). */
export function minimalRotation(a: [number, number, number], b: [number, number, number]): Quat {
  const cross = vecCross(a, b);
  const s = Math.hypot(cross[0], cross[1], cross[2]);
  const c = vecDot(a, b);
  if (s < 1e-9) {
    if (c > 0) return [0, 0, 0, 1];
    const ref: [number, number, number] = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    const axis = vecNorm(vecCross(a, ref));
    return quatFromAxisAngle(axis, Math.PI);
  }
  const axis = [cross[0] / s, cross[1] / s, cross[2] / s] as [number, number, number];
  return quatFromAxisAngle(axis, Math.atan2(s, c));
}

function rotAbout(u: [number, number, number], phi: number): Quat {
  return quatFromAxisAngle(u, phi);
}

function spinAbout(q: Quat, u: [number, number, number]): number {
  const { axis, angle } = quatAxisAngle(q);
  return angle * vecDot(axis, u);
}

/**
 * Solve the calibration for one pivot candidate.
 * `line` MUST be in the parent frame's local space (jointAxisInFrameSpace).
 */
export function solveCalibration(
  pivot: CircleCandidate,
  line: AxisLine,
  calCurrent: RigidTransform,
  opts: SolveOptions = {},
): SolveResult | null {
  const qCur = calCurrent.rotation as unknown as Quat;
  const tCur = calCurrent.translation as [number, number, number];

  let n = pivot.normal as [number, number, number];
  const u = line.dir;
  if (vecDot(n, u) < 0) n = vecScale(n, -1);

  // 1) Minimal rotation n → u.
  let qRot = minimalRotation(n, u);

  // 2) Spin about the axis.
  if (opts.secondCircle && opts.secondLine) {
    const n2 = opts.secondCircle.normal as [number, number, number];
    const u2 = opts.secondLine.dir;
    const v1 = quatRotate(qRot, n2);
    const pv = vecNorm(vecSub(v1, vecScale(u, vecDot(v1, u))));
    const pw = vecNorm(vecSub(u2, vecScale(u, vecDot(u2, u))));
    if (pv && pw) {
      const phi = Math.atan2(vecDot(vecCross(pv, pw), u), vecDot(pv, pw));
      qRot = quatMul(rotAbout(u, phi), qRot);
    }
  } else {
    // Preserve the current spin. Any rotation mapping n→u decomposes as
    // R = R_min · Rot(n, φ) (the coset of the stabilizer of n), so the
    // preserved-spin rotation is R = R_min · qRel where
    // qRel = qRot⁻¹ · qCur is a pure rotation about n.
    let qRel = quatMul(quatConjOf(qRot), qCur);
    if (qRel[3] < 0) qRel = [-qRel[0], -qRel[1], -qRel[2], -qRel[3]]; // canonical sign
    qRel = quatNormalize(qRel);
    qRot = quatMul(qRot, qRel); // qCur = qRot · qRel by construction
  }
  qRot = quatNormalize(qRot);

  // 3) Translation: LSQ in FRAME space.
  //    For each line: (t + R·c_i − p0_i) ⊥ u_i  (2 equations per line).
  const equations: { a: [number, number, number]; b: number; weight: number }[] = [];

  const addLine = (circle: CircleCandidate, ln: AxisLine) => {
    const rc = quatRotate(qRot, circle.center as [number, number, number]);
    const ref: [number, number, number] = Math.abs(ln.dir[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    const a1 = vecNorm(vecCross(ln.dir, ref));
    const a2 = vecNorm(vecCross(ln.dir, a1));
    for (const ax of [a1, a2]) {
      equations.push({ a: ax, b: -vecDot(ax, vecSub(rc, ln.point)), weight: 1 });
    }
  };

  addLine(pivot, line);
  const hasSecond = !!(opts.secondCircle && opts.secondLine);
  if (hasSecond) {
    addLine(opts.secondCircle!, opts.secondLine!);
  } else {
    // Preserve the slide along the axis: (t + R·c − (t_act + R_act·c)) · u = 0.
    const rc = quatRotate(qRot, pivot.center as [number, number, number]);
    const rcCur = quatRotate(qCur, pivot.center as [number, number, number]);
    equations.push({
      a: u,
      b: vecDot(u, vecAdd(tCur, vecSub(rcCur, rc))),
      weight: 1,
    });
  }
  if (opts.keepY) {
    equations.push({ a: [0, 1, 0], b: tCur[1], weight: 1 });
  }
  if (opts.basePlaneY0) {
    const rf = quatRotate(qRot, opts.basePlaneY0.facePoint);
    equations.push({ a: [0, 1, 0], b: -rf[1], weight: 1 }); // t_y + (R·face)_y = 0
  }

  // Normal equations AᵀA t = Aᵀb with weights.
  let ATA = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  let ATb = [0, 0, 0];
  for (const eq of equations) {
    const w = eq.weight;
    const ax = eq.a;
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) ATA[r * 3 + c] += w * w * ax[r] * ax[c];
      ATb[r] += w * w * ax[r] * eq.b;
    }
  }
  const diag = Math.max(ATA[0], ATA[4], ATA[8]);
  if (diag < 1e-9) return null;
  const lambda = 1e-6 * diag;
  ATA[0] += lambda; ATA[4] += lambda; ATA[8] += lambda;

  const t = solve3x3(ATA, ATb);
  if (!t) return null;

  const cal: RigidTransform = {
    translation: [t[0], t[1], t[2]],
    rotation: quatNormalize(qRot) as unknown as [number, number, number, number],
  };

  // Residual: distance of pivot center(s) to the constraint line(s), frame space.
  const rc = applyCal(cal, pivot.center as [number, number, number]);
  let res = distPointLine(rc, line);
  if (hasSecond && opts.secondCircle) {
    const rc2 = applyCal(cal, opts.secondCircle.center as [number, number, number]);
    res += distPointLine(rc2, opts.secondLine!);
  }
  return { cal, residualMm: res };
}

function quatConjOf(q: Quat): Quat {
  return [-q[0], -q[1], -q[2], q[3]];
}
