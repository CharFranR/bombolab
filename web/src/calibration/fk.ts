/**
 * Pure-TS forward kinematics for the FABRI Creator, mirroring
 * crates/bombolab-core/src/kinematics/forward.rs + robot/segment.rs.
 *
 * Produces frames F0..F5 where F0 = base_transform (0,0,57) and
 * Fk = F_{k-1} × T_k. The renderer prepends a world identity frame, so
 * parentJoint k (1..5) maps to Fk and parentJoint -1 (tool) maps to F5.
 *
 * Joint-axis lines are computed in the renderer's Three.js space and then
 * pulled into the part's calibration space using the CURRENT calibration
 * as the seed — this is the positional pre-filter that disambiguates which
 * detected circle is the real pivot.
 */

import { FABRI_DH, type AxisLine, type DhSegment, type Mat4, type RigidTransform } from './types';
import { framePose } from '../renderers/types';

// ─── Mat4 helpers (row-major: element [r*4+c]) ───────────────────────────────

export function mat4Identity(): Mat4 {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

export function mat4Mul(a: Mat4, b: Mat4): Mat4 {
  const out = new Array<number>(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      out[r * 4 + c] =
        a[r * 4 + 0] * b[0 * 4 + c] +
        a[r * 4 + 1] * b[1 * 4 + c] +
        a[r * 4 + 2] * b[2 * 4 + c] +
        a[r * 4 + 3] * b[3 * 4 + c];
    }
  }
  return out as Mat4;
}

/** Inverse of an affine 4×4 (rotation + translation). */
export function mat4InvertAffine(m: Mat4): Mat4 {
  const r = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) r[i * 3 + j] = m[j * 4 + i];
  }
  const tx = m[12], ty = m[13], tz = m[14];
  return [
    r[0], r[1], r[2], 0,
    r[3], r[4], r[5], 0,
    r[6], r[7], r[8], 0,
    -(r[0] * tx + r[3] * ty + r[6] * tz),
    -(r[1] * tx + r[4] * ty + r[7] * tz),
    -(r[2] * tx + r[5] * ty + r[8] * tz),
    1,
  ];
}

export function mat4TransformPoint(m: Mat4, p: [number, number, number]): [number, number, number] {
  return [
    m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
    m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
    m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
  ];
}

// ─── Quaternion helpers ──────────────────────────────────────────────────────

export type Quat = [number, number, number, number]; // [x,y,z,w]

export function quatMul(a: Quat, b: Quat): Quat {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}

export function quatConj(q: Quat): Quat {
  return [-q[0], -q[1], -q[2], q[3]];
}

export function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) return [0, 0, 0, 1];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** Rotation quaternion about a unit axis by angle (radians). */
export function quatFromAxisAngle(axis: [number, number, number], angle: number): Quat {
  const s = Math.sin(angle / 2);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)];
}

/** Rotate vector v by quaternion q. */
export function quatRotate(q: Quat, v: [number, number, number]): [number, number, number] {
  const qv = [q[0], q[1], q[2]] as [number, number, number];
  const cross1 = vecCross(qv, v);
  const cross2 = vecCross(qv, cross1);
  return [
    v[0] + 2 * (q[3] * cross1[0] + cross2[0]),
    v[1] + 2 * (q[3] * cross1[1] + cross2[1]),
    v[2] + 2 * (q[3] * cross1[2] + cross2[2]),
  ];
}

/** Rotate vector v by the inverse of quaternion q. */
export function quatRotateInverse(q: Quat, v: [number, number, number]): [number, number, number] {
  return quatRotate(quatConj(q), v);
}

/** Extract (axis, angle) of a unit quaternion; axis·u gives the signed spin about u. */
export function quatAxisAngle(q: Quat): { axis: [number, number, number]; angle: number } {
  const s = Math.hypot(q[0], q[1], q[2]);
  if (s < 1e-12) return { axis: [1, 0, 0], angle: 0 };
  return { axis: [q[0] / s, q[1] / s, q[2] / s], angle: 2 * Math.atan2(s, q[3]) };
}

// ─── Vector helpers ──────────────────────────────────────────────────────────

export function vecSub(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function vecAdd(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function vecScale(a: [number, number, number], s: number): [number, number, number] {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function vecDot(a: [number, number, number], b: [number, number, number]): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function vecCross(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

export function vecLen(a: [number, number, number]): number {
  return Math.hypot(a[0], a[1], a[2]);
}

export function vecNorm(a: [number, number, number]): [number, number, number] {
  const n = vecLen(a);
  return n < 1e-12 ? [1, 0, 0] : [a[0] / n, a[1] / n, a[2] / n];
}

/** Distance from point p to line (point + unit dir). */
export function distPointLine(p: [number, number, number], line: AxisLine): number {
  const w = vecSub(p, line.point);
  const cross = vecCross(w, line.dir);
  return vecLen(cross);
}

/** Swap DH→Three: (x, y, z) → (x, z, y) — the renderer's C-3 reflection convention. */
export function dhToThree(p: [number, number, number]): [number, number, number] {
  return [p[0], p[2], p[1]];
}

// ─── FK ──────────────────────────────────────────────────────────────────────

function segmentMatrix(q: number, seg: DhSegment): Mat4 {
  if (seg.type === 'twist') {
    // Twist: RotX(alpha + q), translation (a, d, 0).
    const c = Math.cos(seg.alpha + q);
    const s = Math.sin(seg.alpha + q);
    return [
      1, 0, 0, seg.a,
      0, c, -s, seg.d,
      0, s, c, 0,
      0, 0, 0, 1,
    ];
  }
  const theta = q + (seg.theta0 ?? 0);
  const cz = Math.cos(theta), sz = Math.sin(theta);
  const cx = Math.cos(seg.alpha), sx = Math.sin(seg.alpha);
  // R = RotZ(theta) * RotX(alpha)
  const m00 = cz, m01 = -sz * cx, m02 = sz * sx;
  const m10 = sz, m11 = cz * cx, m12 = -cz * sx;
  const m20 = 0, m21 = sx, m22 = cx;
  return [
    m00, m01, m02, seg.a * cz,
    m10, m11, m12, seg.a * sz,
    m20, m21, m22, seg.d,
    0, 0, 0, 1,
  ];
}

/**
 * Forward kinematics → frames F0..F5 (F0 = base_transform (0,0,57)).
 * Returns DH-world Mat4s.
 */
export function fkFrames(q: number[]): Mat4[] {
  const base: Mat4 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 57, 0, 0, 0, 1];
  const frames: Mat4[] = [base];
  let current = base;
  for (let i = 0; i < FABRI_DH.length; i++) {
    const qi = q[i] ?? 0;
    current = mat4Mul(current, segmentMatrix(qi, FABRI_DH[i]));
    frames.push(current);
  }
  return frames; // F0..F5
}

/** Column X of a Mat4 rotation part. */
export function mat4ColX(m: Mat4): [number, number, number] {
  return [m[0], m[4], m[8]];
}

/** Column Z of a Mat4 rotation part. */
export function mat4ColZ(m: Mat4): [number, number, number] {
  return [m[2], m[6], m[10]];
}

export function mat4Translation(m: Mat4): [number, number, number] {
  return [m[3], m[7], m[11]];
}

/**
 * Joint axis line (DH world) of joint `jointK` (1..5):
 *  - Revolute: through origin of F_{jointK-1}, direction = column Z of F_{jointK-1}.
 *  - Twist (joint 4): through origin of F_3, direction = column X of F_3.
 */
export function jointAxisWorld(jointK: number, frames: Mat4[]): AxisLine {
  const prev = frames[jointK - 1];
  const dir = FABRI_DH[jointK - 1].type === 'twist' ? mat4ColX(prev) : mat4ColZ(prev);
  return { point: mat4Translation(prev), dir: vecNorm(dir) };
}

/**
 * Joint axis line expressed in the LOCAL space of `frameIdx` (Three coords).
 * frameIdx 0 → world space directly (dhToThree); frameIdx ≥ 1 → inverse
 * transform of the FK frame pose. This is the space where `cal` lives:
 * the solver constrains `cal·pivot ∈ line`.
 */
export function jointAxisInFrameSpace(
  jointK: number,
  frameIdx: number,
  frames: Mat4[],
): AxisLine {
  const world = jointAxisWorld(jointK, frames);
  let pointLocal: [number, number, number];
  let dirLocal: [number, number, number];
  if (frameIdx === 0) {
    pointLocal = dhToThree(world.point);
    dirLocal = dhToThree(world.dir);
  } else {
    const pose = framePose(frames[frameIdx]);
    pointLocal = quatRotateInverse(pose.quat as unknown as Quat, vecSub(dhToThree(world.point), pose.pos as unknown as [number, number, number]));
    dirLocal = quatRotateInverse(pose.quat as unknown as Quat, dhToThree(world.dir));
  }
  return { point: pointLocal, dir: vecNorm(dirLocal) };
}

/**
 * The joint axis line, expressed in the PART's calibration space (STL space
 * after the CURRENT calibration): the pre-filter seed line. The pivot circle
 * of the part must lie close to this line when the current calibration is
 * roughly right. (The SOLVER must NOT use this line — it solves in frame
 * space, see jointAxisInFrameSpace.)
 */
export function jointAxisInPartSpace(
  jointK: number,
  frameIdx: number,
  frames: Mat4[],
  calCurrent: RigidTransform,
): AxisLine {
  const local = jointAxisInFrameSpace(jointK, frameIdx, frames);
  const q = calCurrent.rotation as unknown as Quat;
  const t = calCurrent.translation as [number, number, number];
  const pointStl = quatRotateInverse(q, vecSub(local.point, t));
  const dirStl = vecNorm(quatRotateInverse(q, local.dir));
  return { point: pointStl, dir: dirStl };
}

/** Apply a rigid transform to a point. */
export function applyCal(cal: RigidTransform, p: [number, number, number]): [number, number, number] {
  return vecAdd(quatRotate(cal.rotation as unknown as Quat, p), cal.translation as [number, number, number]);
}

/** Invert-apply a rigid transform to a point. */
export function invApplyCal(cal: RigidTransform, p: [number, number, number]): [number, number, number] {
  return quatRotateInverse(cal.rotation as unknown as Quat, vecSub(p, cal.translation as [number, number, number]));
}
