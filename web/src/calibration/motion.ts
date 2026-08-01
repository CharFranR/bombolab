/**
 * Motion validation: certify that the solved calibration keeps the pivot
 * on the joint axis across many poses.
 *
 * The pivot world position is F_eff(q) · cal · center; with a correct
 * calibration it stays on the joint axis line for EVERY pose of the parent
 * joint (the pivot is the fixed point of the rotation). A wrong candidate
 * or a broken calibration shows up as drift.
 */

import { applyCal, distPointLine, dhToThree, fkFrames, jointAxisWorld, vecNorm, type Quat } from './fk';
import { framePose } from '../renderers/types';
import type { AxisLine, CircleCandidate, MotionResult, RigidTransform } from './types';

export const MOTION_THRESHOLD_MM = 0.3;

/**
 * Validate the calibration of a part over `nPoses` sweep poses of its
 * parent joint (others frozen at the measurement pose).
 *
 * @param jointK       joint that moves the part (1..5)
 * @param frameIdx     parent frame index (parentJoint or 5 for tool)
 * @param measurementPose frozen pose for the other joints (radians)
 * @param sweepDeg     sweep range ±degrees for jointK
 * @param nPoses       number of poses
 * @param cal          candidate calibration
 * @param pivot        pivot circle center in STL space
 */
export function motionValidation(
  jointK: number,
  frameIdx: number,
  measurementPose: number[],
  sweepDeg: number,
  nPoses: number,
  cal: RigidTransform,
  pivot: CircleCandidate,
): MotionResult {
  const sweepRad = (sweepDeg * Math.PI) / 180;
  let maxDrift = 0;
  let sumSq = 0;

  for (let i = 0; i < nPoses; i++) {
    const t = nPoses === 1 ? 0 : (i / (nPoses - 1)) * 2 - 1;
    const q = measurementPose.slice();
    q[jointK - 1] += t * sweepRad;

    const frames = fkFrames(q);
    // World axis of the moving joint.
    const axisWorld: AxisLine = {
      point: dhToThree(jointAxisWorld(jointK, frames).point),
      dir: dhToThree(jointAxisWorld(jointK, frames).dir),
    };
    axisWorld.dir = vecNorm(axisWorld.dir);

    // Parent frame pose (frameIdx 0 = world identity; else FK frame).
    let pos: [number, number, number] = [0, 0, 0];
    let quat: Quat = [0, 0, 0, 1];
    if (frameIdx > 0) {
      const pose = framePose(frames[frameIdx]);
      pos = pose.pos as unknown as [number, number, number];
      quat = pose.quat as unknown as Quat;
    }

    // Pivot world = F_eff(q) · cal · center (jaw identity, scale identity).
    const inFrame = applyCal(cal, pivot.center as [number, number, number]);
    const pivotWorld: [number, number, number] = [
      pos[0] + rotateByQuat(quat, inFrame)[0],
      pos[1] + rotateByQuat(quat, inFrame)[1],
      pos[2] + rotateByQuat(quat, inFrame)[2],
    ];

    const d = distPointLine(pivotWorld, axisWorld);
    maxDrift = Math.max(maxDrift, d);
    sumSq += d * d;
  }

  const rms = Math.sqrt(sumSq / nPoses);
  return {
    maxDriftMm: maxDrift,
    rmsDriftMm: rms,
    nPoses,
    passed: maxDrift < MOTION_THRESHOLD_MM,
  };
}

/** Rotate v by quaternion q (forward). */
function rotateByQuat(q: Quat, v: [number, number, number]): [number, number, number] {
  const qv = [q[0], q[1], q[2]] as [number, number, number];
  const c1 = cross3(qv, v);
  const c2 = cross3(qv, c1);
  return [
    v[0] + 2 * (q[3] * c1[0] + c2[0]),
    v[1] + 2 * (q[3] * c1[1] + c2[1]),
    v[2] + 2 * (q[3] * c1[2] + c2[2]),
  ];
}

function cross3(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}
