/**
 * Part descriptors for the 11 FABRI Creator STLs.
 *
 * Robot-specific by design: the problem is fully constrained (known
 * hierarchy, DH frames, joint axes, STL-per-link). Each descriptor declares
 * the expected pivot geometry (circle radius band), the parent joint, and
 * per-part flags for extra constraints (base on ground, second reference
 * circle for the wrist, fixed gripper for the jaws).
 */

import type { PartDescriptor } from './types';

export const PART_DESCRIPTORS: PartDescriptor[] = [
  {
    filename: 'Base.stl',
    parentJoint: 0,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Fixed to world. NO auto-correction: the base mates with Eje Central via the frame chain; world Y=0 is arbitrary in the simulator. Diagnostic only.',
  },
  {
    filename: 'Tapa Base.stl',
    parentJoint: 0,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    keepY: true,
    note: 'Fixed to world; centered on J1 axis; height preserved from current cal.',
  },
  {
    filename: 'Eje Central.stl',
    parentJoint: 1,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Bottom bore pivots on J1 axis; spin and slide preserved from current cal.',
  },
  {
    filename: 'Antebrazo.stl',
    parentJoint: 2,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Shoulder-end bore pivots on J2 axis.',
  },
  {
    filename: 'Brazo.stl',
    parentJoint: 3,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Elbow-end bore pivots on J3 axis.',
  },
  {
    filename: 'Muñeca.stl',
    parentJoint: 4,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    secondRefCircle: true,
    note: 'J4 bore pivots on J4 (Twist) axis; J5 bore aligns with joint-5 axis (spin + slide).',
  },
  {
    filename: 'Base de la garra.stl',
    parentJoint: -1,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Tool frame; bore pivots on J5 axis.',
  },
  {
    filename: 'Engranaje1.stl',
    parentJoint: -1,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Tool frame; central bore on J5 axis; spin about axis is free (preserved).',
  },
  {
    filename: 'Engranaje2.stl',
    parentJoint: -1,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 0,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Tool frame; central bore on J5 axis; spin about axis is free (preserved).',
  },
  {
    filename: 'Pinza1.stl',
    parentJoint: -1,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 100,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Tool frame, jaw fixed closed (jawOpen=0); bore on J5 axis.',
  },
  {
    filename: 'Pinza2.stl',
    parentJoint: -1,
    radiusMin: 2,
    radiusMax: 9,
    distToLineMax: 15,
    fixedGripper: 100,
    measurementPose: [0, 0, 0, 0, 0],
    motionRangeDeg: 30,
    note: 'Tool frame, jaw fixed closed (jawOpen=0); bore on J5 axis.',
  },
];

export function descriptorFor(filename: string): PartDescriptor | undefined {
  return PART_DESCRIPTORS.find((d) => d.filename === filename);
}
