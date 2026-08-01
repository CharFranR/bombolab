/**
 * Calibration domain types for the FABRI Creator STL calibrator.
 *
 * All geometry math lives in the renderer's Three.js Y-up space: the
 * calibration transform `cal` is post-multiplied in the renderer pipeline
 * `world = F_k(q) × jaw × scale × cal`, so `cal` lives in the LOCAL space
 * of the parent FK frame (Three coordinates), exactly where the mesh sits
 * after jaw × scale.
 */

import type { Mat4 } from '../kinematics/types';

/** A 3D circle detected on an STL mesh (hole mouth / boss). */
export interface CircleCandidate {
  id: number;
  /** Circle center in STL model space (mm). */
  center: [number, number, number];
  /** Unit normal of the circle plane (STL space). Ambiguous sign. */
  normal: [number, number, number];
  /** Radius in mm. */
  radius: number;
  /** RMS fit residual in mm. */
  residual: number;
  /** Number of vertices used for the fit. */
  nVerts: number;
}

/** An infinite line (axis). */
export interface AxisLine {
  /** Point on the line. */
  point: [number, number, number];
  /** Unit direction. */
  dir: [number, number, number];
}

/** A rigid transform: translation + unit quaternion [x,y,z,w]. */
export interface RigidTransform {
  translation: [number, number, number];
  rotation: [number, number, number, number];
}

/** Per-part semantic descriptor (robot-specific, hand-authored). */
export interface PartDescriptor {
  filename: string;
  /** STL_META parentJoint: 0=world, 1..5=joint frame, -1=tool (frame 5). */
  parentJoint: number;
  /** Expected pivot circle radius range (mm). */
  radiusMin: number;
  radiusMax: number;
  /** Max distance (mm) from pivot circle center to the joint-axis seed line. */
  distToLineMax: number;
  /** Fixed gripper value for jaw parts (Pinza1/2: 100 = closed → jawOpen 0). */
  fixedGripper: number;
  /** Frozen measurement pose (radians, kinematic coordinates). */
  measurementPose: number[];
  /** Joint sweep (degrees) for motion validation. */
  motionRangeDeg: number;
  /** Muñeca: second circle whose normal must align with joint 5 axis. */
  secondRefCircle?: boolean;
  /** Base.stl: bottom face must sit on Y=0 (world ground). */
  basePlaneY0?: boolean;
  /** Tapa Base.stl: keep the current height (Y translation) untouched. */
  keepY?: boolean;
  note?: string;
}

/** Motion validation outcome. */
export interface MotionResult {
  /** Max pivot-to-axis distance (mm) across all poses. */
  maxDriftMm: number;
  /** RMS pivot-to-axis distance (mm) across all poses. */
  rmsDriftMm: number;
  /** Number of poses evaluated. */
  nPoses: number;
  /** True when maxDriftMm is within the certification threshold (0.3mm). */
  passed: boolean;
}

/** Per-part calibration outcome. */
export interface CalibrationResult {
  filename: string;
  status: 'ok' | 'ambiguous' | 'failed';
  /** Candidate chosen as the pivot (undefined when failed). */
  pivot?: CircleCandidate;
  /** Candidates surviving the pre-filter. */
  candidates: CircleCandidate[];
  /** Solved calibration transform (STL → parent frame local space). */
  cal?: RigidTransform;
  /** Residual distances (mm) of pivot center to the constraint line(s). */
  residualMm?: number;
  /** Motion validation of the solved calibration. */
  motion?: MotionResult;
  /** Delta vs the current calibration.json entry. */
  deltaTranslationMm?: number;
  deltaRotationDeg?: number;
  reason?: string;
}

/** Full run output. */
export interface CalibrationOutput {
  results: CalibrationResult[];
  /** calibration.json payload (same shape as web/public/calibration.json). */
  config: {
    version: number;
    stlScale: number;
    entries: { filename: string; translation: [number, number, number]; rotation: [number, number, number, number] }[];
  };
}

/** Context shared across the pipeline (IO-free: the loader is injected). */
export interface CalibrationContext {
  /** Loads a raw STL file by name (injected by the CLI — keeps src/ pure). */
  loader: (filename: string) => ArrayBuffer;
  /** Current calibration entries keyed by filename. */
  currentCal: Map<string, RigidTransform>;
  stlScale: number;
  /** Cache of parsed meshes by filename (avoid re-parsing). */
  meshCache: Map<string, unknown>;
}

/** DH segment definition (mirror of crates/bombolab-core/src/robot/fabri_creator.rs). */
export interface DhSegment {
  type: 'revolute' | 'twist';
  /** Fixed theta offset (radians); absent for Twist (rotates about X). */
  theta0?: number;
  d: number;
  a: number;
  alpha: number;
}

export const FABRI_DH: DhSegment[] = [
  { type: 'revolute', theta0: 0, d: 85, a: 15, alpha: -Math.PI / 2 },
  { type: 'revolute', theta0: -Math.PI / 2, d: 0, a: 120, alpha: 0 },
  { type: 'revolute', theta0: Math.PI / 2, d: 0, a: 90, alpha: -Math.PI / 2 },
  { type: 'twist', d: 15, a: 35, alpha: Math.PI / 2 },
  { type: 'revolute', theta0: 0, d: 0, a: 0, alpha: 0 },
];

export type { Mat4 };
