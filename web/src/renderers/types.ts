import * as THREE from 'three';
import type { Mat4 } from '../kinematics/types';

// ─── Fidelity mode ──────────────────────────────────────────────────────────

export type FidelityMode = 'low' | 'high';

// ─── Pre-computed FK output ─────────────────────────────────────────────────

export interface FramePose {
  pos: [number, number, number];
  quat: [number, number, number, number];
}

// ─── Props every robot renderer receives ────────────────────────────────────

export interface RobotRendererProps {
  frames: FramePose[];
  gripper: number;
  workspacePoints?: [number, number, number][];
  ikTarget?: [number, number, number] | null;
  onIkTargetChange?: (pos: [number, number, number]) => void;
  onDragStart?: () => void;
  onDragEnd?: () => void;
}

// ─── Loaded STL mesh + FK binding ───────────────────────────────────────────

export interface VisualLink {
  mesh: THREE.Mesh;
  parentJoint: number; // index into frames[]
  calibrationTransform: THREE.Matrix4;
}

// ─── DH → Three.js conversion ───────────────────────────────────────────────

/** Converts a DH row-major 4×4 (Z-up convention) into a Three.js FramePose. */
export function framePose(f: Mat4): FramePose {
  // DH → Three.js: X→X, Z→Y(up), Y→Z
  const m = new THREE.Matrix4();
  const te = m.elements;
  // Col 0: X (DH column 0 → Three column 0)
  te[0] = f[0];  te[1] = f[8];  te[2] = f[4];  te[3] = 0;
  // Col 1: Z → Y
  te[4] = f[2];  te[5] = f[10]; te[6] = f[6];  te[7] = 0;
  // Col 2: Y → Z
  te[8] = f[1];  te[9] = f[9];  te[10] = f[5]; te[11] = 0;
  // Col 3: traslación con swap
  te[12] = f[3]; te[13] = f[11]; te[14] = f[7]; te[15] = 1;

  const pos = new THREE.Vector3();
  const quat = new THREE.Quaternion();
  const scale = new THREE.Vector3();
  m.decompose(pos, quat, scale);
  return {
    pos: [pos.x, pos.y, pos.z],
    quat: [quat.x, quat.y, quat.z, quat.w],
  };
}

// ─── Mat4 multiplication ────────────────────────────────────────────────────

export function mulMat4(a: Mat4, b: Mat4): Mat4 {
  const m = (r: number, c: number) =>
    a[r * 4 + 0] * b[0 * 4 + c] +
    a[r * 4 + 1] * b[1 * 4 + c] +
    a[r * 4 + 2] * b[2 * 4 + c] +
    a[r * 4 + 3] * b[3 * 4 + c];
  return [
    m(0,0), m(0,1), m(0,2), m(0,3),
    m(1,0), m(1,1), m(1,2), m(1,3),
    m(2,0), m(2,1), m(2,2), m(2,3),
    m(3,0), m(3,1), m(3,2), m(3,3),
  ];
}
