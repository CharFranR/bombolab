import type { Segment, Mat4, RobotDef } from './types';
import { forwardKinematics, forwardKinematicsRaw } from './forward';

// ─── Mat4 helpers (inline) ──────────────────────────────────────────────────

function mulMat4(a: Mat4, b: Mat4): Mat4 {
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

// ─── 3×3 matrix helpers ────────────────────────────────────────────────────

type Mat3 = [
  number, number, number,
  number, number, number,
  number, number, number,
];

function mat3Mul(a: Mat3, b: Mat3): Mat3 {
  const m = (r: number, c: number) =>
    a[r * 3 + 0] * b[0 * 3 + c] +
    a[r * 3 + 1] * b[1 * 3 + c] +
    a[r * 3 + 2] * b[2 * 3 + c];
  return [
    m(0,0), m(0,1), m(0,2),
    m(1,0), m(1,1), m(1,2),
    m(2,0), m(2,1), m(2,2),
  ];
}

function mat3Det(m: Mat3): number {
  return m[0] * (m[4]*m[8] - m[5]*m[7])
       - m[1] * (m[3]*m[8] - m[5]*m[6])
       + m[2] * (m[3]*m[7] - m[4]*m[6]);
}

function mat3Inv(m: Mat3): Mat3 | null {
  const det = mat3Det(m);
  if (Math.abs(det) < 1e-15) return null;
  const inv = 1 / det;
  return [
    inv * (m[4]*m[8] - m[5]*m[7]),
    inv * (m[2]*m[7] - m[1]*m[8]),
    inv * (m[1]*m[5] - m[2]*m[4]),
    inv * (m[5]*m[6] - m[3]*m[8]),
    inv * (m[0]*m[8] - m[2]*m[6]),
    inv * (m[2]*m[3] - m[0]*m[5]),
    inv * (m[3]*m[7] - m[4]*m[6]),
    inv * (m[1]*m[6] - m[0]*m[7]),
    inv * (m[0]*m[4] - m[1]*m[3]),
  ];
}

function mat3AddScalar(m: Mat3, s: number): Mat3 {
  return [
    m[0]+s, m[1],   m[2],
    m[3],   m[4]+s, m[5],
    m[6],   m[7],   m[8]+s,
  ];
}

// ─── 3×5 matrix helpers ────────────────────────────────────────────────────

/** Jacobiana 3×5 como array de 15 elementos [col0, col1, ...] row-major. */
type Mat3x5 = Float64Array; // 15 elements, 3 rows × 5 cols

function mat3x5TransposeMulVec(j: Mat3x5, v: [number, number, number]): Float64Array {
  const out = new Float64Array(5);
  for (let col = 0; col < 5; col++) {
    out[col] = j[col * 3 + 0] * v[0] + j[col * 3 + 1] * v[1] + j[col * 3 + 2] * v[2];
  }
  return out;
}

// ─── IkSolver ──────────────────────────────────────────────────────────────

export class IkSolver {
  maxIterations: number;
  tolerance: number;  // mm
  damping: number;    // λ
  stepSize: number;   // rad

  constructor(
    maxIterations = 200,
    tolerance = 1.0,
    damping = 0.05,
    stepSize = 0.5,
  ) {
    this.maxIterations = maxIterations;
    this.tolerance = tolerance;
    this.damping = damping;
    this.stepSize = stepSize;
  }

  /** IK de posición: [x, y, z] mm → q (rad). qInit = última solución (tracking). */
  solvePosition(
    target: [number, number, number],
    qInit: number[],
    robot: RobotDef,
  ): { q: number[]; converged: boolean; error: number } {
    const n = Math.min(robot.segments.length, 5);
    const q = qInit.slice();
    const dampingSq = this.damping * this.damping;

    for (let iter = 0; iter < this.maxIterations; iter++) {
      // FK con q actual
      const segments = robot.segments.map((seg, i) => ({ ...seg, q: q[i] ?? 0 }));
      const baseT: Mat4 = [
        1, 0, 0, robot.baseTransform[0],
        0, 1, 0, robot.baseTransform[1],
        0, 0, 1, robot.baseTransform[2],
        0, 0, 0, 1,
      ];
      const fk = forwardKinematicsRaw(segments, baseT);
      const ee = fk.frames[fk.frames.length - 1];
      const toolT: Mat4 = [
        1, 0, 0, robot.toolTransform[0],
        0, 1, 0, robot.toolTransform[1],
        0, 0, 1, robot.toolTransform[2],
        0, 0, 0, 1,
      ];
      const toolPose = mulMat4(ee, toolT);
      const pEe: [number, number, number] = [toolPose[3], toolPose[7], toolPose[11]];

      // Error DH (Z-up)
      const error: [number, number, number] = [
        target[0] - pEe[0],
        target[1] - pEe[1],
        target[2] - pEe[2],
      ];
      const errNorm = Math.sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);

      if (errNorm < this.tolerance) {
        return { q, converged: true, error: errNorm };
      }

      // Jacobiana 3×5: J_i = z_{i-1} × (p_ee - p_{i-1})
      // J1 usa base (z=(0,0,1), p=baseTransform)
      const j = new Float64Array(15); // 3×5 row-major
      const baseP: [number, number, number] = [baseT[3], baseT[7], baseT[11]];

      for (let i = 0; i < n; i++) {
        let z: [number, number, number];
        let p: [number, number, number];
        if (i === 0) {
          z = [0, 0, 1];
          p = baseP;
        } else {
          const prevFrame = fk.frames[i - 1];
          // Z axis of frame i-1 = column 2 (indices 2, 6, 10)
          z = [prevFrame[2], prevFrame[6], prevFrame[10]];
          p = [prevFrame[3], prevFrame[7], prevFrame[11]];
        }

        // cross = z × (p_ee - p)
        const dx = pEe[0] - p[0];
        const dy = pEe[1] - p[1];
        const dz = pEe[2] - p[2];
        const cx = z[1] * dz - z[2] * dy;
        const cy = z[2] * dx - z[0] * dz;
        const cz = z[0] * dy - z[1] * dx;

        j[i * 3 + 0] = cx;
        j[i * 3 + 1] = cy;
        j[i * 3 + 2] = cz;
      }

      // DLS: J·J^T (3×3)
      let jjt: Mat3 = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          let sum = 0;
          for (let col = 0; col < n; col++) {
            sum += j[col * 3 + r] * j[col * 3 + c];
          }
          jjt[r * 3 + c] = sum;
        }
      }

      const reg = mat3AddScalar(jjt, dampingSq);
      const inv = mat3Inv(reg);
      if (!inv) {
        return { q, converged: false, error: errNorm };
      }

      // Δx = inv · error (3×1)
      const deltaX: [number, number, number] = [
        inv[0] * error[0] + inv[1] * error[1] + inv[2] * error[2],
        inv[3] * error[0] + inv[4] * error[1] + inv[5] * error[2],
        inv[6] * error[0] + inv[7] * error[1] + inv[8] * error[2],
      ];

      // Δq = J^T · Δx
      const deltaQ = mat3x5TransposeMulVec(j, deltaX);

      // Step scaling + joint limits
      const dqNorm = Math.sqrt(
        deltaQ[0] * deltaQ[0] + deltaQ[1] * deltaQ[1] +
        deltaQ[2] * deltaQ[2] + deltaQ[3] * deltaQ[3] + deltaQ[4] * deltaQ[4]
      );
      const scale = dqNorm > this.stepSize ? this.stepSize / dqNorm : 1.0;

      const limit = 80 * Math.PI / 180; // ±80°
      for (let idx = 0; idx < n; idx++) {
        q[idx] += deltaQ[idx] * scale;
        q[idx] = Math.max(-limit, Math.min(limit, q[idx]));
      }
    }

    // Error final
    const segments = robot.segments.map((seg, i) => ({ ...seg, q: q[i] ?? 0 }));
    const baseT: Mat4 = [
      1, 0, 0, robot.baseTransform[0],
      0, 1, 0, robot.baseTransform[1],
      0, 0, 1, robot.baseTransform[2],
      0, 0, 0, 1,
    ];
    const fk = forwardKinematicsRaw(segments, baseT);
    const ee = fk.frames[fk.frames.length - 1];
    const toolT: Mat4 = [
      1, 0, 0, robot.toolTransform[0],
      0, 1, 0, robot.toolTransform[1],
      0, 0, 1, robot.toolTransform[2],
      0, 0, 0, 1,
    ];
    const toolPose = mulMat4(ee, toolT);
    const finalErr = Math.sqrt(
      (target[0] - toolPose[3]) ** 2 +
      (target[1] - toolPose[7]) ** 2 +
      (target[2] - toolPose[11]) ** 2
    );

    return { q, converged: false, error: finalErr };
  }
}
