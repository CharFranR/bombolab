import type { DHParams, Mat4, Pose, Segment } from './types';

// ─── Mat4 helpers ──────────────────────────────────────────────────────────

const id = (): Mat4 => [
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1,
];

const mul = (a: Mat4, b: Mat4): Mat4 => {
  const m: number[] = [];
  for (let row = 0; row < 4; row++) {
    for (let col = 0; col < 4; col++) {
      m.push(a[row * 4 + 0] * b[0 * 4 + col] +
             a[row * 4 + 1] * b[1 * 4 + col] +
             a[row * 4 + 2] * b[2 * 4 + col] +
             a[row * 4 + 3] * b[3 * 4 + col]);
    }
  }
  return m as Mat4;
};

// ─── Matriz DH estándar ────────────────────────────────────────────────────

/**
 * A_i = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
 *
 * θ = dh.theta + q  (theta fijo + variable articular para revolute)
 */
export function dhMatrix(dh: DHParams, q: number): Mat4 {
  const theta = dh.theta + q;
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(dh.alpha);
  const sa = Math.sin(dh.alpha);

  return [
    ct,     -st * ca,  st * sa,   dh.a * ct,
    st,      ct * ca, -ct * sa,   dh.a * st,
    0,        sa,       ca,        dh.d,
    0,        0,        0,         1,
  ];
}

/** Construye un segmento con q=0 (home). */
export function segment(dh: DHParams): Segment {
  return { ...dh, q: 0 };
}

// ─── FK ────────────────────────────────────────────────────────────────────

/** Retorna la pose de cada frame [base, J1, J2, ..., Jn]. */
export function forwardKinematics(
  segments: Segment[],
  base: [number, number, number] = [0, 0, 0],
): { frames: Mat4[]; ee: Mat4; pose: Pose } {
  // Base transform
  const [bx, by, bz] = base;
  let current: Mat4 = mul(id(), [
    1, 0, 0, bx,
    0, 1, 0, by,
    0, 0, 1, bz,
    0, 0, 0, 1,
  ]);
  const frames: Mat4[] = [current];

  for (const seg of segments) {
    current = mul(current, dhMatrix(seg, seg.q));
    frames.push(current);
  }

  return { frames, ee: current, pose: mat4ToPose(current) };
}

/** Pose from Mat4 (extrae traslación + rotación 3×3). */
export function mat4ToPose(m: Mat4): Pose {
  return {
    x: m[3], y: m[7], z: m[11],
    rot: [
      m[0], m[1], m[2],
      m[4], m[5], m[6],
      m[8], m[9], m[10],
    ],
  };
}
