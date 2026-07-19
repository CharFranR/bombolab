import type { RobotDef, Segment } from '../kinematics/types';
import { segment } from '../kinematics/forward';

const DEG = Math.PI / 180;

/**
 * FABRI Creator 5-DOF — tabla DH estándar (corregida).
 *
 * | Joint | θ_fijo | d   | a   | α       |
 * |-------|--------|-----|-----|---------|
 * | J1    | 0      | 95  | 15  | -π/2    |
 * | J2    | -π/2   | 0   | 162 | 0       |
 * | J3    | +π/2   | 0   | 111 | -π/2    |
 * | J4    | 0      | 0   | 35  | +π/2    |
 * | J5    | 0      | 0   | 0   | 0       |
 */
export function fabriCreatorSegments(q?: number[]): Segment[] {
  const base = [
    segment({ theta: 0,       d: 95,  a: 15,  alpha: -Math.PI / 2 }),
    segment({ theta: -Math.PI / 2, d: 0,   a: 162, alpha: 0 }),
    segment({ theta: Math.PI / 2,  d: 0,   a: 111, alpha: -Math.PI / 2 }),
    segment({ theta: 0,       d: 0,   a: 35,  alpha: Math.PI / 2 }),
    segment({ theta: 0,       d: 0,   a: 0,   alpha: 0 }),
  ];

  if (q) {
    base.forEach((seg, i) => { seg.q = q[i] ?? 0; });
  }

  return base;
}

/** Robot FABRI Creator completo. */
export function fabriCreator(q?: number[]): RobotDef {
  return {
    name: 'FABRI Creator',
    segments: fabriCreatorSegments(q),
    baseTransform: [0, 0, 57],
    toolTransform: [75, 0, 0],
  };
}
