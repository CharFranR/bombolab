import * as THREE from 'three';

/**
 * Straight-line curve through the path points — preserves sharp drawing
 * corners (CatmullRom would round them off). Used to build the trace tube.
 */
export class PolylineCurve3 extends THREE.Curve<THREE.Vector3> {
  private pts: THREE.Vector3[];

  constructor(points: THREE.Vector3[]) {
    super();
    this.pts = points;
  }

  getPoint(t: number, target = new THREE.Vector3()): THREE.Vector3 {
    const pts = this.pts;
    const n = pts.length - 1;
    if (n < 1) return target.copy(pts[0] ?? new THREE.Vector3());
    const f = Math.min(Math.max(t, 0), 1) * n;
    const i = Math.min(Math.floor(f), n - 1);
    const frac = f - i;
    return target.copy(pts[i]).lerp(pts[Math.min(i + 1, n)], frac);
  }
}

export const TRACE_RADIUS = 1.5; // mm — reads as a solid stroke on the floor

/**
 * Solid tube along the trace path. Progressive reveal via
 * geometry.setDrawRange works NATIVELY on plain BufferGeometry; the previous
 * drei Line (Line2 + LineMaterial) ignores drawRange — it rendered the full
 * stroke at once and degraded into dots. One tube segment per path point, so
 * the reveal advances point by point.
 */
export function buildTraceTube(points: [number, number, number][]): THREE.TubeGeometry | null {
  if (points.length < 2) return null;
  const vecs = points.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
  const curve = new PolylineCurve3(vecs);
  return new THREE.TubeGeometry(curve, points.length, TRACE_RADIUS, 6, false);
}
