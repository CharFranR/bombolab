/**
 * Mesh topology + geometric primitives for the STL calibrator.
 *
 * Builds an indexed mesh from the flat STL vertex stream, computes triangle
 * normals and dihedral angles per edge, clusters high-curvature vertex
 * rings (hole mouths) and fits 3D circles to them.
 */

import { parseStl } from '../utils/stlParser';
import { vecCross, vecDot, vecNorm, vecSub } from './fk';
import type { AxisLine, CircleCandidate } from './types';

export interface IndexedMesh {
  /** Unique vertex positions. */
  positions: [number, number, number][];
  /** Triangles as index triples. */
  triangles: number[][];
  /** Triangle unit normals. */
  triNormals: [number, number, number][];
}

export interface MeshAnalysis {
  mesh: IndexedMesh;
  /** Vertex indices that belong to sharp (high-curvature) rings. */
  sharpVertices: Set<number>;
}

/**
 * Load an STL file (binary) and build the indexed mesh.
 */
export function loadMeshFromBuffer(buffer: ArrayBuffer): IndexedMesh {
  const parsed = parseStl(buffer);
  if (!parsed) throw new Error('STL parse failed');
  const v = parsed.vertices;
  const nTri = v.length / 9;

  const indexOf = new Map<string, number>();
  const positions: [number, number, number][] = [];
  const triangles: number[][] = [];
  const triNormals: [number, number, number][] = [];

  for (let t = 0; t < nTri; t++) {
    const idx: number[] = [];
    for (let k = 0; k < 3; k++) {
      const x = v[t * 9 + k * 3];
      const y = v[t * 9 + k * 3 + 1];
      const z = v[t * 9 + k * 3 + 2];
      // Round to 1e-4 mm for dedup (STL exports duplicate vertices per triangle).
      const key = `${x.toFixed(4)},${y.toFixed(4)},${z.toFixed(4)}`;
      let i = indexOf.get(key);
      if (i === undefined) {
        i = positions.length;
        indexOf.set(key, i);
        positions.push([x, y, z] as [number, number, number]);
      }
      idx.push(i);
    }
    triangles.push(idx);
    const a = positions[idx[0]];
    const b = positions[idx[1]];
    const c = positions[idx[2]];
    triNormals.push(vecNorm(vecCross(vecSub(b, a), vecSub(c, a))));
  }
  return { positions, triangles, triNormals };
}

/**
 * Mark sharp edges (dihedral > threshold) and the vertices incident to them.
 * Hole mouths / boss edges have large dihedral angles; smooth surfaces do not.
 */
export function analyzeSharpness(mesh: IndexedMesh, thresholdRad = 0.2): MeshAnalysis {
  const { triangles, triNormals } = mesh;
  const edgeMap = new Map<string, { a: number; b: number; tris: number[] }>();

  const keyOf = (a: number, b: number) => (a < b ? `${a}|${b}` : `${b}|${a}`);

  for (let t = 0; t < triangles.length; t++) {
    const [i0, i1, i2] = triangles[t];
    for (const [a, b] of [[i0, i1], [i1, i2], [i2, i0]] as const) {
      const k = keyOf(a, b);
      const e = edgeMap.get(k);
      if (e) e.tris.push(t);
      else edgeMap.set(k, { a, b, tris: [t] });
    }
  }

  const sharpVertices = new Set<number>();
  for (const e of edgeMap.values()) {
    if (e.tris.length < 2) continue;
    const [t1, t2] = e.tris;
    const d = vecDot(triNormals[t1], triNormals[t2]);
    const angle = Math.acos(Math.min(1, Math.max(-1, d)));
    if (angle > thresholdRad) {
      sharpVertices.add(e.a);
      sharpVertices.add(e.b);
    }
  }
  return { mesh, sharpVertices };
}

/**
 * Cluster sharp vertices into connected rings (BFS over sharp edges).
 */
export function clusterRings(analysis: MeshAnalysis): number[][] {
  const { mesh, sharpVertices } = analysis;
  const { triangles } = mesh;

  // Adjacency restricted to sharp edges.
  const adj = new Map<number, number[]>();
  const edgeMap = new Map<string, { a: number; b: number; sharp: boolean }>();
  const keyOf = (a: number, b: number) => (a < b ? `${a}|${b}` : `${b}|${a}`);

  for (let t = 0; t < triangles.length; t++) {
    const [i0, i1, i2] = triangles[t];
    for (const [a, b] of [[i0, i1], [i1, i2], [i2, i0]] as const) {
      const k = keyOf(a, b);
      if (edgeMap.has(k)) {
        const e = edgeMap.get(k)!;
        if (e.sharp) continue;
        const sharp = sharpVertices.has(a) && sharpVertices.has(b);
        if (sharp) {
          e.sharp = true;
          if (!adj.has(a)) adj.set(a, []);
          if (!adj.has(b)) adj.set(b, []);
          adj.get(a)!.push(b);
          adj.get(b)!.push(a);
        }
      } else {
        const sharp = sharpVertices.has(a) && sharpVertices.has(b);
        edgeMap.set(k, { a, b, sharp });
        if (sharp) {
          if (!adj.has(a)) adj.set(a, []);
          if (!adj.has(b)) adj.set(b, []);
          adj.get(a)!.push(b);
          adj.get(b)!.push(a);
        }
      }
    }
  }

  const visited = new Set<number>();
  const clusters: number[][] = [];
  for (const start of sharpVertices) {
    if (visited.has(start)) continue;
    const stack = [start];
    visited.add(start);
    const cluster: number[] = [];
    while (stack.length) {
      const v = stack.pop()!;
      cluster.push(v);
      for (const n of adj.get(v) ?? []) {
        if (!visited.has(n)) {
          visited.add(n);
          stack.push(n);
        }
      }
    }
    if (cluster.length >= 12) clusters.push(cluster);
  }
  return clusters;
}

// ─── SVD 3×3 (Jacobi rotations) ──────────────────────────────────────────────

/** Eigen decomposition of a symmetric 3×3 matrix (row-major). */
export function eigenSymmetric3(m: number[]): { values: number[]; vectors: number[][] } {
  let a = [m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]];
  let v = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let iter = 0; iter < 50; iter++) {
    let off = 0;
    for (let p = 0; p < 3; p++) {
      for (let q = p + 1; q < 3; q++) off += a[p * 3 + q] * a[p * 3 + q];
    }
    if (off < 1e-18) break;
    for (let p = 0; p < 3; p++) {
      for (let q = p + 1; q < 3; q++) {
        const apq = a[p * 3 + q];
        if (Math.abs(apq) < 1e-18) continue;
        const app = a[p * 3 + p];
        const aqq = a[q * 3 + q];
        const theta = (aqq - app) / (2 * apq);
        const t = Math.sign(theta) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 3; k++) {
          const akp = a[k * 3 + p];
          const akq = a[k * 3 + q];
          a[k * 3 + p] = c * akp - s * akq;
          a[k * 3 + q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = a[p * 3 + k];
          const aqk = a[q * 3 + k];
          a[p * 3 + k] = c * apk - s * aqk;
          a[q * 3 + k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = v[k * 3 + p];
          const vkq = v[k * 3 + q];
          v[k * 3 + p] = c * vkp - s * vkq;
          v[k * 3 + q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const values = [a[0], a[4], a[8]];
  const vectors: number[][] = [];
  for (let j = 0; j < 3; j++) {
    const col = [v[j], v[3 + j], v[6 + j]];
    const n = Math.hypot(col[0], col[1], col[2]);
    vectors.push(n > 1e-12 ? [col[0] / n, col[1] / n, col[2] / n] : [1, 0, 0]);
  }
  return { values, vectors };
}

// ─── Circle fitting ──────────────────────────────────────────────────────────

/**
 * Fit a 3D circle to a set of points (least squares, Kasa method).
 * Returns center, unit normal, radius and RMS residual.
 */
export function fitCircle(points: number[][]): {
  center: [number, number, number];
  normal: [number, number, number];
  radius: number;
  residual: number;
} | null {
  const n = points.length;
  if (n < 8) return null;
  const m: [number, number, number] = [0, 0, 0];
  for (const p of points) {
    m[0] += p[0]; m[1] += p[1]; m[2] += p[2];
  }
  m[0] /= n; m[1] /= n; m[2] /= n;

  // Covariance → plane normal = eigenvector of smallest eigenvalue.
  const cov = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const p of points) {
    const dx = p[0] - m[0], dy = p[1] - m[1], dz = p[2] - m[2];
    cov[0] += dx * dx; cov[1] += dx * dy; cov[2] += dx * dz;
    cov[3] += dx * dy; cov[4] += dy * dy; cov[5] += dy * dz;
    cov[6] += dx * dz; cov[7] += dy * dz; cov[8] += dz * dz;
  }
  const { values, vectors } = eigenSymmetric3(cov);
  let minIdx = 0;
  if (values[1] < values[minIdx]) minIdx = 1;
  if (values[2] < values[minIdx]) minIdx = 2;
  const normal = vecNorm(vectors[minIdx] as [number, number, number]);

  // Build an orthonormal basis in the plane.
  const ref: [number, number, number] =
    Math.abs(normal[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u1 = vecNorm(cross2(normal, ref));
  const u2 = cross2(normal, u1);

  // Project into 2D and fit circle (Kasa).
  let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sx3 = 0, sy3 = 0, sx2y = 0, sxy2 = 0;
  const pts2: [number, number][] = [];
  for (const p of points) {
    const x = (p[0] - m[0]) * u1[0] + (p[1] - m[1]) * u1[1] + (p[2] - m[2]) * u1[2];
    const y = (p[0] - m[0]) * u2[0] + (p[1] - m[1]) * u2[1] + (p[2] - m[2]) * u2[2];
    pts2.push([x, y]);
    const r2 = x * x + y * y;
    sx += x; sy += y;
    sxx += x * x; syy += y * y; sxy += x * y;
    sx3 += x * r2; sy3 += y * r2;
    sx2y += x * x * y; sxy2 += x * y * y;
  }
  // Normal equations (Kasa): [[Sxx,Sxy,Sx],[Sxy,Syy,Sy],[Sx,Sy,n]]·[a,b,c] = [Sx3,Sy3,Sr2]
  const A = [sxx, sxy, sx, sxy, syy, sy, sx, sy, n];
  const b = [sx3, sy3, sx2y + sxy2];
  const sol = solve3x3(A, b);
  if (!sol) return null;
  const [a, c, d] = sol;
  const cx2 = a / 2, cy2 = c / 2;
  const r2 = d + (a * a + c * c) / 4;
  if (r2 <= 0) return null;
  const radius = Math.sqrt(r2);

  const center: [number, number, number] = [
    m[0] + cx2 * u1[0] + cy2 * u2[0],
    m[1] + cx2 * u1[1] + cy2 * u2[1],
    m[2] + cx2 * u1[2] + cy2 * u2[2],
  ];

  let residual = 0;
  for (const [x, y] of pts2) {
    const dd = Math.hypot(x - cx2, y - cy2) - radius;
    residual += dd * dd;
  }
  residual = Math.sqrt(residual / n);
  return { center, normal, radius, residual };
}

/** Cross product helper. */
function cross2(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return vecCross(a, b);
}

/** Solve a 3×3 linear system by Gaussian elimination with partial pivoting. */
export function solve3x3(A: number[], b: number[]): [number, number, number] | null {
  const M = [A.slice(0, 3), A.slice(3, 6), A.slice(6, 9)];
  const B = b.slice();
  for (let col = 0; col < 3; col++) {
    let piv = col;
    for (let r = col + 1; r < 3; r++) {
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    }
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    [M[col], M[piv]] = [M[piv], M[col]];
    [B[col], B[piv]] = [B[piv], B[col]];
    for (let r = col + 1; r < 3; r++) {
      const f = M[r][col] / M[col][col];
      for (let c = col; c < 3; c++) M[r][c] -= f * M[col][c];
      B[r] -= f * B[col];
    }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r--) {
    let s = B[r];
    for (let c = r + 1; c < 3; c++) s -= M[r][c] * x[c];
    x[r] = s / M[r][r];
  }
  return [x[0], x[1], x[2]];
}

/**
 * Detect circle candidates (hole mouths / boss rings) on a mesh.
 *
 * Approach: spatial grid over the sharp vertices; per-cell LOCAL circle
 * fits (a single cell captures a clean arc of a ring, avoiding the
 * silhouette contamination that mixing 27-cell windows causes); then
 * multi-scale consensus (cells of the same ring must agree on center /
 * radius / normal). Multi-scale (4/6/8mm cells) covers both small bores
 * and large pockets.
 *
 * Generous by design: false positives are acceptable — the solver judges.
 */
export function detectCircles(mesh: IndexedMesh, radiusMin = 1.5, radiusMax = 20): CircleCandidate[] {
  const { mesh: m, sharpVertices } = analyzeSharpness(mesh);
  const positions = m.positions;
  if (sharpVertices.size === 0) return [];

  const allLocalFits: { center: [number, number, number]; normal: [number, number, number]; radius: number; residual: number; pts: number }[] = [];

  for (const CELL of [4, 6, 8]) {
    const keyOf = (cx: number, cy: number, cz: number) => `${cx},${cy},${cz}`;
    const grid = new Map<string, number[]>();
    for (const vi of sharpVertices) {
      const p = positions[vi];
      const k = keyOf(Math.floor(p[0] / CELL), Math.floor(p[1] / CELL), Math.floor(p[2] / CELL));
      const cell = grid.get(k);
      if (cell) cell.push(vi);
      else grid.set(k, [vi]);
    }
    for (const [k, cell] of grid) {
      if (cell.length < 6) continue;
      const pts = cell.map((i) => positions[i]);
      const fit = fitCircle(pts);
      if (!fit) continue;
      if (fit.radius < radiusMin || fit.radius > radiusMax) continue;
      if (fit.residual / fit.radius > 0.15) continue;
      allLocalFits.push({ center: fit.center, normal: fit.normal, radius: fit.radius, residual: fit.residual, pts: pts.length });
    }
  }

  // Consensus clustering of local fits (generous: same ring seen from
  // neighboring windows/scales must agree on center/radius/normal).
  const groups: typeof allLocalFits[] = [];
  for (const fit of allLocalFits) {
    let placed = false;
    for (const g of groups) {
      const ref = g[0];
      const dc = Math.hypot(fit.center[0] - ref.center[0], fit.center[1] - ref.center[1], fit.center[2] - ref.center[2]);
      const dot = Math.abs(fit.normal[0] * ref.normal[0] + fit.normal[1] * ref.normal[1] + fit.normal[2] * ref.normal[2]);
      if (dc < 4 && Math.abs(fit.radius - ref.radius) < 1.0 && dot > 0.75) {
        g.push(fit);
        placed = true;
        break;
      }
    }
    if (!placed) groups.push([fit]);
  }

  const candidates: CircleCandidate[] = [];
  for (const g of groups) {
    if (g.length < 2) continue; // consensus: ≥2 fits agree
    const center: [number, number, number] = [0, 0, 0];
    let radius = 0;
    let residual = 0;
    for (const f of g) {
      center[0] += f.center[0]; center[1] += f.center[1]; center[2] += f.center[2];
      radius += f.radius;
      residual += f.residual;
    }
    center[0] /= g.length; center[1] /= g.length; center[2] /= g.length;
    radius /= g.length;
    residual /= g.length;
    candidates.push({
      id: candidates.length,
      center,
      normal: g[0].normal,
      radius,
      residual,
      nVerts: g.reduce((s, f) => s + f.pts, 0),
    });
  }
  return candidates;
}

/** Bounding box of a mesh. */
export function boundingBox(mesh: IndexedMesh): { min: [number, number, number]; max: [number, number, number] } {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const p of mesh.positions) {
    if (p[0] < minX) minX = p[0];
    if (p[1] < minY) minY = p[1];
    if (p[2] < minZ) minZ = p[2];
    if (p[0] > maxX) maxX = p[0];
    if (p[1] > maxY) maxY = p[1];
    if (p[2] > maxZ) maxZ = p[2];
  }
  return { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] };
}

/** Detect the bottom face plane (Y = min_y of bbox) — used by Base.stl. */
export function bottomFacePoint(mesh: IndexedMesh): { point: [number, number, number]; normal: [number, number, number] } | null {
  let minY = Infinity;
  for (const p of mesh.positions) {
    if (p[1] < minY) minY = p[1];
  }
  // Average the vertices on the bottom band (within 2mm of minY).
  let sx = 0, sz = 0, n = 0;
  for (const p of mesh.positions) {
    if (Math.abs(p[1] - minY) < 2) {
      sx += p[0]; sz += p[2]; n++;
    }
  }
  if (n === 0) return null;
  return { point: [sx / n, minY, sz / n], normal: [0, -1, 0] };
}
