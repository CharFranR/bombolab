export type Rgb = readonly [number, number, number];

export const LEVEL_COLORS = {
  high: [0.96, 0.36, 0.32] as const,
  medium: [0.98, 0.78, 0.34] as const,
  low: [0.36, 0.62, 0.96] as const,
};

export function levelColor(sigmaMin: number): Rgb {
  if (sigmaMin >= 50) return LEVEL_COLORS.high;
  if (sigmaMin >= 25) return LEVEL_COLORS.medium;
  return LEVEL_COLORS.low;
}

export function validateSampleCount(n: number): string | null {
  if (!Number.isFinite(n) || n <= 0) return 'N must be a positive integer';
  return null;
}

export interface WorkspacePoints {
  positions: Float32Array;
  colors: Float32Array;
}

const STRIDE = 5;
const X = 0;
const Y = 1;
const Z = 2;
const SIGMA_MIN = 3;

export function parseWorkspaceBatch(batch: Float64Array): WorkspacePoints {
  const records = batch.length / STRIDE;
  const positions = new Float32Array(records * 3);
  const colors = new Float32Array(records * 3);
  for (let i = 0; i < records; i++) {
    const base = i * STRIDE;
    positions[i * 3] = batch[base + X];
    positions[i * 3 + 1] = batch[base + Z];
    positions[i * 3 + 2] = batch[base + Y];
    const color = levelColor(batch[base + SIGMA_MIN]);
    colors[i * 3] = color[0];
    colors[i * 3 + 1] = color[1];
    colors[i * 3 + 2] = color[2];
  }
  return { positions, colors };
}

export function concatWorkspaceChunks(chunks: WorkspacePoints[]): WorkspacePoints {
  const positions = new Float32Array(chunks.reduce((n, c) => n + c.positions.length, 0));
  const colors = new Float32Array(chunks.reduce((n, c) => n + c.colors.length, 0));
  let p = 0;
  let q = 0;
  for (const chunk of chunks) {
    positions.set(chunk.positions, p);
    colors.set(chunk.colors, q);
    p += chunk.positions.length;
    q += chunk.colors.length;
  }
  return { positions, colors };
}
