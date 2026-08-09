import type { PlanSample } from './planTimeline';

export const V2_CHUNK_MAX = 24;
export const V2_US_MIN = 500;
export const V2_US_MAX = 2400;

export interface ManifestResult {
  lines: string[];
  count: number;
  durationUs: number;
}

export function buildManifest(samples: PlanSample[]): ManifestResult | Error {
  if (samples.length === 0) {
    return new Error('plan vacío');
  }
  const lines: string[] = [];
  let prevFrameT: number | null = null;
  let totalCount = 0;
  for (const s of samples) {
    let dtUs: number;
    if (prevFrameT === null) {
      dtUs = 0;
    } else {
      dtUs = Math.round((s.t - prevFrameT) * 1e6);
      if (dtUs <= 0) {
        return new Error('timeline no creciente');
      }
    }
    const q = s.q_us.map((v) => Math.round(v));
    for (let i = 0; i < q.length; i++) {
      if (q[i] < V2_US_MIN || q[i] > V2_US_MAX) {
        return new Error('joint fuera de rango: ' + q[i]);
      }
    }
    lines.push(`SAMPLE ${q.join(' ')} ${dtUs}`);
    totalCount += 1;
    prevFrameT = s.t;
  }
  const firstT = samples[0].t;
  const lastFrameT = prevFrameT ?? firstT;
  const durationUs = Math.max(0, Math.round((lastFrameT - firstT) * 1e6));
  return { lines, count: totalCount, durationUs };
}

export function sliceLines(lines: string[], chunkMax: number): string[][] {
  const chunks: string[][] = [];
  for (let i = 0; i < lines.length; i += chunkMax) {
    chunks.push(lines.slice(i, i + chunkMax));
  }
  return chunks;
}

export function validateManifest(lines: string[]): { count: number; durationUs: number } | Error {
  let count = 0;
  let sumDt = 0;
  let sawFirst = false;
  for (const line of lines) {
    const m = /^SAMPLE ((?:\d+ ){5}\d+) (\d+)$/.exec(line);
    if (!m) {
      return new Error('línea inválida: ' + line);
    }
    const vals = m[1].split(' ').map(Number);
    const dt = Number(m[2]);
    for (const v of vals) {
      if (v < V2_US_MIN || v > V2_US_MAX) {
        return new Error('joint fuera de rango');
      }
    }
    if (!sawFirst) {
      if (dt !== 0) {
        return new Error('primer dt debe ser 0');
      }
      sawFirst = true;
    } else if (dt <= 0) {
      return new Error('dt debe ser > 0');
    }
    sumDt += dt;
    count += 1;
  }
  if (count === 0) {
    return new Error('manifest vacío');
  }
  return { count, durationUs: sumDt };
}
