import { describe, it, expect } from 'vitest';
import { buildManifest, sliceLines, validateManifest, V2_CHUNK_MAX } from './manifest';
import type { PlanSample } from './planTimeline';

function sample(t: number, q: number[], count = 1): PlanSample {
  return { t, q_us: q, count };
}

const Q = [1500, 1500, 1500, 1500, 1500, 1500];

describe('buildManifest', () => {
  it('emite primer dt=0 y deltas correctos', () => {
    const samples = [sample(0, Q), sample(0.05, [1600, 1500, 1500, 1500, 1500, 1500]), sample(0.1, Q)];
    const r = buildManifest(samples);
    expect(r).not.toBeInstanceOf(Error);
    const m = r as { lines: string[]; count: number; durationUs: number };
    expect(m.lines).toHaveLength(3);
    expect(m.lines[0]).toBe(`SAMPLE ${Q.join(' ')} 0`);
    expect(m.lines[1]).toContain(' 50000');
    expect(m.lines[2]).toContain(' 50000');
    expect(m.count).toBe(3);
    expect(m.durationUs).toBe(100000);
  });

  it('acumula dedupe via count en el delta del siguiente sample', () => {
    const samples = [sample(0, Q, 3), sample(0.2, [1600, 1500, 1500, 1500, 1500, 1500], 1)];
    const r = buildManifest(samples) as { lines: string[]; count: number; durationUs: number };
    expect(r.count).toBe(2);
    expect(r.lines[0]).toBe(`SAMPLE ${Q.join(' ')} 0`);
    expect(r.lines[1]).toContain(' 200000');
    expect(r.durationUs).toBe(200000);
  });

  it('redondea dt a microsegundos', () => {
    const samples = [sample(0, Q), sample(0.0167, Q.map((v) => v + 1))];
    const r = buildManifest(samples) as { lines: string[] };
    expect(r.lines[1]).toContain(' 16700');
  });

  it('rechaza plan vacío', () => {
    expect(buildManifest([])).toBeInstanceOf(Error);
  });

  it('rechaza timeline no creciente', () => {
    const samples = [sample(0, Q), sample(0.05, Q), sample(0.05, Q)];
    expect(buildManifest(samples)).toBeInstanceOf(Error);
  });

  it('rechaza joints fuera de rango', () => {
    const samples = [sample(0, [3000, 1500, 1500, 1500, 1500, 1500])];
    expect(buildManifest(samples)).toBeInstanceOf(Error);
  });
});

describe('sliceLines', () => {
  it('parte en chunks de chunkMax', () => {
    const lines = Array.from({ length: 70 }, (_, i) => `SAMPLE ${Q.join(' ')} ${i === 0 ? 0 : 50000}`);
    const chunks = sliceLines(lines, V2_CHUNK_MAX);
    expect(chunks.map((c) => c.length)).toEqual([24, 24, 22]);
  });
});

describe('validateManifest', () => {
  it('round-trip valida build → count/duration', () => {
    const samples = [sample(0, Q, 2), sample(0.1, [1600, 1500, 1500, 1500, 1500, 1500])];
    const built = buildManifest(samples) as { lines: string[]; count: number; durationUs: number };
    const v = validateManifest(built.lines);
    expect(v).not.toBeInstanceOf(Error);
    const r = v as { count: number; durationUs: number };
    expect(r.count).toBe(built.count);
    expect(r.durationUs).toBe(built.durationUs);
  });

  it('rechaza primer dt != 0', () => {
    expect(validateManifest([`SAMPLE ${Q.join(' ')} 50000`])).toBeInstanceOf(Error);
  });

  it('rechaza línea corrupta', () => {
    expect(validateManifest(['SAMPLE 1500 1500'])).toBeInstanceOf(Error);
  });

  it('rechaza joint fuera de rango', () => {
    expect(validateManifest([`SAMPLE 3000 ${Q.slice(1).join(' ')} 0`])).toBeInstanceOf(Error);
  });
});
