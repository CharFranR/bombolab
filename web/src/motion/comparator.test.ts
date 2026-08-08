import { describe, expect, it } from 'vitest';
import { compare, type TraceMetrics } from './comparator';
import type { PlanSample } from './planTimeline';
import type { TraceSample } from './trace';

function plan(rows: [number, number[], number][]): PlanSample[] {
  return rows.map(([t, q_us, count]) => ({ t, q_us, count }));
}

function trace(rows: [number, number[], number][]): TraceSample[] {
  return rows.map(([ts_ms, q_us, count]) => ({ ts_ms, q_us: q_us as [number, number, number, number, number, number], count }));
}

const Q0 = [1000, 2000, 3000, 4000, 5000, 6000];
const Q1 = [1100, 2000, 3000, 4000, 5000, 6000];
const Q2 = [1200, 2000, 3000, 4000, 5000, 6000];

function zeroMetrics(): TraceMetrics {
  return {
    sendTimeError: { mean: 0, maxAbs: 0, sigma: 0 },
    gaps: { min: 0, mean: 0, max: 0, sigma: 0, count: 0, over1s: 0 },
    deviation: {
      perJoint: Array.from({ length: 6 }, () => ({ mean: 0, max: 0 })),
      globalMax: 0,
      globalMean: 0,
    },
    duration: { plan: 0, real: 0 },
    framesWritten: 0,
    truncated: false,
  };
}

describe('compare — TC-2 ordinal alignment and send-time error', () => {
  it('computes send-time error, gaps, deviation and durations from aligned samples', () => {
    const m = compare(
      plan([[0, Q0, 1], [0.1, Q1, 1], [0.2, Q2, 1]]),
      trace([[5, [1005, 2000, 3000, 4000, 5000, 6000], 1], [115, [1115, 2000, 3000, 4000, 5000, 6000], 1], [235, [1250, 2000, 3000, 4000, 5000, 6000], 1]]),
    );
    expect(m.sendTimeError.mean).toBeCloseTo(55 / 3, 9);
    expect(m.sendTimeError.maxAbs).toBe(35);
    expect(m.sendTimeError.sigma).toBeCloseTo(15.2753, 3);
    expect(m.gaps.min).toBe(110);
    expect(m.gaps.mean).toBe(115);
    expect(m.gaps.max).toBe(120);
    expect(m.gaps.sigma).toBeCloseTo(7.0711, 4);
    expect(m.gaps.count).toBe(2);
    expect(m.gaps.over1s).toBe(0);
    expect(m.deviation.perJoint[0]).toEqual({ mean: 70 / 3, max: 50 });
    for (let j = 1; j < 6; j++) {
      expect(m.deviation.perJoint[j]).toEqual({ mean: 0, max: 0 });
    }
    expect(m.deviation.globalMax).toBe(50);
    expect(m.deviation.globalMean).toBeCloseTo(70 / 18, 9);
    expect(m.duration).toEqual({ plan: 200, real: 235 });
    expect(m.framesWritten).toBe(3);
    expect(m.truncated).toBe(false);
  });

  it('reports negative send-time errors with maxAbs as magnitude', () => {
    const m = compare(
      plan([[0, Q0, 1], [0.1, Q1, 1]]),
      trace([[-5, Q0, 1], [90, Q1, 1]]),
    );
    expect(m.sendTimeError.mean).toBeCloseTo(-7.5, 9);
    expect(m.sendTimeError.maxAbs).toBe(10);
    expect(m.sendTimeError.sigma).toBeCloseTo(3.5355, 3);
  });

  it('computes the plan duration from last sample time plus coalesced hold ticks', () => {
    const m = compare(
      plan([[0, Q0, 1], [1, Q1, 21]]),
      trace([[5, Q0, 1], [1010, Q1, 3]]),
      { planDt: 0.05 },
    );
    expect(m.duration.plan).toBe(2000);
    expect(m.duration.real).toBe(1010);
  });
});

describe('compare — TC-2 empty inputs', () => {
  it('returns empty metrics for an empty plan', () => {
    expect(compare([], trace([[0, Q0, 1]]))).toEqual(zeroMetrics());
  });

  it('returns empty metrics for an empty trace', () => {
    expect(compare(plan([[0, Q0, 1]]), [])).toEqual(zeroMetrics());
  });

  it('returns empty metrics when both are empty', () => {
    expect(compare([], [])).toEqual(zeroMetrics());
  });
});

describe('compare — TC-3 heartbeat-only trace', () => {
  it('yields a single paired sample, no gap stats and zero deviation', () => {
    const m = compare(
      plan([[0, Q0, 1]]),
      trace([[0, Q0, 5]]),
    );
    expect(m.sendTimeError).toEqual({ mean: 0, maxAbs: 0, sigma: 0 });
    expect(m.gaps).toEqual({ min: 0, mean: 0, max: 0, sigma: 0, count: 0, over1s: 0 });
    expect(m.deviation.globalMax).toBe(0);
    expect(m.deviation.globalMean).toBe(0);
    expect(m.duration).toEqual({ plan: 0, real: 0 });
    expect(m.framesWritten).toBe(5);
  });
});

describe('compare — TC-4 throttle gaps and heartbeat exclusion', () => {
  it('keeps gaps above 1 s unsmoothed and counts them', () => {
    const m = compare(
      plan([[0, Q0, 1], [0.11, Q1, 1], [2.6, Q2, 1]]),
      trace([[0, Q0, 1], [110, Q1, 1], [2600, Q2, 1]]),
    );
    expect(m.gaps.min).toBe(110);
    expect(m.gaps.mean).toBe(1300);
    expect(m.gaps.max).toBe(2490);
    expect(m.gaps.sigma).toBeCloseTo(1682.914, 3);
    expect(m.gaps.count).toBe(2);
    expect(m.gaps.over1s).toBe(1);
  });

  it('excludes gap pairs adjacent to heartbeat-deduplicated samples', () => {
    const m = compare(
      plan([[0, Q0, 1], [0.26, Q2, 1]]),
      trace([[0, Q0, 1], [110, Q1, 1], [120, Q1, 4], [250, Q2, 1], [260, Q2, 1]]),
    );
    expect(m.gaps.min).toBe(10);
    expect(m.gaps.mean).toBe(60);
    expect(m.gaps.max).toBe(110);
    expect(m.gaps.sigma).toBeCloseTo(70.7107, 4);
    expect(m.gaps.count).toBe(2);
    expect(m.gaps.over1s).toBe(0);
  });
});
