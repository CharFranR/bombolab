import { describe, expect, it } from 'vitest';
import { LEVEL_COLORS, levelColor, parseWorkspaceBatch, validateSampleCount } from './colors';

describe('levelColor display bands 50/25', () => {
  it('maps sigma_min to high/medium/low with inclusive boundaries', () => {
    expect(levelColor(60)).toEqual(LEVEL_COLORS.high);
    expect(levelColor(50)).toEqual(LEVEL_COLORS.high);
    expect(levelColor(30)).toEqual(LEVEL_COLORS.medium);
    expect(levelColor(25)).toEqual(LEVEL_COLORS.medium);
    expect(levelColor(10)).toEqual(LEVEL_COLORS.low);
    expect(levelColor(0)).toEqual(LEVEL_COLORS.low);
  });

  it('keeps the three bands visually distinct', () => {
    const seen = new Set(
      [levelColor(60), levelColor(30), levelColor(10)].map((c) => c.join(',')),
    );
    expect(seen.size).toBe(3);
  });
});

describe('parseWorkspaceBatch stride-5', () => {
  it('splits records into positions with DH→three swap and per-point colors', () => {
    const batch = new Float64Array([10, 20, 30, 60, 1.5, 40, 50, 60, 10, 2.5]);
    const { positions, colors } = parseWorkspaceBatch(batch);
    expect(Array.from(positions)).toEqual([10, 30, 20, 40, 60, 50]);
    const expectedColors = new Float32Array([...LEVEL_COLORS.high, ...LEVEL_COLORS.low]);
    expect(Array.from(colors)).toEqual(Array.from(expectedColors));
  });

  it('derives color from sigma_min, not from position', () => {
    const batch = new Float64Array([1, 2, 3, 30, 0.5, 4, 5, 6, 60, 0.5]);
    const { colors } = parseWorkspaceBatch(batch);
    expect(Array.from(colors.slice(0, 3))).toEqual(Array.from(new Float32Array(LEVEL_COLORS.medium)));
    expect(Array.from(colors.slice(3))).toEqual(Array.from(new Float32Array(LEVEL_COLORS.high)));
  });

  it('returns empty arrays for an empty batch', () => {
    const { positions, colors } = parseWorkspaceBatch(new Float64Array(0));
    expect(positions.length).toBe(0);
    expect(colors.length).toBe(0);
  });
});

describe('validateSampleCount', () => {
  it('rejects zero and negative counts', () => {
    expect(validateSampleCount(0)).not.toBeNull();
    expect(validateSampleCount(-5)).not.toBeNull();
  });

  it('accepts positive counts', () => {
    expect(validateSampleCount(1000)).toBeNull();
  });
});
