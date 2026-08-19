import { describe, expect, it } from 'vitest';
import { parseFrameFile, splitStrokes } from './plotFile';
import { TRACE_CSV_HEADER } from './csv';

describe('parseFrameFile — trace CSV', () => {
  const CSV = [
    TRACE_CSV_HEADER,
    '0,1000,1472,1379,1523,1162,1678,1',
    '40,1100,1472,1379,1523,1162,1678,2',
  ].join('\n');

  it('parses trace CSV rows into (t, q_us) frames', () => {
    const frames = parseFrameFile(CSV);
    expect(frames).toHaveLength(2);
    expect(frames[0]).toEqual({ t: 0, q_us: [1000, 1472, 1379, 1523, 1162, 1678] });
    expect(frames[1].t).toBe(40);
  });

  it('rejects a CSV row with wrong field count', () => {
    const bad = TRACE_CSV_HEADER + '\n1,2,3\n';
    expect(() => parseFrameFile(bad)).toThrow(/8 campos/);
  });

  it('rejects non-integer joint values', () => {
    const bad = TRACE_CSV_HEADER + '\n0,1.5,1472,1379,1523,1162,1678,1\n';
    expect(() => parseFrameFile(bad)).toThrow(/enteros/);
  });
});

describe('parseFrameFile — SAMPLE protocol lines', () => {
  const LINES = [
    'SAMPLE 1000 1472 1379 1523 1162 1678 0',
    'SAMPLE 1100 1472 1379 1523 1162 1678 40000',
    'SAMPLE 1200 1472 1379 1523 1162 1678 20000',
  ].join('\n');

  it('parses SAMPLE lines with start-time semantics (dt accumulates)', () => {
    const frames = parseFrameFile(LINES);
    expect(frames).toHaveLength(3);
    expect(frames[0]).toEqual({ t: 0, q_us: [1000, 1472, 1379, 1523, 1162, 1678] });
    // Frame 1 starts at t=0 (its dt=40000 runs until 0.04), frame 2 starts at 0.04.
    expect(frames[1].t).toBe(0);
    expect(frames[2].t).toBeCloseTo(0.04, 12);
  });

  it('rejects malformed SAMPLE lines', () => {
    expect(() => parseFrameFile('SAMPLE 1 2 3')).toThrow(/SAMPLE malformada/);
  });

  it('rejects out-of-range µs values', () => {
    const bad = 'SAMPLE 100 1472 1379 1523 1162 1678 0\n';
    expect(() => parseFrameFile(bad)).toThrow(/rango/);
  });
});

describe('parseFrameFile — format detection', () => {
  it('rejects unknown formats with a clear message', () => {
    expect(() => parseFrameFile('G0 X10 Y20')).toThrow(/formato no reconocido/);
  });

  it('rejects empty files', () => {
    expect(() => parseFrameFile('   \n ')).toThrow(/vacío/);
  });
});

describe('splitStrokes — stroke segmentation', () => {
  it('keeps consecutive drawing points in one stroke', () => {
    const pts = [
      { x: 0, y: 0, drawing: true },
      { x: 1, y: 0, drawing: true },
      { x: 2, y: 0, drawing: true },
    ];
    const strokes = splitStrokes(pts);
    expect(strokes).toHaveLength(1);
    expect(strokes[0]).toHaveLength(3);
  });

  it('breaks the stroke when a travel point intervenes', () => {
    const pts = [
      { x: 0, y: 0, drawing: true },
      { x: 1, y: 0, drawing: true },
      { x: 10, y: 10, drawing: false }, // pen up between strokes
      { x: 20, y: 0, drawing: true },
      { x: 21, y: 0, drawing: true },
    ];
    const strokes = splitStrokes(pts);
    expect(strokes).toHaveLength(2);
    expect(strokes[0]).toEqual([
      [0, 0],
      [1, 0],
    ]);
    expect(strokes[1]).toEqual([
      [20, 0],
      [21, 0],
    ]);
  });

  it('breaks the stroke on a large jump even without a travel sample', () => {
    const pts = [
      { x: 0, y: 0, drawing: true },
      { x: 1, y: 0, drawing: true },
      { x: 50, y: 0, drawing: true }, // 49mm jump > gap
      { x: 51, y: 0, drawing: true },
    ];
    const strokes = splitStrokes(pts);
    expect(strokes).toHaveLength(2);
  });

  it('returns empty for no drawing points', () => {
    expect(splitStrokes([{ x: 1, y: 1, drawing: false }])).toEqual([]);
  });
});
