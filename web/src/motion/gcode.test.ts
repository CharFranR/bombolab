/**
 * Fixture tests for the .gcode → MotionCommandJS parser.
 *
 * parseGcode is a pure function (text in → commands out): no wasm, no React,
 * no network. These tests lock in the accepted dialect so dialect regressions
 * (a dud line silently changing the drawing) fail instead of passing unnoticed.
 *
 * Most cases run with `autofit: false` so raw coordinates are asserted 1:1;
 * a dedicated case covers the autofit scale+center path.
 */
import { describe, it, expect } from 'vitest';
import { parseGcode, type GcodeOptions } from './gcode';
import type { MotionCommandJS } from './commands';

function parse(text: string, opts: GcodeOptions = {}): ReturnType<typeof parseGcode> {
  return parseGcode(text, { autofit: false, ...opts });
}

type Move = Extract<MotionCommandJS, { type: 'move' }>;
const moves = (cmds: MotionCommandJS[]): Move[] =>
  cmds.filter((c) => c.type === 'move') as Move[];

describe('parseGcode', () => {
  it('parses the CIPRA fixture (absolute mm, M3/M5 pen control)', () => {
    const cipra = [
      'G21 G90',
      'G0 Z5',
      'M3',
      'G1 X10 Y10 F600',
      'G1 X20 Y20',
      'M5',
      'G0 Z50',
    ].join('\n');

    const r = parse(cipra);
    expect(r.warnings).toEqual([]);
    expect(r.commands).toEqual([
      { type: 'move', target: [0, 0, 85], speed: 40 }, // travel lift before M3
      { type: 'penDown' },
      { type: 'move', target: [10, 10, 80], speed: 10 }, // F600 → 600/60 = 10 mm/s
      { type: 'move', target: [20, 20, 80], speed: 10 },
      { type: 'penUp' },
      { type: 'move', target: [20, 20, 85], speed: 10 },
    ]);
    expect(r.moveCount).toBe(4);
    expect(r.bounds).toEqual({ min: [0, 0], max: [20, 20] });
  });

  it('reads G01 zero-padded coordinates numerically', () => {
    const padded = [
      'G21 G90',
      'G0 Z5',
      'M3',
      'G01 X00010.000 Y00010.000 F600',
      'G01 X00020 Y00020',
    ].join('\n');

    const r = parse(padded);
    expect(r.commands).toEqual([
      { type: 'move', target: [0, 0, 85], speed: 40 }, // travel, default feed
      { type: 'penDown' },
      { type: 'move', target: [10, 10, 80], speed: 10 },
      { type: 'move', target: [20, 20, 80], speed: 10 },
    ]);
    expect(r.moveCount).toBe(3);
  });

  it('parses compact lines with no whitespace', () => {
    const compact = 'G21G90M3\nG0X10Y10\nG1X20Y20';
    const r = parse(compact);
    expect(r.commands).toEqual([
      { type: 'penDown' },
      { type: 'move', target: [10, 10, 80], speed: 40 },
      { type: 'move', target: [20, 20, 80], speed: 40 },
    ]);
    expect(r.moveCount).toBe(2);
  });

  it('accumulates G91 relative moves', () => {
    const relative = ['G21 G91', 'G0 X10 Y10', 'G1 X10 Y0', 'G1 X0 Y10'].join('\n');
    const r = parse(relative);
    // no Z / no M3 → pen is always down; first command is penDown
    expect(r.commands[0]).toEqual({ type: 'penDown' });
    expect(moves(r.commands).map((m) => m.target)).toEqual([
      [10, 10, 80],
      [20, 10, 80],
      [20, 20, 80],
    ]);
  });

  it('stops at M2/M30, ignoring the rest of the file', () => {
    const m2 = ['G21 G90', 'M3', 'G1 X10 Y10', 'M2', 'G1 X50 Y50'].join('\n');
    const r = parse(m2);
    expect(moves(r.commands).map((m) => m.target)).toEqual([[10, 10, 80]]);
    expect(r.warnings.some((w) => w.includes('M2/M30'))).toBe(true);
  });

  it('skips G2/G3 arcs with a warning (v1 unsupported)', () => {
    const arc = ['G21 G90', 'M3', 'G2 X20 Y20 I5 J0', 'G1 X30 Y30'].join('\n');
    const r = parse(arc);
    expect(moves(r.commands).map((m) => m.target)).toEqual([[30, 30, 80]]);
    expect(r.warnings.some((w) => w.includes('G2/G3'))).toBe(true);
  });

  it('returns an empty result for empty/comment-only files', () => {
    const r = parse('  \n; nothing here\n(block comment)\n');
    expect(r.commands).toEqual([]);
    expect(r.warnings).toEqual([]);
    expect(r.bounds).toBeNull();
    expect(r.moveCount).toBe(0);
  });

  it('converts G20 inches to mm', () => {
    const r = parse('G20 G90\nG0 X1 Y1');
    expect(moves(r.commands).map((m) => m.target)).toEqual([[25.4, 25.4, 80]]);
  });

  it('auto-detects pen state from Z when the file has no M3/M5', () => {
    const byZ = [
      'G21 G90',
      'G0 Z5',
      'G1 X10 Y10',
      'G1 X20 Y20 Z0',
      'G0 Z5',
    ].join('\n');
    const r = parse(byZ);
    expect(r.commands).toEqual([
      { type: 'move', target: [0, 0, 85], speed: 40 },
      { type: 'move', target: [10, 10, 85], speed: 40 },
      { type: 'penDown' },
      { type: 'move', target: [20, 20, 80], speed: 40 },
      { type: 'penUp' },
      { type: 'move', target: [20, 20, 85], speed: 40 },
    ]);
    // stroke-start point (where the pen went down) is included in bounds
    expect(r.bounds).toEqual({ min: [10, 10], max: [20, 20] });
  });

  it('G4 dwell: P is milliseconds, S is seconds (wait in seconds)', () => {
    const byP = parse('G21 G90\nG4 P2000\nG1 X10 Y10');
    expect(byP.commands).toEqual([
      { type: 'wait', duration: 2 }, // 2000 ms → 2 s
      { type: 'penDown' },
      { type: 'move', target: [10, 10, 80], speed: 40 },
    ]);

    const byS = parse('G21 G90\nG4 S1.5\nG1 X10 Y10');
    expect(byS.commands).toEqual([
      { type: 'wait', duration: 1.5 }, // seconds, passed through as-is
      { type: 'penDown' },
      { type: 'move', target: [10, 10, 80], speed: 40 },
    ]);
  });

  it('autofits (scales + centers) the drawing to the target area', () => {
    const r = parseGcode('G21 G90\nM3\nG1 X0 Y0\nG1 X100 Y0\n', {
      area: { xMin: 160, xMax: 240, yMin: -35, yMax: 35 },
    });
    const ms = moves(r.commands).map((m) => m.target);
    expect(ms.map((t) => t[0])).toEqual([165, 235]); // 100mm → 70mm of the 80mm band
    expect(ms[0][1]).toBe(0);
    expect(ms[0][2]).toBe(80);
  });
});