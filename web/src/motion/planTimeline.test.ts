import { describe, expect, it } from 'vitest';
import { gripperToServoUs, qToServoUs } from '../serial';
import { DEFAULT_TOOL_TRANSFORM, type RobotDef } from '../kinematics/types';
import type { MotionCommandJS } from './commands';
import { planTimeline, type IkFn, type PlanSample } from './planTimeline';

const robot: RobotDef = {
  name: 'fixture',
  segments: [],
  baseTransform: [0, 0, 0],
  toolTransform: DEFAULT_TOOL_TRANSFORM,
};

const START_Q = [0, 0, 0, 0, 0];

function qUs(q: number[]): number[] {
  return [...qToServoUs(q), gripperToServoUs(50)];
}

function makeIk(calls: [number, number, number][] = [], failAtX?: number): IkFn {
  return (_r, target, _qInit) => {
    calls.push(target);
    if (failAtX !== undefined && target[0] === failAtX) {
      return { q: [0.05, 0, 0, 0, 0], converged: false, error: 42 };
    }
    return { q: [target[0] / 100, target[1] / 100, target[2] / 100, 0, 0], converged: true, error: 0 };
  };
}

function lastT(samples: PlanSample[]): number {
  const last = samples[samples.length - 1];
  return last.t + (last.count - 1) * 0.04;
}

describe('planTimeline — TC-1 analytic timeline', () => {
  it('builds a linear segment at speed with exact analytic duration', () => {
    const calls: [number, number, number][] = [];
    const cmds: MotionCommandJS[] = [{ type: 'move', target: [100, 0, 0], speed: 50 }];
    const tl = planTimeline(cmds, { ik: makeIk(calls), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0] });
    expect(tl.length).toBeGreaterThan(0);
    expect(tl[0]).toEqual({ t: 0, q_us: qUs([0, 0, 0, 0, 0]), count: 1, penDown: false });
    expect(tl[tl.length - 1].t).toBe(2.0);
    expect(lastT(tl)).toBeCloseTo(2.0, 6);
    expect(calls.length).toBe(51);
    for (const s of tl) {
      expect(s.q_us).toHaveLength(6);
      expect(s.q_us[5]).toBe(gripperToServoUs(50));
    }
  });

  it('holds the pose during a wait and keeps the wait in the duration', () => {
    const cmds: MotionCommandJS[] = [
      { type: 'move', target: [100, 0, 0], speed: 50 },
      { type: 'wait', duration: 1.5 },
      { type: 'move', target: [160, 0, 0], speed: 20 },
    ];
    const tl = planTimeline(cmds, { ik: makeIk(), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0] });
    expect(lastT(tl)).toBeCloseTo(6.5, 6);
    const hold = tl.find((s) => s.count > 5);
    expect(hold).toBeDefined();
    expect(hold!.t).toBe(2.0);
    expect(hold!.count).toBe(39);
    expect(hold!.q_us).toEqual(qUs([1, 0, 0, 0, 0]));
    const endpoint = tl[tl.length - 1];
    expect(endpoint.t + (endpoint.count - 1) * 0.04).toBe(6.5);
    expect(endpoint.q_us).toEqual(qUs([1.6, 0, 0, 0, 0]));
  });

  it('treats penUp/penDown as instant with no time and no pose change', () => {
    const cmds: MotionCommandJS[] = [
      { type: 'penDown' },
      { type: 'move', target: [50, 0, 0], speed: 50 },
      { type: 'penUp' },
      { type: 'wait', duration: 1 },
      { type: 'penDown' },
    ];
    const tl = planTimeline(cmds, { ik: makeIk(), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0] });
    expect(lastT(tl)).toBeCloseTo(2.0, 6);
    expect(tl.some((s) => s.count > 5)).toBe(true);
  });

  it('returns an empty timeline for an empty command list', () => {
    const tl = planTimeline([], { ik: makeIk(), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0] });
    expect(tl).toEqual([]);
  });

  it('tracks the pen state through penUp/penDown commands', () => {
    // q stays inside the servo band on channel 0 (max ≈ 1.48 rad before
    // clamping): 100→q=1.0, 120→q=1.2, 130→q=1.3.
    const cmds: MotionCommandJS[] = [
      { type: 'move', target: [100, 0, 0], speed: 50 },
      { type: 'penDown' },
      { type: 'move', target: [120, 0, 0], speed: 50 },
      { type: 'penUp' },
      { type: 'move', target: [130, 0, 0], speed: 50 },
    ];
    const tl = planTimeline(cmds, { ik: makeIk(), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0] });
    expect(tl.length).toBeGreaterThan(0);
    // Before the first penDown: pen is up. Channel 0 direction is −1, so
    // LARGER q → SMALLER µs.
    expect(tl[0].penDown).toBe(false);
    // After penDown: pen is down for the second move (q ∈ (1.0, 1.2)).
    const firstDown = tl.find((s) => s.q_us[0] < qUs([1, 0, 0, 0, 0])[0]);
    expect(firstDown?.penDown).toBe(true);
    // After penUp: pen is up again on the final move (q > 1.2).
    const lastUp = tl.find((s) => s.q_us[0] < qUs([1.2, 0, 0, 0, 0])[0]);
    expect(lastUp?.penDown).toBe(false);
  });
});

describe('planTimeline — playback mirror', () => {
  it('skips the IK solve when the target moved less than 0.5 mm', () => {
    const calls: [number, number, number][] = [];
    const cmds: MotionCommandJS[] = [{ type: 'move', target: [1, 0, 0], speed: 5 }];
    // Grilla explícita de 50 ms: el comportamiento de skip (< 0.5 mm) se prueba
    // sobre esta cadencia, independiente del default (40 ms).
    const tl = planTimeline(cmds, { ik: makeIk(calls), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0], dt: 0.05 });
    expect(calls).toHaveLength(2);
    expect(calls[0]).toEqual([0, 0, 0]);
    expect(calls[1][0]).toBeCloseTo(0.75, 12);
    expect(tl).toHaveLength(2);
    expect(tl[0].count).toBe(3);
    expect(tl[1].t).toBeCloseTo(0.15, 12);
    expect(tl[1].count).toBe(2);
    expect(tl[1].t + (tl[1].count - 1) * 0.05).toBe(0.2);
  });

  it('holds the previous q when the IK solve fails', () => {
    const cmds: MotionCommandJS[] = [{ type: 'move', target: [100, 0, 0], speed: 100 }];
    const failedUs = qUs([0.05, 0, 0, 0, 0])[0];
    // Grilla explícita de 50 ms: el fallo IK está amarrado a x=5 (primer push).
    const tl = planTimeline(cmds, { ik: makeIk([], 5), robot, startQ: START_Q, gripperPct: 50, startTcp: [0, 0, 0], dt: 0.05 });
    expect(tl.some((s) => s.q_us[0] === failedUs)).toBe(false);
    const held = tl[0];
    expect(held.q_us[0]).toBe(qUs([0, 0, 0, 0, 0])[0]);
    expect(held.count).toBe(2);
    const resumed = tl.find((s) => s.t === 0.1)!;
    expect(resumed.q_us[0]).toBe(qUs([0.1, 0, 0, 0, 0])[0]);
  });
});
