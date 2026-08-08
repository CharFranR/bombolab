import { beforeAll, describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import init from '../pkg/bombolab_wasm';
import { analyzePathSingularity, fabriCreator, type SingularityThresholdsJS, type SingularityReportJS, type SingularityWaypointJS } from '../wasm';
import { squareCommands, type MotionCommandJS } from './commands';
import type { RobotDef } from '../kinematics/types';
import { DEFAULT_TOOL_TRANSFORM } from '../kinematics/types';
import { applySingularityDecision, checkSingularityGate, resolveSingularityGate, runSingularityGate, type SingularityGateDeps } from './singularityGate';

beforeAll(async () => {
  await init(readFileSync(new URL('../pkg/bombolab_wasm_bg.wasm', import.meta.url)));
});

function robotWithToolX(offsetX: number): RobotDef {
  return {
    ...fabriCreator(),
    toolTransform: [
      ...DEFAULT_TOOL_TRANSFORM.slice(0, 3),
      offsetX,
      ...DEFAULT_TOOL_TRANSFORM.slice(4),
    ] as RobotDef['toolTransform'],
  };
}

function waypoint(overrides: Partial<SingularityWaypointJS> = {}): SingularityWaypointJS {
  return {
    index: 3,
    target: [85, -15, 142],
    q: [0, 1, 1, 0, 0],
    sigma_min: 2.5,
    kappa: 40,
    yoshikawa: 0.001,
    level: 'ok',
    reason: 'metrics',
    ...overrides,
  };
}

const FOLD_CMDS: MotionCommandJS[] = [{ type: 'move', target: [85, -15, 142], speed: 40 }];
const WARN_THRESHOLDS: SingularityThresholdsJS = { warn_sigma_min: 100, warn_kappa: 1000, block_sigma_min: 5, block_kappa: 100 };

describe('analyzePathSingularity — payload contract', () => {
  it('decodes a healthy path as sampled>0 with every waypoint classified ok', () => {
    const report = analyzePathSingularity(fabriCreator(), squareCommands());
    expect(report.sampled).toBeGreaterThan(0);
    expect(report.worst.length).toBeGreaterThan(0);
    const worst = report.worst[0];
    expect(worst.index).toBeGreaterThanOrEqual(0);
    expect(worst.target).toHaveLength(3);
    expect(worst.q).toHaveLength(5);
    expect(worst.sigma_min).toBeGreaterThan(0);
    expect(worst.kappa).toBeGreaterThan(0);
    expect(worst.level).toBe('ok');
    expect(worst.reason).toBe('metrics');
  });

  it('flags an exact-singular waypoint as block with ik_non_convergence', () => {
    const foldCmds: MotionCommandJS[] = [
      { type: 'move', target: [85, -15, 142], speed: 40 },
    ];
    const report = analyzePathSingularity(fabriCreator(), foldCmds);
    expect(report.sampled).toBeGreaterThan(0);
    expect(report.worst.some((w) => w.level === 'block')).toBe(true);
    const blocker = report.worst.find((w) => w.level === 'block')!;
    expect(blocker.reason).toBe('ik_non_convergence');
  });

  it('returns an empty worst list for an empty command list', () => {
    const report = analyzePathSingularity(fabriCreator(), []);
    expect(report.sampled).toBe(0);
    expect(report.worst).toHaveLength(0);
  });

  it('reflects a custom tool offset in the waypoint metrics', () => {
    const defaultReport = analyzePathSingularity(fabriCreator(), squareCommands());
    const customReport = analyzePathSingularity(robotWithToolX(30), squareCommands());
    expect(customReport.sampled).toBe(defaultReport.sampled);
    expect(
      Math.abs(customReport.worst[0].sigma_min - defaultReport.worst[0].sigma_min),
    ).toBeGreaterThan(1e-6);
  });

  it('honors custom thresholds instead of the calibrated defaults', () => {
    const aggressive = { warn_sigma_min: 100, warn_kappa: 1000, block_sigma_min: 5, block_kappa: 100 };
    const report = analyzePathSingularity(fabriCreator(), squareCommands(), aggressive);
    expect(report.worst[0].level).toBe('warn');
    const blockAll = { warn_sigma_min: 10000, warn_kappa: 10000, block_sigma_min: 5000, block_kappa: 5000 };
    const blockReport = analyzePathSingularity(fabriCreator(), squareCommands(), blockAll);
    expect(blockReport.worst[0].level).toBe('block');
  });
});

describe('resolveSingularityGate — warn/block/ok classification', () => {
  it('blocks on the worst exact-singular waypoint and propagates canRefit', () => {
    const report: SingularityReportJS = {
      sampled: 10,
      worst: [waypoint({ level: 'block', reason: 'metrics', sigma_min: 0.01 })],
    };
    const decision = resolveSingularityGate(report, true);
    expect(decision.action).toBe('block');
    if (decision.action !== 'block') return;
    expect(decision.block.points).toEqual([[85, -15, 142]]);
    expect(decision.block.canRefit).toBe(true);
    expect(decision.block.reason).toContain('singularidad');
  });

  it('distinguishes the IK non-convergence block reason', () => {
    const report: SingularityReportJS = {
      sampled: 10,
      worst: [waypoint({ level: 'block', reason: 'ik_non_convergence' })],
    };
    const decision = resolveSingularityGate(report, false);
    expect(decision.action).toBe('block');
    if (decision.action !== 'block') return;
    expect(decision.block.reason).toContain('IK');
    expect(decision.block.canRefit).toBe(false);
  });

  it('warns on near-singular waypoints with a human-readable message', () => {
    const report: SingularityReportJS = {
      sampled: 10,
      worst: [waypoint({ level: 'warn', sigma_min: 22.34, kappa: 21.5, index: 7 })],
    };
    const decision = resolveSingularityGate(report, false);
    expect(decision.action).toBe('warn');
    if (decision.action !== 'warn') return;
    expect(decision.warning).toContain('#7');
    expect(decision.warning).toContain('singularidad');
    expect(decision.warning).toContain('22.3');
  });

  it('lets block outrank warn when both are present', () => {
    const report: SingularityReportJS = {
      sampled: 10,
      worst: [waypoint({ level: 'warn' }), waypoint({ index: 9, level: 'block' })],
    };
    const decision = resolveSingularityGate(report, false);
    expect(decision.action).toBe('block');
  });

  it('stays silent on a healthy path', () => {
    const report: SingularityReportJS = {
      sampled: 40,
      worst: [waypoint({ level: 'ok', sigma_min: 82 })],
    };
    expect(resolveSingularityGate(report, false).action).toBe('ok');
  });
});

describe('checkSingularityGate — wasm-backed pre-flight', () => {
  it('blocks when the warm-started IK cannot resolve a waypoint', async () => {
    const decision = await checkSingularityGate(fabriCreator(), FOLD_CMDS, { canRefit: true });
    expect(decision.action).toBe('block');
    if (decision.action !== 'block') return;
    expect(decision.block.reason).toContain('IK');
    expect(decision.block.points).toEqual([[85, -15, 142]]);
  });

  it('passes a healthy path silently and returns an ok decision', async () => {
    const decision = await checkSingularityGate(fabriCreator(), squareCommands(), { canRefit: false });
    expect(decision.action).toBe('ok');
  });

  it('passes an empty command list as ok', async () => {
    const decision = await checkSingularityGate(fabriCreator(), [], { canRefit: false });
    expect(decision.action).toBe('ok');
  });

  it('warns when custom thresholds flag the same healthy path', async () => {
    const decision = await checkSingularityGate(fabriCreator(), squareCommands(), {
      canRefit: false,
      thresholds: WARN_THRESHOLDS,
    });
    expect(decision.action).toBe('warn');
  });

  it('does not mutate the command list (draw-anyway leaves the trajectory intact)', async () => {
    const cmds = squareCommands();
    const snapshot = JSON.stringify(cmds);
    await checkSingularityGate(fabriCreator(), cmds, { canRefit: false });
    expect(JSON.stringify(cmds)).toBe(snapshot);
  });
});

describe('applySingularityDecision — UX side effects of a decision', () => {
  function deps(overrides: Partial<SingularityGateDeps> = {}): SingularityGateDeps {
    return {
      canRefit: false,
      confirmFn: () => true,
      setDrawingBlock: () => {},
      setGcodeWarnings: () => {},
      ...overrides,
    };
  }

  it('renders drawingBlock and refuses playback on a block decision', () => {
    let blocked: unknown = null;
    const decision = resolveSingularityGate(
      { sampled: 1, worst: [waypoint({ level: 'block' })] },
      true,
    );
    const proceed = applySingularityDecision(decision, deps({ setDrawingBlock: (b) => { blocked = b; } }));
    expect(proceed).toBe(false);
    expect(blocked).toMatchObject({ canRefit: true, points: [[85, -15, 142]] });
  });

  it('appends the gate warning to existing gcode warnings and proceeds on confirm', () => {
    let warned: string[] = ['parse warning'];
    const decision = resolveSingularityGate(
      { sampled: 1, worst: [waypoint({ level: 'warn' })] },
      false,
    );
    let confirmedMessage = '';
    const proceed = applySingularityDecision(decision, deps({
      setGcodeWarnings: (updater) => { warned = updater(warned); },
      confirmFn: (message) => { confirmedMessage = message; return true; },
    }));
    expect(proceed).toBe(true);
    expect(warned).toHaveLength(2);
    expect(warned[0]).toBe('parse warning');
    expect(warned[1]).toContain('#3');
    expect(warned[1]).toContain('singularidad');
    expect(confirmedMessage).toContain('singularidad');
  });

  it('refuses playback when the user declines the draw-anyway confirm', () => {
    const decision = resolveSingularityGate(
      { sampled: 1, worst: [waypoint({ level: 'warn' })] },
      false,
    );
    const proceed = applySingularityDecision(decision, deps({ confirmFn: () => false }));
    expect(proceed).toBe(false);
  });

  it('is silent on ok decisions: no panels, no confirm', () => {
    let calls = 0;
    const proceed = applySingularityDecision(
      { action: 'ok' },
      deps({
        setDrawingBlock: () => { calls++; },
        setGcodeWarnings: () => { calls++; },
        confirmFn: () => { calls++; return true; },
      }),
    );
    expect(proceed).toBe(true);
    expect(calls).toBe(0);
  });
});

describe('runSingularityGate — startTrajectory pre-flight wiring', () => {
  function deps(overrides: Partial<SingularityGateDeps> = {}): SingularityGateDeps {
    return {
      canRefit: false,
      confirmFn: () => true,
      setDrawingBlock: () => {},
      setGcodeWarnings: () => {},
      ...overrides,
    };
  }

  it('blocks playback on a singular path', async () => {
    let blocked: unknown = null;
    const proceed = await runSingularityGate(fabriCreator(), FOLD_CMDS, deps({
      setDrawingBlock: (b) => { blocked = b; },
    }));
    expect(proceed).toBe(false);
    expect(blocked).not.toBeNull();
  });

  it('starts playback silently on a healthy path', async () => {
    let calls = 0;
    const proceed = await runSingularityGate(fabriCreator(), squareCommands(), deps({
      setDrawingBlock: () => { calls++; },
      setGcodeWarnings: () => { calls++; },
      confirmFn: () => { calls++; return true; },
    }));
    expect(proceed).toBe(true);
    expect(calls).toBe(0);
  });

  it('warns, confirms, and proceeds with the unchanged command list (MP-1)', async () => {
    const cmds = squareCommands();
    const snapshot = JSON.stringify(cmds);
    const proceed = await runSingularityGate(fabriCreator(), cmds, deps({
      canRefit: true,
      confirmFn: () => true,
    }), WARN_THRESHOLDS);
    expect(proceed).toBe(true);
    expect(JSON.stringify(cmds)).toBe(snapshot);
  });
});
