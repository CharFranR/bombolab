import { beforeAll, describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import init from './pkg/bombolab_wasm';
import { fabriCreator, forwardKinematics, robotToWasm, toRobotDef, type WasmRobotDef } from './wasm';
import { DEFAULT_TOOL_TRANSFORM, type Mat4 } from './kinematics/types';
import { mulMat4 } from './renderers/types';

const ROUND_TRIP_EPSILON = 1e-12;
const FK_EPSILON = 1e-9;

beforeAll(async () => {
  await init(readFileSync(new URL('./pkg/bombolab_wasm_bg.wasm', import.meta.url)));
});

function expectMat4Close(actual: Mat4, expected: Mat4, eps: number): void {
  for (let i = 0; i < 16; i++) {
    expect(Math.abs(actual[i] - expected[i])).toBeLessThanOrEqual(eps);
  }
}

function wireRobot(toolTransform: number[]): WasmRobotDef {
  return {
    segments: [],
    base_transform: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
    tool_transform: toolTransform,
  };
}

function makeSegs(): ReturnType<typeof fabriCreator>['segments'] {
  const robot = fabriCreator();
  return robot.segments.map((s, i) => ({ ...s, q: 0.1 * (i + 1) }));
}

function rotatedTool(deg: number): { mat: Mat4; wire: number[] } {
  const c = Math.cos(deg * Math.PI / 180);
  const s = Math.sin(deg * Math.PI / 180);
  return {
    mat: [
      c, -s, 0, 75,
      s, c, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ],
    wire: [
      c, -s, 0, 75,
      s, c, 0, 0,
      0, 0, 1, 0,
    ],
  };
}

describe('toRobotDef / robotToWasm — full-pose round-trip', () => {
  it('round-trips a non-identity tool rotation bit-exactly (wire stays 12 floats)', () => {
    const { wire } = rotatedTool(30);
    const robot = toRobotDef(wireRobot(wire));
    const back = robotToWasm(robot).tool_transform;
    expect(back).toHaveLength(12);
    for (let i = 0; i < 12; i++) {
      expect(Math.abs(back[i] - wire[i])).toBeLessThanOrEqual(ROUND_TRIP_EPSILON);
    }
  });

  it('round-trips the identity tool pose within epsilon', () => {
    const robot = toRobotDef(wireRobot([1, 0, 0, 75, 0, 1, 0, 0, 0, 0, 1, 0]));
    const back = robotToWasm(robot).tool_transform;
    for (let i = 0; i < 12; i++) {
      expect(Math.abs(back[i] - [1, 0, 0, 75, 0, 1, 0, 0, 0, 0, 1, 0][i])).toBeLessThanOrEqual(ROUND_TRIP_EPSILON);
    }
  });

  it('carries the full rotation into the Mat4 (translation + rotation survive)', () => {
    const { mat } = rotatedTool(30);
    const robot = toRobotDef(wireRobot(mat.slice(0, 12) as number[]));
    expectMat4Close(robot.toolTransform, mat, ROUND_TRIP_EPSILON);
  });
});

describe('fabriCreator — default marker payload', () => {
  it('exposes DEFAULT_TOOL_TRANSFORM as its toolTransform', () => {
    expectMat4Close(fabriCreator().toolTransform, DEFAULT_TOOL_TRANSFORM, ROUND_TRIP_EPSILON);
  });

  it('keeps the wire payload identical to the legacy 12-float tool', () => {
    expect(robotToWasm(fabriCreator()).tool_transform).toEqual([1, 0, 0, 117, 0, 1, 0, 0, 0, 0, 1, 0]);
  });
});

describe('forwardKinematics — tool param as single source of truth', () => {
  it('renderer FK (frames.last() * tool) equals wasm FK ee for a rotated tool', () => {
    const segs = makeSegs();
    const { mat } = rotatedTool(30);
    const fk = forwardKinematics(segs, fabriCreator().baseTransform, mat);
    const last = fk.frames[fk.frames.length - 1];
    expectMat4Close(fk.ee, mulMat4(last, mat), FK_EPSILON);
  });

  it('wasm FK ee honors a non-default tool instead of the hardcoded marker', () => {
    const segs = makeSegs();
    const tool: Mat4 = [
      0, -1, 0, 30,
      1, 0, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ];
    const fk = forwardKinematics(segs, fabriCreator().baseTransform, tool);
    const last = fk.frames[fk.frames.length - 1];
    expectMat4Close(fk.ee, mulMat4(last, tool), FK_EPSILON);
  });

  it('defaults to the marker tool when omitted (legacy callers)', () => {
    const segs = makeSegs();
    const fk = forwardKinematics(segs, fabriCreator().baseTransform);
    const last = fk.frames[fk.frames.length - 1];
    expectMat4Close(fk.ee, mulMat4(last, DEFAULT_TOOL_TRANSFORM), FK_EPSILON);
  });
});
