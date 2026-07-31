import init, { fabri_creator as wasmFabriCreator, forward_kinematics as wasmFk, solve_ik as wasmSolveIk, solve_drawing_ik as wasmSolveDrawingIk, solve_drawing_ik_v2 as wasmSolveDrawingIkV2 } from './pkg/bombolab_wasm';
import type { RobotDef, Segment, Mat4 } from './kinematics/types';

let initialized = false;

export async function initWasm(): Promise<void> {
  if (initialized) return;
  console.log('[wasm] initializing...');
  await init();
  initialized = true;
  console.log('[wasm] ready');
}

interface WasmSegment {
  q: number;
  theta: number;
  d: number;
  a: number;
  alpha: number;
  q_min: number;
  q_max: number;
  joint_type: string;
}

interface WasmRobotDef {
  segments: WasmSegment[];
  base_transform: number[];
  tool_transform: number[];
}

interface WasmFkResult {
  frames: number[][];
  ee: number[];
}

interface WasmIkResult {
  q: number[];
  converged: boolean;
  error: number;
}

function toRobotDef(wasm: WasmRobotDef): RobotDef {
  return {
    name: 'FABRI Creator',
    segments: wasm.segments.map((s) => ({
      q: s.q,
      theta: s.theta,
      d: s.d,
      a: s.a,
      alpha: s.alpha,
      q_min: s.q_min,
      q_max: s.q_max,
      joint_type: s.joint_type,
    })),
    baseTransform: [wasm.base_transform[3], wasm.base_transform[7], wasm.base_transform[11]],
    toolTransform: [wasm.tool_transform[3], wasm.tool_transform[7], wasm.tool_transform[11]],
  };
}

function robotToWasm(robot: RobotDef): WasmRobotDef {
  return {
    segments: robot.segments.map((s) => ({
      q: s.q,
      theta: s.theta,
      d: s.d,
      a: s.a,
      alpha: s.alpha,
      q_min: s.q_min ?? -80 * Math.PI / 180,
      q_max: s.q_max ?? 80 * Math.PI / 180,
      joint_type: s.joint_type ?? 'revolute',
    })),
    base_transform: [
      1, 0, 0, robot.baseTransform[0],
      0, 1, 0, robot.baseTransform[1],
      0, 0, 1, robot.baseTransform[2],
    ],
    tool_transform: [
      1, 0, 0, robot.toolTransform[0],
      0, 1, 0, robot.toolTransform[1],
      0, 0, 1, robot.toolTransform[2],
    ],
  };
}

export function fabriCreator(): RobotDef {
  const wasm = wasmFabriCreator() as unknown as WasmRobotDef;
  return toRobotDef(wasm);
}

export function forwardKinematics(segments: Segment[], base: [number, number, number]): { frames: Mat4[]; ee: Mat4 } {
  const robot: RobotDef = {
    name: 'FABRI Creator',
    segments,
    baseTransform: base,
    toolTransform: [75, 0, 0],
  };
  const wasmRobot = robotToWasm(robot);
  const result = wasmFk(wasmRobot) as unknown as WasmFkResult;

  const toMat4 = (arr: number[]): Mat4 => [
    arr[0], arr[1], arr[2], arr[3],
    arr[4], arr[5], arr[6], arr[7],
    arr[8], arr[9], arr[10], arr[11],
    0, 0, 0, 1,
  ];

  // Contract: result.frames[0] is base·T₁ — the first JOINT frame, base
  // already applied. The renderer expects frames[0] to be the WORLD frame
  // (z=0, the ground where the base sits) so the fixed base parts
  // (parentJoint 0) hang from the world, NOT from base_transform().
  // Prepending the identity gives: [world, base·T₁, base·T₁T₂, ...].
  // (Never prepend baseMat here: base is already inside every Rust frame;
  //  doing so creates a phantom frame at (0,0,57) that calibration then
  //  has to absorb — base parts end up buried ~26mm under the ground.)
  const worldMat: Mat4 = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];

  return {
    frames: [worldMat, ...result.frames.map(toMat4)],
    ee: toMat4(result.ee),
  };
}

export function solveIk(
  robot: RobotDef,
  target: [number, number, number],
  qInit: number[],
): { q: number[]; converged: boolean; error: number } {
  const wasmRobot = robotToWasm(robot);
  const result = wasmSolveIk(wasmRobot, new Float64Array(target), new Float64Array(qInit)) as unknown as WasmIkResult;
  return result;
}

export function solveDrawingIk(
  robot: RobotDef,
  target: [number, number, number],
  qInit: number[],
): { q: number[]; converged: boolean; error: number } {
  const wasmRobot = robotToWasm(robot);
  const result = wasmSolveDrawingIk(wasmRobot, new Float64Array(target), new Float64Array(qInit)) as unknown as WasmIkResult;
  return result;
}

export function solveDrawingIkV2(
  robot: RobotDef,
  target: [number, number, number],
  qInit: number[],
): { q: number[]; converged: boolean; error: number } {
  const wasmRobot = robotToWasm(robot);
  const result = wasmSolveDrawingIkV2(wasmRobot, new Float64Array(target), new Float64Array(qInit)) as unknown as WasmIkResult;
  return result;
}
