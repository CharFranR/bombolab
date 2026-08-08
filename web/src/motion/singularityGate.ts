import { analyzePathSingularity, initWasm, type SingularityReportJS, type SingularityThresholdsJS, type SingularityWaypointJS } from '../wasm';
import type { RobotDef } from '../kinematics/types';
import type { MotionCommandJS } from './commands';

export interface SingularityBlockInfo {
  reason: string;
  points: [number, number, number][];
  canRefit: boolean;
}

export type SingularityGateDecision =
  | { action: 'ok' }
  | { action: 'block'; block: SingularityBlockInfo }
  | { action: 'warn'; warning: string };

export interface SingularityGateOptions {
  canRefit: boolean;
  thresholds?: SingularityThresholdsJS;
}

export interface SingularityGateDeps {
  canRefit: boolean;
  confirmFn: (message: string) => boolean;
  setDrawingBlock: (block: SingularityBlockInfo) => void;
  setGcodeWarnings: (updater: (prev: string[]) => string[]) => void;
}

function blockReason(wp: SingularityWaypointJS): string {
  if (wp.reason === 'ik_non_convergence') {
    return `El punto #${wp.index} no pudo resolverse: el IK no converge (posible singularidad o límite articular).`;
  }
  return `El punto #${wp.index} está en una singularidad (σ_min=${wp.sigma_min.toFixed(1)} mm).`;
}

function warnMessage(wp: SingularityWaypointJS): string {
  return `El punto #${wp.index} pasa cerca de una singularidad (σ_min=${wp.sigma_min.toFixed(1)} mm, κ=${wp.kappa.toFixed(1)}).`;
}

export function resolveSingularityGate(report: SingularityReportJS, canRefit: boolean): SingularityGateDecision {
  const blocker = report.worst.find((wp) => wp.level === 'block');
  if (blocker) {
    return {
      action: 'block',
      block: { reason: blockReason(blocker), points: [blocker.target], canRefit },
    };
  }
  const warmer = report.worst.find((wp) => wp.level === 'warn');
  if (warmer) {
    return { action: 'warn', warning: warnMessage(warmer) };
  }
  return { action: 'ok' };
}

export async function checkSingularityGate(
  robot: RobotDef,
  commands: MotionCommandJS[],
  opts: SingularityGateOptions,
): Promise<SingularityGateDecision> {
  await initWasm();
  let report: SingularityReportJS;
  try {
    report = analyzePathSingularity(robot, commands, opts.thresholds);
  } catch (e: any) {
    return {
      action: 'block',
      block: {
        reason: 'No se pudo analizar la trayectoria: ' + (e?.message ?? String(e)),
        points: [],
        canRefit: opts.canRefit,
      },
    };
  }
  return resolveSingularityGate(report, opts.canRefit);
}

export function applySingularityDecision(decision: SingularityGateDecision, deps: SingularityGateDeps): boolean {
  if (decision.action === 'block') {
    deps.setDrawingBlock(decision.block);
    return false;
  }
  if (decision.action === 'warn') {
    deps.setGcodeWarnings((prev) => [...prev, decision.warning]);
    return deps.confirmFn(decision.warning);
  }
  return true;
}

export async function runSingularityGate(
  robot: RobotDef,
  commands: MotionCommandJS[],
  deps: SingularityGateDeps,
  thresholds?: SingularityThresholdsJS,
): Promise<boolean> {
  const decision = await checkSingularityGate(robot, commands, { canRefit: deps.canRefit, thresholds });
  return applySingularityDecision(decision, deps);
}
