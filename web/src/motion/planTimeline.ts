import { gripperToServoUs, qToServoUs } from '../serial';
import type { RobotDef } from '../kinematics/types';
import type { MotionCommandJS } from './commands';

export interface IkFn {
  (robot: RobotDef, target: [number, number, number], qInit: number[]): {
    q: number[];
    converged: boolean;
    error: number;
  };
}

export interface PlanSample {
  t: number;
  q_us: number[];
  count: number;
  /** True while the pen is down (drawing on the plane). Set by planTimeline;
   *  optional so hand-built fixtures / legacy consumers stay valid. Used by
   *  the diagnostic overlay to color drawing vs travel samples. */
  penDown?: boolean;
}

export interface PlanOptions {
  ik: IkFn;
  robot: RobotDef;
  startQ: number[];
  gripperPct: number;
  dt?: number;
  startTcp: [number, number, number];
  /** Apply a centered moving-average filter (window 3) over the raw q_us
   *  values after IK resolution.  Smooths out IK solution jitter that
   *  manifests as visible tremor in the physical drawing.
   *  Default: false (caller opts in). */
  smooth?: boolean;
}

// Plan sampling interval (s): 40 ms step cadence for drawing playback
// (unified with the legacy step delay; speed is set by the gcode F).
export const DEFAULT_PLAN_DT = 0.04;

const MIN_PUSH_MM = 0.5;
const EPS = 1e-6;

/**
 * Centered moving-average filter (window 3) applied independently to each
 * servo channel.  Because the whole trajectory is planned upfront we can
 * look ahead, so this filter adds zero phase lag — the smoothed path
 * follows the same timeline as the raw one.
 *
 * The first and last samples are left unchanged so that trajectory
 * endpoints stay exactly on-target.
 */
function smoothSamples(samples: PlanSample[]): PlanSample[] {
  if (samples.length <= 2) return samples;
  const channels = samples[0].q_us.length;
  const out = samples.map((s) => ({ ...s, q_us: [...s.q_us] }));

  for (let ch = 0; ch < channels; ch++) {
    for (let i = 1; i < out.length - 1; i++) {
      const prev = out[i - 1].q_us[ch];
      const curr = out[i].q_us[ch];
      const next = out[i + 1].q_us[ch];
      out[i].q_us[ch] = (prev + curr + next) / 3;
    }
  }
  return out;
}

function dist3(a: [number, number, number], b: [number, number, number]): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function sameQus(a: number[], b: number[]): boolean {
  for (let i = 0; i < a.length; i++) {
    if (a[i] !== b[i]) return false;
  }
  return true;
}

export function planTimeline(cmds: MotionCommandJS[], opts: PlanOptions): PlanSample[] {
  const { ik, robot, startQ, gripperPct } = opts;
  const dt = opts.dt ?? DEFAULT_PLAN_DT;
  if (cmds.length === 0) return [];
  const samples: PlanSample[] = [];
  let tcp: [number, number, number] = [...opts.startTcp];
  let lastQ: number[] | null = null;
  let lastPushed: [number, number, number] | null = null;
  let penDown = false;

  const emit = (time: number, qq: number[]): void => {
    const qus = [...qToServoUs(qq), gripperToServoUs(gripperPct)];
    const last = samples[samples.length - 1];
    if (last !== undefined && sameQus(last.q_us, qus)) {
      last.count += 1;
      return;
    }
    samples.push({ t: time, q_us: qus, count: 1, penDown });
  };

  const push = (time: number, target: [number, number, number]): void => {
    if (lastPushed !== null && dist3(lastPushed, target) <= MIN_PUSH_MM) {
      emit(time, lastQ ?? startQ);
      return;
    }
    lastPushed = [...target];
    const result = ik(robot, target, lastQ ?? startQ);
    if (result.converged && result.error < 10) {
      lastQ = result.q;
      emit(time, result.q);
    } else {
      emit(time, lastQ ?? startQ);
    }
  };

  push(0, tcp);
  let t = 0;
  for (const cmd of cmds) {
    if (cmd.type === 'penUp') {
      penDown = false;
      continue;
    }
    if (cmd.type === 'penDown') {
      penDown = true;
      continue;
    }
    if (cmd.type === 'wait') {
      const waitEnd = t + cmd.duration;
      let k = 1;
      while (t + k * dt < waitEnd - 1e-9) {
        emit(t + k * dt, lastQ ?? startQ);
        k += 1;
      }
      emit(waitEnd, lastQ ?? startQ);
      t = waitEnd;
      continue;
    }
    const segEnd: [number, number, number] = [...cmd.target];
    const dist = dist3(tcp, segEnd);
    if (dist <= EPS || cmd.speed <= 0) {
      tcp = segEnd;
      continue;
    }
    const segT0 = t;
    const dur = dist / cmd.speed;
    const moveEnd = segT0 + dur;
    let k = 1;
    while (segT0 + k * dt < moveEnd - 1e-9) {
      const frac = (k * dt) / dur;
      const pos: [number, number, number] = [
        tcp[0] + (segEnd[0] - tcp[0]) * frac,
        tcp[1] + (segEnd[1] - tcp[1]) * frac,
        tcp[2] + (segEnd[2] - tcp[2]) * frac,
      ];
      push(segT0 + k * dt, pos);
      k += 1;
    }
    push(moveEnd, segEnd);
    t = moveEnd;
    tcp = segEnd;
  }

  return opts.smooth ? smoothSamples(samples) : samples;
}
