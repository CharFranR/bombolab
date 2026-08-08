import { DEFAULT_PLAN_DT, type PlanSample } from './planTimeline';
import type { TraceSample } from './trace';

export interface SendTimeErrorStats {
  mean: number;
  maxAbs: number;
  sigma: number;
}

export interface GapStats {
  min: number;
  mean: number;
  max: number;
  sigma: number;
  count: number;
  over1s: number;
}

export interface JointDeviation {
  mean: number;
  max: number;
}

export interface DeviationStats {
  perJoint: JointDeviation[];
  globalMax: number;
  globalMean: number;
}

export interface TraceMetrics {
  sendTimeError: SendTimeErrorStats;
  gaps: GapStats;
  deviation: DeviationStats;
  duration: { plan: number; real: number };
  framesWritten: number;
  truncated: boolean;
}

export interface CompareOptions {
  planDt?: number;
}

const MS_PER_S = 1000;

function meanOf(values: number[]): number {
  if (values.length === 0) return 0;
  return values.reduce((a, b) => a + b, 0) / values.length;
}

function sigmaOf(values: number[]): number {
  if (values.length < 2) return 0;
  const m = meanOf(values);
  const variance = values.reduce((acc, v) => acc + (v - m) * (v - m), 0) / (values.length - 1);
  return Math.sqrt(variance);
}

function emptyMetrics(): TraceMetrics {
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

export function compare(plan: PlanSample[], trace: TraceSample[], opts?: CompareOptions): TraceMetrics {
  if (plan.length === 0 || trace.length === 0) return emptyMetrics();
  const planDt = opts?.planDt ?? DEFAULT_PLAN_DT;
  const paired = Math.min(plan.length, trace.length);
  const errors: number[] = [];
  const deltas: number[][] = Array.from({ length: 6 }, () => []);
  for (let k = 0; k < paired; k++) {
    errors.push(trace[k].ts_ms - plan[k].t * MS_PER_S);
    for (let j = 0; j < 6; j++) {
      deltas[j].push(Math.abs(trace[k].q_us[j] - plan[k].q_us[j]));
    }
  }
  const gaps: number[] = [];
  for (let k = 0; k < trace.length - 1; k++) {
    if (trace[k].count > 1 || trace[k + 1].count > 1) continue;
    gaps.push(trace[k + 1].ts_ms - trace[k].ts_ms);
  }
  const allDeltas = deltas.flat();
  const lastPlan = plan[plan.length - 1];
  const lastTrace = trace[trace.length - 1];
  return {
    sendTimeError: {
      mean: meanOf(errors),
      maxAbs: errors.reduce((m, v) => Math.max(m, Math.abs(v)), 0),
      sigma: sigmaOf(errors),
    },
    gaps: {
      min: gaps.length > 0 ? Math.min(...gaps) : 0,
      mean: meanOf(gaps),
      max: gaps.length > 0 ? Math.max(...gaps) : 0,
      sigma: sigmaOf(gaps),
      count: gaps.length,
      over1s: gaps.filter((g) => g > 1000).length,
    },
    deviation: {
      perJoint: deltas.map((d) => ({
        mean: meanOf(d),
        max: d.length > 0 ? Math.max(...d) : 0,
      })),
      globalMax: allDeltas.length > 0 ? Math.max(...allDeltas) : 0,
      globalMean: meanOf(allDeltas),
    },
    duration: {
      plan: (lastPlan.t + (lastPlan.count - 1) * planDt) * MS_PER_S,
      real: lastTrace.ts_ms,
    },
    framesWritten: trace.reduce((acc, s) => acc + s.count, 0),
    truncated: false,
  };
}
