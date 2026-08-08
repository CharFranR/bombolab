import { describe, expect, it } from 'vitest';
import { renderToStaticMarkup } from 'react-dom/server';
import { compare } from '../motion/comparator';
import { exportTraceCsv } from '../motion/csv';
import type { PlanSample } from '../motion/planTimeline';
import type { TraceResult } from '../motion/trace';
import PlanExecPanel, {
  deviationSeries,
  gapStatsOf,
  gapsOf,
  importTraceText,
  sendTimeErrors,
  summaryOf,
} from './PlanExecPanel';

const TRACE: TraceResult = {
  samples: [
    { ts_ms: 0, q_us: [544, 1472, 1379, 1523, 1162, 1678], count: 1 },
    { ts_ms: 50.1234567890123, q_us: [1472, 1472, 1379, 1523, 1162, 1678], count: 2 },
    { ts_ms: 105.5, q_us: [1473, 1472, 1379, 1523, 1162, 1678], count: 1 },
  ],
  t0: 0,
  truncated: false,
  framesWritten: 4,
  dedupe: 1,
};

const PLAN: PlanSample[] = [
  { t: 0, q_us: [540, 1472, 1379, 1523, 1162, 1678], count: 1 },
  { t: 0.05, q_us: [1470, 1472, 1379, 1523, 1162, 1678], count: 1 },
  { t: 0.1, q_us: [1475, 1472, 1379, 1523, 1162, 1678], count: 1 },
];

const GAP_TRACE: TraceResult = {
  samples: [
    { ts_ms: 0, q_us: [544, 1472, 1379, 1523, 1162, 1678], count: 1 },
    { ts_ms: 100, q_us: [1472, 1472, 1379, 1523, 1162, 1678], count: 1 },
    { ts_ms: 105, q_us: [1473, 1472, 1379, 1523, 1162, 1678], count: 2 },
    { ts_ms: 1105, q_us: [1474, 1472, 1379, 1523, 1162, 1678], count: 1 },
    { ts_ms: 1606, q_us: [1475, 1472, 1379, 1523, 1162, 1678], count: 1 },
    { ts_ms: 2700, q_us: [1476, 1472, 1379, 1523, 1162, 1678], count: 1 },
  ],
  t0: 0,
  truncated: false,
  framesWritten: 7,
  dedupe: 1,
};

const PLAN5: PlanSample[] = [
  { t: 0, q_us: [540, 1472, 1379, 1523, 1162, 1678], count: 1 },
  { t: 0.05, q_us: [1470, 1472, 1379, 1523, 1162, 1678], count: 1 },
  { t: 0.1, q_us: [1475, 1472, 1379, 1523, 1162, 1678], count: 1 },
  { t: 0.15, q_us: [1480, 1472, 1379, 1523, 1162, 1678], count: 1 },
  { t: 0.2, q_us: [1485, 1472, 1379, 1523, 1162, 1678], count: 1 },
];

describe('importTraceText — CS-4 CSV import simétrico', () => {
  it('round-trips an exported CSV into the exact trace (samples + derived fields)', () => {
    const result = importTraceText(exportTraceCsv(TRACE));
    expect(result).toEqual({ trace: TRACE });
  });

  it('surfaces malformed CSV as an error without any partial trace', () => {
    const bad = importTraceText('garbage');
    expect('error' in bad).toBe(true);
    expect('trace' in bad).toBe(false);
    const wrongArity = importTraceText('ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count\n0,544,1472\n');
    expect('error' in wrongArity).toBe(true);
    const ok = importTraceText(exportTraceCsv(TRACE));
    expect('trace' in ok).toBe(true);
  });
});

describe('summaryOf — métricas derivadas de la traza (sin plan)', () => {
  it('reproduces samples, framesWritten, dedupe and real duration', () => {
    const s = summaryOf(TRACE, null);
    expect(s.samples).toBe(3);
    expect(s.framesWritten).toBe(4);
    expect(s.dedupe).toBe(1);
    expect(s.durationRealMs).toBe(105.5);
    expect(s.truncated).toBe(false);
  });

  it('reports no compared metrics when there is no plan', () => {
    const s = summaryOf(TRACE, null);
    expect(s.sendTimeError).toBeNull();
    expect(s.deviation).toBeNull();
    expect(s.durationPlanMs).toBeNull();
  });

  it('derives gap stats from the trace alone, excluding heartbeat pairs', () => {
    const s = summaryOf(GAP_TRACE, null);
    expect(s.gaps.count).toBe(3);
    expect(s.gaps.min).toBe(100);
    expect(s.gaps.max).toBe(1094);
    expect(s.gaps.mean).toBeCloseTo(565, 6);
    expect(s.gaps.over1s).toBe(1);
    expect(s.gaps.sigma).toBeCloseTo(Math.sqrt(((100 - 565) ** 2 + (501 - 565) ** 2 + (1094 - 565) ** 2) / 2), 6);
  });
});

describe('summaryOf — métricas comparadas (con plan)', () => {
  it('matches the comparator exactly for send-time error, deviation and plan duration', () => {
    const s = summaryOf(TRACE, PLAN);
    const m = compare(PLAN, TRACE.samples);
    expect(s.sendTimeError).toEqual(m.sendTimeError);
    expect(s.deviation).toEqual(m.deviation);
    expect(s.durationPlanMs).toBe(m.duration.plan);
  });

  it('exposes concrete compared values from the fixture', () => {
    const s = summaryOf(TRACE, PLAN);
    expect(s.sendTimeError?.maxAbs).toBeCloseTo(5.5, 10);
    expect(s.deviation?.perJoint[0]).toEqual({ mean: 8 / 3, max: 4 });
    expect(s.deviation?.perJoint[1]).toEqual({ mean: 0, max: 0 });
    expect(s.durationPlanMs).toBeCloseTo(100, 10);
  });
});

describe('sendTimeErrors / deviationSeries — series para charts', () => {
  it('computes the send-time error per paired sample', () => {
    const errors = sendTimeErrors(PLAN, TRACE.samples);
    expect(errors).toHaveLength(3);
    expect(errors[0]).toBe(0);
    expect(errors[1]).toBeCloseTo(0.1234567890123, 10);
    expect(errors[2]).toBeCloseTo(5.5, 10);
    const m = compare(PLAN, TRACE.samples);
    expect(Math.max(...errors.map((v) => Math.abs(v)))).toBeCloseTo(m.sendTimeError.maxAbs, 10);
  });

  it('computes the per-joint commanded-vs-plan deviation series', () => {
    const series = deviationSeries(PLAN, TRACE.samples);
    expect(series).toHaveLength(6);
    expect(series[0]).toEqual([4, 2, 2]);
    expect(series[1]).toEqual([0, 0, 0]);
  });
});

describe('gapsOf / gapStatsOf — exclusión heartbeat', () => {
  it('excludes pairs adjacent to a count>1 sample and keeps throttle gaps raw', () => {
    expect(gapsOf(GAP_TRACE.samples)).toEqual([100, 501, 1094]);
    const m = compare(PLAN5, GAP_TRACE.samples);
    expect(gapStatsOf(gapsOf(GAP_TRACE.samples)).count).toBe(m.gaps.count);
    expect(gapStatsOf(gapsOf(GAP_TRACE.samples)).min).toBe(m.gaps.min);
    expect(gapStatsOf(gapsOf(GAP_TRACE.samples)).max).toBe(m.gaps.max);
    expect(gapStatsOf(gapsOf(GAP_TRACE.samples)).over1s).toBe(m.gaps.over1s);
  });
});

describe('render — CS-4 vocabulario honesto (X-1)', () => {
  it('labels metrics as commanded vs planned and never claims accuracy', () => {
    const html = renderToStaticMarkup(<PlanExecPanel trace={TRACE} plan={PLAN} />);
    expect(html).toContain('Comandado vs planificado');
    expect(html).toContain('Importar CSV');
    expect(html).toContain('frames comandados');
    expect(html).toContain('error de envío');
    expect(html).toContain('desviación por joint');
    expect(html).toContain('&gt;1 s: 0');
    expect(html).not.toMatch(/accuracy|exactitud|precisión/i);
  });

  it('renders an empty state without crashing when there is no trace', () => {
    const html = renderToStaticMarkup(<PlanExecPanel trace={null} plan={null} />);
    expect(html).toContain('sin traza');
    expect(html).toContain('Importar CSV');
    expect(html).not.toContain('<svg');
  });

  it('shows trace-only metrics and no compared metrics when the plan is absent', () => {
    const html = renderToStaticMarkup(<PlanExecPanel trace={GAP_TRACE} plan={null} />);
    expect(html).toContain('gaps entre frames');
    expect(html).toContain('&gt;1 s: 1');
    expect(html).not.toContain('error de envío');
    expect(html).not.toContain('desviación por joint');
  });
});
