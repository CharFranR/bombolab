import { useMemo, useState } from 'react';
import { compare, type DeviationStats, type GapStats, type SendTimeErrorStats } from '../motion/comparator';
import { parseTraceCsv } from '../motion/csv';
import type { PlanSample } from '../motion/planTimeline';
import type { TraceResult, TraceSample } from '../motion/trace';

const MS_PER_S = 1000;
const JOINT_LABELS = ['J1', 'J2', 'J3', 'J4', 'J5', 'Pinza'];

function meanOf(values: number[]): number {
  return values.length === 0 ? 0 : values.reduce((a, b) => a + b, 0) / values.length;
}

function sigmaOf(values: number[]): number {
  if (values.length < 2) return 0;
  const m = meanOf(values);
  const variance = values.reduce((acc, v) => acc + (v - m) * (v - m), 0) / (values.length - 1);
  return Math.sqrt(variance);
}

export function gapsOf(samples: TraceSample[]): number[] {
  const gaps: number[] = [];
  for (let k = 0; k < samples.length - 1; k++) {
    if (samples[k].count > 1 || samples[k + 1].count > 1) continue;
    gaps.push(samples[k + 1].ts_ms - samples[k].ts_ms);
  }
  return gaps;
}

export function gapStatsOf(gaps: number[]): GapStats {
  return {
    min: gaps.length > 0 ? Math.min(...gaps) : 0,
    mean: meanOf(gaps),
    max: gaps.length > 0 ? Math.max(...gaps) : 0,
    sigma: sigmaOf(gaps),
    count: gaps.length,
    over1s: gaps.filter((g) => g > 1000).length,
  };
}

export function sendTimeErrors(plan: PlanSample[], trace: TraceSample[]): number[] {
  const paired = Math.min(plan.length, trace.length);
  const errors: number[] = [];
  for (let k = 0; k < paired; k++) {
    errors.push(trace[k].ts_ms - plan[k].t * MS_PER_S);
  }
  return errors;
}

export function deviationSeries(plan: PlanSample[], trace: TraceSample[]): number[][] {
  const paired = Math.min(plan.length, trace.length);
  const deltas: number[][] = Array.from({ length: 6 }, () => []);
  for (let k = 0; k < paired; k++) {
    for (let j = 0; j < 6; j++) {
      deltas[j].push(Math.abs(trace[k].q_us[j] - plan[k].q_us[j]));
    }
  }
  return deltas;
}

export interface PanelSummary {
  samples: number;
  framesWritten: number;
  dedupe: number;
  durationRealMs: number;
  truncated: boolean;
  gaps: GapStats;
  sendTimeError: SendTimeErrorStats | null;
  deviation: DeviationStats | null;
  durationPlanMs: number | null;
}

export function summaryOf(trace: TraceResult, plan: PlanSample[] | null): PanelSummary {
  const metrics = plan ? compare(plan, trace.samples) : null;
  const last = trace.samples[trace.samples.length - 1];
  return {
    samples: trace.samples.length,
    framesWritten: trace.framesWritten,
    dedupe: trace.dedupe,
    durationRealMs: last ? last.ts_ms : 0,
    truncated: trace.truncated,
    gaps: gapStatsOf(gapsOf(trace.samples)),
    sendTimeError: metrics ? metrics.sendTimeError : null,
    deviation: metrics ? metrics.deviation : null,
    durationPlanMs: metrics ? metrics.duration.plan : null,
  };
}

export function importTraceText(text: string): { trace: TraceResult } | { error: string } {
  try {
    return { trace: parseTraceCsv(text) };
  } catch (e) {
    return { error: e instanceof Error ? e.message : String(e) };
  }
}

function fmtMs(v: number): string {
  return v.toFixed(1);
}

function fmtS(v: number): string {
  return (v / MS_PER_S).toFixed(2);
}

function fmtUs(v: number): string {
  return v.toFixed(0);
}

function PanelBody({ active, plan }: { active: TraceResult; plan: PlanSample[] | null }) {
  const summary = summaryOf(active, plan);
  return (
    <>
      <div style={{ fontSize: 10, fontFamily: 'monospace', color: '#aaa', marginBottom: 4 }}>
        frames comandados {summary.framesWritten} · muestras {summary.samples} · dedupe{' '}
        {summary.dedupe}
        {summary.truncated ? ' · traza truncada' : ''}
      </div>
      <div style={{ fontSize: 10, color: '#888', marginBottom: 4 }}>
        duración real {fmtS(summary.durationRealMs)} s
        {summary.durationPlanMs !== null ? ` · planificada ${fmtS(summary.durationPlanMs)} s` : ''}
      </div>
      {summary.sendTimeError !== null && (
        <div style={{ fontSize: 10, color: '#888', marginBottom: 4 }}>
          error de envío (comandado vs planificado): media {fmtMs(summary.sendTimeError.mean)} ms · máx
          |{fmtMs(summary.sendTimeError.maxAbs)}| ms · σ {fmtMs(summary.sendTimeError.sigma)} ms
        </div>
      )}
      <div style={{ fontSize: 10, color: '#888', marginBottom: 4 }}>
        gaps entre frames: {summary.gaps.count} · min {fmtMs(summary.gaps.min)} ms · media{' '}
        {fmtMs(summary.gaps.mean)} ms · máx {fmtMs(summary.gaps.max)} ms · σ {fmtMs(summary.gaps.sigma)}{' '}
        ms · &gt;1 s: {summary.gaps.over1s}
      </div>
      {summary.deviation !== null && (
        <div style={{ fontSize: 10, color: '#888', marginBottom: 4 }}>
          desviación por joint (comandado vs planificado):{' '}
          {summary.deviation.perJoint
            .map((d, j) => `${JOINT_LABELS[j]} media ${fmtUs(d.mean)} µs · máx ${fmtUs(d.max)} µs`)
            .join(' · ')}
          {' · global media '}
          {fmtUs(summary.deviation.globalMean)} µs · máx {fmtUs(summary.deviation.globalMax)} µs
        </div>
      )}
    </>
  );
}

export default function PlanExecPanel({
  trace,
  plan,
}: {
  trace: TraceResult | null;
  plan: PlanSample[] | null;
}) {
  const [imported, setImported] = useState<TraceResult | null>(null);
  const [importError, setImportError] = useState<string | null>(null);
  const active = imported ?? trace;

  const handleImport = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      const result = importTraceText(String(reader.result));
      if ('error' in result) {
        setImportError(result.error);
      } else {
        setImported(result.trace);
        setImportError(null);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div style={{ borderTop: '1px solid #333', padding: '8px 16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 6, flexWrap: 'wrap' }}>
        <span style={{ fontSize: 11, color: '#888' }}>Comandado vs planificado</span>
        <label style={{ fontSize: 10, color: '#69c', cursor: 'pointer' }}>
          Importar CSV
          <input
            type="file"
            accept=".csv"
            style={{ display: 'none' }}
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) handleImport(f);
            }}
          />
        </label>
        {imported && trace !== null && (
          <button
            onClick={() => setImported(null)}
            style={{ fontSize: 10, color: '#c96', background: 'none', border: 'none', cursor: 'pointer' }}
          >
            (volver al run actual)
          </button>
        )}
      </div>
      {importError && (
        <div style={{ fontSize: 10, color: '#e55', marginBottom: 6 }}>CSV inválido: {importError}</div>
      )}
      {!active ? (
        <div style={{ fontSize: 10, color: '#555' }}>sin traza — corré una trayectoria o importá un CSV</div>
      ) : (
        <PanelBody active={active} plan={plan} />
      )}
    </div>
  );
}
