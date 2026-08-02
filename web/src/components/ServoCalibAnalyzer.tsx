/**
 * Servo calibration analyzer — permanent development tool.
 *
 * Turns the manual calibration log (or an imported CSV) into statistics and
 * visualizations per joint:
 *   - backlash events (dead runs) with their angle and direction,
 *   - mean / median / σ per joint and per direction (CW/CCW),
 *   - histogram of event values,
 *   - scatter of backlash vs joint angle (reveals patterns like "play grows
 *     near joint limits" that averages hide).
 *
 * The goal is NOT a perfect mechanical model: it is to answer ONE question —
 * "does a single compensation value per joint explain the data?" If σ is
 * small, a fixed take-up makes sense; if σ is large, no fixed compensation
 * should be implemented. The measurement itself decides the model.
 */

import { useMemo, useState } from 'react';

export interface CalibEntry {
  joint: number;
  from: number;
  to: number;
  moved: boolean;
}

export interface BacklashEvent {
  /** Dead degrees lost in the run (number of consecutive "no" steps). */
  value: number;
  /** Joint angle (degrees) where the run started. */
  angle: number;
  /** Direction of the run. */
  dir: 'cw' | 'ccw';
  /** Reversal = run right after a direction change (backlash); miss = isolated. */
  kind: 'reversal' | 'miss';
}

export interface JointStats {
  joint: number;
  events: BacklashEvent[];
  count: number;
  mean: number;
  median: number;
  sigma: number;
  min: number;
  max: number;
  cw: number[]; // event values by direction
  ccw: number[];
}

function mean(xs: number[]): number {
  return xs.length === 0 ? 0 : xs.reduce((a, b) => a + b, 0) / xs.length;
}

function median(xs: number[]): number {
  if (xs.length === 0) return 0;
  const s = [...xs].sort((a, b) => a - b);
  const mid = Math.floor(s.length / 2);
  return s.length % 2 === 0 ? (s[mid - 1] + s[mid]) / 2 : s[mid];
}

function sigma(xs: number[]): number {
  if (xs.length < 2) return 0;
  const m = mean(xs);
  return Math.sqrt(mean(xs.map((x) => (x - m) * (x - m))));
}

/**
 * Extract backlash events from the raw verdict log.
 *
 * A dead run is a maximal run of consecutive `moved=false` entries in the
 * same direction. If the run starts right after a direction change it is a
 * reversal event (gear play taken up); otherwise an isolated miss. The run
 * value is its length in degrees — the motion the joint failed to deliver.
 */
export function analyzeBacklash(log: CalibEntry[]): Map<number, BacklashEvent[]> {
  const byJoint = new Map<number, BacklashEvent[]>();
  const order: number[] = [];
  for (const e of log) {
    if (!byJoint.has(e.joint)) {
      byJoint.set(e.joint, []);
      order.push(e.joint);
    }
  }

  for (const j of order) {
    const es = log.filter((e) => e.joint === j);
    const events: BacklashEvent[] = [];
    let prevDir = 0; // direction of the last non-zero entry
    let runStart = -1;
    let runAngle = 0;
    let runDir: 'cw' | 'ccw' = 'cw';
    let runPrevDir = 0; // direction BEFORE the run started (for reversal detection)

    const closeRun = (endIdx: number) => {
      if (runStart < 0) return;
      const len = endIdx - runStart;
      if (len > 0) {
        const first = es[runStart];
        const isReversal = runPrevDir !== 0 && Math.sign(first.to - first.from) !== runPrevDir;
        events.push({
          value: len,
          angle: runAngle,
          dir: runDir,
          kind: isReversal ? 'reversal' : 'miss',
        });
      }
      runStart = -1;
    };

    for (let i = 0; i < es.length; i++) {
      const e = es[i];
      const dir = Math.sign(e.to - e.from);
      if (dir === 0) continue;
      if (dir !== prevDir && runStart >= 0) closeRun(i);
      const beforeDir = prevDir;
      prevDir = dir;
      runDir = dir > 0 ? 'cw' : 'ccw';
      if (!e.moved) {
        if (runStart < 0) {
          runStart = i;
          runAngle = e.from;
          runPrevDir = beforeDir;
        }
      } else {
        closeRun(i);
      }
    }
    closeRun(es.length);
    byJoint.set(j, events);
  }
  return byJoint;
}

export function statsFor(joint: number, events: BacklashEvent[]): JointStats {
  const values = events.map((e) => e.value);
  return {
    joint,
    events,
    count: values.length,
    mean: mean(values),
    median: median(values),
    sigma: sigma(values),
    min: values.length ? Math.min(...values) : 0,
    max: values.length ? Math.max(...values) : 0,
    cw: events.filter((e) => e.dir === 'cw').map((e) => e.value),
    ccw: events.filter((e) => e.dir === 'ccw').map((e) => e.value),
  };
}

// ─── CSV import ────────────────────────────────────────────────────────────

export function parseCalibCsv(text: string): CalibEntry[] {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length > 0 && !l.startsWith('joint'));
  return lines.map((l) => {
    const [joint, from, to, moved] = l.split(',');
    return {
      joint: parseInt(joint, 10),
      from: parseInt(from, 10),
      to: parseInt(to, 10),
      moved: moved.trim() === 'si',
    };
  });
}

// ─── Charts (dependency-free SVG/div) ──────────────────────────────────────

function Histogram({ stats }: { stats: JointStats }) {
  const maxVal = Math.max(1, stats.max);
  const counts = new Array(maxVal + 1).fill(0);
  for (const e of stats.events) counts[e.value] = (counts[e.value] ?? 0) + 1;
  const maxCount = Math.max(1, ...counts);
  return (
    <div style={{ display: 'flex', alignItems: 'flex-end', gap: 3, height: 44, marginTop: 4 }}>
      {counts.slice(1).map((c, i) => (
        <div key={i} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 2 }}>
          <div
            style={{
              width: 14,
              height: Math.max(2, (c / maxCount) * 36),
              background: c > 0 ? '#aa8' : '#2a2a2a',
              borderRadius: 2,
            }}
          />
          <span style={{ fontSize: 8, color: '#777' }}>{i + 1}°</span>
        </div>
      ))}
    </div>
  );
}

function Scatter({ stats }: { stats: JointStats }) {
  const W = 380;
  const H = 130;
  const maxVal = Math.max(1, stats.max);
  const x = (angle: number) => (angle / 180) * W;
  const y = (v: number) => H - 8 - (v / (maxVal + 1)) * (H - 20);
  return (
    <svg width={W} height={H} style={{ marginTop: 4, background: '#1a1a1e', borderRadius: 4 }}>
      {[0, 45, 90, 135, 180].map((a) => (
        <g key={a}>
          <line x1={x(a)} y1={4} x2={x(a)} y2={H - 10} stroke="#2c2c32" strokeWidth={1} />
          <text x={x(a) - 6} y={H - 1} fontSize={8} fill="#666">{a}°</text>
        </g>
      ))}
      {stats.events.map((e, i) => (
        <circle
          key={i}
          cx={x(e.angle)}
          cy={y(e.value)}
          r={3.5}
          fill={e.kind === 'reversal' ? (e.dir === 'cw' ? '#69c' : '#c96') : '#888'}
        />
      ))}
      <text x={4} y={10} fontSize={8} fill="#777">backlash vs ángulo (● reversión · ● miss)</text>
    </svg>
  );
}

// ─── Component ─────────────────────────────────────────────────────────────

export default function ServoCalibAnalyzer({ log }: { log: CalibEntry[] }) {
  const [imported, setImported] = useState<CalibEntry[] | null>(null);
  const entries = imported ?? log;
  const byJoint = useMemo(() => analyzeBacklash(entries), [entries]);
  const stats = useMemo(
    () => [...byJoint.entries()].map(([j, ev]) => statsFor(j, ev)),
    [byJoint],
  );

  const handleImport = (file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      try {
        setImported(parseCalibCsv(String(reader.result)));
      } catch (e) {
        console.error('[analyzer] CSV inválido', e);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div style={{ borderTop: '1px solid #333', padding: '8px 16px' }}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 6 }}>
        <span style={{ fontSize: 11, color: '#888' }}>Analizador de backlash</span>
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
        {imported && (
          <button
            onClick={() => setImported(null)}
            style={{ fontSize: 10, color: '#c96', background: 'none', border: 'none', cursor: 'pointer' }}
          >
            (volver al log actual)
          </button>
        )}
        {entries.length === 0 && (
          <span style={{ fontSize: 10, color: '#555' }}>sin datos — corré la calibración</span>
        )}
      </div>

      {stats.map((s) => (
        <div key={s.joint} style={{ marginBottom: 10, border: '1px solid #2c2c32', borderRadius: 6, padding: 8 }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }}>
            <span style={{ fontSize: 12, color: '#ccc', fontWeight: 600 }}>J{s.joint}</span>
            <span style={{ fontSize: 10, color: '#888', fontFamily: 'monospace' }}>
              n={s.count} · media {s.mean.toFixed(1)}° · mediana {s.median.toFixed(1)}° · σ{' '}
              {s.sigma.toFixed(1)}° · [{s.min}–{s.max}]°
            </span>
          </div>
          <div style={{ fontSize: 10, color: '#777' }}>
            CW: {s.cw.length ? `${mean(s.cw).toFixed(1)}° media (${s.cw.join('°, ')}°)` : '—'} · CCW:{' '}
            {s.ccw.length ? `${mean(s.ccw).toFixed(1)}° media (${s.ccw.join('°, ')}°)` : '—'}
          </div>
          <Histogram stats={s} />
          <Scatter stats={s} />
          <div style={{ fontSize: 9, color: '#666', marginTop: 2 }}>
            {s.sigma <= 0.75 && s.count >= 3
              ? 'σ baja → compensación fija viable'
              : s.count >= 3
                ? 'σ alta → un valor fijo NO explica los datos'
                : 'faltan muestras (σ requiere ≥3 eventos)'}
          </div>
        </div>
      ))}
    </div>
  );
}
