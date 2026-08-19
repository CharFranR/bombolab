/**
 * TEMPORARY draw-test view: upload a trace CSV (Exportar traza) or raw
 * SAMPLE protocol lines and plot the commanded tool-tip path on a 2D
 * Cartesian view (top-down of the drawing plane, mm).
 *
 * Diagnostic purpose: compare what the robot was COMMANDED to draw (this
 * plot) against the physical drawing — isolates IK/protocol error from
 * mechanical error.
 */
import { useMemo, useRef, useState } from 'react';
import { fabriCreator, forwardKinematics } from '../wasm';
import { parseFrameFile, framesToPlot, type FramePlot } from '../motion/plotFile';
import { TRACE_CSV_HEADER } from '../motion/csv';

interface PlotBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

function boundsOf(plot: FramePlot): PlotBounds | null {
  const all = [...plot.drawing, ...plot.travel];
  if (all.length === 0) return null;
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  for (const [x, y] of all) {
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  return { minX, maxX, minY, maxY };
}

const MARGIN = 10; // mm around the data
const VIEW_W = 600;
const VIEW_H = 600;

/** SVG polyline points string for a point list (viewBox coords). */
function toSvgPoints(
  pts: [number, number][],
  b: PlotBounds,
): string {
  const xSpan = Math.max(b.maxX - b.minX + 2 * MARGIN, 1);
  const ySpan = Math.max(b.maxY - b.minY + 2 * MARGIN, 1);
  const scale = Math.min(VIEW_W / xSpan, VIEW_H / ySpan);
  // Center the drawing in the viewBox.
  const offX = (VIEW_W - xSpan * scale) / 2;
  const offY = (VIEW_H - ySpan * scale) / 2;
  return pts
    .map(([x, y]) => {
      const sx = offX + (x - b.minX + MARGIN) * scale;
      const sy = VIEW_H - (offY + (y - b.minY + MARGIN) * scale); // Y up
      return `${sx.toFixed(2)},${sy.toFixed(2)}`;
    })
    .join(' ');
}

/** Grid lines every `step` mm in viewBox coords. */
function gridLines(b: PlotBounds): { x: number[]; y: number[] } {
  const step = 10;
  const xs: number[] = [];
  for (let gx = Math.ceil(b.minX / step) * step; gx <= b.maxX; gx += step) xs.push(gx);
  const ys: number[] = [];
  for (let gy = Math.ceil(b.minY / step) * step; gy <= b.maxY; gy += step) ys.push(gy);
  return { x: xs, y: ys };
}

export default function DrawTestPanel() {
  const inputRef = useRef<HTMLInputElement>(null);
  const [fileName, setFileName] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [plot, setPlot] = useState<FramePlot | null>(null);
  const [format, setFormat] = useState<'csv' | 'sample' | null>(null);

  const robot = useMemo(() => {
    try {
      return fabriCreator();
    } catch {
      return null;
    }
  }, []);

  const handleFile = (file: File) => {
    setError(null);
    setPlot(null);
    setFormat(null);
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const text = String(reader.result);
        if (!robot) throw new Error('WASM no disponible — recargá la página');
        const frames = parseFrameFile(text);
        const p = framesToPlot(robot, frames);
        setPlot(p);
        setFormat(text.trim().startsWith('SAMPLE ') ? 'sample' : 'csv');
        setFileName(file.name);
      } catch (e) {
        setError((e as Error).message);
        setFileName(file.name);
      }
    };
    reader.readAsText(file);
  };

  const bounds = useMemo(() => (plot ? boundsOf(plot) : null), [plot]);

  return (
    <div style={{ padding: 12, borderTop: '1px solid #333' }}>
      <div style={{ fontSize: 12, color: '#ccc', marginBottom: 6 }}>
        Test de dibujo (temporal) — plano cartesiano XY [mm]
      </div>
      <div style={{ fontSize: 10, color: '#777', marginBottom: 8 }}>
        Subí el export de traza CSV (Exportar traza) o líneas SAMPLE del protocolo v2. El plot
        muestra lo que el robot fue <b>comandado</b> a dibujar (µs → FK): cian = lápiz abajo
        (plano z=80), gris = viaje.
      </div>

      <input
        ref={inputRef}
        type="file"
        accept=".csv,.txt,text/csv,text/plain"
        style={{ display: 'none' }}
        onChange={(e) => {
          const f = e.target.files?.[0];
          if (f) handleFile(f);
        }}
      />
      <button
        onClick={() => inputRef.current?.click()}
        style={{
          padding: '6px 12px',
          background: '#464',
          border: 'none',
          borderRadius: 4,
          color: '#ccc',
          fontSize: 12,
          cursor: 'pointer',
        }}
      >
        Subir archivo…
      </button>
      {fileName && (
        <span style={{ fontSize: 10, color: '#888', marginLeft: 8 }}>
          {fileName} · {format === 'csv' ? 'trace CSV' : format === 'sample' ? 'SAMPLE lines' : ''}
        </span>
      )}

      {error && (
        <div role="alert" style={{ fontSize: 11, color: '#e55', marginTop: 6 }}>
          {error}
        </div>
      )}

      {plot && bounds && (
        <>
          <svg
            viewBox={`0 0 ${VIEW_W} ${VIEW_H}`}
            style={{
              width: '100%',
              maxWidth: 560,
              marginTop: 10,
              background: '#141418',
              border: '1px solid #333',
              borderRadius: 4,
            }}
          >
            {/* Grid */}
            {gridLines(bounds).x.map((gx) => {
              const xSpan = Math.max(bounds.maxX - bounds.minX + 2 * MARGIN, 1);
              const scale = Math.min(VIEW_W / xSpan, VIEW_H / Math.max(bounds.maxY - bounds.minY + 2 * MARGIN, 1));
              const offX = (VIEW_W - xSpan * scale) / 2;
              const sx = offX + (gx - bounds.minX + MARGIN) * scale;
              return (
                <line key={`gx-${gx}`} x1={sx} y1={0} x2={sx} y2={VIEW_H} stroke="#2a2a30" strokeWidth={0.5} />
              );
            })}
            {gridLines(bounds).y.map((gy) => {
              const ySpan = Math.max(bounds.maxY - bounds.minY + 2 * MARGIN, 1);
              const scale = Math.min(VIEW_W / Math.max(bounds.maxX - bounds.minX + 2 * MARGIN, 1), VIEW_H / ySpan);
              const offY = (VIEW_H - ySpan * scale) / 2;
              const sy = VIEW_H - (offY + (gy - bounds.minY + MARGIN) * scale);
              return (
                <line key={`gy-${gy}`} x1={0} y1={sy} x2={VIEW_W} y2={sy} stroke="#2a2a30" strokeWidth={0.5} />
              );
            })}

            {/* Travel points (pen up) — dim gray DOTS, never connected:
                connecting them would draw phantom chords. */}
            {plot.travel.map(([x, y], i) => {
              const xSpan = Math.max(bounds.maxX - bounds.minX + 2 * MARGIN, 1);
              const ySpan = Math.max(bounds.maxY - bounds.minY + 2 * MARGIN, 1);
              const scale = Math.min(VIEW_W / xSpan, VIEW_H / ySpan);
              const offX = (VIEW_W - xSpan * scale) / 2;
              const offY = (VIEW_H - ySpan * scale) / 2;
              const sx = offX + (x - bounds.minX + MARGIN) * scale;
              const sy = VIEW_H - (offY + (y - bounds.minY + MARGIN) * scale);
              return <circle key={`t-${i}`} cx={sx} cy={sy} r={1.2} fill="#777788" opacity={0.35} />;
            })}
            {/* Drawing path — one polyline PER STROKE (never connect across
                pen-up gaps). */}
            {plot.drawingStrokes.map((stroke, si) =>
              stroke.length > 1 ? (
                <polyline
                  key={`s-${si}`}
                  points={toSvgPoints(stroke, bounds)}
                  fill="none"
                  stroke="#44ddff"
                  strokeWidth={1.6}
                />
              ) : null,
            )}
            {/* Sample dots on the drawing path */}
            {plot.drawing.map(([x, y], i) => {
              const xSpan = Math.max(bounds.maxX - bounds.minX + 2 * MARGIN, 1);
              const ySpan = Math.max(bounds.maxY - bounds.minY + 2 * MARGIN, 1);
              const scale = Math.min(VIEW_W / xSpan, VIEW_H / ySpan);
              const offX = (VIEW_W - xSpan * scale) / 2;
              const offY = (VIEW_H - ySpan * scale) / 2;
              const sx = offX + (x - bounds.minX + MARGIN) * scale;
              const sy = VIEW_H - (offY + (y - bounds.minY + MARGIN) * scale);
              return <circle key={i} cx={sx} cy={sy} r={1.3} fill="#44ddff" opacity={0.6} />;
            })}
          </svg>

          <div style={{ fontSize: 10, color: '#888', marginTop: 6, fontFamily: 'monospace' }}>
            {plot.frames} frames · dibujo {plot.drawing.length} pts · viaje {plot.travel.length} pts · X
            [{bounds.minX.toFixed(1)}, {bounds.maxX.toFixed(1)}] · Y [{bounds.minY.toFixed(1)},{' '}
            {bounds.maxY.toFixed(1)}] mm
          </div>
          <div style={{ fontSize: 10, color: '#555', marginTop: 4 }}>
            Probar: exportá la traza del cuadrado demo (Exportar traza CSV) y subila acá — el plot
            debe mostrar el cuadrado. Si el papel difiere del plot, el error es mecánico.
          </div>
        </>
      )}
    </div>
  );
}
