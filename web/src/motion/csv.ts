import type { TraceResult, TraceSample } from './trace';

export const TRACE_CSV_HEADER = 'ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count';

function sampleRow(s: TraceSample): string {
  return [s.ts_ms, ...s.q_us, s.count].join(',');
}

export function exportTraceCsv(result: TraceResult): string {
  const lines = [TRACE_CSV_HEADER, ...result.samples.map(sampleRow), ''];
  return lines.join('\n');
}

/**
 * Descarga un Blob con nombre de archivo, robusto en Chrome y Firefox:
 * el anchor debe estar en el DOM (requisito de Firefox) y la object URL NO debe
 * revocarse sincrónicamente tras el click (revocarla antes de que el navegador
 * arranque la descarga la cancela silenciosamente). Revocar con setTimeout.
 */
export function downloadBlob(filename: string, blob: Blob): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.style.display = 'none';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function downloadTraceCsv(result: TraceResult, filename = 'trace.csv'): void {
  downloadBlob(filename, new Blob([exportTraceCsv(result)], { type: 'text/csv' }));
}

/** Copia texto al portapapeles con fallback (textarea + execCommand) para
 *  entornos donde la descarga directa está bloqueada. */
export async function copyText(text: string): Promise<boolean> {
  try {
    await navigator.clipboard.writeText(text);
    return true;
  } catch {
    try {
      const ta = document.createElement('textarea');
      ta.value = text;
      ta.style.position = 'fixed';
      ta.style.opacity = '0';
      document.body.appendChild(ta);
      ta.select();
      const ok = document.execCommand('copy');
      document.body.removeChild(ta);
      return ok;
    } catch {
      return false;
    }
  }
}

export function parseTraceCsv(text: string): TraceResult {
  const lines = text.split('\n');
  if (lines.length === 0 || lines[0] !== TRACE_CSV_HEADER) {
    throw new Error('CSV header does not match the trace format');
  }
  const samples: TraceSample[] = [];
  let framesWritten = 0;
  let dedupe = 0;
  for (let i = 1; i < lines.length; i++) {
    const line = lines[i].trim();
    if (line.length === 0) continue;
    const fields = line.split(',');
    if (fields.length !== 8) {
      throw new Error(`Row ${i + 1}: expected 8 fields, got ${fields.length}`);
    }
    const ts = Number(fields[0]);
    if (!Number.isFinite(ts)) throw new Error(`Row ${i + 1}: ts_ms is not a number`);
    const q = fields.slice(1, 7).map((f) => Number(f));
    if (q.some((v) => !Number.isInteger(v))) {
      throw new Error(`Row ${i + 1}: joint values must be integers`);
    }
    const count = Number(fields[7]);
    if (!Number.isInteger(count) || count < 1) {
      throw new Error(`Row ${i + 1}: count must be a positive integer`);
    }
    samples.push({ ts_ms: ts, q_us: q as [number, number, number, number, number, number], count });
    framesWritten += count;
    dedupe += count - 1;
  }
  return {
    samples,
    t0: 0,
    truncated: false,
    framesWritten,
    dedupe,
  };
}
