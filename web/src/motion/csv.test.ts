import { afterEach, describe, expect, it, vi } from 'vitest';
import { copyText, downloadBlob, downloadTraceCsv, exportTraceCsv, parseTraceCsv, TRACE_CSV_HEADER } from './csv';
import type { TraceResult } from './trace';

function sampleTrace(): TraceResult {
  return {
    samples: [
      { ts_ms: 0, q_us: [544, 1472, 1379, 1523, 1162, 1678], count: 1 },
      { ts_ms: 50.1234567890123, q_us: [1472, 1472, 1379, 1523, 1162, 1678], count: 2 },
      { ts_ms: 105.5, q_us: [1473, 1472, 1379, 1523, 1162, 1678], count: 1 },
    ],
    t0: 1723123456789.5,
    truncated: false,
    framesWritten: 4,
    dedupe: 1,
  };
}

describe('csv — CS-1 export shape', () => {
  it('emits the exact header and one row per sample with 8 fields', () => {
    const csv = exportTraceCsv(sampleTrace());
    const lines = csv.split('\n');
    expect(lines[0]).toBe(TRACE_CSV_HEADER);
    expect(lines[0]).toBe('ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count');
    expect(lines).toHaveLength(5);
    expect(lines[4]).toBe('');
    for (const line of lines.slice(1, 4)) {
      expect(line.split(',')).toHaveLength(8);
    }
    expect(lines[1]).toBe('0,544,1472,1379,1523,1162,1678,1');
  });

  it('round-trips samples, counts and derived fields exactly', () => {
    const imported = parseTraceCsv(exportTraceCsv(sampleTrace()));
    expect(imported.samples).toEqual(sampleTrace().samples);
    expect(imported.framesWritten).toBe(4);
    expect(imported.dedupe).toBe(1);
    expect(imported.truncated).toBe(false);
  });

  it('accepts an empty trace as a valid CSV with no rows', () => {
    const empty: TraceResult = {
      samples: [],
      t0: 0,
      truncated: false,
      framesWritten: 0,
      dedupe: 0,
    };
    expect(parseTraceCsv(exportTraceCsv(empty)).samples).toEqual([]);
  });
});

describe('csv — CS-2 corrupt input', () => {
  it('throws on a wrong header', () => {
    expect(() => parseTraceCsv('a,b\n1,2\n')).toThrow();
  });

  it('throws on rows with the wrong arity', () => {
    const csv = 'ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count\n0,544,1472,1379,1523,1162,1678\n';
    expect(() => parseTraceCsv(csv)).toThrow();
  });

  it('throws on non-numeric values and non-integer joints', () => {
    const badTs = 'ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count\nabc,544,1472,1379,1523,1162,1678,1\n';
    expect(() => parseTraceCsv(badTs)).toThrow();
    const badJoint = 'ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count\n0,54.4,1472,1379,1523,1162,1678,1\n';
    expect(() => parseTraceCsv(badJoint)).toThrow();
  });

  it('throws on non-positive counts', () => {
    const badCount = 'ts_ms,j1_us,j2_us,j3_us,j4_us,j5_us,j6_us,count\n0,544,1472,1379,1523,1162,1678,0\n';
    expect(() => parseTraceCsv(badCount)).toThrow();
  });

  it('leaves no partial state behind: valid parses still succeed after a throw', () => {
    expect(() => parseTraceCsv('garbage')).toThrow();
    expect(parseTraceCsv(exportTraceCsv(sampleTrace())).samples).toHaveLength(3);
  });
});

describe('csv — CS-3 double export', () => {
  it('produces byte-identical files without mutating the trace', () => {
    const trace = sampleTrace();
    const first = exportTraceCsv(trace);
    const second = exportTraceCsv(trace);
    expect(first).toBe(second);
    expect(trace.samples).toHaveLength(3);
    expect(trace.samples[0].ts_ms).toBe(0);
  });

  it('is stable across an export-import-export cycle', () => {
    const original = exportTraceCsv(sampleTrace());
    const reexported = exportTraceCsv(parseTraceCsv(original));
    expect(reexported).toBe(original);
  });
});

describe('csv — CS-4 downloadBlob (fix: export buttons no-op)', () => {
  const originalDocument = globalThis.document;
  const originalURL = globalThis.URL;

  afterEach(() => {
    globalThis.document = originalDocument;
    globalThis.URL = originalURL;
  });

  it('attaches the anchor to the DOM, clicks it, removes it and defers the revoke', () => {
    const clicked: string[] = [];
    const appended: unknown[] = [];
    const removed: unknown[] = [];
    const revoked: string[] = [];
    globalThis.document = {
      createElement: () => ({
        href: '',
        download: '',
        style: {},
        click: () => clicked.push((globalThis.document as any)._lastHref),
      }),
      body: {
        appendChild: (n: any) => {
          appended.push(n);
          (globalThis.document as any)._lastHref = n.href;
        },
        removeChild: (n: any) => removed.push(n),
      },
    } as any;
    globalThis.URL = {
      createObjectURL: () => 'blob:fake-1',
      revokeObjectURL: (u: string) => revoked.push(u),
    } as any;

    const timers = vi.useFakeTimers();
    try {
      downloadBlob('trace-test.csv', new Blob(['a,b,c'], { type: 'text/csv' }));
      const anchor = appended[0] as any;
      expect(anchor.download).toBe('trace-test.csv');
      expect(anchor.style.display).toBe('none');
      expect(clicked).toEqual(['blob:fake-1']);
      expect(removed).toHaveLength(1);
      // la URL NO se revoca sincrónicamente (eso cancelaba la descarga)
      expect(revoked).toEqual([]);
      timers.advanceTimersByTime(1000);
      expect(revoked).toEqual(['blob:fake-1']);
    } finally {
      timers.useRealTimers();
    }
  });

  it('copyText usa navigator.clipboard cuando está disponible', async () => {
    const originalClipboard = (navigator as any).clipboard;
    (navigator as any).clipboard = { writeText: (t: string) => Promise.resolve(t) };
    const ok = await copyText('a,b,c');
    expect(ok).toBe(true);
    (navigator as any).clipboard = originalClipboard;
  });

  it('copyText cae al fallback textarea+execCommand si clipboard no existe', async () => {
    const originalClipboard = (navigator as any).clipboard;
    (navigator as any).clipboard = undefined;
    let execCalled = false;
    globalThis.document = {
      createElement: () => ({ value: '', style: {}, select: () => {}, }),
      body: { appendChild: () => {}, removeChild: () => {} },
    } as any;
    globalThis.document.execCommand = () => { execCalled = true; return true; };
    const ok = await copyText('a,b,c');
    expect(ok).toBe(true);
    expect(execCalled).toBe(true);
    (navigator as any).clipboard = originalClipboard;
  });

  it('downloadTraceCsv routes through downloadBlob with the trace content', () => {
    const appended: unknown[] = [];
    globalThis.document = {
      createElement: () => ({ href: '', download: '', style: {}, click: () => {} }),
      body: {
        appendChild: (n: any) => {
          appended.push(n);
          (globalThis.document as any)._lastHref = n.href;
        },
        removeChild: () => {},
      },
    } as any;
    globalThis.URL = {
      createObjectURL: (b: any) => {
        (globalThis as any)._blobSize = b.size;
        return 'blob:fake-2';
      },
      revokeObjectURL: () => {},
    } as any;
    downloadTraceCsv(sampleTrace(), 'mi-traza.csv');
    expect((appended[0] as any).download).toBe('mi-traza.csv');
    expect((globalThis as any)._blobSize).toBeGreaterThan(0);
  });
});
