export interface TraceSample {
  ts_ms: number;
  q_us: [number, number, number, number, number, number];
  count: number;
}

export interface TraceResult {
  samples: TraceSample[];
  t0: number;
  truncated: boolean;
  framesWritten: number;
  dedupe: number;
}

export const TRACE_CAPACITY = 100000;

function parseWire(wire: Uint8Array): [number, number, number, number, number, number] | null {
  const parts = new TextDecoder().decode(wire).trim().split(',');
  if (parts.length !== 6) return null;
  const vals = parts.map((p) => Number(p));
  if (vals.some((v) => !Number.isInteger(v))) return null;
  return vals as [number, number, number, number, number, number];
}

function sameQ(a: readonly number[], b: readonly number[]): boolean {
  for (let i = 0; i < 6; i++) {
    if (a[i] !== b[i]) return false;
  }
  return true;
}

export class TraceRecorder {
  private samples: TraceSample[] = [];
  private t0: number | null = null;
  private recording = false;
  private finalized = false;
  private truncated = false;
  private framesWritten = 0;
  private dedupe = 0;
  private readonly now: () => number;
  private readonly capacity: number;

  constructor(now: () => number = () => performance.now(), capacity: number = TRACE_CAPACITY) {
    this.now = now;
    this.capacity = capacity;
  }

  start(): void {
    this.samples = [];
    this.t0 = null;
    this.recording = true;
    this.finalized = false;
    this.truncated = false;
    this.framesWritten = 0;
    this.dedupe = 0;
  }

  isRecording(): boolean {
    return this.recording;
  }

  record(wire: Uint8Array): void {
    if (!this.recording) return;
    const q = parseWire(wire);
    if (q === null) return;
    const nowMs = this.now();
    if (this.t0 === null) this.t0 = nowMs;
    const last = this.samples[this.samples.length - 1];
    if (last !== undefined && sameQ(last.q_us, q)) {
      last.count += 1;
      this.dedupe += 1;
      this.framesWritten += 1;
      return;
    }
    if (this.samples.length >= this.capacity) {
      this.truncated = true;
      this.recording = false;
      return;
    }
    this.samples.push({ ts_ms: nowMs - this.t0, q_us: q, count: 1 });
    this.framesWritten += 1;
  }

  stop(): TraceResult | null {
    if (this.finalized || this.t0 === null) return null;
    this.finalized = true;
    this.recording = false;
    return {
      samples: this.samples,
      t0: this.t0,
      truncated: this.truncated,
      framesWritten: this.framesWritten,
      dedupe: this.dedupe,
    };
  }

  discard(): void {
    this.samples = [];
    this.t0 = null;
    this.recording = false;
    this.finalized = true;
    this.truncated = false;
    this.framesWritten = 0;
    this.dedupe = 0;
  }
}
