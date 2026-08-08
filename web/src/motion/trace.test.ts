import { describe, expect, it } from 'vitest';
import { encodeWire } from '../serial';
import { TraceRecorder, TRACE_CAPACITY } from './trace';

function fakeClock(offsets: number[]) {
  let i = 0;
  return () => (i < offsets.length ? offsets[i++] : offsets[offsets.length - 1]);
}

function wire(j1: number, j2: number, j3: number, j4: number, j5: number, g: number): Uint8Array {
  return encodeWire([j1, j2, j3, j4, j5, g]);
}

describe('TraceRecorder — MT-1 full run recorded', () => {
  it('records every frame with relative timestamps and t0 = first frame time', () => {
    const rec = new TraceRecorder(fakeClock([1000, 1010, 1020, 1030]));
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(91, 90, 81, 95, 60, 110));
    rec.record(wire(92, 90, 81, 95, 60, 110));
    rec.record(wire(93, 90, 81, 95, 60, 110));
    const result = rec.stop();
    expect(result).not.toBeNull();
    expect(result!.t0).toBe(1000);
    expect(result!.samples).toHaveLength(4);
    expect(result!.samples.map((s) => s.ts_ms)).toEqual([0, 10, 20, 30]);
    expect(result!.samples.map((s) => s.count)).toEqual([1, 1, 1, 1]);
    expect(result!.framesWritten).toBe(4);
    expect(result!.dedupe).toBe(0);
    expect(result!.truncated).toBe(false);
  });

  it('parses the 6 wire values as integer microseconds', () => {
    const rec = new TraceRecorder(fakeClock([0, 5]));
    rec.start();
    rec.record(wire(544, 2400, 1234, 5678, 900, 1000));
    rec.record(wire(545, 2400, 1234, 5678, 900, 1000));
    const result = rec.stop()!;
    expect(result.samples[0].q_us).toEqual([544, 2400, 1234, 5678, 900, 1000]);
    expect(result.samples[1].q_us).toEqual([545, 2400, 1234, 5678, 900, 1000]);
  });

  it('records nothing before start and ignores frames after stop', () => {
    const rec = new TraceRecorder(fakeClock([0, 10]));
    rec.record(wire(90, 90, 81, 95, 60, 110));
    expect(rec.stop()).toBeNull();
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    const result = rec.stop()!;
    expect(result.samples).toHaveLength(1);
    rec.record(wire(91, 90, 81, 95, 60, 110));
    expect(rec.stop()).toBeNull();
  });

  it('ignores malformed wires without crashing the send path', () => {
    const rec = new TraceRecorder(fakeClock([0, 1]));
    rec.start();
    const bad = new TextEncoder().encode('90,90,81,95,60\n');
    rec.record(bad);
    rec.record(wire(90, 90, 81, 95, 60, 110));
    const result = rec.stop()!;
    expect(result.samples).toHaveLength(1);
  });
});

describe('TraceRecorder — MT-2 heartbeat dedupe', () => {
  it('coalesces consecutive identical frames into one sample with count > 1 and first ts', () => {
    const rec = new TraceRecorder(fakeClock([0, 1000, 2000, 3000, 3010]));
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(91, 90, 81, 95, 60, 110));
    const result = rec.stop()!;
    expect(result.samples).toHaveLength(2);
    expect(result.samples[0]).toEqual({ ts_ms: 0, q_us: [90, 90, 81, 95, 60, 110], count: 4 });
    expect(result.samples[1]).toEqual({ ts_ms: 3010, q_us: [91, 90, 81, 95, 60, 110], count: 1 });
    expect(result.framesWritten).toBe(5);
    expect(result.dedupe).toBe(3);
  });

  it('keeps identical frames separated by a different frame as distinct samples', () => {
    const rec = new TraceRecorder(fakeClock([0, 10, 20]));
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(91, 90, 81, 95, 60, 110));
    rec.record(wire(90, 90, 81, 95, 60, 110));
    const result = rec.stop()!;
    expect(result.samples).toHaveLength(3);
    expect(result.samples[0].count).toBe(1);
    expect(result.samples[2].count).toBe(1);
  });
});

describe('TraceRecorder — MT-3 stop and discard', () => {
  it('stop finalizes the trace; a second stop returns null', () => {
    const rec = new TraceRecorder(fakeClock([0, 10]));
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    const first = rec.stop()!;
    expect(first.samples).toHaveLength(1);
    expect(rec.stop()).toBeNull();
  });

  it('discard clears the trace and makes stop unavailable', () => {
    const rec = new TraceRecorder(fakeClock([0, 10, 20]));
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.discard();
    expect(rec.stop()).toBeNull();
    rec.start();
    rec.record(wire(91, 90, 81, 95, 60, 110));
    expect(rec.stop()!.samples[0].q_us[0]).toBe(91);
  });
});

describe('TraceRecorder — MT-4/MT-5 capacity overflow', () => {
  it('caps samples, stops recording and flags truncation', () => {
    const clock = (() => {
      let t = 0;
      return () => (t += 10);
    })();
    const rec = new TraceRecorder(clock, 3);
    rec.start();
    rec.record(wire(90, 90, 81, 95, 60, 110));
    rec.record(wire(91, 90, 81, 95, 60, 110));
    rec.record(wire(92, 90, 81, 95, 60, 110));
    rec.record(wire(93, 90, 81, 95, 60, 110));
    rec.record(wire(94, 90, 81, 95, 60, 110));
    const result = rec.stop()!;
    expect(result.samples).toHaveLength(3);
    expect(result.truncated).toBe(true);
    expect(result.framesWritten).toBe(3);
  });

  it('keeps the default capacity at 100000 samples', () => {
    expect(TRACE_CAPACITY).toBe(100000);
  });
});
