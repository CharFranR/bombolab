/**
 * R19 tests for the bombolab-side CIPRA subscriber.
 *
 * object test the pure pieces that keep the WS subscriber honest:
 *  - protocol.ts  — the vendored TS mirror of the CIPRA envelope contract
 *  - jobStore.ts  — the pending/received/job state machine
 *  - loadGcodeText.ts — the shared gcode→trajectory pipeline used by both the
 *    file picker and the WS arrival path
 *  - cipra.ts     — the WebSocket adapter (URL building, message planning, ACK).
 *
 * Follows the gcode.test.ts convention: pure functions, no jsdom/React.
 */
import { describe, it, expect } from 'vitest';
import { validateEnvelope, buildAckRequest, buildErrorRequest, SCHEMA_VERSION } from './cipra/protocol';
import { jobReducer, initialJobState, jobById, jobStatus } from './cipra/jobStore';
import { loadGcodeText, type LoadGcodeTextDeps } from './cipra/loadGcodeText';

/** Reachable drawing-plane heights (mirror of reachability DRAW/TRAVEL_PLANE_Z).
 *  Kept literal here to keep the node test free of the wasm module chain. */
const DRAW_PLANE_Z = 80;
const TRAVEL_PLANE_Z = 85;

const READY_FIXTURE = {
  type: 'gcode.ready',
  version: 1,
  id: 'job-1',
  name: 'logo.gcode',
  meta: {},
  payload: 'G21 G90\nM3\nG1 X10 Y10 F600',
};

describe('cipra/protocol.ts — mirrored envelope contract', () => {
  it('accepts a well-formed gcode.ready envelope', () => {
    expect(validateEnvelope(READY_FIXTURE)).toEqual({ valid: true, error: null });
  });

  it('rejects a version mismatch with E_PROTOCOL_VERSION', () => {
    expect(validateEnvelope({ ...READY_FIXTURE, version: 2 })).toEqual({
      valid: false,
      error: 'E_PROTOCOL_VERSION',
    });
  });

  it('rejects an envelope missing a required key with E_INVALID_ENVELOPE', () => {
    const { id: _omit, ...noId } = READY_FIXTURE;
    expect(validateEnvelope(noId)).toEqual({ valid: false, error: 'E_INVALID_ENVELOPE' });
  });

  it('rejects a gcode.ready with an empty payload with E_INVALID_ENVELOPE', () => {
    expect(validateEnvelope({ ...READY_FIXTURE, payload: '   ' })).toEqual({
      valid: false,
      error: 'E_INVALID_ENVELOPE',
    });
  });

  it('rejects an unknown envelope type with E_INVALID_ENVELOPE', () => {
    expect(validateEnvelope({ ...READY_FIXTURE, type: 'gcode.gibberish' })).toEqual({
      valid: false,
      error: 'E_INVALID_ENVELOPE',
    });
  });

  it('rejects a non-object message with E_INVALID_ENVELOPE', () => {
    expect(validateEnvelope('not-an-envelope')).toEqual({
      valid: false,
      error: 'E_INVALID_ENVELOPE',
    });
  });

  it('builds a canonical gcode.ack for the received id (R10)', () => {
    expect(buildAckRequest('job-1')).toEqual({
      type: 'gcode.ack',
      version: SCHEMA_VERSION,
      id: 'job-1',
      name: '',
      meta: {},
      payload: '',
    });
  });

  it('builds a gcode.error carrying the canonical code (S2)', () => {
    const err = buildErrorRequest('E_PROTOCOL_VERSION', 'job-1');
    expect(err.type).toBe('gcode.error');
    expect(err.version).toBe(SCHEMA_VERSION);
    expect(err.id).toBe('job-1');
    expect(err.meta.code).toBe('E_PROTOCOL_VERSION');
    expect(err.meta.message).toBeTypeOf('string');
    expect(err.payload).toBe('');
  });
});

function arrive(id: string, name = id): ReturnType<typeof jobReducer> extends never ? never : { type: 'ARRIVE'; job: { id: string; name: string; payload: string } } {
  return { type: 'ARRIVE', job: { id, name, payload: 'G21\nM3\nG1 X1 Y1' } };
}

describe('cipra/jobStore.ts — pending-job state machine (R8/R9)', () => {
  it('queues a new job as pending', () => {
    const s = jobReducer(initialJobState, arrive('a'));
    expect(s.jobs).toHaveLength(1);
    expect(jobStatus(s, 'a')).toBe('pending');
  });

  it('accepts pending → accepted', () => {
    const s = jobReducer(jobReducer(initialJobState, arrive('a')), { type: 'ACCEPT', id: 'a' });
    expect(jobStatus(s, 'a')).toBe('accepted');
    expect(s.drawingId).toBeNull();
  });

  it('draws accepted → drawing (single-active)', () => {
    let s = jobReducer(initialJobState, arrive('a'));
    s = jobReducer(s, { type: 'ACCEPT', id: 'a' });
    s = jobReducer(s, { type: 'DRAW', id: 'a' });
    expect(jobStatus(s, 'a')).toBe('drawing');
    expect(s.drawingId).toBe('a');
  });

  it('fails to draw a second job while one is drawing (single-active, R9)', () => {
    let s = jobReducer(initialJobState, arrive('a'));
    s = jobReducer(s, arrive('b'));
    s = jobReducer(s, { type: 'ACCEPT', id: 'a' });
    s = jobReducer(s, { type: 'DRAW', id: 'a' });
    s = jobReducer(s, { type: 'ACCEPT', id: 'b' });
    const t = jobReducer(s, { type: 'DRAW', id: 'b' });
    expect(t.drawingId).toBe('a'); // current drawing untouched
    expect(jobStatus(t, 'b')).toBe('accepted'); // did not start
  });

  it('discards pending → discarded (terminal)', () => {
    const s = jobReducer(jobReducer(initialJobState, arrive('a')), { type: 'DISCARD', id: 'a' });
    expect(jobStatus(s, 'a')).toBe('discarded');
  });

  it('ignores a duplicate id — idempotent, keeps the original (S11)', () => {
    let s = jobReducer(initialJobState, arrive('a', 'first'));
    s = jobReducer(s, arrive('a', 'second'));
    expect(s.jobs).toHaveLength(1);
    expect(jobById(s, 'a')?.name).toBe('first');
  });

  it('queues an arrival while drawing as pending + notice, no auto-switch (R13)', () => {
    let s = jobReducer(initialJobState, arrive('a'));
    s = jobReducer(s, { type: 'ACCEPT', id: 'a' });
    s = jobReducer(s, { type: 'DRAW', id: 'a' });
    s = jobReducer(s, arrive('b'));
    expect(jobStatus(s, 'b')).toBe('pending'); // queued, not auto-drawn
    expect(s.drawingId).toBe('a'); // current drawing untouched
    expect(s.lastNotice).toEqual({ jobId: 'b', whileDrawing: true });
  });

  it('keeps another pending job selectable while one is drawing (S6)', () => {
    let s = jobReducer(initialJobState, arrive('a'));
    s = jobReducer(s, arrive('b'));
    s = jobReducer(s, { type: 'ACCEPT', id: 'b' });
    s = jobReducer(s, { type: 'DRAW', id: 'b' });
    expect(jobStatus(s, 'a')).toBe('pending');
    expect(jobStatus(s, 'b')).toBe('drawing');
  });

  it('completes drawing → completed (terminal) and clears the active id', () => {
    let s = jobReducer(initialJobState, arrive('a'));
    s = jobReducer(s, { type: 'ACCEPT', id: 'a' });
    s = jobReducer(s, { type: 'DRAW', id: 'a' });
    s = jobReducer(s, { type: 'COMPLETE', id: 'a' });
    expect(jobStatus(s, 'a')).toBe('completed');
    expect(s.drawingId).toBeNull();
  });

  it('a later arrival after a discard opens a fresh pending (S7)', () => {
    let s = jobReducer(initialJobState, arrive('a'));
    s = jobReducer(s, { type: 'DISCARD', id: 'a' });
    s = jobReducer(s, arrive('b'));
    expect(jobStatus(s, 'b')).toBe('pending');
  });

  it('treats an invalid transition as a strict no-op (same reference)', () => {
    const s = jobReducer(initialJobState, arrive('a'));
    const next = jobReducer(s, { type: 'DRAW', id: 'a' }); // pending cannot draw directly
    expect(next).toBe(s);
  });
});

function makeLoaderDeps(overrides: Partial<LoadGcodeTextDeps> = {}): LoadGcodeTextDeps {
  return {
    safeDrawingArea: async () => ({ xMin: 160, xMax: 240, yMin: -35, yMax: 35 }),
    parseGcode: () => ({
      commands: [{ type: 'move', target: [180, 0, DRAW_PLANE_Z], speed: 40 }],
      warnings: [],
      bounds: { min: [179, -1], max: [181, 1] },
      moveCount: 1,
    }),
    startTrajectory: async () => true,
    setValidating: () => {},
    setGcodeError: () => {},
    setGcodeWarnings: () => {},
    setGcodeName: () => {},
    ...overrides,
  };
}

describe('cipra/loadGcodeText.ts — shared gcode→trajectory pipeline (R12)', () => {
  it('parses with the safe drawing area and starts the trajectory in-reach (R11)', async () => {
    let parsed = false;
    let startedKey = '';
    const deps = makeLoaderDeps({
      parseGcode: (text, opts) => {
        parsed = true;
        expect(opts.area).toEqual({ xMin: 160, xMax: 240, yMin: -35, yMax: 35 });
        expect(opts.planeZ).toBe(DRAW_PLANE_Z);
        expect(opts.travelZ).toBe(TRAVEL_PLANE_Z);
        return {
          commands: [{ type: 'penDown' }, { type: 'move', target: [40, 0, DRAW_PLANE_Z], speed: 40 }],
          warnings: ['arco omitido'],
          bounds: { min: [40, 0], max: [40, 0] },
          moveCount: 1,
        };
      },
      startTrajectory: async (_c, key) => {
        startedKey = key;
        return true;
      },
    });
    const res = await loadGcodeText('G21\nM3\nG1 X10 Y10', 'logo.gcode', deps);
    expect(parsed).toBe(true);
    expect(startedKey).toBe('gcode');
    expect(res).toEqual({ ok: true });
  });

  it('blocks (no draw) when the workspace rejects an out-of-reach trajectory (R11/S9)', async () => {
    let started = false;
    const deps = makeLoaderDeps({
      startTrajectory: async () => {
        started = true;
        return false; // workspace block — nothing queued
      },
    });
    const res = await loadGcodeText('G21\nM3\nG1 X999 Y999', 'far.gcode', deps);
    // `startTrajectory` WAS consulted (the pre-flight gate ran), but it refused.
    expect(started).toBe(true);
    expect(res).toEqual({ ok: false, reason: 'blocked' });
  });

  it('surfaces a visible error and does NOT start when there are no drawable moves', async () => {
    const gcodeError: string[] = [];
    const deps = makeLoaderDeps({
      parseGcode: () => ({ commands: [], warnings: [], bounds: null, moveCount: 0 }),
      setGcodeError: (e) => {
        if (e !== null) gcodeError.push(e);
      },
      startTrajectory: async () => {
        throw new Error('must not be called');
      },
    });
    const res = await loadGcodeText('G4 P1', 'empty.gcode', deps);
    expect(gcodeError.length).toBe(1);
    expect(res).toEqual({ ok: false, reason: 'no-drawable' });
  });

  it('always clears the spinner in the finally block (validating=false)', async () => {
    const flags: boolean[] = [];
    const deps = makeLoaderDeps({
      setValidating: (v) => flags.push(v),
    });
    await loadGcodeText('G21\nG1 X1 Y1', 'x.gcode', deps);
    expect(flags[0]).toBe(true);
    expect(flags[flags.length - 1]).toBe(false);
  });
});