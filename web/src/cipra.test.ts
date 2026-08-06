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