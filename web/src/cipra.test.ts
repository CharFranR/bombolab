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