/**
 * CIPRA WebSocket envelope protocol — bombolab (consumer) mirror.
 *
 * This is a faithful TypeScript mirror of the canonical contract owned by the
 * CIPRA backend at ``cipra_api/ws/protocol.py``. The envelope shape, version
 * and error codes here MUST stay in lock-step with that module — the publisher
 * and the subscriber vendored copies of the same contract.
 *
 * Keep this module JSON-only and free of React/WebSocket imports so it stays
 * trivially unit-testable and mirrorable in the other direction.
 */

export const SCHEMA_VERSION = 1;

/** Hard cap on a `gcode.ready` payload (review fix #1). Enforced in the
 *  receive path BEFORE the payload is parsed as G-Code or fed to the job
 *  store; an oversized envelope is rejected as `E_INVALID_ENVELOPE` and the
 *  publisher is told with a `gcode.error`. 512 KiB covers any real generated
 *  program while bounding memory and downstream parse cost. */
export const MAX_PAYLOAD_BYTES = 512 * 1024;

export const ENVELOPE_KEYS = ['type', 'version', 'id', 'name', 'meta', 'payload'] as const;

// Message / event types (mirror of the Python T_* constants).
export const T_GCODE_READY = 'gcode.ready';
export const T_GCODE_ACK = 'gcode.ack';
export const T_GCODE_ERROR = 'gcode.error';
export const T_NO_JOB = 'no-job';
export const T_PRESENCE = 'presence';

const KNOWN_TYPES = new Set<string>([T_GCODE_READY, T_GCODE_ACK, T_GCODE_ERROR, T_NO_JOB]);

// Canonical error codes (mirror of the backend ERROR_CODES map).
export const ERROR_CODES: Record<string, string> = {
  E_PROTOCOL_VERSION: 'Unsupported protocol version.',
  E_INVALID_ENVELOPE: 'Invalid or malformed envelope.',
  E_EMPTY_PAYLOAD: 'Empty G-Code payload; publish suppressed.',
  E_NO_JOB: 'No job is held in the current snapshot.',
  E_PARSE_GCODE: 'Failed to parse G-Code.',
  E_UNREACHABLE: 'Move is outside the reachable drawing area.',
  // bombolab → publisher EXTENSION (not in the backend's canonical map): the
  // pending-job queue is at capacity and the arrival was NOT enqueued. The
  // backend validates envelope shape only, so it tolerates unknown codes.
  E_QUEUE_FULL: 'Pending job queue is full; arrival not enqueued.',
};

export type MessageType =
  | typeof T_GCODE_READY
  | typeof T_GCODE_ACK
  | typeof T_GCODE_ERROR
  | typeof T_NO_JOB
  | typeof T_PRESENCE;

/** Canonical envelope. `payload` holds the full G-Code text for `gcode.ready`. */
export interface Envelope {
  type: string;
  version: number;
  id: string;
  name: string;
  meta: Record<string, unknown>;
  payload: string;
}

/** A validated `gcode.ready` envelope (guaranteed non-empty payload). */
export interface GcodeReadyEnvelope extends Envelope {
  type: typeof T_GCODE_READY;
}

export type ErrorCode = keyof typeof ERROR_CODES;

export interface ValidationResult {
  valid: boolean;
  error: ErrorCode | null;
}

/** Validate that `message` is a well-formed envelope (R4).
 *
 *  Mirrors the backend `validate_envelope`: required keys, known type, matching
 *  protocol version, non-empty id, and (for `gcode.ready`) a non-empty payload.
 */
export function validateEnvelope(message: unknown): ValidationResult {
  if (message === null || typeof message !== 'object' || Array.isArray(message)) {
    return { valid: false, error: 'E_INVALID_ENVELOPE' };
  }
  const env = message as Record<string, unknown>;
  if (!ENVELOPE_KEYS.every((k) => typeof env[k] !== 'undefined')) {
    return { valid: false, error: 'E_INVALID_ENVELOPE' };
  }
  const type = env.type;
  if (typeof type !== 'string' || !KNOWN_TYPES.has(type)) {
    return { valid: false, error: 'E_INVALID_ENVELOPE' };
  }
  if (env.version !== SCHEMA_VERSION) {
    return { valid: false, error: 'E_PROTOCOL_VERSION' };
  }
  if (typeof env.id !== 'string' || env.id.length === 0) {
    return { valid: false, error: 'E_INVALID_ENVELOPE' };
  }
  if (type === T_GCODE_READY && (typeof env.payload !== 'string' || env.payload.trim().length === 0)) {
    return { valid: false, error: 'E_INVALID_ENVELOPE' };
  }
  // Review fix #1: a payload above MAX_PAYLOAD_BYTES is rejected here too, so
  // ANY direct caller of validateEnvelope gets the same strict boundary.
  if (type === T_GCODE_READY && typeof env.payload === 'string' && env.payload.length > MAX_PAYLOAD_BYTES) {
    return { valid: false, error: 'E_INVALID_ENVELOPE' };
  }
  return { valid: true, error: null };
}

/** True when a parsed message carries a payload above MAX_PAYLOAD_BYTES
 *  (review fix #1). Kept separate from `validateEnvelope` so the receive path
 *  can CLASSIFY an oversized arrival and reply `gcode.error E_INVALID_ENVELOPE`
 *  instead of silently dropping it as generic malformed input. */
export function isOversizePayload(message: unknown): boolean {
  if (message === null || typeof message !== 'object' || Array.isArray(message)) return false;
  const payload = (message as Record<string, unknown>).payload;
  return typeof payload === 'string' && payload.length > MAX_PAYLOAD_BYTES;
}

/** Type-narrowing guard for a validated `gcode.ready` envelope. */
export function isGcodeReadyEnvelope(message: unknown): message is GcodeReadyEnvelope {
  const r = validateEnvelope(message);
  if (!r.valid) return false;
  return (message as Envelope).type === T_GCODE_READY;
}

/** Build a canonical `gcode.ack` for the received job *id* (R10). */
export function buildAckRequest(id: string): Envelope {
  return {
    type: T_GCODE_ACK,
    version: SCHEMA_VERSION,
    id,
    name: '',
    meta: {},
    payload: '',
  };
}

/** Build a `gcode.error` envelope carrying a canonical error code (S2). */
export function buildErrorRequest(code: ErrorCode, id?: string): Envelope {
  return {
    type: T_GCODE_ERROR,
    version: SCHEMA_VERSION,
    id: id ?? '',
    name: '',
    meta: { code, message: ERROR_CODES[code] ?? code },
    payload: '',
  };
}