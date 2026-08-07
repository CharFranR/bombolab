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
import { describe, it, expect, vi, afterEach } from 'vitest';
import {
  validateEnvelope,
  buildAckRequest,
  buildErrorRequest,
  SCHEMA_VERSION,
  MAX_PAYLOAD_BYTES,
} from './cipra/protocol';
import {
  jobReducer,
  initialJobState,
  jobById,
  jobStatus,
  MAX_PENDING_JOBS,
  queueFull,
  canEnqueue,
} from './cipra/jobStore';
import {
  loadGcodeText,
  mapDrawFailureToErrorCode,
  type LoadGcodeTextDeps,
} from './cipra/loadGcodeText';
import { buildGcodeWsUrl, planIncoming, GcodeClient, getConnectionStatusLabel } from './cipra';

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

class FakeWebSocket {
  static instances: FakeWebSocket[] = [];
  url: string;
  sent: string[] = [];
  onopen: ((ev: unknown) => void) | null = null;
  onmessage: ((ev: { data: unknown }) => void) | null = null;
  onclose: ((ev: unknown) => void) | null = null;
  onerror: ((ev: unknown) => void) | null = null;
  constructor(url: string) {
    this.url = url;
    FakeWebSocket.instances.push(this);
  }
  send(data: string): void {
    this.sent.push(data);
  }
  close(): void {
    this.onclose?.({});
  }
  open(): void {
    this.onopen?.({});
  }
  deliver(data: string): void {
    this.onmessage?.({ data });
  }
}

afterEach(() => {
  FakeWebSocket.instances.length = 0;
  vi.restoreAllMocks();
});

describe('cipra.ts — WebSocket adapter (K4-04)', () => {
  describe('buildGcodeWsUrl', () => {
    it('builds ws://host:8000/ws/gcode/ from an http page origin', () => {
      expect(buildGcodeWsUrl({ protocol: 'http:', hostname: '192.168.1.7' }, null)).toBe(
        'ws://192.168.1.7:8000/ws/gcode/',
      );
    });

    it('upgrades to wss:// on an https page origin', () => {
      expect(buildGcodeWsUrl({ protocol: 'https:', hostname: 'bombolab.local' }, null)).toBe(
        'wss://bombolab.local:8000/ws/gcode/',
      );
    });

    it('prefers an explicit env override URL when present', () => {
      expect(
        buildGcodeWsUrl({ protocol: 'http:', hostname: 'ignored' }, 'ws://other:9000/ws/gcode/'),
      ).toBe('ws://other:9000/ws/gcode/');
    });
  });

  describe('getConnectionStatusLabel', () => {
    it('maps each status to its user-facing banner copy (R15)', () => {
      expect(getConnectionStatusLabel('connected')).toBe('CIPRA: en línea');
      expect(getConnectionStatusLabel('connecting')).toBe('CIPRA: conectando…');
      expect(getConnectionStatusLabel('disconnected')).toBe('CIPRA: sin conexión');
    });
  });

  describe('planIncoming', () => {
    it('plans ack+arrive for a validated gcode.ready', () => {
      expect(planIncoming(JSON.stringify(READY_FIXTURE))).toEqual({
        kind: 'arrive',
        envelope: READY_FIXTURE,
      });
    });

    it('plans a protocol-version error reply for a version mismatch (S2)', () => {
      expect(planIncoming(JSON.stringify({ ...READY_FIXTURE, version: 2 }))).toEqual({
        kind: 'version-mismatch',
        id: 'job-1',
      });
    });

    it('plans ignore for unparseable data', () => {
      expect(planIncoming('not-json')).toEqual({ kind: 'invalid' });
    });

    it('plans ignore for non-ready envelopes', () => {
      const nojob = { type: 'no-job', version: 1, id: 'x', name: '', meta: {}, payload: '' };
      expect(planIncoming(JSON.stringify(nojob))).toEqual({ kind: 'ignore' });
    });
  });

  describe('ACK on validated receipt (R10/S8) + status', () => {
    function makeClient(): { client: GcodeClient; sockets: FakeWebSocket[]; onReady: ReturnType<typeof vi.fn> } {
      const sockets: FakeWebSocket[] = [];
      const client = new GcodeClient('ws://test:8000/ws/gcode/', (url) => {
        const s = new FakeWebSocket(url);
        sockets.push(s);
        return s as unknown as WebSocket;
      });
      const onReady = vi.fn();
      client.onReady = onReady;
      client.connect();
      return { client, sockets, onReady };
    }

    it('sends a gcode.ack for a validated receipt before surfacing the job', () => {
      const { sockets, onReady } = makeClient();
      const ws = sockets[0];
      ws.open();
      ws.deliver(JSON.stringify(READY_FIXTURE));
      expect(ws.sent).toContainEqual(
        JSON.stringify({ type: 'gcode.ack', version: 1, id: 'job-1', name: '', meta: {}, payload: '' }),
      );
      expect(onReady).toHaveBeenCalledWith(expect.objectContaining({ id: 'job-1' }));
    });

    it('replies E_PROTOCOL_VERSION error but does NOT ack/surface on version mismatch (S2)', () => {
      const { sockets, onReady } = makeClient();
      const ws = sockets[0];
      ws.open();
      ws.deliver(JSON.stringify({ ...READY_FIXTURE, version: 99 }));
      expect(onReady).not.toHaveBeenCalled();
      const outbound = ws.sent.map((s) => JSON.parse(s));
      expect(outbound.some((o) => o.type === 'gcode.error' && o.meta?.code === 'E_PROTOCOL_VERSION')).toBe(true);
      expect(outbound.some((o) => o.type === 'gcode.ack')).toBe(false);
    });

    it('reports connecting → connected on open and powers the offline banner', () => {
      const statuses: string[] = [];
      const sockets: FakeWebSocket[] = [];
      const client = new GcodeClient('ws://x/ws/gcode/', (url) => {
        const s = new FakeWebSocket(url);
        sockets.push(s);
        return s as unknown as WebSocket;
      });
      client.onStatus = (s) => statuses.push(s);
      client.connect();
      expect(statuses).toContain('connecting');
      sockets[0].open();
      expect(statuses).toContain('connected');
      expect(client.status).toBe('connected');
      sockets[0].close();
      expect(statuses).toContain('disconnected');
      expect(client.status).toBe('disconnected');
      client.disconnect(); // cancel the reconnect timer
    });
  });
});

// ─── Review-fix harnesses ────────────────────────────────────────────────────
// A module-level client factory so the review-fix suites (below) share the
// same fake-socket setup as the ACK suite without touching existing tests.
function makeCipraClient(): { client: GcodeClient; sockets: FakeWebSocket[]; onReady: ReturnType<typeof vi.fn> } {
  const sockets: FakeWebSocket[] = [];
  const client = new GcodeClient('ws://test:8000/ws/gcode/', (url) => {
    const s = new FakeWebSocket(url);
    sockets.push(s);
    return s as unknown as WebSocket;
  });
  const onReady = vi.fn();
  client.onReady = onReady;
  client.connect();
  return { client, sockets, onReady };
}

function outboundJson(sockets: FakeWebSocket[]): Record<string, any>[] {
  return sockets[0].sent.map((s) => JSON.parse(s));
}

describe('review fix #1 — payload size bound (MAX_PAYLOAD_BYTES)', () => {
  it('exports MAX_PAYLOAD_BYTES = 512 KiB', () => {
    expect(MAX_PAYLOAD_BYTES).toBe(512 * 1024);
  });

  it('rejects a gcode.ready payload above the limit with E_INVALID_ENVELOPE', () => {
    expect(
      validateEnvelope({ ...READY_FIXTURE, payload: 'x'.repeat(MAX_PAYLOAD_BYTES + 1) }),
    ).toEqual({ valid: false, error: 'E_INVALID_ENVELOPE' });
  });

  it('accepts a payload exactly at the 512 KiB boundary', () => {
    expect(validateEnvelope({ ...READY_FIXTURE, payload: 'x'.repeat(MAX_PAYLOAD_BYTES) })).toEqual({
      valid: true,
      error: null,
    });
  });

  it('plans an oversized arrival as oversize (never fed to the store)', () => {
    const plan = planIncoming(
      JSON.stringify({ ...READY_FIXTURE, payload: 'y'.repeat(MAX_PAYLOAD_BYTES + 1) }),
    );
    expect(plan).toEqual({ kind: 'oversize', id: 'job-1' });
  });

  it('receive path replies E_INVALID_ENVELOPE for an oversized arrival (no ack, no store feed)', () => {
    const { sockets, onReady } = makeCipraClient();
    sockets[0].open();
    sockets[0].deliver(
      JSON.stringify({ ...READY_FIXTURE, payload: 'y'.repeat(MAX_PAYLOAD_BYTES + 1) }),
    );
    expect(onReady).not.toHaveBeenCalled();
    const outbound = outboundJson(sockets);
    expect(
      outbound.some((o) => o.type === 'gcode.error' && o.meta?.code === 'E_INVALID_ENVELOPE'),
    ).toBe(true);
    expect(outbound.some((o) => o.type === 'gcode.ack')).toBe(false);
  });
});

describe('review fix #1 — pending queue bound (MAX_PENDING_JOBS=5)', () => {
  it('exports MAX_PENDING_JOBS = 5', () => {
    expect(MAX_PENDING_JOBS).toBe(5);
  });

  it('rejects a new ARRIVE at capacity as a strict no-op (same reference)', () => {
    let s = initialJobState;
    for (let i = 0; i < MAX_PENDING_JOBS; i++) s = jobReducer(s, arrive(`j${i}`));
    expect(queueFull(s)).toBe(true);
    expect(canEnqueue(s)).toBe(false);
    const before = s;
    const next = jobReducer(s, arrive('overflow'));
    expect(next).toBe(before);
    expect(jobStatus(next, 'overflow')).toBeUndefined();
    expect(next.jobs).toHaveLength(MAX_PENDING_JOBS);
  });

  it('allows arrivals up to the cap and reports queueFull only at it', () => {
    let s = initialJobState;
    expect(queueFull(s)).toBe(false);
    for (let i = 0; i < MAX_PENDING_JOBS - 1; i++) s = jobReducer(s, arrive(`k${i}`));
    expect(canEnqueue(s)).toBe(true);
    expect(queueFull(s)).toBe(false);
    s = jobReducer(s, arrive('last'));
    expect(queueFull(s)).toBe(true);
    expect(s.jobs).toHaveLength(MAX_PENDING_JOBS);
  });
});

describe('review fix #1 — receive path emits error instead of store feed (mock WS)', () => {
  it('replies E_QUEUE_FULL and skips the store feed when the queue is at capacity', () => {
    const { client, sockets, onReady } = makeCipraClient();
    client.canAcceptJob = () => false; // caller sees a full queue
    sockets[0].open();
    sockets[0].deliver(JSON.stringify(READY_FIXTURE));
    expect(onReady).not.toHaveBeenCalled();
    const outbound = outboundJson(sockets);
    expect(outbound.some((o) => o.type === 'gcode.error' && o.meta?.code === 'E_QUEUE_FULL')).toBe(
      true,
    );
    // Delivery is still confirmed (R10): the ACK is a receipt, not a queue slot.
    expect(outbound.some((o) => o.type === 'gcode.ack')).toBe(true);
  });

  it('feeds the store normally (onReady) when the queue can enqueue', () => {
    const { client, sockets, onReady } = makeCipraClient();
    client.canAcceptJob = () => true;
    sockets[0].open();
    sockets[0].deliver(JSON.stringify(READY_FIXTURE));
    expect(onReady).toHaveBeenCalledWith(expect.objectContaining({ id: 'job-1' }));
    expect(
      outboundJson(sockets).some((o) => o.type === 'gcode.error' && o.meta?.code === 'E_QUEUE_FULL'),
    ).toBe(false);
  });
});

describe('review fix #5 — sendError + draw-failure mapping (E_PARSE_GCODE / E_UNREACHABLE)', () => {
  it('sendError emits a canonical gcode.error envelope with code and job id', () => {
    const { client, sockets } = makeCipraClient();
    client.sendError('E_PARSE_GCODE', 'job-9');
    expect(JSON.parse(sockets[0].sent[0])).toEqual({
      type: 'gcode.error',
      version: 1,
      id: 'job-9',
      name: '',
      meta: { code: 'E_PARSE_GCODE', message: expect.any(String) },
      payload: '',
    });
  });

  it('sendError omits the job id when none is attached', () => {
    const { client, sockets } = makeCipraClient();
    client.sendError('E_UNREACHABLE');
    expect(JSON.parse(sockets[0].sent[0])).toMatchObject({
      type: 'gcode.error',
      meta: { code: 'E_UNREACHABLE' },
      id: '',
    });
  });

  it('maps parse-level draw failures (exception, nothing drawable) to E_PARSE_GCODE', () => {
    expect(mapDrawFailureToErrorCode('exception')).toBe('E_PARSE_GCODE');
    expect(mapDrawFailureToErrorCode('no-drawable')).toBe('E_PARSE_GCODE');
  });

  it('maps workspace/reachability rejection to E_UNREACHABLE', () => {
    expect(mapDrawFailureToErrorCode('blocked')).toBe('E_UNREACHABLE');
  });
});