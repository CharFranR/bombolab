/**
 * CIPRA WebSocket subscriber adapter (bombolab consumer side).
 *
 * Connects to ``ws://<host>:8000/ws/gcode/`` (host derived from the page
 * origin at runtime; a VITE_CIPRA_WS_URL override wins when present), reconnects
 * with exponential backoff, and applies the subscriber contract:
 *
 *  - on a VALIDATED ``gcode.ready`` receipt it ACKs with ``gcode.ack {id}``
 *    immediately (R10) and only then surfaces the job via ``onReady`` —
 *    ACK is decoupled from any user decision;
 *  - on a version mismatch it replies ``gcode.error E_PROTOCOL_VERSION`` and
 *    never queues the message (S2);
 *  - connection status is exposed for a "sin conexión" banner (R15).
 *
 * The pure pieces (URL building, message planning) are exported for tests; the
 * client takes an injectable socket factory so tests never touch the network.
 */
import {
  validateEnvelope,
  isGcodeReadyEnvelope,
  buildAckRequest,
  buildErrorRequest,
  type Envelope,
  type GcodeReadyEnvelope,
} from './cipra/protocol';

export type CipraConnectionStatus = 'disconnected' | 'connecting' | 'connected';

/** User-facing (Spanish-neutral) label for the connection indicator (R15). */
export function getConnectionStatusLabel(status: CipraConnectionStatus): string {
  switch (status) {
    case 'connected':
      return 'CIPRA: en línea';
    case 'connecting':
      return 'CIPRA: conectando…';
    case 'disconnected':
      return 'CIPRA: sin conexión';
  }
}

const GCODE_WS_PORT = 8000;

/** Derive the gcode WS URL from the page origin (host only; port is fixed).
 *  An explicit env override replaces the whole URL when provided. */
export function buildGcodeWsUrl(
  pageLocation: { protocol: string; hostname: string },
  envUrl?: string | null,
): string {
  if (envUrl) return envUrl;
  const proto = pageLocation.protocol === 'https:' ? 'wss' : 'ws';
  const host = pageLocation.hostname || 'localhost';
  return `${proto}://${host}:${GCODE_WS_PORT}/ws/gcode/`;
}

/** Read the VITE_CIPRA_WS_URL override (optional; never required). */
export function readEnvWsUrl(): string | undefined {
  try {
    const env = (import.meta as unknown as { env?: Record<string, string | undefined> }).env;
    return env?.VITE_CIPRA_WS_URL;
  } catch {
    return undefined;
  }
}

export type IncomingPlan =
  | { kind: 'arrive'; envelope: GcodeReadyEnvelope }
  | { kind: 'version-mismatch'; id: string | undefined }
  | { kind: 'invalid' }
  | { kind: 'ignore' };

/** Pure decision on a raw inbound message: what the client must do (R4/S2). */
export function planIncoming(raw: string): IncomingPlan {
  let message: unknown;
  try {
    message = JSON.parse(raw);
  } catch {
    return { kind: 'invalid' };
  }
  const { valid, error } = validateEnvelope(message);
  if (!valid) {
    if (error === 'E_PROTOCOL_VERSION') {
      const id = (message as { id?: unknown }).id;
      return { kind: 'version-mismatch', id: typeof id === 'string' ? id : undefined };
    }
    return { kind: 'invalid' };
  }
  if (isGcodeReadyEnvelope(message)) return { kind: 'arrive', envelope: message };
  return { kind: 'ignore' };
}

const BASE_RECONNECT_MS = 500;
const MAX_RECONNECT_MS = 10_000;

export class GcodeClient {
  status: CipraConnectionStatus = 'disconnected';
  onStatus?: (s: CipraConnectionStatus) => void;
  onReady?: (env: GcodeReadyEnvelope) => void;

  private readonly wsUrl: string;
  private readonly socketFactory: (url: string) => WebSocket;
  private socket: WebSocket | null = null;
  private reconnectAttempt = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private stopped = false;

  constructor(
    url: string,
    socketFactory: (url: string) => WebSocket = (u) => new WebSocket(u),
  ) {
    this.wsUrl = url;
    this.socketFactory = socketFactory;
  }

  connect(): void {
    this.stopped = false;
    this.openSocket();
  }

  disconnect(): void {
    this.stopped = true;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    try {
      this.socket?.close();
    } catch {
      /* socket already gone */
    }
    this.socket = null;
    this.setStatus('disconnected');
  }

  private openSocket(): void {
    if (this.stopped) return;
    this.setStatus('connecting');
    const ws = this.socketFactory(this.wsUrl);
    this.socket = ws;
    ws.onopen = () => {
      this.reconnectAttempt = 0;
      this.setStatus('connected');
    };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') this.handleMessage(ev.data);
    };
    ws.onerror = () => {
      /* close event follows */
    };
    ws.onclose = () => {
      this.setStatus('disconnected');
      if (this.stopped) return;
      this.scheduleReconnect();
    };
  }

  private handleMessage(raw: string): void {
    const plan = planIncoming(raw);
    switch (plan.kind) {
      case 'arrive':
        // ACK fires on validated receipt (R10), before any user decision.
        this.send(buildAckRequest(plan.envelope.id));
        this.onReady?.(plan.envelope);
        break;
      case 'version-mismatch':
        this.send(buildErrorRequest('E_PROTOCOL_VERSION', plan.id));
        break;
      case 'invalid':
      case 'ignore':
        break;
    }
  }

  private send(env: Envelope): void {
    try {
      this.socket?.send(JSON.stringify(env));
    } catch {
      /* connection dropped mid-send; reconnect handles it */
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return; // never double-schedule
    const delay = Math.min(
      BASE_RECONNECT_MS * 2 ** this.reconnectAttempt,
      MAX_RECONNECT_MS,
    );
    this.reconnectAttempt += 1;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.openSocket();
    }, delay);
  }

  private setStatus(s: CipraConnectionStatus): void {
    if (this.status === s) return;
    this.status = s;
    this.onStatus?.(s);
  }
}