/**
 * Pending/cipra-job queue state machine (R8/R9).
 *
 * Pure reducer — no React, no WebSocket imports — so the ordering/transition
 * rules are unit-testable in isolation. The WS adapter only feeds ARRIVE; the
 * UI drives ACCEPT / DRAW / DISCARD / COMPLETE based on user decisions.
 *
 * No automatic decisions happen in the reducer: an arrival while the robot is
 * drawing is queued as `pending` and recorded in `lastNotice` so the UI can
 * show a banner; it is never auto-started.
 */
export type JobStatus = 'pending' | 'accepted' | 'drawing' | 'completed' | 'discarded';

export interface CipraJob {
  id: string;
  name: string;
  status: JobStatus;
  payload: string;
  createdAt: number;
}

export interface JobQueueState {
  jobs: CipraJob[];
  drawingId: string | null;
  /** Latest arrival worth surfacing (banner). null once dismissed/next arrival. */
  lastNotice: { jobId: string; whileDrawing: boolean } | null;
}

export const initialJobState: JobQueueState = {
  jobs: [],
  drawingId: null,
  lastNotice: null,
};

export type JobAction =
  | { type: 'ARRIVE'; job: { id: string; name: string; payload: string } }
  | { type: 'ACCEPT'; id: string }
  | { type: 'DRAW'; id: string }
  | { type: 'COMPLETE'; id: string }
  | { type: 'DISCARD'; id: string };

export function jobById(state: JobQueueState, id: string): CipraJob | undefined {
  return state.jobs.find((j) => j.id === id);
}

export function jobStatus(state: JobQueueState, id: string): JobStatus | undefined {
  return jobById(state, id)?.status;
}

/** Pending jobs the user can still pick to draw (single-active drawing). */
export function pendingJobs(state: JobQueueState): CipraJob[] {
  return state.jobs.filter((j) => j.status === 'pending');
}

export function jobReducer(state: JobQueueState, action: JobAction): JobQueueState {
  switch (action.type) {
    case 'ARRIVE': {
      const { id, name, payload } = action.job;
      // Duplicate id → idempotent: keep the original, ignore the re-delivery (S11).
      if (state.jobs.some((j) => j.id === id)) return state;
      const whileDrawing = state.drawingId !== null;
      const job: CipraJob = { id, name, status: 'pending', payload, createdAt: Date.now() };
      return { ...state, jobs: [...state.jobs, job], lastNotice: { jobId: id, whileDrawing } };
    }
    case 'ACCEPT': {
      const job = jobById(state, action.id);
      if (!job || job.status !== 'pending') return state;
      return withUnchangedOrder(state, action.id, 'accepted', state.drawingId);
    }
    case 'DRAW': {
      const job = jobById(state, action.id);
      if (!job || job.status !== 'accepted') return state;
      if (state.drawingId !== null && state.drawingId !== action.id) return state; // single-active
      return withUnchangedOrder(state, action.id, 'drawing', action.id);
    }
    case 'COMPLETE': {
      const job = jobById(state, action.id);
      if (!job || job.status !== 'drawing') return state;
      return withUnchangedOrder(
        state,
        action.id,
        'completed',
        state.drawingId === action.id ? null : state.drawingId,
      );
    }
    case 'DISCARD': {
      const job = jobById(state, action.id);
      if (!job) return state;
      if (job.status === 'completed' || job.status === 'discarded') return state; // terminal
      return withUnchangedOrder(
        state,
        action.id,
        'discarded',
        state.drawingId === action.id ? null : state.drawingId,
      );
    }
    default:
      return state;
  }
}

/** Map one job's status, preserving array order and the drawing pointer. */
function withUnchangedOrder(
  state: JobQueueState,
  id: string,
  status: JobStatus,
  drawingId: string | null,
): JobQueueState {
  return {
    ...state,
    drawingId,
    jobs: state.jobs.map((j) => (j.id === id ? { ...j, status } : j)),
  };
}