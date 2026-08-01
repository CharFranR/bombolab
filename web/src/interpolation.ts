/**
 * Client-side servo interpolation — a TypeScript mirror of the Rust
 * `communication::interpolation` module (same algorithms: per-joint
 * stepping with final-value padding, all joints arrive simultaneously).
 *
 * The web viewer sends frames directly via WebSerial with no pacing, so
 * IK and demo movements jump straight to the solved pose. `ServoInterpolator`
 * replans from the LAST SENT pose (so retargeting mid-motion continues from
 * where the arm actually is) and emits one frame per `delayMs` tick.
 */

import { encodeWire } from './serial';

export interface InterpolationConfig {
  /** Angle increment per step in degrees. */
  stepSize: number;
  /** Delay between steps in milliseconds. */
  delayMs: number;
}

/** Viewer pacing: 5° per step, 50 ms between steps (user-tuned; the Rust
 *  `InterpolationConfig::default()` keeps 100 ms for serial-test/CLI). */
export const DEFAULT_INTERPOLATION: InterpolationConfig = { stepSize: 5, delayMs: 50 };

/**
 * Linear interpolation of one channel from `current` to `target`, stepping
 * by `stepSize` degrees. Mirrors Rust `interpolate_joint`: exclusive start,
 * inclusive end; a distance under one step snaps straight to the target.
 */
export function interpolateJoint(current: number, target: number, stepSize: number): number[] {
  if (current === target) return [];
  const distance = Math.abs(target - current);
  if (stepSize >= distance) return [target];

  const direction = target > current ? 1 : -1;
  const steps: number[] = [];
  let pos = current;
  for (;;) {
    pos += direction * stepSize;
    steps.push(pos);
    if (pos === target) break;
    const next = pos + direction * stepSize;
    // If the next step would overshoot the target, append target and finish.
    if (direction * (next - target) >= 0) {
      steps.push(target);
      break;
    }
  }
  return steps;
}

/**
 * Interpolate all channels independently, then pad shorter ones with their
 * final value so all rows align (max steps across channels). Mirrors Rust
 * `interpolate_all`.
 */
export function interpolateAll(current: number[], target: number[], stepSize: number): number[][] {
  const channelSteps = current.map((c, i) => interpolateJoint(c, target[i], stepSize));
  const maxLen = channelSteps.reduce((m, s) => Math.max(m, s.length), 0);
  if (maxLen === 0) return [];

  const result: number[][] = [];
  for (let stepIdx = 0; stepIdx < maxLen; stepIdx++) {
    const row = channelSteps.map((steps, j) =>
      stepIdx < steps.length ? steps[stepIdx] : steps.length > 0 ? steps[steps.length - 1] : current[j],
    );
    result.push(row);
  }
  return result;
}

/**
 * Serial command scheduler. `moveTo` cancels any in-flight plan and replans
 * from the last-sent pose, then sends the first frame immediately and the
 * rest on a `delayMs` timer. `keepAlive` resends the last-sent frame so the
 * firmware failsafe (5 s watchdog) never parks the arm while connected.
 */
export class ServoInterpolator {
  private current: number[]; // last sent [j1..j5, gripper] in servo degrees
  private steps: number[][] = [];
  private stepIdx = 0;
  private timer: ReturnType<typeof setInterval> | null = null;

  constructor(
    private readonly send: (wire: Uint8Array) => void,
    initialPose: number[],
    private readonly config: InterpolationConfig = DEFAULT_INTERPOLATION,
  ) {
    this.current = [...initialPose];
  }

  /** Plan and start a smooth move from the last-sent pose to `target`. */
  moveTo(target: number[]): void {
    this.stopTimer();
    const steps = interpolateAll(this.current, target, this.config.stepSize);
    if (steps.length === 0) return;
    this.steps = steps;
    this.stepIdx = 0;
    // Send the first frame immediately, then pace the rest (mirrors the
    // Rust send-then-sleep loop).
    this.tick();
    this.timer = setInterval(() => this.tick(), this.config.delayMs);
  }

  /** Re-send the last-sent pose (failsafe heartbeat). */
  keepAlive(): void {
    this.send(encodeWire(this.current));
  }

  /** Cancel any in-flight interpolation and timers. */
  stop(): void {
    this.stopTimer();
    this.steps = [];
  }

  private tick(): void {
    if (this.stepIdx >= this.steps.length) {
      this.stopTimer();
      return;
    }
    this.current = this.steps[this.stepIdx++];
    this.send(encodeWire(this.current));
  }

  private stopTimer(): void {
    if (this.timer !== null) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }
}
