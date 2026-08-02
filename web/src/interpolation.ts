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
  /**
   * Backlash take-up per channel (degrees, 6 values: J1..J5, gripper).
   * When a channel reverses direction, the commanded value is offset by
   * ±backlash in the new direction so the gear play is taken up immediately
   * and the arm lands where commanded. Persistent offset, flipped on each
   * reversal. 0 disables the channel. This is a HARDWARE-layer correction
   * (RobotController), not a trajectory concern.
   */
  backlash?: number[];
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
  private current: number[]; // commanded pose in FLOAT servo degrees
  private lastSent: number[]; // last frame actually written (integer degrees)
  private residue: number[]; // fractional leftover per channel (quantization)
  private steps: number[][] = [];
  private stepIdx = 0;
  private timer: ReturnType<typeof setInterval> | null = null;
  private idleResolver: (() => void) | null = null;
  // Backlash take-up state (per channel).
  private lastTarget: number[] = [];
  private lastDir: number[] = [];
  private playOffset: number[] = [];

  constructor(
    private readonly send: (wire: Uint8Array) => void,
    initialPose: number[],
    private config: InterpolationConfig = DEFAULT_INTERPOLATION,
  ) {
    this.current = [...initialPose];
    this.residue = new Array(initialPose.length).fill(0);
    this.lastSent = this.quantize(initialPose);
    this.lastTarget = [...initialPose];
    this.lastDir = new Array(initialPose.length).fill(0);
    this.playOffset = new Array(initialPose.length).fill(0);
  }

  /** Enable/disable backlash take-up at runtime (A/B testing without reconnect). */
  setBacklash(backlash: number[] | undefined): void {
    this.config.backlash = backlash;
    if (!backlash) {
      this.playOffset = new Array(this.lastTarget.length).fill(0);
      this.lastDir = new Array(this.lastTarget.length).fill(0);
    }
  }

  /**
   * Backlash take-up: command `target + playOffset` where the offset flips
   * sign when the channel reverses direction. The gear play absorbs the
   * offset, so the arm ends where commanded while the play is taken up
   * immediately instead of releasing as a jump at the next corner.
   */
  private applyBacklash(target: number[]): number[] {
    const b = this.config.backlash;
    if (!b) {
      this.lastTarget = [...target];
      return target;
    }
    const eff = new Array(target.length);
    for (let i = 0; i < target.length; i++) {
      const d = Math.sign(target[i] - this.lastTarget[i]);
      if (d !== 0 && d !== this.lastDir[i]) {
        this.lastDir[i] = d;
        this.playOffset[i] = d * (b[i] ?? 0);
      }
      eff[i] = target[i] + this.playOffset[i];
    }
    this.lastTarget = [...target];
    return eff;
  }

  /**
   * Quantize a float pose to whole degrees, carrying the fractional residue
   * per channel: 0.3, 0.6, 0.9, 1.2 → sends 1° and keeps 0.2. This avoids
   * systematically losing sub-degree motion (the wire protocol only accepts
   * integers). The measured servo deadband is a SEPARATE concern — do not
   * fold it into this accumulator.
   */
  private quantize(pose: number[]): number[] {
    const out = new Array(pose.length);
    for (let i = 0; i < pose.length; i++) {
      const v = pose[i] + this.residue[i];
      const q = Math.round(v);
      this.residue[i] = v - q;
      // Wide safety clamp (µs wire: 544..2400; degrees legacy: 5..175):
      // the firmware rejects out-of-range frames, so never leave either band.
      out[i] = Math.max(0, Math.min(5000, q));
    }
    return out;
  }

  private sendFrame(pose: number[]): void {
    this.lastSent = this.quantize(pose);
    this.send(encodeWire(this.lastSent));
  }

  /** Plan and start a smooth move from the last-sent pose to `target`. */
  moveTo(target: number[]): void {
    this.stopTimer();
    const eff = this.applyBacklash(target);
    const steps = interpolateAll(this.current, eff, this.config.stepSize);
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
    this.send(encodeWire(this.lastSent));
  }

  /**
   * Rebase the tracked pose to `pose` WITHOUT sending. Used after raw
   * calibration writes that bypassed the interpolator: the next moveTo
   * continues from where the arm actually is.
   */
  sync(pose: number[]): void {
    this.stopTimer();
    this.steps = [];
    this.current = [...pose];
    this.lastSent = this.quantize(pose);
    this.lastTarget = [...pose];
    this.lastDir = new Array(pose.length).fill(0);
    this.playOffset = new Array(pose.length).fill(0);
  }

  /**
   * Resolves when the current interpolation queue is fully drained — i.e.
   * every scheduled frame has been SENT over serial. Deterministic (queue
   * length × delay), NOT physical servo feedback: the arm may still be
   * moving. Retargeting (a new moveTo) supersedes the awaited plan and
   * resolves the promise early.
   */
  whenIdle(): Promise<void> {
    if (this.steps.length === 0 || this.stepIdx >= this.steps.length) {
      return Promise.resolve();
    }
    return new Promise((resolve) => {
      this.idleResolver = resolve;
    });
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
    this.sendFrame(this.current);
  }

  private resolveIdle(): void {
    if (this.idleResolver !== null) {
      const resolve = this.idleResolver;
      this.idleResolver = null;
      resolve();
    }
  }

  private stopTimer(): void {
    if (this.timer !== null) {
      clearInterval(this.timer);
      this.timer = null;
    }
    // Queue drained or plan superseded: anyone awaiting whenIdle may proceed.
    this.resolveIdle();
  }
}
