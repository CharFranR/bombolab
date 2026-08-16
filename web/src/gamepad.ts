/**
 * Gamepad → robot command mapping.
 *
 * Pure, framework-free logic (no React, no DOM): velocity-style rate control
 * — a stick axis means "move this joint at this speed", not "go to this
 * position". The App integration accumulates `q += v * dt` per animation
 * frame and clamps to each segment's limits; this module only turns raw
 * gamepad axes/buttons into per-joint velocity commands.
 *
 * Standard Xbox-style mapping (documented in the Gamepad panel):
 *   L-Stick X  → Base (Yaw)      J0
 *   L-Stick Y  → Shoulder        J1
 *   R-Stick Y  → Elbow           J2
 *   R-Stick X  → Wrist Roll      J3
 *   RT − LT    → Wrist Pitch     J4   (bipolar triggers)
 *   A          → Gripper open
 *   B          → Gripper close
 */

/** Minimal structural view of a gamepad so this module is testable without
 *  browser types. A real `Gamepad` satisfies this shape. */
export interface GamepadLike {
  readonly axes: readonly number[];
  readonly buttons: readonly { readonly pressed: boolean }[];
}

/** Per-joint velocity command from one poll of the gamepad. */
export interface GamepadCommand {
  /** rad/s per joint (index 4 = last joint of the 5-DOF arm). */
  readonly jointVelocities: number[];
  /** %/s added to the gripper state (0–100, 100 = closed). */
  readonly gripperDeltaPct: number;
}

/** One analog input source of the mapping. */
export type AnalogSource =
  | { readonly kind: 'axis'; readonly axis: number; readonly joint: number; readonly direction: 1 | -1 }
  /** `axes[axisA] - axes[axisB]` merged into one joint (e.g. RT − LT). */
  | { readonly kind: 'bipolar'; readonly axisA: number; readonly axisB: number; readonly joint: number; readonly direction: 1 | -1 };

export interface GamepadButtonBinding {
  readonly button: number;
  /** +1 = increase gripper percent (close), -1 = decrease (open). */
  readonly sign: 1 | -1;
}

export interface GamepadMappingConfig {
  /** Analog deadzone (fraction of full deflection, 0–1). */
  readonly deadzone: number;
  /** rad/s per joint at full deflection. */
  readonly jointSpeedRad: number[];
  /** %/s gripper velocity at full deflection. */
  readonly gripperSpeedPct: number;
  readonly analog: readonly AnalogSource[];
  readonly gripperButtons: readonly GamepadButtonBinding[];
}

export const DEFAULT_GAMEPAD_MAPPING: GamepadMappingConfig = {
  deadzone: 0.12,
  jointSpeedRad: [1.2, 0.8, 0.8, 1.5, 1.5],
  gripperSpeedPct: 60,
  analog: [
    { kind: 'axis', axis: 0, joint: 0, direction: 1 }, // L-Stick X → Base (Yaw)
    { kind: 'axis', axis: 1, joint: 1, direction: 1 }, // L-Stick Y → Shoulder
    { kind: 'axis', axis: 3, joint: 2, direction: 1 }, // R-Stick Y → Elbow
    { kind: 'axis', axis: 2, joint: 3, direction: 1 }, // R-Stick X → Wrist Roll
    { kind: 'bipolar', axisA: 7, axisB: 6, joint: 4, direction: 1 }, // RT − LT → Wrist Pitch
  ],
  gripperButtons: [
    { button: 0, sign: -1 }, // A → open
    { button: 1, sign: 1 }, // B → close
  ],
};

/** Standard scaled deadzone: |v| inside the deadzone → 0; outside it, the
 *  remaining range is remapped linearly to [-1, 1] so motion starts smoothly
 *  instead of in jumps. Input is clamped to [-1, 1] first. */
export function applyDeadzone(value: number, deadzone: number): number {
  const v = Math.max(-1, Math.min(1, value));
  if (deadzone >= 1) return 0;
  if (deadzone <= 0) return v;
  if (Math.abs(v) <= deadzone) return 0;
  return (v - Math.sign(v) * deadzone) / (1 - deadzone);
}

/** Clamp a joint angle to [q_min, q_max], defaulting to ±π when a segment
 *  declares no limits. Degenerate limits (min > max) are left untouched. */
export function clampAngle(q: number, qMin?: number, qMax?: number): number {
  const lo = qMin ?? -Math.PI;
  const hi = qMax ?? Math.PI;
  if (lo > hi) return q;
  return Math.max(lo, Math.min(hi, q));
}

/** Map a gamepad poll to per-joint velocities (rad/s) + gripper delta (%/s).
 *  Axes inside the deadzone and unpressed buttons contribute zero. */
export function mapGamepadToCommands(
  gamepad: GamepadLike,
  config: GamepadMappingConfig = DEFAULT_GAMEPAD_MAPPING,
): GamepadCommand {
  const nOutput = Math.max(
    config.jointSpeedRad.length,
    ...config.analog.map((src) => src.joint + 1),
  );
  const jointVelocities = new Array(nOutput).fill(0);
  for (const src of config.analog) {
    const raw = src.kind === 'axis'
      ? (gamepad.axes[src.axis] ?? 0)
      : (gamepad.axes[src.axisA] ?? 0) - (gamepad.axes[src.axisB] ?? 0);
    const v = applyDeadzone(raw, config.deadzone) * src.direction;
    if (v !== 0) {
      const speed = config.jointSpeedRad[src.joint] ?? 0;
      if (speed !== 0) jointVelocities[src.joint] = v * speed;
    }
  }
  let gripperDeltaPct = 0;
  for (const binding of config.gripperButtons) {
    if (gamepad.buttons[binding.button]?.pressed) {
      gripperDeltaPct += binding.sign * config.gripperSpeedPct;
    }
  }
  return { jointVelocities, gripperDeltaPct };
}
