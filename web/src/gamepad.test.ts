import { describe, expect, it } from 'vitest';
import {
  applyDeadzone,
  clampAngle,
  DEFAULT_GAMEPAD_MAPPING,
  mapGamepadToCommands,
  type GamepadLike,
} from './gamepad';

/** Build a fake gamepad: `axes` values, `pressed` = indices of pressed buttons. */
function pad(axes: number[], pressed: number[] = []): GamepadLike {
  return {
    axes,
    buttons: Array.from({ length: 17 }, (_, i) => ({ pressed: pressed.includes(i) })),
  };
}

describe('applyDeadzone', () => {
  it('zeroes values inside the deadzone, boundary included', () => {
    expect(applyDeadzone(0, 0.12)).toBe(0);
    expect(applyDeadzone(0.05, 0.12)).toBe(0);
    expect(applyDeadzone(0.12, 0.12)).toBe(0);
    expect(applyDeadzone(-0.12, 0.12)).toBe(0);
  });

  it('scales values outside the deadzone linearly to [-1, 1]', () => {
    expect(applyDeadzone(1, 0.12)).toBeCloseTo(1, 6);
    expect(applyDeadzone(-1, 0.12)).toBeCloseTo(-1, 6);
    // (0.5 - 0.12) / (1 - 0.12)
    expect(applyDeadzone(0.5, 0.12)).toBeCloseTo(0.4318, 3);
    expect(applyDeadzone(-0.5, 0.12)).toBeCloseTo(-0.4318, 3);
  });

  it('starts smoothly just past the deadzone edge', () => {
    const near = applyDeadzone(0.121, 0.12);
    expect(near).toBeGreaterThan(0);
    expect(near).toBeLessThan(0.01);
  });

  it('is transparent with a zero deadzone and clamps input to [-1, 1]', () => {
    expect(applyDeadzone(0.7, 0)).toBe(0.7);
    expect(applyDeadzone(2, 0)).toBe(1);
  });

  it('zeroes everything when the deadzone covers the full range', () => {
    expect(applyDeadzone(1, 1)).toBe(0);
  });
});

describe('clampAngle', () => {
  it('clamps to the segment limits', () => {
    expect(clampAngle(2.5, -1, 1)).toBe(1);
    expect(clampAngle(-2.5, -1, 1)).toBe(-1);
    expect(clampAngle(0.5, -1, 1)).toBe(0.5);
  });

  it('defaults to ±π when the segment has no limits', () => {
    expect(clampAngle(4)).toBe(Math.PI);
    expect(clampAngle(-4)).toBe(-Math.PI);
    expect(clampAngle(1)).toBe(1);
  });

  it('uses each bound independently', () => {
    expect(clampAngle(0.5, 0)).toBe(0.5);
    expect(clampAngle(-0.5, 0)).toBe(0);
    expect(clampAngle(2, undefined, 1)).toBe(1);
  });

  it('leaves the value untouched on a degenerate min > max config', () => {
    expect(clampAngle(3, 2, 1)).toBe(3);
  });
});

describe('mapGamepadToCommands — centered / no input', () => {
  it('returns all-zero commands for a centered, untouched gamepad', () => {
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 0, 0]));
    expect(cmd.jointVelocities).toEqual([0, 0, 0, 0, 0]);
    expect(cmd.gripperDeltaPct).toBe(0);
  });

  it('ignores stick deflection inside the deadzone', () => {
    const cmd = mapGamepadToCommands(pad([0.1, -0.1, 0.1, 0, 0, 0, 0, 0]));
    expect(cmd.jointVelocities).toEqual([0, 0, 0, 0, 0]);
  });
});

describe('mapGamepadToCommands — stick mapping and directions', () => {
  it('L-Stick X full right → Base J0 at +max speed, full left → -max', () => {
    const right = mapGamepadToCommands(pad([1, 0, 0, 0, 0, 0, 0, 0]));
    expect(right.jointVelocities[0]).toBeCloseTo(DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[0], 6);
    const left = mapGamepadToCommands(pad([-1, 0, 0, 0, 0, 0, 0, 0]));
    expect(left.jointVelocities[0]).toBeCloseTo(-DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[0], 6);
  });

  it('L-Stick Y full up (−1) → Shoulder J1 at −max (down-pull = +q)', () => {
    const up = mapGamepadToCommands(pad([0, -1, 0, 0, 0, 0, 0, 0]));
    expect(up.jointVelocities[1]).toBeCloseTo(-DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[1], 6);
    const down = mapGamepadToCommands(pad([0, 1, 0, 0, 0, 0, 0, 0]));
    expect(down.jointVelocities[1]).toBeCloseTo(DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[1], 6);
  });

  it('R-Stick Y full up → Elbow J2 negative; R-Stick X full right → Wrist Roll J3 positive', () => {
    const elbow = mapGamepadToCommands(pad([0, 0, 0, -1, 0, 0, 0, 0]));
    expect(elbow.jointVelocities[2]).toBeCloseTo(-DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[2], 6);
    const roll = mapGamepadToCommands(pad([0, 0, 1, 0, 0, 0, 0, 0]));
    expect(roll.jointVelocities[3]).toBeCloseTo(DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[3], 6);
  });

  it('each axis only moves its own joint', () => {
    const cmd = mapGamepadToCommands(pad([1, 0, 0, 0, 0, 0, 0, 0]));
    expect(cmd.jointVelocities.slice(1)).toEqual([0, 0, 0, 0]);
  });
});

describe('mapGamepadToCommands — bipolar triggers (Wrist Pitch J4)', () => {
  it('RT only → J4 at +max speed (RT − LT)', () => {
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 0, 1]));
    expect(cmd.jointVelocities[4]).toBeCloseTo(DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[4], 6);
  });

  it('LT only → J4 at −max speed', () => {
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 1, 0]));
    expect(cmd.jointVelocities[4]).toBeCloseTo(-DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[4], 6);
  });

  it('both triggers pressed → cancel out to zero', () => {
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 1, 1]));
    expect(cmd.jointVelocities[4]).toBe(0);
  });

  it('partial trigger deflection scales linearly past the deadzone', () => {
    const half = (0.5 - DEFAULT_GAMEPAD_MAPPING.deadzone) / (1 - DEFAULT_GAMEPAD_MAPPING.deadzone);
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 0, 0.5]));
    expect(cmd.jointVelocities[4]).toBeCloseTo(DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[4] * half, 6);
  });
});

describe('mapGamepadToCommands — gripper buttons', () => {
  it('A (open) → negative delta, B (close) → positive delta', () => {
    const open = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 0, 0], [0]));
    expect(open.gripperDeltaPct).toBe(-DEFAULT_GAMEPAD_MAPPING.gripperSpeedPct);
    const close = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 0, 0], [1]));
    expect(close.gripperDeltaPct).toBe(DEFAULT_GAMEPAD_MAPPING.gripperSpeedPct);
  });

  it('A + B together → zero net delta', () => {
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 0, 0, 0, 0, 0], [0, 1]));
    expect(cmd.gripperDeltaPct).toBe(0);
  });

  it('gripper buttons do not affect joint velocities', () => {
    const cmd = mapGamepadToCommands(pad([1, 0, 0, 0, 0, 0, 0, 0], [0]));
    expect(cmd.jointVelocities[0]).toBeCloseTo(DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[0], 6);
  });
});

describe('mapGamepadToCommands — custom config', () => {
  it('respects a smaller deadzone: small deflection now yields motion', () => {
    const config = { ...DEFAULT_GAMEPAD_MAPPING, deadzone: 0.05 };
    const cmd = mapGamepadToCommands(pad([0.1, 0, 0, 0, 0, 0, 0, 0]), undefined);
    const cfgCmd = mapGamepadToCommands(pad([0.1, 0, 0, 0, 0, 0, 0, 0]), config);
    expect(cmd.jointVelocities[0]).toBe(0);
    expect(cfgCmd.jointVelocities[0]).toBeGreaterThan(0);
  });

  it('honors direction inversion per source', () => {
    const config = {
      ...DEFAULT_GAMEPAD_MAPPING,
      analog: DEFAULT_GAMEPAD_MAPPING.analog.map((src, i) =>
        i === 0 ? { ...src, direction: -1 as const } : src,
      ),
    };
    const cmd = mapGamepadToCommands(pad([1, 0, 0, 0, 0, 0, 0, 0]), config);
    expect(cmd.jointVelocities[0]).toBeCloseTo(-DEFAULT_GAMEPAD_MAPPING.jointSpeedRad[0], 6);
  });

  it('keeps working when speed config lacks an entry for a mapped joint', () => {
    const config = { ...DEFAULT_GAMEPAD_MAPPING, jointSpeedRad: [1.2, 0.8] };
    const cmd = mapGamepadToCommands(pad([0, 0, 0, 1, 0, 0, 0, 0]), config);
    expect(cmd.jointVelocities[2]).toBe(0);
  });
});
