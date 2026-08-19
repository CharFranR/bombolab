import { describe, expect, it } from 'vitest';
import type { MotionCommandJS } from './commands';
import {
  compensateMotionCommands,
  compensateTarget,
  determinant2x2,
  isWithinCalibrationRegion,
  X_MAX,
  X_MIN,
  Y_MAX,
  Y_MIN,
  invertAffineMatrix,
  loadPostIkCalibration,
  validatePostIkCalibration,
  velocityCompensationFactor,
  type PostIkCalibration,
} from './postIkCalibration';

function calibration(overrides: Partial<PostIkCalibration> = {}): PostIkCalibration {
  return {
    version: 1,
    backlash_offset_x_mm: 0,
    k_velocity_factor: 0,
    calibration_points: [
      {
        center_x: 0,
        center_y: 0,
        a11: 1,
        a12: 0,
        a21: 0,
        a22: 1,
        b_x: 0,
        b_y: 0,
      },
    ],
    ...overrides,
  };
}

describe('post-IK calibration', () => {
  it('loads and validates the version, numeric fields, and positive determinants', async () => {
    const loaded = await loadPostIkCalibration('/post-ik-calibration.json', async () => ({
      ok: true,
      status: 200,
      json: async () => calibration(),
    }) as Response);
    expect(loaded.version).toBe(1);
    expect(() => validatePostIkCalibration({ ...loaded, version: 2 })).toThrow(/version/);
    expect(() => validatePostIkCalibration({
      ...loaded,
      calibration_points: [{ ...loaded.calibration_points[0], a22: -1 }],
    })).toThrow(/determinant/);
  });

  it('inverts a positive-determinant 2x2 affine matrix', () => {
    const matrix = { a11: 2, a12: 1, a21: 1, a22: 1 };
    const inverse = invertAffineMatrix(matrix);
    expect(determinant2x2(matrix)).toBe(1);
    expect(inverse).toEqual({ a11: 1, a12: -1, a21: -1, a22: 2 });
  });

  it('averages duplicate exact centers instead of selecting the first point', () => {
    const cal = calibration({
      calibration_points: [
        { center_x: 260, center_y: -45, a11: 1, a12: 0, a21: 0, a22: 1, b_x: 10, b_y: 0 },
        { center_x: 260, center_y: -45, a11: 1, a12: 0, a21: 0, a22: 1, b_x: 20, b_y: 0 },
      ],
    });
    expect(compensateTarget([260, -45, 80], cal)).toEqual([245, -45, 80]);
  });

  it('inverts the recentralized model correctly when A differs from identity (regression)', () => {
    // Calibration with a realistic scale error (the robot lands ~5% short in
    // both axes), recentralized with b = center − A·center so the center is
    // exactly fixed. The center must map to itself, and a point 40mm east of
    // the center must be commanded 40/0.95 mm away. The pre-fix code computed
    // A⁻¹·target − b and broke both invariants whenever A ≠ I.
    const center: [number, number] = [260, -45];
    const scale = 0.95;
    const cal = calibration({
      calibration_points: [{
        center_x: center[0],
        center_y: center[1],
        a11: scale,
        a12: 0,
        a21: 0,
        a22: scale,
        b_x: center[0] - scale * center[0],
        b_y: center[1] - scale * center[1],
      }],
    });

    // The recentralized center is an exact fixed point.
    expect(compensateTarget([center[0], center[1], 80], cal)).toEqual([center[0], center[1], 80]);

    // A nominal point 40mm east must be commanded 40/scale mm east of center.
    const east: [number, number, number] = [center[0] + 40, center[1], 80];
    const corrected = compensateTarget(east, cal);
    expect(corrected[0]).toBeCloseTo(center[0] + 40 / scale, 6);
    expect(corrected[1]).toBeCloseTo(center[1], 6);
  });

  it('exposes the conservative rectangular calibration region', () => {
    expect({ X_MIN, X_MAX, Y_MIN, Y_MAX }).toEqual({ X_MIN: 210, X_MAX: 310, Y_MIN: -70, Y_MAX: -20 });
    expect(isWithinCalibrationRegion(X_MIN, Y_MIN)).toBe(true);
    expect(isWithinCalibrationRegion(X_MAX, Y_MAX)).toBe(true);
    expect(isWithinCalibrationRegion(X_MIN - 1, -45)).toBe(false);
    expect(isWithinCalibrationRegion(260, Y_MIN - 1)).toBe(false);
  });

  it('leaves external targets identical to their original coordinates', () => {
    const cal = calibration({
      backlash_offset_x_mm: 5,
      k_velocity_factor: 0.001,
      calibration_points: [{
        center_x: 260,
        center_y: -45,
        a11: 1,
        a12: 0,
        a21: 0,
        a22: 1,
        b_x: 10,
        b_y: -3,
      }],
    });
    const target: MotionCommandJS = { type: 'move', target: [160, -30, 80], speed: 30 };
    expect(compensateTarget(target.target, cal)).toEqual(target.target);
    expect(compensateMotionCommands([target], cal, { start: [260, -45, 80] })).toEqual([target]);
  });

  it('compensates targets inside the calibration region', () => {
    const cal = calibration({
      calibration_points: [{
        center_x: 260,
        center_y: -45,
        a11: 1,
        a12: 0,
        a21: 0,
        a22: 1,
        b_x: 2,
        b_y: -3,
      }],
    });
    expect(compensateTarget([260, -45, 80], cal)).toEqual([258, -42, 80]);
  });

  it('resets X backlash direction when crossing out of the calibration region', () => {
    const commands: MotionCommandJS[] = [
      { type: 'move', target: [270, -45, 80], speed: 20 },
      { type: 'move', target: [200, -45, 80], speed: 20 },
      { type: 'move', target: [220, -45, 80], speed: 20 },
    ];
    const result = compensateMotionCommands(commands, calibration({ backlash_offset_x_mm: 5 }), {
      start: [260, -45, 80],
    });
    expect(result).toEqual([
      { type: 'move', target: [270, -45, 80], speed: 20 },
      { type: 'move', target: [200, -45, 80], speed: 20 },
      { type: 'move', target: [220, -45, 80], speed: 20 },
    ]);
  });

  it('does not compensate a segment entering the valid region from outside', () => {
    const cal = calibration({
      calibration_points: [{
        center_x: 260,
        center_y: -45,
        a11: 1,
        a12: 0,
        a21: 0,
        a22: 1,
        b_x: 20,
        b_y: -10,
      }],
    });
    const commands: MotionCommandJS[] = [
      { type: 'move', target: [200, -45, 80], speed: 20 },
      { type: 'move', target: [260, -45, 80], speed: 20 },
    ];
    expect(compensateMotionCommands(commands, cal, { start: [200, -45, 80] })).toEqual(commands);
  });

  it('preserves pen, wait, and their ordering while compensating moves', () => {
    const commands: MotionCommandJS[] = [
      { type: 'penDown' },
      { type: 'move', target: [10, 20, 80], speed: 20 },
      { type: 'wait', duration: 0.5 },
      { type: 'penUp' },
    ];
    expect(compensateMotionCommands(commands, calibration(), { start: [0, 0, 80] })).toEqual(commands);
  });

  it('applies X backlash only when the movement direction reverses', () => {
    const commands: MotionCommandJS[] = [
      { type: 'move', target: [270, -45, 80], speed: 20 },
      { type: 'penUp' },
      { type: 'wait', duration: 1 },
      { type: 'move', target: [260, -45, 80], speed: 20 },
      { type: 'move', target: [270, -45, 80], speed: 20 },
    ];
    const result = compensateMotionCommands(commands, calibration({ backlash_offset_x_mm: 5 }), { start: [260, -45, 80] });
    expect(result[0]).toEqual({ type: 'move', target: [270, -45, 80], speed: 20 });
    expect(result[3]).toEqual({ type: 'move', target: [255, -45, 80], speed: 20 });
    expect(result[4]).toEqual({ type: 'move', target: [275, -45, 80], speed: 20 });
  });

  it('converts mm/s to mm/min before applying the velocity factor', () => {
    const cal = calibration({ k_velocity_factor: 0.001 });
    expect(velocityCompensationFactor(30, cal)).toBeCloseTo(1.6);
    const result = compensateMotionCommands(
      [{ type: 'move', target: [260, -45, 80], speed: 30 }],
      cal,
      { start: [250, -45, 80] },
    );
    expect(result[0]).toEqual({ type: 'move', target: [266, -45, 80], speed: 30 });
  });

  it('changes the target used by the integrated command transform', () => {
    const cal = calibration({
      calibration_points: [{
        center_x: 260,
        center_y: -45,
        a11: 1,
        a12: 0,
        a21: 0,
        a22: 1,
        b_x: 2,
        b_y: -3,
      }],
    });
    const result = compensateMotionCommands(
      [{ type: 'move', target: [260, -45, 80], speed: 20 }],
      cal,
      { start: [250, -45, 80] },
    );
    expect(result[0]).toEqual({ type: 'move', target: [258, -42, 80], speed: 20 });
  });
});
