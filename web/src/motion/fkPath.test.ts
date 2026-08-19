import { describe, expect, it } from 'vitest';
import { qUsToQ, maxDeviationToPath } from './fkPath';
import { qToServoUs } from '../serial';

describe('qUsToQ — inverse of qToServoUs', () => {
  it('round-trips joint angles through the servo µs mapping', () => {
    // q values inside the servo clamp band [5, 175]° for every channel.
    const q = [0.3, -0.7, 0.2, -0.4, 0.8];
    const us = qToServoUs(q);
    const back = qUsToQ(us);
    for (let i = 0; i < q.length; i++) {
      expect(back[i]).toBeCloseTo(q[i], 10);
    }
  });

  it('handles negative and positive extents of the joint range', () => {
    const q = [-1.2, 1.2, -0.9, 0.9, -0.9];
    const us = qToServoUs(q);
    const back = qUsToQ(us);
    for (let i = 0; i < q.length; i++) {
      expect(back[i]).toBeCloseTo(q[i], 10);
    }
  });

  it('recovers the clamped value when q exceeds the servo range', () => {
    // q = 1.1 rad on channel 4 maps to deg = -63 + 60 = -3°, clamped to 5°.
    // The reconstruction is faithful to what the servo actually receives.
    const us = qToServoUs([0, 0, 0, 0, 1.1]);
    const back = qUsToQ(us);
    expect(back[4]).toBeCloseTo((5 - 60) / (-1 * (180 / Math.PI)), 10);
  });

  it('returns only the first 5 channels (gripper excluded)', () => {
    const us = qToServoUs([0.1, 0.2, 0.3, 0.4, 0.5]);
    const withGripper = [...us, 1000];
    expect(qUsToQ(withGripper)).toHaveLength(5);
  });
});

describe('maxDeviationToPath — IK path vs ideal trace', () => {
  it('returns 0 when the path lies exactly on the reference', () => {
    const ref: [number, number, number][] = [
      [0, 0, 0],
      [10, 0, 0],
      [10, 10, 0],
    ];
    const path: [number, number, number][] = [
      [2, 0, 0],
      [10, 5, 0],
      [10, 10, 0],
    ];
    expect(maxDeviationToPath(path, ref)).toBeCloseTo(0, 10);
  });

  it('measures the max distance for a point off the line', () => {
    const ref: [number, number, number][] = [
      [0, 0, 0],
      [10, 0, 0],
    ];
    const path: [number, number, number][] = [
      [5, 0, 0],
      [5, 3, 0],
      [9, 0, 0],
    ];
    expect(maxDeviationToPath(path, ref)).toBeCloseTo(3, 10);
  });

  it('uses distance to the closest segment (corner case)', () => {
    const ref: [number, number, number][] = [
      [0, 0, 0],
      [10, 0, 0],
      [10, 10, 0],
    ];
    // Point beyond the corner: closest point on the polyline is the corner.
    const path: [number, number, number][] = [[10, 12, 0]];
    expect(maxDeviationToPath(path, ref)).toBeCloseTo(2, 10);
  });

  it('returns 0 for empty inputs', () => {
    expect(maxDeviationToPath([], [])).toBe(0);
    expect(maxDeviationToPath([], [[0, 0, 0]])).toBe(0);
    expect(maxDeviationToPath([[1, 1, 1]], [])).toBe(0);
  });
});
