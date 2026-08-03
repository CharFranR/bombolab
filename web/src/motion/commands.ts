/**
 * Motion command types for the trajectory layer (TS side of the wasm
 * motion player). Mirror of the Rust `trajectory::MotionCommand`.
 *
 * Providers (app layer) emit these; the core planner adapts them; the
 * core player executes them with frame deltas. The robot mode and the
 * hardware controller decide what a command means for the physical arm.
 */

/** Serialized form understood by `motion_player_new` (wasm). */
export type MotionCommandJS =
  | { type: 'move'; target: [number, number, number]; speed: number }
  | { type: 'penUp' }
  | { type: 'penDown' }
  | { type: 'wait'; duration: number };

/**
 * 5×5 cm square demo centered at (cx, cy) on the drawing plane z.
 *
 * The first command is PenDown: semantic intent ("start drawing"); the
 * planner materializes it (slice 1: pass-through, the marker is always
 * down while holding the pen). `speed` in mm/s.
 */
export function squareCommands(
  cx = 200,
  cy = 0,
  z = 80,
  half = 25,
  speed = 40,
): MotionCommandJS[] {
  const corners: [number, number, number][] = [
    [cx - half, cy - half, z],
    [cx + half, cy - half, z],
    [cx + half, cy + half, z],
    [cx - half, cy + half, z],
  ];
  return [
    { type: 'penDown' },
    ...corners.map((target) => ({ type: 'move' as const, target, speed })),
  ];
}

/**
 * Extract the drawing path (moves at the drawing plane z) from a command
 * list — the shape each demo will draw. Used for the 3D preview trace.
 */
export function drawingPath(commands: MotionCommandJS[], z = 80): [number, number, number][] {
  const pts: [number, number, number][] = [];
  for (const c of commands) {
    if (c.type === 'move' && Math.abs(c.target[2] - z) < 1) pts.push(c.target);
  }
  return pts;
}

/**
 * Diagnostic line grid: horizontal and vertical lines with the pen lifted
 * (z + 5 mm) between them. Many more segments than the square, so error
 * patterns become visible immediately: does the step appear only at
 * direction reversals? near joint extremes? is it speed-dependent?
 */
export function diagnosticLinesCommands(z = 80, speed = 40): MotionCommandJS[] {
  const lift = z + 5;
  const cmds: MotionCommandJS[] = [];

  // Horizontal lines: y = -30..30 step 10, x 160 → 240
  for (let y = -30; y <= 30; y += 10) {
    const start: [number, number, number] = [160, y, z];
    const end: [number, number, number] = [240, y, z];
    if (cmds.length > 0) {
      cmds.push({ type: 'move', target: [160, y, lift], speed: 60 }); // travel with pen up
    }
    cmds.push({ type: 'penDown' });
    cmds.push({ type: 'move', target: start, speed });
    cmds.push({ type: 'move', target: end, speed });
  }

  // Vertical lines: x = 165..225 step 15, y -35 → 35
  for (let x = 165; x <= 225; x += 15) {
    const start: [number, number, number] = [x, -35, z];
    const end: [number, number, number] = [x, 35, z];
    cmds.push({ type: 'move', target: [x, -35, lift], speed: 60 }); // travel with pen up
    cmds.push({ type: 'penDown' });
    cmds.push({ type: 'move', target: start, speed });
    cmds.push({ type: 'move', target: end, speed });
  }

  return cmds;
}

/**
 * Base-centered arc: J1 sweeps `sweepDeg` monotonically (no reversal), J2/J3
 * stay nearly constant. This separates the two escalón components: if the
 * arc shows a REGULAR staircase, the 1°-integer wire granularity dominates
 * (fixable with a µs-pulse protocol); if it is clean, the play at direction
 * reversals dominates (the square's corners are the worst case).
 *
 * CONSTRAINT: J1's servo range [5,175] with direction −1 gives q1 ∈ [−85°, 85°]
 * → the arc must stay inside θ ∈ [−85°, 85°]. The defaults sweep −70°→+70°
 * (verified: 0 solver failures at radius 180 / z=80).
 */
export function arcCommands(
  radius = 180,
  startDeg = -70,
  sweepDeg = 140,
  z = 80,
  speed = 40,
): MotionCommandJS[] {
  const cmds: MotionCommandJS[] = [{ type: 'penDown' }];
  for (let d = 0; d <= sweepDeg; d += 1) {
    const th = ((startDeg + d) * Math.PI) / 180;
    cmds.push({
      type: 'move',
      target: [radius * Math.cos(th), radius * Math.sin(th), z],
      speed,
    });
  }
  return cmds;
}
