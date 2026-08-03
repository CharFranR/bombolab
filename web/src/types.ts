/** A single drawing-trajectory step for the FABRI tool-pen (CIPRA G-code).
 *  `q` are joint angles in radians, already resolved by IK (drawing mode)
 *  in the Rust `gcode-bridge` crate. The solver respects the RobotDef's
 *  q_min/q_max, so callers can trust the loaded values as-is. */
export interface TrajectoryStep {
  q: [number, number, number, number, number];
  gripper?: number;
}

/** JSON shape of a trajectory file: `{ steps: TrajectoryStep[] }`. */
export interface TrajectoryFile {
  steps: TrajectoryStep[];
}