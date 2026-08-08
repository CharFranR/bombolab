/**
 * Shared "load gcode text → validate → draw" pipeline (R12).
 *
 * Extracted from App.tsx `handleGcodeFile` so the file-picker path and the
 * CIPRA WebSocket arrival path reuse the exact same logic: safe drawing area →
 * parse → pre-flight reachability gate (via `startTrajectory`) → draw.
 *
 * All side-effects are injected as deps so the module is unit-testable without
 * React, wasm or a network. The workspace-refusal path delegates to
 * `startTrajectory` (it sets the `drawingBlock` panel itself) so NO move is
 * ever commanded for an out-of-reach trajectory. The robot-mode guard is owned
 * by `startTrajectory` — it refuses to run unless `robotMode === 'drawing'`.
 *
 * Plane heights default to the reachability module constants (80/85) but are
 * overridable to keep this module free of the wasm import chain.
 */
import type { MotionCommandJS } from '../motion/commands';
import type { GcodeOptions, GcodeParseResult } from '../motion/gcode';
import type { ErrorCode } from './protocol';

export interface DrawingAreaLike {
  xMin: number;
  xMax: number;
  yMin: number;
  yMax: number;
}

export interface LoadGcodeTextResult {
  ok: boolean;
  reason?: 'no-drawable' | 'blocked';
}

export interface LoadGcodeTextDeps {
  /** Builds the autofit target area for the robot's reachable workspace. */
  safeDrawingArea: () => Promise<DrawingAreaLike>;
  /** Injected from motion/gcode.ts `parseGcode`. */
  parseGcode: (text: string, opts: GcodeOptions) => GcodeParseResult;
  /** Pre-flight gates reachability + `robotMode==='drawing'`; starts playback. */
  startTrajectory: (cmds: MotionCommandJS[], key: string) => Promise<boolean>;
  setValidating: (v: boolean) => void;
  setGcodeError: (e: string | null) => void;
  setGcodeWarnings: (w: string[]) => void;
  setGcodeName: (name: string) => void;
  /** Drawing-plane heights. Defaults to 80/85 (mirror DRAW_PLANE_Z/TRAVEL_PLANE_Z). */
  planeZ?: number;
  travelZ?: number;
}

/** Map a draw-time failure to the canonical error code the publisher
 *  understands (review fix #5). Parse-level failures — a thrown parse/
 *  validation exception or a program with nothing drawable — map to
 *  E_PARSE_GCODE; a workspace/reachability rejection (startTrajectory
 *  refused) maps to E_UNREACHABLE. The ACK already confirmed DELIVERY, so
 *  this code tells the publisher WHY the job could not be drawn. */
export function mapDrawFailureToErrorCode(
  reason: LoadGcodeTextResult['reason'] | 'exception',
): ErrorCode {
  if (reason === 'blocked') return 'E_UNREACHABLE';
  return 'E_PARSE_GCODE';
}

export async function loadGcodeText(
  text: string,
  name: string,
  deps: LoadGcodeTextDeps,
): Promise<LoadGcodeTextResult> {
  const {
    safeDrawingArea,
    parseGcode,
    startTrajectory,
    setValidating,
    setGcodeError,
    setGcodeWarnings,
    setGcodeName,
    planeZ = 80,
    travelZ = 85,
  } = deps;

  setGcodeError(null);
  setGcodeName(name);
  setValidating(true);
  try {
    const area = await safeDrawingArea();
    const result = parseGcode(text, { area, planeZ, travelZ });
    if (result.commands.length === 0) {
      setGcodeError('El archivo no contiene movimientos dibujables (G0/G1 con lápiz).');
      return { ok: false, reason: 'no-drawable' };
    }
    setGcodeWarnings(result.warnings);
    const started = await startTrajectory(result.commands, 'gcode');
    if (!started) return { ok: false, reason: 'blocked' };
    return { ok: true };
  } finally {
    setValidating(false);
  }
}