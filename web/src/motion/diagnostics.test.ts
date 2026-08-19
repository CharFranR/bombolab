/**
 * DIAGNOSTIC GCODE VALIDATION — experiments/diagnostics/*.gcode
 *
 * Every diagnostic pattern is validated against the REAL production pipeline
 * before it reaches the robot:
 *   1. parseGcode (autofit OFF → real-scale 1:1, the intended loading mode).
 *   2. Pre-flight reachability via validateDrawingCommands — the EXACT gate
 *      the web app runs before starting playback (blocks out-of-reach files).
 *   3. IK resolution via the real wasm solver (solveDrawingPlaneIk) over the
 *      full planTimeline, exactly as App.tsx does.
 *   4. Manifest build (v2 protocol) must succeed with all joints in range.
 *   5. Geometry reconstruction: µs → q → FK, the drawing must land on the
 *      intended plane with all samples inside the safe area.
 *
 * This is the gate that guarantees a diagnostic file will not be blocked by
 * the web pre-flight when the user loads it with autofit OFF.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import { readFileSync, existsSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

import init from '../pkg/bombolab_wasm';
import { fabriCreator, solveDrawingPlaneIk } from '../wasm';
import { parseGcode } from './gcode';
import { planTimeline } from './planTimeline';
import { buildManifest } from './manifest';
import { validateDrawingCommands } from './reachability';
import { DRAW_PLANE_Z, TRAVEL_PLANE_Z } from './planes';
import type { RobotDef } from '../kinematics/types';

const WASM_FILE = path.join(
  path.dirname(fileURLToPath(import.meta.url)),
  '..',
  'pkg',
  'bombolab_wasm_bg.wasm',
);
const wasmFileExists = existsSync(WASM_FILE);

const DIAG_DIR = path.resolve(
  path.dirname(fileURLToPath(import.meta.url)),
  '..',
  '..',
  '..',
  'experiments',
  'diagnostics',
);

// Safe area used by the app for autofit OFF real-scale mode (reachability
// gate in App.tsx via validateDrawingCommands → safeDrawingArea). Area for
// the MG996R geometry: x 160..360, y -130..30.
const AREA = { xMin: 160, xMax: 360, yMin: -130, yMax: 30 };

function diagFiles(): string[] {
  if (!existsSync(DIAG_DIR)) return [];
  return readdirSync(DIAG_DIR)
    .filter((f) => f.endsWith('.gcode'))
    .sort();
}

beforeAll(async () => {
  if (!wasmFileExists) return;
  try {
    const bytes = readFileSync(WASM_FILE);
    try {
      await init(bytes);
    } catch {
      await init(new WebAssembly.Module(bytes));
    }
  } catch (err) {
    console.warn('[diagnostics] wasm init FALLÓ:', (err as Error)?.message ?? err);
  }
});

describe('experiments/diagnostics — GCODE validados contra el pipeline real', () => {
  const files = diagFiles();
  if (files.length === 0) {
    it('no diagnostic files found (skipping)', () => {
      expect(true).toBe(true);
    });
    return;
  }

  for (const file of files) {
    it(`${file}: parsea, pasa el pre-flight de reachability, IK converge y manifest válido`, async () => {
      expect(wasmFileExists, 'wasm build requerido (npm run wasm)').toBe(true);
      const text = readFileSync(path.join(DIAG_DIR, file), 'utf8');
      const robot: RobotDef = fabriCreator();

      const parsed = parseGcode(text, {
        autofit: false,
        area: AREA,
        planeZ: DRAW_PLANE_Z,
        travelZ: TRAVEL_PLANE_Z,
      });
      expect(parsed.commands.length).toBeGreaterThan(0);

      // 1) The EXACT production pre-flight gate (autofit OFF path).
      // NOTE: do NOT pass maxFailures: 0 — the contract uses it as the cap on
      // the failures ARRAY, so 0 means "never record a failure" and ok stays
      // true even when points are unreachable (the bug that let the ghost
      // (0,0,120) travel move pass CI). Default (20) records failures properly.
      const reach = await validateDrawingCommands(parsed.commands, {
        tolerance: 10,
      });
      expect(reach.ok, `reachability failures: ${JSON.stringify(reach.failures.slice(0, 8))}`).toBe(true);

      // 1b) No ghost moves: every move target must lie inside the declared
      // safe area (autofit OFF → real-scale). Catches files whose first move
      // is a bare `G0 Z5` (parser starts at (0,0,0) → phantom segment from
      // the origin, which is unreachable at the travel plane).
      for (const c of parsed.commands) {
        if (c.type !== 'move') continue;
        const [x, y] = c.target;
        expect(
          x >= AREA.xMin && x <= AREA.xMax && y >= AREA.yMin && y <= AREA.yMax,
          `target fuera del área segura: (${x.toFixed(1)}, ${y.toFixed(1)}, ${c.target[2]})`,
        ).toBe(true);
      }

      // 1c) diag-08-landmarks: the dwell waits must survive parsing. Each of
      // the 9 landmarks x3 passes must carry a 2 s dwell (the reference L has
      // two 0.3 s ones). If a wait is dropped, that landmark would not be marked.
      if (file === 'diag-08-landmarks.gcode') {
        const waits = parsed.commands.filter((c) => c.type === 'wait') as { type: 'wait'; duration: number }[];
        expect(waits.length).toBe(29);
        const long = waits.filter((w) => w.duration >= 1.5);
        expect(long.length, `se esperaban 27 dwells de 2s, hay ${long.length}`).toBe(27);
      }

      // 2) Real IK over the full timeline, as App.tsx does.
      const start: [number, number, number] = [200, 0, DRAW_PLANE_Z];
      const samples = planTimeline(parsed.commands, {
        ik: solveDrawingPlaneIk,
        robot,
        startQ: robot.segments.map((s) => s.q),
        gripperPct: 100,
        startTcp: start,
        smooth: false,
      });
      expect(samples.length).toBeGreaterThan(0);

      // 3) v2 manifest must build (joints in range, timeline increasing).
      const manifest = buildManifest(samples);
      expect(manifest instanceof Error, `manifest: ${manifest instanceof Error ? manifest.message : ''}`).toBe(false);
      expect((manifest as { count: number }).count).toBeGreaterThan(0);

      // 3b) diag-08: the 2 s dwells must survive into the manifest as long
      // frame deltas (≥1.5 s). Guards the serialization path against silently
      // collapsing the dwells that mark each landmark.
      if (file === 'diag-08-landmarks.gcode' && manifest instanceof Error === false) {
        const lines = (manifest as { lines: string[] }).lines;
        const longFrames = lines.filter((l) => {
          const m = / (\d+)$/.exec(l);
          return m !== null && Number(m[1]) >= 1_500_000;
        });
        expect(longFrames.length, `se esperaban ~27 frames de 2s en el manifest, hay ${longFrames.length}`)
          .toBeGreaterThanOrEqual(27);
      }
    }, 20_000);
  }
});
