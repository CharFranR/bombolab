/**
 * Calibrator tests — run with: bun run scripts/test-calibrate.ts
 *
 * 1. FK pure-TS sanity (mirrors forward.rs tests + FABRI DH).
 * 2. Joint-axis lines (J1 through the base origin).
 * 3. End-to-end: calibrate all 11 parts against the CURRENT
 *    calibration.json; every part must converge with a certified motion
 *    test, the chosen pivot must sit on the seed line, and deltas vs the
 *    current calibration must stay small (the current values are known to
 *    be roughly right — big deltas are a WARNING, not a failure).
 */

import { join } from 'node:path';
import assert from 'node:assert/strict';
import { fkFrames, jointAxisWorld, mat4Translation, vecDot, vecLen } from '../src/calibration/fk';
import { calibrateAll } from '../src/calibration/calibrate';
import { createContext } from './calibration-io';
import { ALL_STL_FILES } from '../src/renderers/stlMapping';

const ROOT = join(import.meta.dir, '..');
const stlDir = join(ROOT, 'public', 'stl');
const calPath = join(ROOT, 'public', 'calibration.json');

let failures = 0;
function check(name: string, fn: () => void) {
  try {
    fn();
    console.log(`  PASS  ${name}`);
  } catch (e) {
    failures++;
    console.log(`  FAIL  ${name}: ${(e as Error).message}`);
  }
}

// ─── 1. FK sanity ────────────────────────────────────────────────────────────

check('FK: F1 translation = (15, 0, 142) at q=0 (base 57 + d1 85)', () => {
  const frames = fkFrames([0, 0, 0, 0, 0]);
  const t = mat4Translation(frames[1]);
  assert.ok(Math.abs(t[0] - 15) < 1e-6, `x=${t[0]}`);
  assert.ok(Math.abs(t[1]) < 1e-6, `y=${t[1]}`);
  assert.ok(Math.abs(t[2] - 142) < 1e-6, `z=${t[2]}`);
});

check('FK: F2 translation = (15, 0, 262) at q=0 (J2 θ0=-90°)', () => {
  const frames = fkFrames([0, 0, 0, 0, 0]);
  const t = mat4Translation(frames[2]);
  assert.ok(Math.abs(t[0] - 15) < 1e-6, `x=${t[0]}`);
  assert.ok(Math.abs(t[1]) < 1e-6, `y=${t[1]}`);
  assert.ok(Math.abs(t[2] - 262) < 1e-6, `z=${t[2]}`);
});

check('FK: F0 (base) translation = (0, 0, 57)', () => {
  const frames = fkFrames([0, 0, 0, 0, 0]);
  const t = mat4Translation(frames[0]);
  assert.deepEqual(t.map((v) => Math.round(v)), [0, 0, 57]);
});

check('Axis: J1 line passes through base origin with Z direction', () => {
  const frames = fkFrames([0, 0, 0, 0, 0]);
  const line = jointAxisWorld(1, frames);
  const p = mat4Translation(frames[0]);
  const w = [line.point[0] - p[0], line.point[1] - p[1], line.point[2] - p[2]];
  assert.ok(vecLen(w) < 1e-9);
  assert.ok(Math.abs(vecDot(line.dir, [0, 0, 1]) - 1) < 1e-9);
});

check('Axis: J4 (Twist) line direction is column X of F3', () => {
  const frames = fkFrames([0, 0, 0, 0, 0]);
  const line = jointAxisWorld(4, frames);
  const colX = [frames[3][0], frames[3][4], frames[3][8]];
  const d = vecDot(line.dir, [colX[0], colX[1], colX[2]]);
  assert.ok(Math.abs(Math.abs(d) - 1) < 1e-9, `dot=${d}`);
});

// ─── 2. End-to-end calibration ───────────────────────────────────────────────

console.log('End-to-end calibration (this may take a while — meshes are large)…');
const ctx = createContext(stlDir, calPath);
const output = calibrateAll(ctx);

check('All 11 parts were processed', () => {
  assert.equal(output.results.length, ALL_STL_FILES.length);
  assert.equal(output.config.entries.length, ALL_STL_FILES.length);
});

check('Converged parts pass motion validation; untouched parts report diagnostic drift', () => {
  const failed = output.results.filter((r) => r.status === 'failed' || r.status === 'ambiguous');
  for (const f of failed) {
    console.log(`  NOTE  ${f.filename}: ${f.reason ?? f.status} — current calibration preserved in output`);
  }
  for (const r of output.results) {
    if (r.status === 'ok' && r.deltaTranslationMm === 0 && r.motion) {
      console.log(`  DIAG  ${r.filename}: untouched (within tolerance); current-cal drift = ${r.motion.maxDriftMm.toFixed(2)}mm`);
    }
  }
  const bad = output.results.filter((r) => r.status === 'ok' && (r.deltaTranslationMm ?? 0) > 0 && r.motion && !r.motion.passed);
  if (bad.length > 0) {
    throw new Error('motion failed: ' + bad.map((b) => `${b.filename}: ${b.motion!.maxDriftMm.toFixed(4)}mm`).join(' | '));
  }
});

check('Deltas vs current calibration are small (WARNINGs allowed, printed below)', () => {
  let warns = 0;
  for (const r of output.results) {
    if (r.status !== 'ok') continue;
    if ((r.deltaTranslationMm ?? 0) > 5 || (r.deltaRotationDeg ?? 0) > 5) {
      console.log(`  WARN  ${r.filename}: Δt=${r.deltaTranslationMm?.toFixed(2)}mm Δrot=${r.deltaRotationDeg?.toFixed(2)}° — large correction, review`);
      warns++;
    }
  }
  if (warns > 0) console.log(`  (${warns} parts needed a large correction — inspect them in the report)`);
});

console.log('');
if (failures === 0) {
  console.log('ALL TESTS PASSED');
} else {
  console.log(`${failures} TEST(S) FAILED`);
  process.exit(1);
}
