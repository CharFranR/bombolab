/**
 * Calibrator CLI — generates calibration.json for the FABRI Creator STLs.
 *
 * Usage (from web/):
 *   bun run scripts/calibrate.ts [--write] [--force] [--part <file>]
 *                                [--pick <file>:<candidateId>] [--json]
 *                                [--stl-dir <dir>] [--cal <path>] [--out <path>]
 *
 * Defaults: stl-dir=public/stl, cal=public/calibration.json,
 * out=public/calibration.generated.json (never touches the real file
 * without --write; --force allows overwriting calibration.json itself).
 *
 * The pipeline is fully deterministic (no AI): candidates are detected on
 * the mesh, pre-filtered by descriptor + joint-axis seed line, solved by
 * the constraint solver, and certified by motion validation. The VLM hook
 * (vlm.ts) is a future increment for ambiguous ties.
 */

import { writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { calibrateAll } from '../src/calibration/calibrate';
import { createContext } from './calibration-io';
import { ALL_STL_FILES } from '../src/renderers/stlMapping';
import type { CalibrationResult } from '../src/calibration/types';

function parseArgs(argv: string[]) {
  const args: Record<string, string> = {};
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a.startsWith('--')) {
      const key = a.slice(2);
      const next = argv[i + 1];
      if (next !== undefined && !next.startsWith('--')) {
        args[key] = next;
        i++;
      } else {
        args[key] = 'true';
      }
    }
  }
  return args;
}

const ROOT = join(import.meta.dir, '..');
const args = parseArgs(process.argv.slice(2));

const stlDir = args['stl-dir'] ? join(ROOT, args['stl-dir']) : join(ROOT, 'public', 'stl');
const calPath = args['cal'] ? join(ROOT, args['cal']) : join(ROOT, 'public', 'calibration.json');
const outPath = args['out'] ? join(ROOT, args['out']) : join(ROOT, 'public', 'calibration.generated.json');

const ctx = createContext(stlDir, calPath);
const only = args['part'] ?? undefined;

let output;
if (only) {
  output = calibrateAll(ctx, { only });
} else {
  output = calibrateAll(ctx);
}

// Apply --pick to re-run the chosen part with a forced candidate.
const pick = args['pick'];
if (pick && only) {
  const [file, idStr] = pick.split(':');
  if (file && idStr && file === only) {
    const { calibratePart } = await import('../src/calibration/calibrate');
    const { descriptorFor } = await import('../src/calibration/descriptors');
    const part = descriptorFor(file);
    if (part) {
      const res = calibratePart(part, ctx, { pickId: Number(idStr) });
      output.results = output.results.map((r) => (r.filename === file ? res : r));
      output.config.entries = output.config.entries.map((e) =>
        e.filename === file && res.cal ? { filename: e.filename, translation: res.cal!.translation, rotation: res.cal!.rotation } : e,
      );
    }
  }
}

// Apply --base-anchor "x,z": shift the world-fixed Base so the anchor point
// (the hole the user picked in the simulator) lands on the J1 axis (0,0).
// This is the chain-coherence fix: the base mates the visual yaw axis with
// the model axis (0,57,0) that Eje Central is aligned to.
const baseAnchor = args['base-anchor'];
if (baseAnchor) {
  const [ax, az] = baseAnchor.split(',').map(Number);
  if (Number.isFinite(ax) && Number.isFinite(az)) {
    const entry = output.config.entries.find((e) => e.filename === 'Base.stl');
    if (entry) {
      const dx = -ax;
      const dz = -az;
      entry.translation = [entry.translation[0] + dx, entry.translation[1], entry.translation[2] + dz];
      const res = output.results.find((r) => r.filename === 'Base.stl');
      if (res) res.reason = `base anchor (${ax}, ${az}) → shifted (${dx.toFixed(2)}, 0, ${dz.toFixed(2)})mm to center the hole on the J1 axis`;
      console.log(`\nBase anchor applied: hole at (${ax}, ${az}) world → shifted by (${dx.toFixed(2)}, 0, ${dz.toFixed(2)})mm`);
    }
  } else {
    console.error('Invalid --base-anchor (expected "x,z")');
  }
}

function fmtResult(r: CalibrationResult): string {
  const lines = [`${r.status.toUpperCase().padEnd(9)} ${r.filename}`];
  if (r.candidates.length > 0) {
    lines.push(`  candidates detected : ${r.candidates.length} (pre-filter → ${r.pivot ? `#${r.pivot.id} r=${r.pivot.radius.toFixed(2)}mm residual=${r.residualMm?.toFixed(3) ?? '?'}mm` : 'none'})`);
  }
  if (r.motion) {
    lines.push(`  motion validation  : max drift ${r.motion.maxDriftMm.toFixed(4)}mm / RMS ${r.motion.rmsDriftMm.toFixed(4)}mm over ${r.motion.nPoses} poses → ${r.motion.passed ? 'PASS' : 'FAIL'}`);
  }
  if (r.deltaTranslationMm !== undefined) {
    lines.push(`  delta vs current    : Δt=${r.deltaTranslationMm.toFixed(3)}mm  Δrot=${r.deltaRotationDeg?.toFixed(3)}°`);
  }
  if (r.cal) {
    const t = r.cal.translation.map((v) => v.toFixed(4));
    const q = r.cal.rotation.map((v) => v.toFixed(6));
    lines.push(`  solved cal          : t=[${t.join(', ')}] q=[${q.join(', ')}]`);
  }
  if (r.reason) lines.push(`  reason              : ${r.reason}`);
  return lines.join('\n');
}

const statuses = output.results.reduce<Record<string, number>>((acc, r) => {
  acc[r.status] = (acc[r.status] ?? 0) + 1;
  return acc;
}, {});

if (args['json']) {
  console.log(JSON.stringify(output, null, 2));
} else {
  console.log('=== FABRI Creator STL calibration ===');
  for (const r of output.results) console.log(fmtResult(r));
  console.log('-------------------------------------');
  console.log(`status: ${Object.entries(statuses).map(([k, v]) => `${k}=${v}`).join(' ')}`);
  console.log(`stlDir=${stlDir}`);
  console.log(`current=${calPath}`);
}

// Dump all detected candidates per part for the in-simulator overlay.
if (args['candidates']) {
  const { detectCircles } = await import('../src/calibration/mesh');
  const { loadMeshFromBuffer } = await import('../src/calibration/mesh');
  const { descriptorFor } = await import('../src/calibration/descriptors');
  const parts = [];
  for (const filename of ALL_STL_FILES) {
    const part = descriptorFor(filename);
    if (!part || part.parentJoint === 0) continue; // world parts: no candidates used
    const buffer = ctx.loader(filename);
    const mesh = loadMeshFromBuffer(buffer);
    const circles = detectCircles(mesh, 1.5, 40);
    parts.push({ filename, candidates: circles.map((c) => ({ id: c.id, center: c.center, radius: Number(c.radius.toFixed(2)), normal: c.normal })) });
  }
  const outPath2 = args['out-candidates'] ? join(ROOT, args['out-candidates']) : join(ROOT, 'public', 'calibration-candidates.json');
  writeFileSync(outPath2, JSON.stringify({ version: 1, parts }, null, 2) + '\n', 'utf-8');
  console.log(`\nWrote candidates ${outPath2}`);
}

if (args['write']) {
  const target = args['force'] ? calPath : outPath;
  writeFileSync(target, JSON.stringify(output.config, null, 2) + '\n', 'utf-8');
  console.log(`\nWrote ${target}${args['force'] ? ' (--force: overwrote current calibration.json)' : ' (use --force to overwrite calibration.json)'}`);
}
