/**
 * IO helpers for the calibrator CLI (Node-side only — keeps src/ pure).
 */

import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import type { CalibrationContext, RigidTransform } from '../src/calibration/types';

export function loadCurrentCalibration(filePath: string): { entries: Map<string, RigidTransform>; stlScale: number } {
  const raw = JSON.parse(readFileSync(filePath, 'utf-8')) as {
    version?: number;
    stlScale?: number;
    entries: { filename: string; translation: number[]; rotation: number[] }[];
  };
  const entries = new Map<string, RigidTransform>();
  for (const e of raw.entries) {
    entries.set(e.filename, {
      translation: [e.translation[0], e.translation[1], e.translation[2]],
      rotation: [e.rotation[0], e.rotation[1], e.rotation[2], e.rotation[3]],
    });
  }
  return { entries, stlScale: raw.stlScale ?? 1 };
}

export function createContext(stlDir: string, calPath: string): CalibrationContext {
  const { entries, stlScale } = loadCurrentCalibration(calPath);
  return {
    loader: (filename: string) => {
      const buf = readFileSync(join(stlDir, filename));
      return buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength) as ArrayBuffer;
    },
    currentCal: entries,
    stlScale,
    meshCache: new Map(),
  };
}
