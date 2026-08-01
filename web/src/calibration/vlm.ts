/**
 * VLM hook — DECISION SUPPORT ONLY. Not called by the deterministic
 * pipeline (calibrate.ts) unless a caller explicitly wires it up.
 *
 * The role of the vision model is strictly semantic: "which candidate is
 * the servo pivot?" — never geometry. All metrics come from the mesh and
 * the solver; the VLM output is a ranking of candidate ids, which the
 * solver then judges.
 *
 * Intended flow (future increment):
 *  1. Render the part (views with candidates drawn + numbered overlay).
 *  2. Send image(s) + candidate table + this prompt to a VLM.
 *  3. Parse { best, alternatives, none } and pass to calibratePart as a
 *     candidate priority hint (still solved and judged by the solver).
 */

import type { CircleCandidate, PartDescriptor } from './types';

export interface VlmCandidateRow {
  id: number;
  radiusMm: number;
  centerMm: [number, number, number];
  note: string;
}

export interface VlmPromptInput {
  part: PartDescriptor;
  candidates: CircleCandidate[];
}

export interface VlmOutputSchema {
  /** The candidate id that is the servo pivot, or null if none fit. */
  best: number | null;
  /** Runner-up candidate ids (for multi-hypothesis solving). */
  alternatives: number[];
  /** Human-readable justification. */
  reason: string;
}

/** Build the candidate table rendered with the views. */
export function candidateTable(input: VlmPromptInput): VlmCandidateRow[] {
  return input.candidates.map((c) => ({
    id: c.id,
    radiusMm: Number(c.radius.toFixed(2)),
    centerMm: [Number(c.center[0].toFixed(1)), Number(c.center[1].toFixed(1)), Number(c.center[2].toFixed(1))],
    note: '',
  }));
}

/** Build the prompt for the VLM (views are attached separately). */
export function vlmPrompt(input: VlmPromptInput): string {
  const rows = candidateTable(input)
    .map((r) => `  ${String.fromCharCode(65 + r.id)}: agujero radio ${r.radiusMm}mm, centro STL (${r.centerMm.join(', ')})mm`)
    .join('\n');
  const jointName =
    input.part.parentJoint === -1 ? 'tool (joint 5)' :
    input.part.parentJoint === 0 ? 'world (J1 axis)' :
    `joint ${input.part.parentJoint}`;
  return [
    `Este STL pertenece al robot FABRI Creator y representa la pieza "${input.part.filename}".`,
    `Debe unirse a ${jointName}.`,
    'Los círculos numerados en la imagen son candidatos a pivote (boca del servo / agujero de montaje).',
    'Respondé SOLO con JSON: { "best": <id|null>, "alternatives": [<id>], "reason": "..." }',
    'Reglas:',
    '  - best = el candidato cuyo agujero es el pivote real del joint (el que el eje del joint debe atravesar).',
    '  - alternatives = otros candidatos plausibles (si los hay), ordenados por probabilidad.',
    '  - best = null solo si NINGÚN candidato puede ser el pivote.',
    'Candidatos:',
    rows,
  ].join('\n');
}

/**
 * Adapter stub — wire to the model provider of choice (OpenAI-compatible
 * chat completions with image_url parts). Returns the validated JSON.
 */
export async function vlmSelect(
  _input: VlmPromptInput,
  _views: unknown[],
  _config: { apiKey: string; model: string; baseUrl?: string },
): Promise<VlmOutputSchema> {
  // Not implemented in this increment. The deterministic pipeline runs
  // without it: every surviving candidate is solved and the solver picks
  // the minimum residual (with ratio < 0.5); true ties are reported as
  // ambiguous and resolved via CLI --pick or the future VLM path.
  throw new Error('vlmSelect is a stub — wire the provider before use');
}
