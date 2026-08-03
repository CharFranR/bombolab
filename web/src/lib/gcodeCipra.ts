// G-Code handling for the CIPRA dialect, ported from the Rust `gcode-bridge`
// crate so the browser and the crate agree on the exact same algorithm and
// defaults (parser, workspace bounds, auto-scaling, mapping).
//
// CIPRA emits a purely geometric dialect over an A4 portrait plane
// (210×297 mm): G21 (mm), G90 (absolute), G0/G1 (rapid/draw with X Y), M3
// (pen down), M5 (pen up).

export type GPoint = [number, number];

/** Target drawing rectangle in robot millimetres. */
export interface DrawingBounds {
  x_min: number;
  x_max: number;
  y_min: number;
  y_max: number;
}

export interface MappingConfig {
  target: DrawingBounds;
  z_draw: number;
  z_travel: number;
  /** Optional explicit scale; `null` → auto-scale to fit the target. */
  scale: number | null;
}

// ─── Workspace (mirrors crate workspace.rs) ─────────────────────────────────

export function defaultForFabri(): DrawingBounds {
  return { x_min: 150, x_max: 250, y_min: -50, y_max: 50 };
}

export function defaultMapping(): MappingConfig {
  return { target: defaultForFabri(), z_draw: 80, z_travel: 86, scale: null };
}

function boundsWidth(b: DrawingBounds): number {
  return b.x_max - b.x_min;
}
function boundsHeight(b: DrawingBounds): number {
  return b.y_max - b.y_min;
}

/** Scale that fits a `w`×`h` box inside `b`, preserving aspect, never > 1. */
export function fitScale(b: DrawingBounds, w: number, h: number): number {
  if (w <= 0 || h <= 0 || !Number.isFinite(w) || !Number.isFinite(h)) return 1;
  const sx = boundsWidth(b) / w;
  const sy = boundsHeight(b) / h;
  return Math.min(sx, sy, 1);
}

// ─── Parser (mirrors crate parser.rs) ──────────────────────────────────────────

function stripComment(line: string): string {
  const semicolon = line.indexOf(';');
  const paren = line.indexOf('(');
  let cut = -1;
  if (semicolon >= 0) cut = semicolon;
  if (paren >= 0 && (cut < 0 || paren < cut)) cut = paren;
  return cut >= 0 ? line.slice(0, cut) : line;
}

/** Parse `X<..>` / `Y<..>` tokens from a command line. Returns [x, y]. */
function parseXY(tokens: string[]): GPoint {
  let x = 0;
  let y = 0;
  for (const t of tokens) {
    if (t[0] === 'X') x = parseFloat(t.slice(1));
    else if (t[0] === 'Y') y = parseFloat(t.slice(1));
  }
  return [x, y];
}

export interface ParseResult {
  strokes: GPoint[][];
  error?: string;
}

/** Parse a G-code document; each stroke is a connected pen-down sequence. */
export function parseGcode(input: string): ParseResult {
  const strokes: GPoint[][] = [];
  let current: GPoint[] = [];
  let penDown = false;
  let pos: GPoint = [0, 0];

  for (const rawLine of input.split(/\r?\n/)) {
    const trimmed = stripComment(rawLine).trim();
    if (!trimmed) continue;
    const tokens = trimmed.split(/\s+/);
    const key = tokens[0];

    if (key === 'G0' || key === 'G1') {
      pos = parseXY(tokens);
      if (penDown) current.push(pos);
    } else if (key === 'M3') {
      if (!penDown) {
        penDown = true;
        current.push(pos);
      }
    } else if (key === 'M5') {
      if (penDown) {
        penDown = false;
        if (current.length > 0) strokes.push(current);
        current = [];
      }
    }
    // G21/G90 are preamble; unknown codes tolerated.
  }

  return { strokes };
}

// ─── Mapper (mirrors crate mapper.rs) ─────────────────────────────────────────

/** Effective scale: explicit override wins, else auto-fit. */
function effectiveScale(drawingW: number, drawingH: number, config: MappingConfig): number {
  if (config.scale !== null) return Math.max(config.scale, 0);
  return fitScale(config.target, drawingW, drawingH);
}

/**
 * Map a single A4 point to a robot-space target.
 *
 * `z` is `'draw'` (z_draw) or `'travel'` (z_travel, pen up between strokes).
 */
export function mapPoint(
  x: number,
  y: number,
  drawingW: number,
  drawingH: number,
  config: MappingConfig,
  z: 'draw' | 'travel',
): [number, number, number] {
  const scale = effectiveScale(drawingW, drawingH, config);
  const t = config.target;
  const offsetX = t.x_min + (boundsWidth(t) - drawingW * scale) / 2;
  const offsetY = t.y_min + (boundsHeight(t) - drawingH * scale) / 2;
  const rx = offsetX + x * scale;
  const ry = offsetY + y * scale;
  const rz = z === 'travel' ? config.z_travel : config.z_draw;
  return [rx, ry, rz];
}

/** Union of all stroke bounding boxes; null when there are no points. */
export interface BoundingBox {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

export function drawingBoundingBox(strokes: GPoint[][]): BoundingBox | null {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const s of strokes) {
    for (const [x, y] of s) {
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
  }
  return Number.isFinite(minX) ? { minX, minY, maxX, maxY } : null;
}