// G-Code handling for the CIPRA dialect, ported from the Rust `gcode-bridge`
// crate so the browser and the crate agree on the exact same algorithm and
// defaults (parser, workspace bounds, auto-scaling, mapping).
//
// CIPRA emits a purely geometric dialect over an A4 portrait plane
// (210×297 mm): G21 (mm), G90 (absolute), G0/G1 (rapid/draw with X Y), M3
// (pen down), M5 (pen up). Codes may be zero-padded (`G00`, `G01`) and motion
// lines may be compact (`G1X50Y20`); both are normalized to `G0`/`G1`.

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

/** Error raised by the strict parser; `message` mirrors the Rust ParseError. */
class GcodeParseError extends Error {}

/**
 * Normalize a command word: strip the leading `G`/`M` letter and any leading
 * zeros from the numeric part, then re-prefix the letter, so `G01` → `G1`,
 * `G00` → `G0`, `M05` → `M5`. Returns the normalized command plus, for
 * compact forms like `G1X50Y20` (no whitespace), the remaining coordinate
 * text. Returns `null` when the word is not a command.
 */
function normalizeCommand(token: string): { cmd: string; rest: string | null } | null {
  const letter = token[0];
  if (letter !== 'G' && letter !== 'M') return null;
  let i = 1;
  while (i < token.length && token[i] >= '0' && token[i] <= '9') i++;
  const digits = token.slice(1, i);
  if (digits === '') return null;
  const remaining = token.slice(i);
  const trimmed = digits.replace(/^0+/, '');
  return {
    cmd: letter + (trimmed === '' ? '0' : trimmed),
    rest: remaining === '' ? null : remaining,
  };
}

/**
 * Parse `X<..>` / `Y<..>` values from a motion command. Works for both spaced
 * (`X10 Y20`) and compact (`X10Y20`) forms: a value is the leading numeric
 * part of its token, so the next axis letter terminates it. Values must be
 * strict numbers (same semantics as Rust `token.parse::<f64>()`); anything
 * else raises `GcodeParseError`.
 */
function parseXY(command: string): GPoint {
  let x = 0;
  let y = 0;
  let rest = command;
  for (;;) {
    const m = /[XY]/.exec(rest);
    if (!m) break;
    const axis = m[0];
    const after = rest.slice(m.index + 1);
    const token = after.split(/\s+/)[0];
    if (token === undefined || token === '') {
      throw new GcodeParseError(`malformed command: ${command}`);
    }
    let numEnd = 0;
    while (numEnd < token.length && /[-+.\deE]/.test(token[numEnd])) numEnd++;
    const numPart = token.slice(0, numEnd);
    const trailing = token.slice(numEnd);
    if (trailing !== '' && trailing[0] !== 'X' && trailing[0] !== 'Y') {
      throw new GcodeParseError(`invalid number "${token}"`);
    }
    if (!/^-?\d+(\.\d+)?([eE][+-]?\d+)?$/.test(numPart)) {
      throw new GcodeParseError(`invalid number "${token}"`);
    }
    const value = Number(numPart);
    if (axis === 'X') x = value;
    else y = value;
    rest = after.slice(numPart.length);
  }
  return [x, y];
}

export interface ParseResult {
  strokes: GPoint[][];
  error?: string;
}

/**
 * Parse a G-code document; each stroke is a connected pen-down sequence.
 * Returns `error` (with the strokes parsed so far) when a line is malformed
 * or a coordinate value is not a valid number — never silently drops commands.
 */
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

    const normalized = normalizeCommand(key);
    if (!normalized) continue;
    const { cmd, rest } = normalized;

    if (cmd === 'G0' || cmd === 'G1') {
      let next: GPoint;
      try {
        next = parseXY(rest ?? trimmed);
      } catch (err) {
        if (err instanceof GcodeParseError) return { strokes, error: err.message };
        throw err;
      }
      pos = next;
      if (penDown) current.push(pos);
    } else if (cmd === 'M3') {
      if (!penDown) {
        penDown = true;
        current.push(pos);
      }
    } else if (cmd === 'M5') {
      if (penDown) {
        penDown = false;
        if (current.length > 0) strokes.push(current);
        current = [];
      }
    }
    // G21/G90 are preamble; unknown codes tolerated.
  }

  // Trailing partial stroke (document ends with the pen still down), mirroring
  // the Rust parser in crates/gcode-bridge/src/parser.rs.
  if (penDown && current.length > 0) strokes.push(current);

  return { strokes };
}

/**
 * Zero-dependency self-test mirroring the Rust parser tests and the shared
 * parity fixture `shared/gcode-parity.json` (the single source of truth for
 * parser behavior). Not called at runtime; import and call it manually (or
 * from a runner) to verify parity.
 *
 * Keep these cases in sync with the fixture when either side changes, and run
 * the Rust integration test `crates/gcode-bridge/tests/parity.rs`.
 */
export function runParserSelfTests(): { ok: boolean; failures: string[] } {
  const failures: string[] = [];
  const expect = (name: string, got: GPoint[][], want: GPoint[][]): void => {
    const same =
      got.length === want.length &&
      got.every((s, i) => {
        const w = want[i];
        return s.length === w.length && s.every((p, j) => p[0] === w[j][0] && p[1] === w[j][1]);
      });
    if (!same) failures.push(`${name}: got ${JSON.stringify(got)}, want ${JSON.stringify(want)}`);
  };
  const expectError = (name: string, result: ParseResult): void => {
    if (!result.error) failures.push(`${name}: expected error, got ${JSON.stringify(result.strokes)}`);
  };

  expect('simple path', parseGcode('G21 G90\nG0 X10 Y10\nM3\nG1 X50 Y50\nM5\n').strokes, [
    [
      [10, 10],
      [50, 50],
    ],
  ]);
  expect(
    'multi stroke',
    parseGcode('G21 G90\nG0 X10 Y10\nM3\nG1 X50 Y50\nM5\nG0 X60 Y60\nM3\nG1 X70 Y70\nG1 X80 Y80\nM5\n').strokes,
    [
      [
        [10, 10],
        [50, 50],
      ],
      [
        [60, 60],
        [70, 70],
        [80, 80],
      ],
    ],
  );
  expect('zero padded', parseGcode('G21 G90\nG00 X5 Y5\nM3\nG01 X10 Y20\nM5\n').strokes, [
    [
      [5, 5],
      [10, 20],
    ],
  ]);
  expect('compact', parseGcode('G21 G90\nG0X10Y10\nM3\nG1X50Y20\nM5\n').strokes, [
    [
      [10, 10],
      [50, 20],
    ],
  ]);
  expect('compact zero padded', parseGcode('M3\nG01X10Y20\nM5\n').strokes, [
    [
      [0, 0],
      [10, 20],
    ],
  ]);
  expect('comments', parseGcode('G21 G90 ; mm and absolute\n  (a comment)\nM3\nG1 X1 Y2\nM5\n').strokes, [
    [
      [0, 0],
      [1, 2],
    ],
  ]);
  expect('trailing partial stroke', parseGcode('G21 G90\nG0 X1 Y1\nM3\nG1 X5 Y5\n').strokes, [
    [
      [1, 1],
      [5, 5],
    ],
  ]);
  expect('empty', parseGcode('G21 G90\nM5\n').strokes, []);
  expect(
    'mixed travel and padded draw',
    parseGcode('G0 X0 Y0\nM3\nG01 X10 Y10\nG01 X20 Y20\nM5\nG0 X30 Y30\nM3\nG01 X40 Y40\nM5\n').strokes,
    [
      [
        [0, 0],
        [10, 10],
        [20, 20],
      ],
      [
        [30, 30],
        [40, 40],
      ],
    ],
  );
  expectError('invalid number with trailing garbage', parseGcode('M3\nG1 X10abc Y20\nM5\n'));
  expectError('bad number', parseGcode('M3\nG0 Xabc Y10\n'));

  return { ok: failures.length === 0, failures };
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