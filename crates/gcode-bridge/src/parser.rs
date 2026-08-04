//! Parser for the minimal G-code dialect emitted by CIPRA.
//!
//! CIPRA produces a purely geometric dialect of G-code, described below:
//!
//! ```gcode
//! G21 G90        — preamble (millimetres, absolute positioning)
//! G0 X.. Y..     — rapid travel (pen up) to a path start
//! M3             — pen down (tool on)
//! G1 X.. Y..     — draw a straight segment
//! M5             — pen up (tool off)
//! ```
//!
//! The parser is deliberately tolerant of formatting (whitespace, comments,
//! unknown M/G codes) because CIPRA may later extend the dialect. It exposes
//! the drawing as a list of [`Stroke`]s: each stroke is a connected sequence
//! of `(x, y)` millimetre points drawn with the pen down.

/// A point on the A4 drawing plane, in millimetres (`x` ∈ width, `y` ∈ height).
pub type Point2D = (f64, f64);

/// A connected sequence of points drawn with the pen down.
#[derive(Debug, Clone, PartialEq)]
pub struct Stroke {
    pub points: Vec<Point2D>,
}

impl Stroke {
    /// The stroke's bounding box (min_x, min_y, max_x, max_y).
    pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
        let mut it = self.points.iter();
        let (x0, y0) = *it.next()?;
        let mut b = (x0, y0, x0, y0);
        for (x, y) in it {
            b.0 = b.0.min(*x);
            b.1 = b.1.min(*y);
            b.2 = b.2.max(*x);
            b.3 = b.3.max(*y);
        }
        Some(b)
    }
}

/// Errors produced while parsing a G-code document.
#[derive(Debug, Clone, PartialEq)]
pub enum ParseError {
    /// A command token was malformed (e.g. a coordinate missing its value).
    MalformedLine { line: usize, text: String },
    /// A coordinate value could not be parsed as a number.
    InvalidNumber { line: usize, token: String },
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::MalformedLine { line, text } => {
                write!(f, "malformed command on line {line}: {text:?}")
            }
            ParseError::InvalidNumber { line, token } => {
                write!(f, "invalid number {token:?} on line {line}")
            }
        }
    }
}

/// Parse a G-code document using the current position as implicit state for
/// strokes: `M3` starts drawing from the current position, `G1`/`G0` append
/// points when the pen is down, `M5` ends the stroke.
pub fn parse_gcode(input: &str) -> Result<Vec<Stroke>, ParseError> {
    let mut strokes: Vec<Stroke> = Vec::new();
    let mut current: Vec<Point2D> = Vec::new();
    let mut pen_down = false;
    let mut pos: Point2D = (0.0, 0.0);

    for (i, raw) in input.lines().enumerate() {
        let line = strip_comment(raw);
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let key = trimmed
            .split_whitespace()
            .next()
            .ok_or_else(|| ParseError::MalformedLine {
                line: i + 1,
                text: trimmed.into(),
            })?;

        if let Some((cmd, compact)) = normalize_command(key) {
            match cmd.as_str() {
                "G0" | "G1" => {
                    // Compact lines like `G1X50Y20` carry the coordinates in
                    // the command token itself; parse them from the remainder.
                    let coords = compact.unwrap_or(trimmed);
                    let (x, y) = parse_xy(coords, i + 1)?;
                    pos = (x, y);
                    if pen_down {
                        current.push(pos);
                    }
                }
                "M3" => {
                    // Tool on: start of a drawing stroke at the current position.
                    if !pen_down {
                        pen_down = true;
                        current.push(pos);
                    }
                }
                "M5" if pen_down => {
                    // Tool off: end of the current stroke.
                    pen_down = false;
                    if !current.is_empty() {
                        strokes.push(Stroke { points: current });
                    }
                    current = Vec::new();
                }
                // G21 (units mm) and G90 (absolute positioning) are preamble;
                // unknown M/G codes are tolerated to future-proof the dialect.
                _ => {}
            }
        }
    }

    // Trailing partial stroke (document ends with the pen still down).
    if pen_down && !current.is_empty() {
        strokes.push(Stroke { points: current });
    }

    Ok(strokes)
}

/// Remove comment content after `(` ... `)` (Fanuc style) or trailing `;`.
fn strip_comment(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
    let mut depth = 0usize;
    for c in line.chars() {
        match c {
            '(' => depth += 1,
            ')' => {
                depth = depth.saturating_sub(1);
            }
            _ if depth == 0 && c == ';' => break,
            _ if depth == 0 => out.push(c),
            _ => {}
        }
    }
    out
}

/// Normalize a command word: strip the leading `G`/`M` letter and any leading
/// zeros from the numeric part, then re-prefix the letter, so `G01` → `G1`,
/// `G00` → `G0`, `M05` → `M5`. Returns the normalized command plus, for
/// compact forms like `G1X50Y20` (no whitespace), the remaining coordinate
/// text. Returns `None` when the word is not a command (e.g. `G21`-style
/// unknown codes stay normalized but unmatched, and non-commands are skipped).
fn normalize_command(token: &str) -> Option<(String, Option<&str>)> {
    let (letter, rest) = token.split_at(1);
    if letter != "G" && letter != "M" {
        return None;
    }
    let digits_len = rest.bytes().take_while(|b| b.is_ascii_digit()).count();
    let (digits, remaining) = rest.split_at(digits_len);
    if digits.is_empty() {
        return None;
    }
    let trimmed = digits.trim_start_matches('0');
    let number = if trimmed.is_empty() { "0" } else { trimmed };
    let compact = if remaining.is_empty() {
        None
    } else {
        Some(remaining)
    };
    Some((format!("{letter}{number}"), compact))
}

/// Extract `X<..>` and `Y<..>` values from a motion command. Works for both
/// spaced (`X10 Y20`) and compact (`X10Y20`) forms: a value is the leading
/// numeric part of its token, so the next axis letter terminates it.
fn parse_xy(command: &str, line: usize) -> Result<Point2D, ParseError> {
    let mut x: Option<f64> = None;
    let mut y: Option<f64> = None;

    let mut rest = command;
    while let Some(idx) = rest.find(['X', 'Y']) {
        let axis = rest.as_bytes()[idx] as char;
        let after = &rest[idx + 1..];
        let token = after
            .split_whitespace()
            .next()
            .ok_or_else(|| ParseError::MalformedLine {
                line,
                text: command.into(),
            })?;
        let num_len = token
            .bytes()
            .take_while(|b| b.is_ascii_digit() || matches!(b, b'-' | b'+' | b'.' | b'e' | b'E'))
            .count();
        let (num, trailing) = token.split_at(num_len);
        // A trailing `X`/`Y` starts the next axis value (compact form);
        // anything else is a malformed number, e.g. `X10abc`.
        if !trailing.is_empty() && !trailing.starts_with(['X', 'Y']) {
            return Err(ParseError::InvalidNumber {
                line,
                token: token.into(),
            });
        }
        let value: f64 = num.parse().map_err(|_| ParseError::InvalidNumber {
            line,
            token: token.into(),
        })?;
        match axis {
            'X' => x = Some(value),
            'Y' => y = Some(value),
            _ => unreachable!(),
        }
        // Advance past this axis value to find the next one; a compact trailing
        // part (e.g. `Y20` in `X50Y20`) is rescanned as the next axis.
        rest = &after[num_len..];
    }

    Ok((x.unwrap_or(0.0), y.unwrap_or(0.0)))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Inline cases below mirror the shared parity fixture
    // `shared/gcode-parity.json` (the single source of truth, also consumed by
    // `tests/parity.rs` and the TS self-tests in `web/src/lib/gcodeCipra.ts`).
    // Keep this set in sync with the fixture when either side changes.

    #[test]
    fn parses_simple_path_cipra_fixture() {
        let gcode = "G21 G90\nG0 X10.00 Y10.00\nM3\nG1 X50.00 Y50.00\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 1);
        assert_eq!(
            strokes[0].points,
            vec![(10.0, 10.0), (50.0, 50.0)]
        );
    }

    #[test]
    fn parses_multi_path_fixture() {
        let gcode = "G21 G90\nG0 X10.00 Y10.00\nM3\nG1 X50.00 Y50.00\nM5\n\
                     G0 X60.00 Y60.00\nM3\nG1 X70.00 Y70.00\nG1 X80.00 Y80.00\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 2);
        assert_eq!(strokes[0].points, vec![(10.0, 10.0), (50.0, 50.0)]);
        assert_eq!(
            strokes[1].points,
            vec![(60.0, 60.0), (70.0, 70.0), (80.0, 80.0)]
        );
    }

    #[test]
    fn tolerates_comments_and_whitespace() {
        let gcode = "G21 G90 ; mm and absolute\n  (a comment)\nM3\nG1 X1 Y2\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 1);
        assert_eq!(strokes[0].points, vec![(0.0, 0.0), (1.0, 2.0)]);
    }

    #[test]
    fn returns_empty_when_no_strokes() {
        let gcode = "G21 G90\nM5\n";
        assert_eq!(parse_gcode(gcode).unwrap(), vec![]);
    }

    #[test]
    fn reports_malformed_number() {
        let gcode = "M3\nG0 Xabc Y10\n";
        assert!(matches!(
            parse_gcode(gcode),
            Err(ParseError::InvalidNumber { .. })
        ));
    }

    #[test]
    fn parses_zero_padded_codes() {
        let gcode = "G21 G90\nG00 X5 Y5\nM3\nG01 X10 Y20\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 1);
        assert_eq!(strokes[0].points, vec![(5.0, 5.0), (10.0, 20.0)]);
    }

    #[test]
    fn parses_compact_motion_no_whitespace() {
        let gcode = "M3\nG1X50Y20\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 1);
        assert_eq!(strokes[0].points, vec![(0.0, 0.0), (50.0, 20.0)]);
    }

    #[test]
    fn parses_zero_padded_compact_motion() {
        let gcode = "M3\nG01X10Y20\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 1);
        assert_eq!(strokes[0].points, vec![(0.0, 0.0), (10.0, 20.0)]);
    }

    #[test]
    fn parses_mixed_travel_and_zero_padded_draw() {
        let gcode = "G0 X0 Y0\nM3\nG01 X10 Y10\nG01 X20 Y20\nM5\n\
                     G0 X30 Y30\nM3\nG01 X40 Y40\nM5\n";
        let strokes = parse_gcode(gcode).unwrap();
        assert_eq!(strokes.len(), 2);
        assert_eq!(
            strokes[0].points,
            vec![(0.0, 0.0), (10.0, 10.0), (20.0, 20.0)]
        );
        assert_eq!(strokes[1].points, vec![(30.0, 30.0), (40.0, 40.0)]);
    }

    #[test]
    fn reports_invalid_number_with_trailing_garbage() {
        let gcode = "M3\nG1 X10abc Y20\n";
        assert!(matches!(
            parse_gcode(gcode),
            Err(ParseError::InvalidNumber { .. })
        ));
    }
}