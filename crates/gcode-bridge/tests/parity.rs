//! Parity test: consumes the shared Rust/TS fixture `shared/gcode-parity.json`
//! (the single source of truth for parser behavior) and verifies that the
//! Rust parser produces the exact expected strokes and error kinds. Keep the
//! fixture in sync with `web/src/lib/gcodeCipra.ts` (`runParserSelfTests`).

use gcode_bridge::parser::{parse_gcode, ParseError};
use serde_json::Value;

const FIXTURE: &str = include_str!("../../../shared/gcode-parity.json");

fn expected_strokes(value: &Value) -> Vec<Vec<(f64, f64)>> {
    value
        .as_array()
        .expect("strokes must be an array")
        .iter()
        .map(|stroke| {
            stroke
                .as_array()
                .expect("stroke must be an array")
                .iter()
                .map(|pt| {
                    let xy = pt.as_array().expect("point must be an array");
                    (
                        xy[0].as_f64().expect("x must be a number"),
                        xy[1].as_f64().expect("y must be a number"),
                    )
                })
                .collect()
        })
        .collect()
}

#[test]
fn parity_cases_match_shared_fixture() {
    let fixture: Value =
        serde_json::from_str(FIXTURE).expect("shared/gcode-parity.json must be valid JSON");

    for case in fixture["cases"].as_array().expect("cases must be an array") {
        let name = case["name"].as_str().unwrap_or("<unnamed>");
        let gcode = case["gcode"].as_str().expect("case gcode must be a string");
        let want = expected_strokes(&case["strokes"]);

        let strokes = parse_gcode(gcode)
            .unwrap_or_else(|e| panic!("case {name:?} must parse Ok, got Err({e})"));
        let got: Vec<Vec<(f64, f64)>> = strokes.into_iter().map(|s| s.points).collect();
        assert_eq!(got, want, "case {name:?} strokes must match the fixture");
    }
}

#[test]
fn parity_error_cases_match_shared_fixture() {
    let fixture: Value =
        serde_json::from_str(FIXTURE).expect("shared/gcode-parity.json must be valid JSON");

    for case in fixture["errorCases"]
        .as_array()
        .expect("errorCases must be an array")
    {
        let name = case["name"].as_str().unwrap_or("<unnamed>");
        let gcode = case["gcode"].as_str().expect("case gcode must be a string");
        let error = case["error"].as_str().expect("case error must be a string");

        let err = parse_gcode(gcode).unwrap_err();
        match error {
            "InvalidNumber" => {
                assert!(
                    matches!(err, ParseError::InvalidNumber { .. }),
                    "case {name:?} must be InvalidNumber, got {err}"
                )
            }
            "MalformedLine" => {
                assert!(
                    matches!(err, ParseError::MalformedLine { .. }),
                    "case {name:?} must be MalformedLine, got {err}"
                )
            }
            other => panic!("case {name:?}: unknown expected error kind {other:?}"),
        }
    }
}
