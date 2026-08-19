#!/usr/bin/env python3
"""Generate the post-IK calibration document."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence


BACKLASH_SAMPLES_MM = (14.0, 12.0, 11.0)
NOMINAL_VELOCITY = 180.0
MEASURED_VELOCITY = 170.0
VELOCITY_SAMPLE_START = 1200.0
VELOCITY_SAMPLE_END = 3600.0

CALIBRATION_POINTS = [
    {
        "center_x": 260.0,
        "center_y": -40.0,
        "a11": 0.621182047517,
        "a12": -0.141182047517,
        "a21": 0.075074514407,
        "a22": 0.844925485593,
        "b_x": -152.330973304695,
        "b_y": 43.379135966028,
    },
    {
        "center_x": 260.0,
        "center_y": -45.0,
        "a11": 0.649011515437,
        "a12": 0.010988484563,
        "a21": 0.025132708137,
        "a22": 0.934867291863,
        "b_x": -135.797936436556,
        "b_y": 84.791159425132,
    },
    {
        "center_x": 260.0,
        "center_y": -45.0,
        "a11": 0.397219826657,
        "a12": 0.029317527394,
        "a21": 0.034183297378,
        "a22": 1.327153651769,
        "b_x": -78.777154930883,
        "b_y": -8.887657318376,
    },
]


def calibration_values() -> tuple[float, float]:
    """Return the post-IK scalar calibration values from the measured samples."""

    backlash_offset_x_mm = sum(BACKLASH_SAMPLES_MM) / len(BACKLASH_SAMPLES_MM)
    k_velocity_factor = (NOMINAL_VELOCITY / MEASURED_VELOCITY - 1.0) / (
        VELOCITY_SAMPLE_END - VELOCITY_SAMPLE_START
    )
    return backlash_offset_x_mm, k_velocity_factor


def validate_excluded_tests() -> list[dict[str, str]]:
    """Print raw indicators for validation-only tests excluded from calibration.

    These tests provide checks on the calibration, not additional calibration
    sources.  The indicators intentionally stay as ranges or raw annotations;
    they are not combined into a global RMS value.
    """

    indicators = [
        {
            "test": "Prueba 2",
            "nominal": "radius=40 mm",
            "raw": "annotations=10..38 mm",
            "error": "annotation-minus-nominal range=-30..-2 mm",
        },
        {
            "test": "Prueba 4",
            "nominal": "outer rectangle=140 x 92 mm",
            "raw": "measured outer chains (raw chain annotations)",
            "error": "compare each outer-chain measurement with 140 mm and 92 mm sides",
        },
        {
            "test": "Prueba 7",
            "nominal": "triangle=80 mm; square=50 mm",
            "raw": "annotated triangle and square chain data",
            "error": "compare each annotated chain with its 80 mm or 50 mm nominal side",
        },
    ]

    print("Validation-only tests (not calibration sources):")
    for indicator in indicators:
        print(
            f"  {indicator['test']}: nominal={indicator['nominal']}; "
            f"raw={indicator['raw']}; error_indicator={indicator['error']}"
        )
    print("No global RMS is computed for excluded validation tests.")
    return indicators


def generate_calibration(output_path: Path | None = None) -> Path:
    """Write the post-IK calibration without modifying the STL calibration file."""

    repository_root = Path(__file__).resolve().parent
    output = output_path or repository_root / "web" / "public" / "post-ik-calibration.json"
    backlash_offset_x_mm, k_velocity_factor = calibration_values()
    calibration_points = []
    for source in CALIBRATION_POINTS:
        point = dict(source)
        cx = point["center_x"]
        cy = point["center_y"]
        point["b_x"] = cx - (point["a11"] * cx + point["a12"] * cy)
        point["b_y"] = cy - (point["a21"] * cx + point["a22"] * cy)
        calibration_points.append(point)

    calibration = {
        "version": 1,
        "backlash_offset_x_mm": backlash_offset_x_mm,
        "k_velocity_factor": k_velocity_factor,
        "calibration_points": calibration_points,
    }

    with output.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(calibration, stream, indent=2, ensure_ascii=False)
        stream.write("\n")

    print(f"Calibration output: {output}")
    print(f"backlash_offset_x_mm={backlash_offset_x_mm:.15g}")
    print(f"k_velocity_factor={k_velocity_factor:.15g}")
    print(f"calibration_points={len(CALIBRATION_POINTS)}")
    return output


def main(argv: Sequence[str] | None = None) -> None:
    repository_root = Path(__file__).resolve().parent
    default_output = repository_root / "web" / "public" / "post-ik-calibration.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output,
        help="path for the generated post-IK calibration JSON",
    )
    args = parser.parse_args(argv)
    generate_calibration(args.output)
    validate_excluded_tests()


if __name__ == "__main__":
    main()
