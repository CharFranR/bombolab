#!/usr/bin/env python3
"""Fit the post-IK affine calibration from measured landmark coordinates.

Reads a CSV of measurements taken from `diag-08-landmarks.gcode` (a 3x3
landmark grid drawn 3 times) and produces `web/public/post-ik-calibration.json`.

Measurement protocol (see `experiments/diagnostics/calib-landmarks.md`):
  - The G-code draws a 6 mm cross centered on each nominal landmark.
  - Measure the CENTER of each cross with a ruler aligned to the grid
    (x along the rows, y perpendicular to them). Record absolute mm.
  - One row per landmark per pass.

CSV columns: `landmark, pass, x_mm, y_mm`
  landmark: L01..L09 (nominal grid coordinates are fixed in this script)
  pass:     1..3 (the three passes drawn by the G-code)
  x_mm, y_mm: measured coordinates of the cross center

Pipeline:
  1. Repeatability — per-landmark spread across the 3 passes. A static
     post-IK map only makes sense if the error is repeatable; a spread
     above ~3 mm is a strong warning that no affine correction will help.
  2. Affine fit — measured = A * commanded + b via least squares (the same
     matrix form the original generator used).
  3. Leave-one-out cross-validation — RMS of predicting each landmark from
     a fit on the other 11. Guards against overfitting small samples.
  4. Recentralization — b = C - A*C around the grid center C, so the center
     maps to itself (the runtime convention expected by the web code).
  5. Sanity checks — positive determinant, A not absurdly far from identity.
  6. Write the calibration JSON (version 1).

`backlash_offset_x_mm` and `k_velocity_factor` are NOT derived from the
landmark grid; pass them via `--backlash-mm` / `--k-velocity`, or keep the
current values from `web/public/post-ik-calibration.json`.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_CSV = REPO_ROOT / "experiments" / "diagnostics" / "measurements" / "landmarks.csv"
DEFAULT_OUTPUT = REPO_ROOT / "web" / "public" / "post-ik-calibration.json"

# Nominal grid of diag-08-landmarks.gcode (x, y) in robot coordinates (mm).
GRID_X = [215, 260, 305]
GRID_Y = [-25, -45, -65]
GRID_CENTER = (260.0, -45.0)
REPEATABILITY_WARNING_MM = 3.0
# Current values, used unless overridden on the command line.
CURRENT_BACKLASH_MM = 12.333333333333334
CURRENT_K_VELOCITY = 2.4509803921568633e-05


def landmarks() -> dict[str, tuple[float, float]]:
    """Map L01..L09 -> nominal (x, y), row-major from top-left."""
    out: dict[str, tuple[float, float]] = {}
    i = 1
    for y in GRID_Y:
        for x in GRID_X:
            out[f"L{i:02d}"] = (float(x), float(y))
            i += 1
    return out


def nominal_ordered() -> list[tuple[str, float, float]]:
    return [(name, x, y) for name, (x, y) in sorted(landmarks().items())]


def load_measurements(path: Path) -> dict[str, list[tuple[float, float]]]:
    """Return {landmark: [(x, y), ...]} across passes, preserving order."""
    if not path.exists():
        return {}
    data: dict[str, list[tuple[float, float]]] = {}
    with path.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            name = (row.get("landmark") or "").strip().upper()
            try:
                x = float(row["x_mm"])
                y = float(row["y_mm"])
            except (KeyError, ValueError) as exc:
                raise SystemExit(f"fit_calibration: fila inválida en {path}: {row} ({exc})")
            if name not in landmarks():
                raise SystemExit(f"fit_calibration: landmark desconocido '{name}' (se esperaba L01..L09)")
            data.setdefault(name, []).append((x, y))
    return data


def repeatability(meas: dict[str, list[tuple[float, float]]]) -> dict[str, float]:
    """Max pairwise distance among the measured passes of each landmark (mm)."""
    out: dict[str, float] = {}
    for name, samples in meas.items():
        if len(samples) < 2:
            out[name] = 0.0
            continue
        worst = 0.0
        for i in range(len(samples)):
            for j in range(i + 1, len(samples)):
                worst = max(worst, np.hypot(samples[i][0] - samples[j][0], samples[i][1] - samples[j][1]))
        out[name] = float(worst)
    return out


def fit_affine(nominal: np.ndarray, measured: np.ndarray):
    """measured ≈ A·nominal + b via lstsq. Returns A (2x2), b (2,), residuals."""
    H = np.hstack([nominal, np.ones((nominal.shape[0], 1))])  # n x 3
    theta, *_ = np.linalg.lstsq(H, measured, rcond=None)       # 3 x 2
    a11, a21 = theta[0]
    a12, a22 = theta[1]
    bx, by = theta[2]
    A = np.array([[a11, a12], [a21, a22]])
    b = np.array([bx, by])
    pred = H @ theta
    residuals = measured - pred
    return A, b, residuals


def leave_one_out(nominal: np.ndarray, measured: np.ndarray) -> float:
    """LOO-CV RMS (mm): predict each landmark from a fit on the other 11."""
    n = nominal.shape[0]
    errors = []
    for i in range(n):
        idx = [j for j in range(n) if j != i]
        A, b, _ = fit_affine(nominal[idx], measured[idx])
        pred = A @ nominal[i] + b
        errors.append(np.linalg.norm(pred - measured[i]))
    return float(np.sqrt(np.mean(np.square(errors))))


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="mediciones CSV (plantilla si no existe)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="JSON de salida")
    parser.add_argument("--backlash-mm", type=float, default=CURRENT_BACKLASH_MM)
    parser.add_argument("--k-velocity", type=float, default=CURRENT_K_VELOCITY)
    args = parser.parse_args(argv)

    # ── Template ─────────────────────────────────────────────────────────────
    if not args.csv.exists():
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["landmark", "pass", "x_mm", "y_mm"])
            for name, x, y in nominal_ordered():
                for p in range(1, 4):
                    writer.writerow([name, p, "", ""])
        print(f"fit_calibration: plantilla CSV creada en {args.csv}")
        print("  Llenala con las mediciones de diag-08-landmarks.gcode y volvé a ejecutar.")
        return

    meas = load_measurements(args.csv)
    missing = [name for name, *_ in nominal_ordered() if name not in meas]
    if missing:
        print(f"fit_calibration: faltan landmarks: {missing}")
        sys.exit(2)
    n_per = {name: len(v) for name, v in meas.items()}
    # One sample per landmark is enough to fit; repeatability needs >=2. The
    # 3 passes of each landmark may be entered in ANY order — the spread and
    # the average are order-independent.

    print("=== 1. Repetibilidad (dispersión entre pasadas por landmark, mm) ===")
    spread = repeatability(meas)
    worst = 0.0
    for name, *_ in nominal_ordered():
        s = spread.get(name, 0.0)
        worst = max(worst, s)
        flag = "  <-- CUIDADO" if s > REPEATABILITY_WARNING_MM else ""
        print(f"  {name}: {s:6.2f}{flag}")
    print(f"  dispersión máxima: {worst:.2f} mm")
    if worst > REPEATABILITY_WARNING_MM:
        print(f"  ADVERTENCIA: dispersión > {REPEATABILITY_WARNING_MM} mm. Un mapa affine")
        print("  estático difícilmente va a corregir un error no repetible.")
        print("  Considerá primero el problema mecánico antes de seguir.")

    # ── Averages → affine fit ────────────────────────────────────────────────
    names = [name for name, *_ in nominal_ordered()]
    lm = landmarks()
    nominal = np.array([lm[n] for n in names], dtype=float)
    measured = np.array([[np.mean([s[0] for s in meas[n]]), np.mean([s[1] for s in meas[n]])] for n in names])

    A, b, residuals = fit_affine(nominal, measured)
    rms_fit = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))
    cv_rms = leave_one_out(nominal, measured)

    print("\n=== 2. Ajuste affine  measured = A·commanded + b ===")
    print(f"  A =\n{np.array2string(A, precision=6)}")
    print(f"  b_local = ({b[0]:.6f}, {b[1]:.6f})")
    print(f"  det(A) = {np.linalg.det(A):.6f}")
    print(f"  RMS ajuste = {rms_fit:.4f} mm")
    print(f"  RMS leave-one-out = {cv_rms:.4f} mm  (CV alto => sobreajuste o modelo inadecuado)")

    if not np.linalg.det(A) > 0:
        print("  ERROR: determinante no positivo (el TS rechaza este JSON).")
        sys.exit(2)
    scale = np.linalg.norm(A - np.eye(2), ord="fro")
    print(f"  ||A - I||_F = {scale:.4f}  (referencia: matriz actual ~0.6 = claramente sospechosa)")
    if cv_rms > rms_fit * 1.5 + 0.5:
        print("  ADVERTENCIA: la validación cruzada degrada mucho. El modelo affine no generaliza bien.")

    # ── Recentralización ─────────────────────────────────────────────────────
    cx, cy = GRID_CENTER
    b_center_x = cx - (A[0, 0] * cx + A[0, 1] * cy)
    b_center_y = cy - (A[1, 0] * cx + A[1, 1] * cy)
    print("\n=== 3. Recentralización alrededor del centro de la retícula ===")
    print(f"  centro C = ({cx}, {cy})")
    print(f"  b_center = ({b_center_x:.6f}, {b_center_y:.6f})")

    # ── Sanity: in-range commanded points stay in the safe area ─────────────
    xs = [215.0, 245.0, 275.0, 305.0]
    ys = [-65.0, -45.0, -25.0]
    worst_out = 0.0
    for x in xs:
        for y in ys:
            t = np.array([x, y])
            cmd = np.linalg.inv(A) @ (t - np.array([b_center_x, b_center_y]))
            d = np.max(np.abs(cmd - t))
            worst_out = max(worst_out, float(d))
    print(f"  mayor desplazamiento comandado respecto al target (extremos): {worst_out:.2f} mm")

    # ── Write JSON ───────────────────────────────────────────────────────────
    doc = {
        "version": 1,
        "backlash_offset_x_mm": args.backlash_mm,
        "k_velocity_factor": args.k_velocity,
        "calibration_points": [
            {
                "center_x": cx,
                "center_y": cy,
                "a11": A[0, 0],
                "a12": A[0, 1],
                "a21": A[1, 0],
                "a22": A[1, 1],
                "b_x": b_center_x,
                "b_y": b_center_y,
            }
        ],
    }
    with args.output.open("w", encoding="utf-8", newline="\n") as fh:
        json.dump(doc, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    print(f"\nfit_calibration: JSON escrito en {args.output}")
    print(f"  (1 calibration point, affine global recentralizado en ({cx}, {cy}))")


if __name__ == "__main__":
    main()
