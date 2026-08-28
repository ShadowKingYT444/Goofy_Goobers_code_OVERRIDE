#!/usr/bin/env python3
"""Demonstrate why wheel and effective-track encoder scales are coupled."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
RAW = REPORT / "rotation_sweep_live_01" / "raw.log"
SCALE = 0.8847477281
PHYSICAL_WHEEL_IN = 2.75
CALIBRATED_TRACK_AT_PHYSICAL_SCALE_IN = 12.0086
EFFECTIVE_WHEEL_IN = PHYSICAL_WHEEL_IN * SCALE
EFFECTIVE_TRACK_IN = CALIBRATED_TRACK_AT_PHYSICAL_SCALE_IN * SCALE

TURN = re.compile(
    r"ROT_SWEEP target_deg=(?P<target>[\d.]+) phase=turn .*?"
    r"encoder_deg=(?P<encoder>[\d.-]+) gps_deg=(?P<gps>[\d.-]+) "
    r"imu_deg=(?P<imu>[\d.-]+)"
)


def metrics(prediction: np.ndarray, truth: np.ndarray) -> dict[str, float]:
    error = prediction - truth
    return {
        "rmse_deg": float(np.sqrt(np.mean(error**2))),
        "mean_abs_error_deg": float(np.mean(np.abs(error))),
        "max_abs_error_deg": float(np.max(np.abs(error))),
    }


def main() -> None:
    text = RAW.read_bytes().decode("utf-8", errors="ignore")
    matches = list(TURN.finditer(text))
    if not matches:
        raise SystemExit("no live rotation sweep points found")
    target = np.asarray([float(match["target"]) for match in matches])
    original_encoder = np.abs([float(match["encoder"]) for match in matches])
    imu = np.abs([float(match["imu"]) for match in matches])
    gps = np.abs([float(match["gps"]) for match in matches])

    wheel_only_encoder = original_encoder * SCALE
    paired_encoder = original_encoder * (SCALE / SCALE)
    track_from_imu_at_physical_scale = (
        CALIBRATED_TRACK_AT_PHYSICAL_SCALE_IN * original_encoder / imu
    )

    fields = (
        "target_deg",
        "original_encoder_deg",
        "imu_deg",
        "gps_deg",
        "wheel_only_scaled_encoder_deg",
        "paired_scaled_encoder_deg",
        "imu_inferred_track_at_physical_scale_in",
    )
    rows = [
        dict(zip(fields, (float(value) for value in values)))
        for values in zip(
            target,
            original_encoder,
            imu,
            gps,
            wheel_only_encoder,
            paired_encoder,
            track_from_imu_at_physical_scale,
        )
    ]
    csv_path = REPORT / "drive_geometry_scale_consistency.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    original_metrics = metrics(original_encoder, imu)
    wheel_only_metrics = metrics(wheel_only_encoder, imu)
    paired_metrics = metrics(paired_encoder, imu)
    summary = {
        "source": str(RAW.relative_to(ROOT)),
        "live_turn_points": len(rows),
        "encoder_distance_scale": SCALE,
        "physical_wheel_diameter_in": PHYSICAL_WHEEL_IN,
        "effective_wheel_diameter_in": EFFECTIVE_WHEEL_IN,
        "track_width_at_physical_wheel_scale_in": CALIBRATED_TRACK_AT_PHYSICAL_SCALE_IN,
        "paired_effective_track_width_in": EFFECTIVE_TRACK_IN,
        "diameter_to_track_ratio_before": PHYSICAL_WHEEL_IN / CALIBRATED_TRACK_AT_PHYSICAL_SCALE_IN,
        "diameter_to_track_ratio_after": EFFECTIVE_WHEEL_IN / EFFECTIVE_TRACK_IN,
        "original_encoder_vs_imu": original_metrics,
        "incorrect_wheel_only_scale_vs_imu": wheel_only_metrics,
        "paired_scale_vs_imu": paired_metrics,
        "mean_imu_inferred_track_45_to_90_deg_at_physical_scale_in": float(
            np.mean(track_from_imu_at_physical_scale[target >= 45.0])
        ),
        "interpretation": (
            "Scaling effective wheel diameter without scaling effective track width would "
            "change a previously validated dimensionless turn ratio. Applying the same "
            "linear scale to both preserves the live encoder/IMU angular relationship."
        ),
        "limitations": [
            "The straight-distance scale was fitted to P7 GPS, not independent tape/laser truth.",
            "P7 later showed strong view-dependent position failures, so the scale remains provisional.",
            "The live points were open-loop outward turns and include endpoint coast.",
            "P6 is the relative angular reference, not an external metrology instrument.",
            "The 90-degree point lacks a completed return because the sweep was safety-interrupted.",
            "A new closed-loop live sweep is still required after firmware upload.",
        ],
    }
    (REPORT / "drive_geometry_scale_consistency_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(16.5, 5.8), constrained_layout=True)
    fig.suptitle(
        "Differential-Drive Scale Coupling — live encoder turns vs P6",
        fontsize=16,
        fontweight="bold",
    )
    axes[0].plot(target, imu, "o-", linewidth=2.4, label="P6 IMU")
    axes[0].plot(target, original_encoder, "o-", label="validated original ratio")
    axes[0].plot(target, wheel_only_encoder, "o--", label="wrong: wheel scale only")
    axes[0].set(
        xlabel="Nominal turn (deg)",
        ylabel="Measured angle magnitude (deg)",
        title="Five live outward turns",
    )
    axes[0].legend(frameon=False)

    width = 4.0
    axes[1].bar(
        target - width / 2,
        np.abs(original_encoder - imu),
        width,
        label="paired wheel + track",
    )
    axes[1].bar(
        target + width / 2,
        np.abs(wheel_only_encoder - imu),
        width,
        label="wheel only",
    )
    axes[1].set(
        xlabel="Nominal turn (deg)",
        ylabel="Absolute disagreement with P6 (deg)",
        title="Calibration error consequence",
    )
    axes[1].legend(frameon=False)

    axes[2].axis("off")
    note = (
        "PAIRED LINEAR CALIBRATION\n"
        f"wheel: {PHYSICAL_WHEEL_IN:.3f} → {EFFECTIVE_WHEEL_IN:.3f} in\n"
        f"track: {CALIBRATED_TRACK_AT_PHYSICAL_SCALE_IN:.4f} → {EFFECTIVE_TRACK_IN:.4f} in\n"
        f"shared scale: {SCALE:.6f}\n\n"
        "ANGLE RMSE VS P6\n"
        f"paired scale: {paired_metrics['rmse_deg']:.2f}°\n"
        f"wheel only: {wheel_only_metrics['rmse_deg']:.2f}°\n"
        f"wheel-only max: {wheel_only_metrics['max_abs_error_deg']:.2f}°\n\n"
        "The paired change does not claim a physical 10.62-in track.\n"
        "It is the effective track expressed in the same calibrated\n"
        "encoder-distance scale as the 2.433-in effective wheel."
    )
    axes[2].text(
        0.02,
        0.98,
        note,
        va="top",
        fontsize=11.5,
        linespacing=1.45,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#ffb347"},
    )
    for axis in axes[:2]:
        axis.grid(alpha=0.17)
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"drive_geometry_scale_consistency_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
