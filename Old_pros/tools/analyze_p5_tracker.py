#!/usr/bin/env python3
"""Compare P5 lateral tracker motion with rotation-geometry expectation."""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
SOURCE = REPORT / "rotation_sweep_live_02" / "raw.log"
REAR_LEVER_IN = 5.18
TURN = re.compile(
    r"ROT_SWEEP target_deg=(?P<target>[\d.]+) phase=turn .*?"
    r"imu_deg=(?P<imu>[\d.-]+).*?h5_delta_cdeg=(?P<cdeg>[\d.-]+) "
    r"h5_in=(?P<actual>[\d.-]+)"
)


def main() -> None:
    text = SOURCE.read_bytes().decode("utf-8", errors="ignore")
    matches = list(TURN.finditer(text))
    if not matches:
        raise SystemExit("no P5 rotation records found")
    rows = []
    for match in matches:
        imu_deg = abs(float(match["imu"]))
        expected_in = REAR_LEVER_IN * math.radians(imu_deg)
        actual_in = abs(float(match["actual"]))
        rows.append({
            "target_deg": float(match["target"]),
            "imu_turn_deg": imu_deg,
            "p5_delta_centideg": float(match["cdeg"]),
            "p5_actual_in": actual_in,
            "expected_rear_wheel_in": expected_in,
            "actual_to_expected_fraction": actual_in / expected_in,
            "missing_motion_in": expected_in - actual_in,
        })
    with (REPORT / "p5_tracker_rotation_comparison.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    actual = np.asarray([row["p5_actual_in"] for row in rows])
    expected = np.asarray([row["expected_rear_wheel_in"] for row in rows])
    target = np.asarray([row["target_deg"] for row in rows])
    ratio = np.divide(actual, expected)
    summary = {
        "source": str(SOURCE.relative_to(ROOT)),
        "rear_lever_in": REAR_LEVER_IN,
        "turn_points": len(rows),
        "expected_motion_range_in": [float(np.min(expected)), float(np.max(expected))],
        "observed_motion_range_in": [float(np.min(actual)), float(np.max(actual))],
        "maximum_observed_fraction_of_expected": float(np.max(ratio)),
        "minimum_missing_motion_in": float(np.min(expected - actual)),
        "maximum_missing_motion_in": float(np.max(expected - actual)),
        "interpretation": (
            "P5 reported at most 0.002 in while rear-offset geometry predicts 1.64-8.51 in. "
            "This is a mechanical wheel/shaft/coupling failure, not usable lateral odometry."
        ),
        "production_state": "fusion disabled; raw telemetry retained",
        "repair_acceptance": [
            "mark shaft/wheel and confirm visible rotation during an in-place turn",
            "verify P5 sign and approximately linear travel versus P6 angle",
            "repeat CW and CCW sweeps and fit lever from multiple angles",
            "only re-enable after translation and rotation residual validation",
        ],
    }
    (REPORT / "p5_tracker_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(16.5, 5.8), constrained_layout=True)
    fig.suptitle(
        "P5 Lateral Tracker Mechanical Audit — live in-place turns",
        fontsize=16,
        fontweight="bold",
    )
    axes[0].plot(target, expected, "o-", linewidth=2.5, label="expected from P6 × 5.18-in lever")
    axes[0].plot(target, actual, "o-", linewidth=2.5, label="P5 observed")
    axes[0].set(xlabel="Nominal turn (deg)", ylabel="Rear wheel travel (in)", title="Expected vs observed travel")
    axes[0].legend(frameon=False)
    axes[0].grid(alpha=0.16)

    axes[1].bar(target, ratio * 100.0, width=7.5, color="#ff6b6b")
    axes[1].set(xlabel="Nominal turn (deg)", ylabel="Observed / expected (%)", title="Mechanical response fraction")
    axes[1].grid(axis="y", alpha=0.16)
    for x, value in zip(target, ratio * 100.0):
        axes[1].text(x, value, f"{value:.3f}%", ha="center", va="bottom", fontsize=9)

    axes[2].axis("off")
    note = (
        "FIVE LIVE CCW TURNS\n"
        f"• expected: {np.min(expected):.2f}-{np.max(expected):.2f} in\n"
        f"• observed: {np.min(actual):.3f}-{np.max(actual):.3f} in\n"
        f"• best response: {np.max(ratio) * 100.0:.3f}% of expected\n"
        f"• worst missing travel: {np.max(expected - actual):.2f} in\n\n"
        "CONCLUSION\n"
        "The Smart Port sensor enumerates, but the wheel/shaft does\n"
        "not transmit chassis motion. P5 cannot observe lateral slip.\n\n"
        "PRODUCTION\n"
        "Fusion disabled; raw P5 telemetry retained for repair testing.\n"
        "Do not tune software around a mechanically stationary wheel."
    )
    axes[2].text(
        0.02,
        0.98,
        note,
        va="top",
        fontsize=11.5,
        linespacing=1.45,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#ff6b6b"},
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"p5_tracker_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
