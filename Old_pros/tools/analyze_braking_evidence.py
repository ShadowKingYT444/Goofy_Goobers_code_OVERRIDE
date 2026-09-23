#!/usr/bin/env python3
"""Summarize the pre-brake straight trials without extrapolating safety."""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
SOURCE = REPORT / "distance_multispeed_live_01" / "endpoint_results.csv"
EFFECTIVE_WHEEL_DIAMETER_IN = 2.4330552523
P1_STOP_IN = 8.0


def main() -> None:
    rows = list(csv.DictReader(SOURCE.open(encoding="utf-8")))
    by_rpm: dict[float, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_rpm[float(row["rpm"])].append(row)

    evidence = []
    for rpm in sorted(by_rpm):
        group = by_rpm[rpm]
        commanded_speed_in_s = (
            rpm * math.pi * EFFECTIVE_WHEEL_DIAMETER_IN / 60.0
        )
        max_target_overshoot_in = max(
            float(row["encoder"]) - float(row["target"]) for row in group
        )
        max_return_residual_in = max(
            abs(float(row["encoder_residual"])) for row in group
        )
        max_gps_return_residual_in = max(
            abs(float(row["gps_residual"])) for row in group
        )
        evidence.append(
            {
                "rpm": rpm,
                "effective_commanded_speed_in_s": commanded_speed_in_s,
                "max_old_coast_target_overshoot_in": max_target_overshoot_in,
                "max_encoder_return_residual_in": max_return_residual_in,
                "max_gps_return_residual_in": max_gps_return_residual_in,
                "travel_during_20ms_in": commanded_speed_in_s * 0.020,
                "travel_during_100ms_in": commanded_speed_in_s * 0.100,
                "p1_threshold_minus_old_overshoot_in": (
                    P1_STOP_IN - max_target_overshoot_in
                ),
            }
        )

    table_path = REPORT / "braking_evidence.csv"
    with table_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=evidence[0].keys())
        writer.writeheader()
        writer.writerows(evidence)

    summary = {
        "source": str(SOURCE.relative_to(ROOT)),
        "trial_count": len(rows),
        "p1_stop_threshold_in": P1_STOP_IN,
        "effective_wheel_diameter_in": EFFECTIVE_WHEEL_DIAMETER_IN,
        "evidence": evidence,
        "strongest_measured_statement": (
            "The old zero-voltage/coast trials overshot their encoder target "
            f"by at most {max(x['max_old_coast_target_overshoot_in'] for x in evidence):.3f} in "
            "through 35 RPM."
        ),
        "not_measured": [
            "physical deceleration and stopping distance after the new PROS brake() command",
            "P1 lens-to-front-bumper offset",
            "stopping behavior above 35 RPM or at navigation power commands",
            "sensor transport latency and target-dependent P1 acquisition latency",
        ],
        "safety_interpretation": (
            "The 8-inch threshold exceeds all recorded old-coast target overshoots, "
            "but this is not a collision-clearance proof. Do not subtract the values "
            "as bumper clearance until the geometric offset and new brake stop are measured."
        ),
    }
    (REPORT / "braking_evidence_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    rpms = [row["rpm"] for row in evidence]
    speeds = [row["effective_commanded_speed_in_s"] for row in evidence]
    overshoot = [row["max_old_coast_target_overshoot_in"] for row in evidence]
    latency20 = [row["travel_during_20ms_in"] for row in evidence]
    latency100 = [row["travel_during_100ms_in"] for row in evidence]

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.7), constrained_layout=True)
    fig.suptitle(
        "Forward Stop Evidence — measured old coast behavior, not a brake-distance claim",
        fontsize=16,
        fontweight="bold",
    )

    axes[0].plot(rpms, overshoot, marker="o", linewidth=2.5, color="#ff9f43")
    axes[0].axhline(P1_STOP_IN, color="#5cffad", linestyle="--", linewidth=2)
    axes[0].set(
        xlabel="Commanded wheel RPM",
        ylabel="Worst encoder target overshoot (in)",
        title="9 live 2/5/10-in trials",
    )
    axes[0].text(10.5, 7.6, "P1 command threshold: 8 in", color="#5cffad")
    for rpm, value in zip(rpms, overshoot):
        axes[0].annotate(f"{value:.2f}", (rpm, value), xytext=(0, 8),
                         textcoords="offset points", ha="center")

    axes[1].plot(speeds, latency20, marker="o", label="20-ms control phase")
    axes[1].plot(speeds, latency100, marker="o", label="100-ms sensor phase")
    axes[1].set(
        xlabel="Effective commanded speed (in/s)",
        ylabel="Distance traveled (in)",
        title="Timing-only travel (no braking)",
    )
    axes[1].legend(frameon=False)

    axes[2].axis("off")
    warning = (
        "WHAT THIS PROVES\n"
        "• Old coast overshoot ≤ 2.03 in through 35 RPM\n"
        "• 20-ms software phase travel ≤ 0.13 in at 35 RPM\n\n"
        "WHAT IS STILL UNKNOWN\n"
        "• New brake() physical stop distance\n"
        "• P1 lens → front bumper offset\n"
        "• P1 acquisition/transport latency\n"
        "• Behavior above 35 RPM\n\n"
        "Therefore 8 in is a conservative command threshold,\n"
        "not yet a certified obstacle clearance."
    )
    axes[2].text(
        0.02,
        0.98,
        warning,
        va="top",
        ha="left",
        fontsize=12,
        linespacing=1.45,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#ff6b6b"},
    )
    for axis in axes[:2]:
        axis.grid(alpha=0.18)

    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"braking_evidence_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
