#!/usr/bin/env python3
"""Convert the provisional dead-reckoning envelope into outage travel budgets."""

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
CONFIG = (ROOT / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)


def setting(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG)
    if match is None:
        raise RuntimeError(f"missing production setting {name}")
    return float(match.group(1))


SCALE_FRACTION = setting("kDeadReckoningScaleEnvelopeFraction")
HEADING_ENVELOPE_DEG = setting("kDeadReckoningHeadingEnvelopeDeg")
ERROR_PER_TRAVEL = math.hypot(
    SCALE_FRACTION, math.tan(math.radians(HEADING_ENVELOPE_DEG))
)
BASE_ERRORS_IN = (0.0, 0.39, 0.75, 1.5, 3.25)
TARGET_ENVELOPES_IN = (1.0, 2.0, 3.0, 6.0)
SPEEDS_IN_S = (5.0, 10.0, 20.0)


def envelope(travel_in: np.ndarray | float, base_error_in: float) -> np.ndarray | float:
    return base_error_in + np.asarray(travel_in) * ERROR_PER_TRAVEL


def budget(target_in: float, base_error_in: float) -> float:
    return max(0.0, (target_in - base_error_in) / ERROR_PER_TRAVEL)


def main() -> None:
    rows = []
    for base in BASE_ERRORS_IN:
        for target in TARGET_ENVELOPES_IN:
            rows.append(
                {
                    "error_at_last_absolute_fix_in": base,
                    "target_total_envelope_in": target,
                    "maximum_outage_travel_in": budget(target, base),
                }
            )
    with (REPORT / "gps_outage_distance_budgets.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    time_rows = []
    for base in BASE_ERRORS_IN:
        for target in TARGET_ENVELOPES_IN:
            travel_budget = budget(target, base)
            for speed in SPEEDS_IN_S:
                time_rows.append(
                    {
                        "error_at_last_absolute_fix_in": base,
                        "target_total_envelope_in": target,
                        "forward_speed_in_s": speed,
                        "maximum_outage_travel_in": travel_budget,
                        "maximum_outage_time_s": travel_budget / speed,
                    }
                )
    with (REPORT / "gps_outage_time_budgets.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(time_rows[0]))
        writer.writeheader()
        writer.writerows(time_rows)

    summary = {
        "measured_scale_envelope_fraction": SCALE_FRACTION,
        "provisional_heading_controller_allowance_deg": HEADING_ENVELOPE_DEG,
        "combined_error_growth_in_per_in_travel": ERROR_PER_TRAVEL,
        "combined_error_growth_percent_of_travel": ERROR_PER_TRAVEL * 100.0,
        "model": "base absolute-position error + travel*hypot(scale_fraction,tan(heading_envelope))",
        "important_exclusions": [
            "systematic encoder-scale bias because P7, not tape/laser truth, calibrated the scale",
            "wheel slip or being pushed",
            "collision and controller endpoint error",
            "starting-pose placement error beyond the selected base bound",
            "long-duration or temperature-dependent IMU drift",
        ],
        "status": "provisional engineering allowance, not measured IMU accuracy or a probabilistic confidence interval",
        "representative_time_budgets_s": {
            "base_0_39_target_1": {
                f"{speed:g}_in_s": budget(1.0, 0.39) / speed
                for speed in SPEEDS_IN_S
            },
            "base_0_75_target_1": {
                f"{speed:g}_in_s": budget(1.0, 0.75) / speed
                for speed in SPEEDS_IN_S
            },
            "base_0_39_target_2": {
                f"{speed:g}_in_s": budget(2.0, 0.39) / speed
                for speed in SPEEDS_IN_S
            },
        },
    }
    (REPORT / "gps_outage_distance_budget_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )

    travel = np.linspace(0.0, 120.0, 601)
    matrix = np.asarray(
        [[budget(target, base) for target in TARGET_ENVELOPES_IN]
         for base in BASE_ERRORS_IN]
    )

    plt.style.use("dark_background")
    fig, (curve, table) = plt.subplots(
        1,
        2,
        figsize=(15.5, 6.8),
        gridspec_kw={"width_ratios": [1.35, 1]},
        constrained_layout=True,
    )
    fig.patch.set_facecolor("#07111f")
    curve.set_facecolor("#0c1b2e")
    colors = ("#57c7ff", "#5cffad", "#ffd166", "#ff9f68", "#ff6b8a")
    for base, color in zip(BASE_ERRORS_IN, colors):
        curve.plot(
            travel,
            envelope(travel, base),
            color=color,
            linewidth=2.3,
            label=f"last-fix base {base:.2f} in",
        )
    for target in TARGET_ENVELOPES_IN:
        curve.axhline(target, color="white", alpha=0.16, linewidth=1)
    curve.set_xlim(0, 120)
    curve.set_ylim(0, 6.5)
    curve.set_xlabel("encoder-reported travel while no absolute fix is accepted (in)")
    curve.set_ylabel("reported minimum position-error envelope (in)")
    curve.set_title("Error budget grows with travel, not outage clock time")
    curve.grid(alpha=0.16)
    curve.legend(loc="upper left", fontsize=9)
    curve.text(
        0.98,
        0.04,
        f"growth = {ERROR_PER_TRAVEL * 100:.2f}% of travel\n"
        "excludes scale-reference bias, slip/push,\nplacement beyond base, long-term drift",
        transform=curve.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        color="#ffd166",
        bbox={"facecolor": "#07111f", "alpha": 0.78, "edgecolor": "#ffd166"},
    )

    table.set_facecolor("#0c1b2e")
    image = table.imshow(matrix, cmap="viridis", aspect="auto", vmin=0, vmax=120)
    table.set_xticks(range(len(TARGET_ENVELOPES_IN)),
                     [f"{value:g} in" for value in TARGET_ENVELOPES_IN])
    table.set_yticks(range(len(BASE_ERRORS_IN)),
                     [f"{value:.2f} in" for value in BASE_ERRORS_IN])
    table.set_xlabel("allowed total envelope")
    table.set_ylabel("error already present at last accepted fix")
    table.set_title("Maximum additional travel before budget is exhausted")
    for row in range(matrix.shape[0]):
        for column in range(matrix.shape[1]):
            value = matrix[row, column]
            table.text(
                column,
                row,
                f"{value:.1f}\"",
                ha="center",
                va="center",
                color="white" if value < 80 else "#07111f",
                fontsize=10,
                fontweight="bold",
            )
    fig.colorbar(image, ax=table, label="outage travel budget (in)")
    fig.suptitle(
        "GPS-outage budget: P7-referenced scale + provisional heading allowance",
        fontsize=16,
        fontweight="bold",
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"gps_outage_distance_budget_dashboard.{suffix}", dpi=180)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
