#!/usr/bin/env python3
"""Sensitivity map for P1 lens range, bumper setback, and physical stop travel."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
STOP_THRESHOLD_IN = 8.0
OFFICIAL_CLOSE_ERROR_IN = 15.0 / 25.4
OLD_COAST_MAX_IN = 2.032


def minimum_clearance(setback_in: np.ndarray, stop_travel_in: np.ndarray) -> np.ndarray:
    # Worst case is an under-reported range: the true lens-to-obstacle gap can
    # be threshold minus the quoted close-range error.
    return STOP_THRESHOLD_IN - OFFICIAL_CLOSE_ERROR_IN - setback_in - stop_travel_in


def main() -> None:
    setbacks = np.linspace(0.0, 8.0, 161)
    stop_travels = np.linspace(0.0, 4.0, 161)
    x, y = np.meshgrid(setbacks, stop_travels)
    clearance = minimum_clearance(x, y)

    scenarios = []
    for setback in (0.0, 2.0, 4.0, 6.0, 8.0):
        for travel in (0.0, 0.5, 1.0, OLD_COAST_MAX_IN, 3.0, 4.0):
            scenarios.append(
                {
                    "lens_behind_front_bumper_in": setback,
                    "post_detection_stop_travel_in": travel,
                    "worst_case_remaining_clearance_in": float(
                        minimum_clearance(np.asarray(setback), np.asarray(travel))
                    ),
                }
            )
    with (REPORT / "p1_stop_margin_scenarios.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(scenarios[0]))
        writer.writeheader()
        writer.writerows(scenarios)

    maximum_combined_budget = STOP_THRESHOLD_IN - OFFICIAL_CLOSE_ERROR_IN
    summary = {
        "configured_lens_threshold_in": STOP_THRESHOLD_IN,
        "official_below_200mm_error_in": OFFICIAL_CLOSE_ERROR_IN,
        "maximum_setback_plus_stop_travel_for_nonnegative_clearance_in": maximum_combined_budget,
        "old_coast_trial_max_overshoot_in": OLD_COAST_MAX_IN,
        "maximum_lens_setback_if_old_coast_repeats_in": maximum_combined_budget - OLD_COAST_MAX_IN,
        "status": "sensitivity only; current lens-to-bumper offset and brake-mode stop travel are unmeasured",
        "formula": "clearance >= threshold - 0.5906 - lens_setback - stop_travel",
    }
    (REPORT / "p1_stop_margin_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )

    plt.style.use("dark_background")
    fig, (axis, note) = plt.subplots(
        1, 2, figsize=(15, 6.5), gridspec_kw={"width_ratios": [1.45, 1]},
        constrained_layout=True,
    )
    fig.patch.set_facecolor("#07111f")
    axis.set_facecolor("#0c1b2e")
    levels = np.linspace(-4.6, 7.5, 25)
    plot = axis.contourf(x, y, clearance, levels=levels, cmap="RdYlGn", extend="both")
    zero = axis.contour(x, y, clearance, levels=[0], colors=["white"], linewidths=2.5)
    axis.clabel(zero, fmt={0: "0 in clearance"}, inline=True, fontsize=10)
    axis.axhline(OLD_COAST_MAX_IN, color="#57c7ff", linestyle="--", linewidth=2,
                 label=f"old coast max {OLD_COAST_MAX_IN:.2f} in")
    axis.set_xlabel("P1 lens behind front bumper (in)")
    axis.set_ylabel("travel after threshold is observed (in)")
    axis.set_title("Worst-case obstacle clearance after stopping")
    axis.grid(alpha=0.18)
    axis.legend(loc="upper right")
    colorbar = fig.colorbar(plot, ax=axis, label="remaining bumper clearance (in)")
    colorbar.ax.axhline(0, color="white", linewidth=1)

    note.set_facecolor("#0c1b2e")
    note.axis("off")
    note.text(0.03, 0.94, "Known", fontsize=16, fontweight="bold", color="#57c7ff")
    note.text(
        0.03,
        0.86,
        "P1 stop threshold      8.00 in\n"
        "VEX close error        +/-0.59 in\n"
        f"Old coast overshoot    {OLD_COAST_MAX_IN:.2f} in max",
        fontsize=13,
        linespacing=1.55,
        va="top",
    )
    note.text(0.03, 0.58, "Still unmeasured", fontsize=16, fontweight="bold", color="#ff6b6b")
    note.text(
        0.03,
        0.50,
        "• lens setback from the front bumper\n"
        "• new brake-mode physical travel\n"
        "• detection/control latency on a real target\n"
        "• target material and incidence angle",
        fontsize=13,
        linespacing=1.55,
        va="top",
    )
    note.text(
        0.03,
        0.20,
        "Nonnegative clearance requires:\n"
        f"setback + stop travel <= {maximum_combined_budget:.2f} in\n\n"
        "This is not collision certification.",
        fontsize=14,
        fontweight="bold",
        color="#ffd166",
        va="top",
    )
    fig.suptitle(
        "P1 emergency-stop margin — geometry sensitivity, not a live brake test",
        fontsize=16,
        fontweight="bold",
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"p1_stop_margin_dashboard.{suffix}", dpi=180)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
