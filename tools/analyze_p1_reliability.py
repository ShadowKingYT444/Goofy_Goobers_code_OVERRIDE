#!/usr/bin/env python3
"""Characterize P1 target availability and low-confidence range variation."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
STATIONARY = REPORT / "stationary_04_120s" / "telemetry.csv"
ROTATION_RUNS = (
    REPORT / "rotation_sweep_live_01" / "telemetry.csv",
    REPORT / "rotation_sweep_live_02" / "telemetry.csv",
)
NO_TARGET_MM = 9999
MIN_MM = 20
MAX_MM = 2000
CONFIDENCE_THRESHOLD = 20


def read(path: Path) -> list[dict[str, str]]:
    return list(csv.DictReader(path.open(encoding="utf-8")))


def runs(mask: np.ndarray, dt_s: float) -> list[float]:
    padded = np.concatenate(([False], mask, [False])).astype(int)
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    return [float((end - start) * dt_s) for start, end in zip(starts, ends)]


def main() -> None:
    rows = read(STATIONARY)
    time_s = np.asarray([float(row["host_s"]) for row in rows])
    time_s -= time_s[0]
    distance_mm = np.asarray([int(row["p1_mm"]) for row in rows])
    confidence = np.asarray([int(row["p1_conf"]) for row in rows])
    dt_s = float(np.median(np.diff(time_s)))
    physical = (distance_mm >= MIN_MM) & (distance_mm <= MAX_MM)
    precision_usable = physical & (
        (distance_mm <= 200) | (confidence >= CONFIDENCE_THRESHOLD)
    )
    no_target = distance_mm == NO_TARGET_MM
    physical_ranges_in = distance_mm[physical] / 25.4
    burst_s = runs(physical, dt_s)

    rotation = []
    for path in ROTATION_RUNS:
        rotation.extend(read(path))
    rotation_distance = np.asarray([int(row["p1_mm"]) for row in rotation])
    rotation_physical = (rotation_distance >= MIN_MM) & (rotation_distance <= MAX_MM)

    summary = {
        "source": str(STATIONARY.relative_to(ROOT)),
        "stationary_samples": int(len(rows)),
        "stationary_elapsed_s": float(time_s[-1]),
        "stationary_physical_return_count": int(np.sum(physical)),
        "stationary_physical_return_fraction": float(np.mean(physical)),
        "stationary_no_target_fraction": float(np.mean(no_target)),
        "stationary_precision_usable_count": int(np.sum(precision_usable)),
        "stationary_precision_usable_fraction": float(np.mean(precision_usable)),
        "physical_return_range_in": {
            "minimum": float(np.min(physical_ranges_in)),
            "median": float(np.median(physical_ranges_in)),
            "maximum": float(np.max(physical_ranges_in)),
            "standard_deviation": float(np.std(physical_ranges_in)),
            "peak_to_peak": float(np.ptp(physical_ranges_in)),
        },
        "physical_return_confidence": {
            "minimum": int(np.min(confidence[physical])),
            "median": float(np.median(confidence[physical])),
            "maximum": int(np.max(confidence[physical])),
        },
        "return_bursts": {
            "count": len(burst_s),
            "median_duration_s": float(np.median(burst_s)),
            "maximum_duration_s": float(np.max(burst_s)),
        },
        "rotation_sweep_samples": len(rotation),
        "rotation_sweep_physical_return_count": int(np.sum(rotation_physical)),
        "rotation_sweep_physical_return_fraction": float(np.mean(rotation_physical)),
        "interpretation": (
            "The stationary sensor intermittently saw a roughly 40-in target, but every "
            "return beyond 200 mm was below the configured 20/63 precision threshold. "
            "P1 can fail closed on a missing device/API fault and stop on a detected "
            "close return; it cannot guarantee detection when the API reports 9999/no "
            "target. It is not suitable for long-range pose correction."
        ),
        "not_measured": [
            "tape-measured target distance",
            "accuracy against controlled targets inside 8 inches",
            "detection latency for a newly appearing obstacle",
            "target color/material/angle dependence",
            "lens-to-front-bumper offset and physical stopping distance",
        ],
    }
    (REPORT / "p1_reliability_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    timeline = np.where(physical, distance_mm / 25.4, np.nan)
    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(15.5, 9), constrained_layout=True)
    fig.suptitle(
        "P1 Forward Distance Reliability — live stationary and rotation captures",
        fontsize=16,
        fontweight="bold",
    )
    axes[0, 0].scatter(time_s[physical], timeline[physical], s=9, color="#58d6ff")
    axes[0, 0].set(xlabel="Elapsed time (s)", ylabel="Physical return (in)", title="Intermittent stationary target returns")
    axes[0, 0].grid(alpha=0.16)

    axes[0, 1].scatter(
        physical_ranges_in,
        confidence[physical],
        s=14,
        alpha=0.65,
        color="#ffb347",
    )
    axes[0, 1].axhline(CONFIDENCE_THRESHOLD, color="#ff6b6b", linestyle="--", label="precision gate 20/63")
    axes[0, 1].set(xlabel="Reported distance (in)", ylabel="Confidence / 63", title="Every long return fails precision confidence")
    axes[0, 1].legend(frameon=False)
    axes[0, 1].grid(alpha=0.16)

    labels = ("physical return", "9999 no target", "other/fault")
    values = (
        int(np.sum(physical)),
        int(np.sum(no_target)),
        int(len(rows) - np.sum(physical) - np.sum(no_target)),
    )
    axes[1, 0].bar(labels, values, color=("#5cffad", "#64748b", "#ff6b6b"))
    axes[1, 0].set(ylabel="Samples", title="120-second availability")
    axes[1, 0].set_ylim(0, max(values) * 1.18)
    axes[1, 0].tick_params(axis="x", rotation=15)
    for index, value in enumerate(values):
        axes[1, 0].text(index, value, f"{value:,}\n{value / len(rows):.1%}", ha="center", va="bottom")
    axes[1, 0].grid(axis="y", alpha=0.16)

    axes[1, 1].axis("off")
    note = (
        "STATIONARY 120 s\n"
        f"• physical returns: {np.mean(physical):.1%}\n"
        f"• precision-usable returns: {np.mean(precision_usable):.1%}\n"
        f"• return median: {np.median(physical_ranges_in):.2f} in\n"
        f"• return std / span: {np.std(physical_ranges_in):.2f} / {np.ptp(physical_ranges_in):.2f} in\n"
        f"• confidence: {np.min(confidence[physical])}-{np.max(confidence[physical])} / 63\n"
        f"• bursts: {len(burst_s)}, max {np.max(burst_s):.2f} s\n\n"
        "ROTATION CAPTURES\n"
        f"• physical returns: {np.sum(rotation_physical)}/{len(rotation)}\n\n"
        "USE\n"
        "Close physical return ≤8 in: emergency stop.\n"
        "9999: documented healthy/no-target.\n"
        "Long low-confidence returns: never precision fusion.\n\n"
        "Pending: close-target accuracy/latency, bumper offset, brake distance."
    )
    axes[1, 1].text(
        0.02,
        0.98,
        note,
        va="top",
        fontsize=11.5,
        linespacing=1.4,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#58d6ff"},
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"p1_reliability_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
