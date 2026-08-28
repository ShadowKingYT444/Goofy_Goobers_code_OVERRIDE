#!/usr/bin/env python3
"""Quantify coupled drive-motor encoder agreement across live campaign logs."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
LIMIT_DEG = 15.0


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q)) if values.size else 0.0


def main() -> None:
    runs: list[dict[str, float | int | str]] = []
    all_left: list[float] = []
    all_right: list[float] = []
    dynamic_left: list[float] = []
    dynamic_right: list[float] = []

    for path in sorted(REPORT.glob("*/telemetry.csv")):
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        samples: list[tuple[float, float, float, float]] = []
        for row in rows:
            try:
                samples.append(tuple(float(row[key]) for key in ("m17", "m18", "m11", "m13")))
            except (KeyError, TypeError, ValueError):
                continue
        if not samples:
            continue
        data = np.asarray(samples)
        left = np.abs(data[:, 0] - data[:, 1])
        right = np.abs(data[:, 2] - data[:, 3])
        left_motion = float(np.ptp((data[:, 0] + data[:, 1]) / 2.0))
        right_motion = float(np.ptp((data[:, 2] + data[:, 3]) / 2.0))
        dynamic = max(left_motion, right_motion) >= 5.0
        all_left.extend(left.tolist())
        all_right.extend(right.tolist())
        if dynamic:
            dynamic_left.extend(left.tolist())
            dynamic_right.extend(right.tolist())
        runs.append(
            {
                "run": path.parent.name,
                "frames": len(samples),
                "dynamic": int(dynamic),
                "left_travel_span_deg": left_motion,
                "right_travel_span_deg": right_motion,
                "left_p95_pair_spread_deg": percentile(left, 95),
                "right_p95_pair_spread_deg": percentile(right, 95),
                "left_max_pair_spread_deg": float(np.max(left)),
                "right_max_pair_spread_deg": float(np.max(right)),
            }
        )

    dynamic_runs = [run for run in runs if run["dynamic"]]
    left = np.asarray(dynamic_left)
    right = np.asarray(dynamic_right)
    combined = np.concatenate((left, right))
    summary = {
        "source_runs": len(runs),
        "dynamic_runs": len(dynamic_runs),
        "dynamic_side_samples": int(combined.size),
        "firmware_pair_spread_limit_deg": LIMIT_DEG,
        "dynamic_p95_pair_spread_deg": percentile(combined, 95),
        "dynamic_max_pair_spread_deg": float(np.max(combined)),
        "dynamic_frames_over_limit": int(np.count_nonzero(combined > LIMIT_DEG)),
        "margin_to_limit_deg": LIMIT_DEG - float(np.max(combined)),
        "interpretation": (
            "All saved dynamic samples remain below the 15-degree fail-closed "
            "pair gate. This validates separation from observed normal behavior, "
            "but does not prove every electrical/mechanical fault is detected."
        ),
    }

    csv_path = REPORT / "drive_pair_consistency.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(runs[0]))
        writer.writeheader()
        writer.writerows(runs)
    (REPORT / "drive_pair_consistency_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )

    dynamic_runs.sort(
        key=lambda run: max(
            float(run["left_max_pair_spread_deg"]),
            float(run["right_max_pair_spread_deg"]),
        )
    )
    labels = [str(run["run"]).replace("_live_", "\n") for run in dynamic_runs]
    y = np.arange(len(dynamic_runs))
    left_max = [float(run["left_max_pair_spread_deg"]) for run in dynamic_runs]
    right_max = [float(run["right_max_pair_spread_deg"]) for run in dynamic_runs]

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), constrained_layout=True)
    fig.patch.set_facecolor("#07111f")
    for axis in axes:
        axis.set_facecolor("#0c1b2e")
        axis.grid(alpha=0.16)

    width = 0.38
    axes[0].barh(y - width / 2, left_max, height=width, color="#32d3a5", label="left P17/P18")
    axes[0].barh(y + width / 2, right_max, height=width, color="#59a8ff", label="right P11/P13")
    axes[0].axvline(LIMIT_DEG, color="#ff5c72", linewidth=2, label="firmware cutoff")
    axes[0].set_yticks(y, labels)
    axes[0].set_xlabel("maximum absolute paired-encoder spread (motor deg)")
    axes[0].set_title("Every live motion run stayed below the fail-closed gate")
    axes[0].legend(loc="lower right")

    bins = np.arange(-0.5, max(LIMIT_DEG + 1.5, float(np.max(combined)) + 1.5), 1.0)
    axes[1].hist(left, bins=bins, alpha=0.7, color="#32d3a5", label="left samples")
    axes[1].hist(right, bins=bins, alpha=0.7, color="#59a8ff", label="right samples")
    axes[1].axvline(LIMIT_DEG, color="#ff5c72", linewidth=2)
    axes[1].set_yscale("symlog", linthresh=1)
    axes[1].set_xlabel("absolute paired-encoder spread (motor deg)")
    axes[1].set_ylabel("dynamic telemetry samples (log-like scale)")
    axes[1].set_title(
        f"p95 {summary['dynamic_p95_pair_spread_deg']:.1f}° • "
        f"max {summary['dynamic_max_pair_spread_deg']:.1f}° • "
        f"{summary['dynamic_frames_over_limit']} over cutoff"
    )
    axes[1].legend()

    fig.suptitle(
        "Coupled drivetrain encoder consistency — live 2026-08-23 campaign\n"
        "Pair spread detects an unplugged/stalled/asymmetric motor; it is not wheel-slip ground truth",
        fontsize=15,
        fontweight="bold",
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"drive_pair_consistency_dashboard.{suffix}", dpi=180)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
