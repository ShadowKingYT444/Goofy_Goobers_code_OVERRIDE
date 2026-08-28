#!/usr/bin/env python3
"""Analyze the live 120-second P6 stationary heading capture.

This deliberately reports heading stability, not gyro-rate Allan deviation or
absolute heading accuracy: the capture contains quantized integrated rotation
samples and has no independent angular truth reference.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
SOURCE = REPORT / "stationary_04_120s" / "telemetry.csv"


def two_sample_heading_deviation(
    heading_deg: np.ndarray, sample_interval_s: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return non-overlapping adjacent-cluster heading deviation by tau."""
    requested_tau = np.geomspace(sample_interval_s, 30.0, 28)
    cluster_sizes = np.unique(np.maximum(1, np.rint(requested_tau / sample_interval_s).astype(int)))
    taus: list[float] = []
    deviations: list[float] = []
    pairs: list[int] = []
    for size in cluster_sizes:
        cluster_count = len(heading_deg) // size
        if cluster_count < 3:
            continue
        means = heading_deg[: cluster_count * size].reshape(cluster_count, size).mean(axis=1)
        difference = np.diff(means)
        taus.append(float(size * sample_interval_s))
        deviations.append(float(np.sqrt(np.mean(difference**2) / 2.0)))
        pairs.append(int(len(difference)))
    return np.asarray(taus), np.asarray(deviations), np.asarray(pairs)


def main() -> None:
    rows = list(csv.DictReader(SOURCE.open(encoding="utf-8")))
    host_s = np.asarray([float(row["host_s"]) for row in rows], dtype=float)
    heading = np.asarray([float(row["imu"]) for row in rows], dtype=float)
    drive = {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in ("m17", "m18", "m11", "m13")
    }
    host_s -= host_s[0]
    dt = float(np.median(np.diff(host_s)))
    slope_deg_s, intercept_deg = np.polyfit(host_s, heading, 1)
    fit = slope_deg_s * host_s + intercept_deg
    residual = heading - fit
    taus, deviations, pairs = two_sample_heading_deviation(heading, dt)
    unique, counts = np.unique(heading, return_counts=True)

    cluster_csv = REPORT / "imu_heading_two_sample_deviation.csv"
    with cluster_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("cluster_time_s", "heading_deviation_deg", "adjacent_cluster_pairs"))
        writer.writerows(zip(taus, deviations, pairs))

    summary = {
        "source": str(SOURCE.relative_to(ROOT)),
        "samples": int(len(heading)),
        "elapsed_s": float(host_s[-1]),
        "median_sample_interval_s": dt,
        "median_rate_hz": 1.0 / dt,
        "heading_min_deg": float(heading.min()),
        "heading_median_deg": float(np.median(heading)),
        "heading_max_deg": float(heading.max()),
        "heading_peak_to_peak_deg": float(np.ptp(heading)),
        "heading_std_deg": float(np.std(heading)),
        "detrended_heading_std_deg": float(np.std(residual)),
        "linear_fit_slope_deg_per_s": float(slope_deg_s),
        "linear_fit_slope_deg_per_min": float(slope_deg_s * 60.0),
        "linear_fit_change_over_capture_deg": float(slope_deg_s * host_s[-1]),
        "observed_heading_values_deg": [float(value) for value in unique],
        "observed_heading_value_counts": [int(value) for value in counts],
        "drive_encoder_span_deg": {
            name: float(np.ptp(values)) for name, values in drive.items()
        },
        "two_sample_heading_deviation": [
            {
                "cluster_time_s": float(tau),
                "heading_deviation_deg": float(deviation),
                "adjacent_cluster_pairs": int(pair_count),
            }
            for tau, deviation, pair_count in zip(taus, deviations, pairs)
        ],
        "interpretation": (
            "P6 integrated heading was stationary to one 0.01-degree output step over this "
            "120-second, zero-encoder-motion capture. The tiny linear fit is descriptive only."
        ),
        "not_measured": [
            "absolute heading accuracy against an external angular reference",
            "gyro-rate bias or angle-random-walk from raw angular-rate samples",
            "temperature dependence",
            "long-term drift beyond the two-minute capture",
            "repeatability after power cycles or calibration cycles",
            "dynamic heading error while accelerating or vibrating",
        ],
    }
    (REPORT / "imu_stability_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(15.5, 9), constrained_layout=True)
    fig.suptitle(
        "P6 IMU Stationary Heading Stability — live 120-second capture",
        fontsize=17,
        fontweight="bold",
    )

    axes[0, 0].step(host_s, heading, where="post", color="#58d6ff", linewidth=1.4, label="P6 rotation")
    axes[0, 0].plot(host_s, fit, color="#ffb347", linewidth=2.0, label="linear fit")
    axes[0, 0].set(
        xlabel="Elapsed time (s)",
        ylabel="Integrated heading (deg)",
        title="Only two quantized values observed",
    )
    axes[0, 0].legend(frameon=False)

    axes[0, 1].bar([f"{value:.2f}" for value in unique], counts, color="#8c7dff")
    axes[0, 1].set(
        xlabel="Reported P6 heading (deg)",
        ylabel="Samples",
        title="0.01-degree output quantization",
    )
    for index, count in enumerate(counts):
        axes[0, 1].text(index, count, str(count), ha="center", va="bottom")

    positive = deviations > 0.0
    axes[1, 0].loglog(taus[positive], deviations[positive], marker="o", color="#5cffad")
    axes[1, 0].set(
        xlabel="Adjacent cluster time (s)",
        ylabel="Two-sample heading deviation (deg)",
        title="Heading stability by averaging interval",
    )
    axes[1, 0].text(
        0.03,
        0.04,
        "Integrated heading statistic; not gyro-rate Allan deviation",
        transform=axes[1, 0].transAxes,
        fontsize=9,
        color="#cbd5e1",
    )

    axes[1, 1].axis("off")
    note = (
        "OBSERVED\n"
        f"• {len(heading):,} samples at {1.0 / dt:.2f} Hz\n"
        f"• peak-to-peak: {np.ptp(heading):.3f}°\n"
        f"• standard deviation: {np.std(heading):.5f}°\n"
        f"• fitted trend: {slope_deg_s * 60.0:.6f}°/min\n"
        f"• fitted 120-s change: {slope_deg_s * host_s[-1]:.6f}°\n"
        "• all four drive-encoder spans: 0.000°\n\n"
        "LIMITS\n"
        "No external truth angle, raw gyro rate, temperature sweep,\n"
        "power-cycle repeat, vibration test, or long-duration capture.\n"
        "This supports short-term dead reckoning during GPS outages;\n"
        "it does not certify absolute or long-term heading accuracy."
    )
    axes[1, 1].text(
        0.02,
        0.98,
        note,
        va="top",
        fontsize=12,
        linespacing=1.42,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#58d6ff"},
    )

    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.grid(alpha=0.16)
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"imu_stability_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
