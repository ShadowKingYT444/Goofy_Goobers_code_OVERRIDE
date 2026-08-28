#!/usr/bin/env python3
"""Compare representative stationary captures across sensor failure modes."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-25"
OUT = REPORT / "stationary_campaign_comparison"
IN_PER_M = 39.37007874015748
CAPTURES = [
    ("quiet prior", ROOT / "reports/sensor_campaign_2026-08-23/stationary_04_120s", False),
    ("P7 walking", ROOT / "reports/sensor_campaign_2026-08-23/reconnect_stationary_01", False),
    ("bad P6: 5 min", REPORT / "post_powercycle_stationary_5min_01", False),
    ("bad P6: next 10", REPORT / "post_powercycle_stationary_10min_02", False),
    ("warm P6: 12 min", REPORT / "imu_raw_rate_recalibration_12min_01", True),
]


def finite(rows: list[dict[str, str]], key: str) -> np.ndarray:
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return np.asarray(values, dtype=float)


def analyze(label: str, path: Path, enhanced_only: bool) -> tuple[dict[str, object], np.ndarray, np.ndarray]:
    with (path / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if enhanced_only:
        rows = [row for row in rows if row.get("imu_gyro_z") not in (None, "", "nan", "NaN")]
    host = finite(rows, "host_s")
    time_s = host - host[0]
    imu = finite(rows, "imu")
    gps_x = finite(rows, "gps_x") * IN_PER_M
    gps_y = finite(rows, "gps_y") * IN_PER_M
    gps_h = finite(rows, "gps_heading")
    p1_mm = finite(rows, "p1_mm")
    p1_conf = finite(rows, "p1_conf")
    n = min(time_s.size, imu.size)
    imu_slope = float(np.polyfit(time_s[:n], imu[:n], 1)[0] * 60.0) if n > 2 else math.nan
    gps_n = min(gps_x.size, gps_y.size)
    gps_endpoint = float(np.hypot(gps_x[gps_n - 1] - gps_x[0],
                                  gps_y[gps_n - 1] - gps_y[0])) if gps_n else math.nan
    physical = (p1_mm >= 20) & (p1_mm <= 2000)
    motor_spans = [float(np.ptp(finite(rows, key))) for key in ("m17", "m18", "m11", "m13")]
    result = {
        "label": label,
        "source": str(path.relative_to(ROOT)),
        "samples": len(rows),
        "duration_s": float(time_s[-1]) if time_s.size else 0.0,
        "maximum_motor_span_deg": max(motor_spans),
        "imu_span_deg": float(np.ptp(imu)),
        "imu_slope_deg_per_min": imu_slope,
        "gps_endpoint_in": gps_endpoint,
        "gps_x_span_in": float(np.ptp(gps_x)),
        "gps_y_span_in": float(np.ptp(gps_y)),
        "gps_heading_span_deg": float(np.ptp(gps_h)),
        "p1_physical_fraction": float(np.mean(physical)),
        "p1_median_in": float(np.median(p1_mm[physical]) / 25.4) if np.any(physical) else None,
        "p1_std_in": float(np.std(p1_mm[physical]) / 25.4) if np.any(physical) else None,
        "p1_confidence_median": float(np.median(p1_conf[physical])) if np.any(physical) else None,
    }
    return result, time_s[:n] / 60.0, imu[:n]


def main() -> None:
    analyzed = [analyze(*capture) for capture in CAPTURES]
    results = [item[0] for item in analyzed]
    OUT.mkdir(parents=True, exist_ok=True)
    payload = {
        "verdict": "stationary precision is conditional on sensor view and calibration quality",
        "captures": results,
        "limitations": [
            "no surveyed position or angle truth",
            "P1 scenes and targets differ across captures",
            "webcam did not prove physical stillness during the bad calibration",
        ],
    }
    (OUT / "summary.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    labels = [str(result["label"]) for result in results]
    colors = ["#66c2ff", "#ffb347", "#ff6b6b", "#ef476f", "#5cffad"]
    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(17, 10), constrained_layout=True)
    fig.suptitle("Stationary localization sensors: good states versus live failure states",
                 fontsize=19, fontweight="bold")

    for (result, time_min, imu), color in zip(analyzed, colors):
        axes[0, 0].plot(time_min, imu - imu[0], color=color, linewidth=1.6,
                        label=result["label"])
    axes[0, 0].set(title="P6 change from capture start", xlabel="capture minutes",
                   ylabel="relative rotation (deg)")
    axes[0, 0].grid(alpha=0.18)
    axes[0, 0].legend(frameon=False)

    x = np.arange(len(labels))
    gps_endpoint = [float(result["gps_endpoint_in"]) for result in results]
    axes[0, 1].bar(x, gps_endpoint, color=colors)
    axes[0, 1].set(title="P7 apparent endpoint motion while encoders were fixed",
                   ylabel="endpoint displacement (in)", xticks=x, xticklabels=labels)
    axes[0, 1].tick_params(axis="x", rotation=18)
    axes[0, 1].grid(axis="y", alpha=0.18)
    for index, value in enumerate(gps_endpoint):
        axes[0, 1].text(index, value, f"{value:.2f}", ha="center", va="bottom")

    imu_span = [max(float(result["imu_span_deg"]), 0.001) for result in results]
    axes[1, 0].bar(x, imu_span, color=colors)
    axes[1, 0].set_yscale("log")
    axes[1, 0].set(title="P6 peak-to-peak span (log scale)", ylabel="degrees",
                   xticks=x, xticklabels=labels)
    axes[1, 0].tick_params(axis="x", rotation=18)
    axes[1, 0].grid(axis="y", alpha=0.18, which="both")
    for index, (drawn, result) in enumerate(zip(imu_span, results)):
        axes[1, 0].text(index, drawn, f"{float(result['imu_span_deg']):.3g}°",
                        ha="center", va="bottom")

    availability = [100.0 * float(result["p1_physical_fraction"]) for result in results]
    axes[1, 1].bar(x, availability, color=colors)
    axes[1, 1].set(title="P1 physical-return availability is scene dependent",
                   ylabel="physical returns (%)", ylim=(0, 112), xticks=x,
                   xticklabels=labels)
    axes[1, 1].tick_params(axis="x", rotation=18)
    axes[1, 1].grid(axis="y", alpha=0.18)
    for index, (value, result) in enumerate(zip(availability, results)):
        median = result["p1_median_in"]
        note = f"{value:.1f}%"
        if median is not None:
            note += f"\nmed {float(median):.1f} in"
        axes[1, 1].text(index, value, note, ha="center", va="bottom")

    for suffix in ("png", "svg"):
        fig.savefig(OUT / f"comparison.{suffix}", dpi=180)
    plt.close(fig)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
