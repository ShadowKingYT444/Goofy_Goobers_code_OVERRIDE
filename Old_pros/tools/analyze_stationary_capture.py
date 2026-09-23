#!/usr/bin/env python3
"""Analyze one capture_sensor_log stationary telemetry directory."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


IN_PER_M = 39.37007874015748


def finite_column(rows: list[dict[str, str]], key: str) -> np.ndarray:
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return np.asarray(values, dtype=float)


def stats(values: np.ndarray) -> dict[str, float | int | None]:
    if values.size == 0:
        return {"count": 0, "min": None, "median": None, "max": None,
                "std": None, "peak_to_peak": None}
    return {
        "count": int(values.size),
        "min": float(values.min()),
        "median": float(np.median(values)),
        "max": float(values.max()),
        "std": float(values.std()),
        "peak_to_peak": float(np.ptp(values)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument(
        "--require-enhanced-imu",
        action="store_true",
        help="discard pre-upload rows that lack raw P6 gyro/acceleration fields",
    )
    args = parser.parse_args()
    capture = args.capture.resolve()
    with (capture / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    original_row_count = len(rows)
    if args.require_enhanced_imu:
        rows = [
            row for row in rows
            if row.get("imu_gyro_z") not in (None, "", "nan", "NaN")
        ]
    if len(rows) < 2:
        raise SystemExit("capture has fewer than two telemetry rows")

    host = finite_column(rows, "host_s")
    time_s = host - host[0]
    gps_x = finite_column(rows, "gps_x") * IN_PER_M
    gps_y = finite_column(rows, "gps_y") * IN_PER_M
    gps_h = finite_column(rows, "gps_heading")
    gps_rms = finite_column(rows, "gps_error") * IN_PER_M
    gps_gyro_z = finite_column(rows, "gps_gyro_z")
    imu = finite_column(rows, "imu")
    raw_imu = finite_column(rows, "rawimu")
    imu_gyro_x = finite_column(rows, "imu_gyro_x")
    imu_gyro_y = finite_column(rows, "imu_gyro_y")
    imu_gyro_z = finite_column(rows, "imu_gyro_z")
    imu_acc_x = finite_column(rows, "imu_acc_x")
    imu_acc_y = finite_column(rows, "imu_acc_y")
    imu_acc_z = finite_column(rows, "imu_acc_z")
    p1 = finite_column(rows, "p1_mm") / 25.4
    p1_conf = finite_column(rows, "p1_conf")
    motors = {key: finite_column(rows, key) for key in ("m17", "m18", "m11", "m13")}

    count = min(time_s.size, gps_x.size, gps_y.size)
    gps_t = time_s[:count]
    gps_x = gps_x[:count]
    gps_y = gps_y[:count]
    center_x = float(np.median(gps_x))
    center_y = float(np.median(gps_y))
    radius = np.hypot(gps_x - center_x, gps_y - center_y)
    endpoint = float(math.hypot(gps_x[-1] - gps_x[0], gps_y[-1] - gps_y[0]))
    slope_x = float(np.polyfit(gps_t, gps_x, 1)[0]) if count > 2 else math.nan
    slope_y = float(np.polyfit(gps_t, gps_y, 1)[0]) if count > 2 else math.nan
    imu_t = time_s[:imu.size]
    imu_slope = float(np.polyfit(imu_t, imu, 1)[0]) if imu.size > 2 else math.nan
    gps_h_t = time_s[:gps_h.size]
    gps_h_slope = (
        float(np.polyfit(gps_h_t, gps_h, 1)[0]) if gps_h.size > 2 else math.nan
    )
    imu_windows = []
    for start_s in np.arange(0.0, max(float(time_s[-1]), 0.0) + 1e-9, 30.0):
        mask = (imu_t >= start_s) & (imu_t < start_s + 30.0)
        if np.any(mask):
            imu_windows.append({
                "start_s": float(start_s),
                "end_s": float(min(start_s + 30.0, time_s[-1])),
                "median_deg": float(np.median(imu[mask])),
                "min_deg": float(np.min(imu[mask])),
                "max_deg": float(np.max(imu[mask])),
            })

    fusion_rejects: dict[str, int] = {}
    ai_rejects: dict[str, int] = {}
    fusion_path = capture / "fusion.jsonl"
    if fusion_path.exists():
        for line in fusion_path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                continue
            gps_reason = str(frame.get("gps_reject", "missing"))
            ai_reason = str(frame.get("ai_reject", "missing"))
            fusion_rejects[gps_reason] = fusion_rejects.get(gps_reason, 0) + 1
            ai_rejects[ai_reason] = ai_rejects.get(ai_reason, 0) + 1

    summary = {
        "capture": str(capture),
        "samples": len(rows),
        "original_samples": original_row_count,
        "selection": "enhanced_imu_only" if args.require_enhanced_imu else "all_rows",
        "elapsed_s": float(time_s[-1]),
        "gps_x_in": stats(gps_x),
        "gps_y_in": stats(gps_y),
        "gps_heading_deg": stats(gps_h),
        "gps_reported_rms_in": stats(gps_rms),
        "gps_raw_gyro_z_dps": stats(gps_gyro_z),
        "gps_endpoint_displacement_in": endpoint,
        "gps_radial_error_about_median_in": stats(radius),
        "gps_linear_slope_in_per_min": {"x": slope_x * 60.0, "y": slope_y * 60.0},
        "imu_heading_deg": stats(imu),
        "raw_imu_heading_deg": stats(raw_imu),
        "imu_gyro_dps": {
            "x": stats(imu_gyro_x), "y": stats(imu_gyro_y), "z": stats(imu_gyro_z)
        },
        "imu_acceleration_g": {
            "x": stats(imu_acc_x), "y": stats(imu_acc_y), "z": stats(imu_acc_z)
        },
        "imu_linear_slope_deg_per_min": imu_slope * 60.0,
        "gps_heading_linear_slope_deg_per_min": gps_h_slope * 60.0,
        "imu_30s_windows": imu_windows,
        "p1_range_in": stats(p1[p1 < 9999.0 / 25.4]),
        "p1_confidence": stats(p1_conf),
        "drive_encoder_span_deg": {key: float(np.ptp(value)) for key, value in motors.items()},
        "stationarity_corroboration": {
            "all_drive_encoder_spans_zero": all(np.ptp(value) == 0.0 for value in motors.values()),
            "gps_heading_span_deg": float(np.ptp(gps_h)),
            "gps_endpoint_displacement_in": endpoint,
            "webcam_robot_visible": False,
        },
        "gps_reject_counts": fusion_rejects,
        "ai_reject_counts": ai_rejects,
        "limitations": [
            "stationary precision is not absolute accuracy",
            "P1 has no tape-measured target truth in this capture",
            "GPS slopes are descriptive linear fits, not guaranteed future drift rates",
            "camera is outside this telemetry stream and P8 saw no tag unless ai_reject_counts says otherwise",
        ],
    }
    (capture / "stationary_analysis.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    plt.rcParams.update({"axes.grid": True, "grid.alpha": 0.25})
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    fig.suptitle("Post-power-cycle stationary sensor characterization", fontsize=17)
    axes[0, 0].plot(gps_x, gps_y, color="#dc2626", linewidth=1)
    axes[0, 0].scatter([gps_x[0], gps_x[-1]], [gps_y[0], gps_y[-1]],
                       c=["#16a34a", "#2563eb"], label="start/end")
    axes[0, 0].set(title="P7 apparent XY motion while encoders are still",
                   xlabel="native GPS X (in)", ylabel="native GPS Y (in)")
    axes[0, 0].axis("equal")
    axes[0, 0].legend()
    axes[0, 1].plot(time_s[:imu.size], imu, label="P6 IMU", color="#7c3aed")
    axes[0, 1].plot(time_s[:gps_h.size], gps_h - np.median(gps_h),
                    label="P7 heading - median", color="#0891b2")
    axes[0, 1].set(title="Heading stability", xlabel="seconds", ylabel="degrees")
    axes[0, 1].legend()
    axes[1, 0].plot(time_s[:p1.size], p1, color="#f97316")
    axes[1, 0].set(title="P1 forward range", xlabel="seconds", ylabel="inches")
    axes[1, 1].plot(time_s[:p1_conf.size], p1_conf, color="#16a34a")
    axes[1, 1].set(title="P1 confidence", xlabel="seconds", ylabel="0-63")
    fig.savefig(capture / "stationary_dashboard.png", dpi=180)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
