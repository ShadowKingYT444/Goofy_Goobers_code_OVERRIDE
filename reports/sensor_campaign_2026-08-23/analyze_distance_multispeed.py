#!/usr/bin/env python3
"""Analyze the 2026-08-23 multi-speed straight out-and-back sweep."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
RUN = ROOT / "distance_multispeed_live_01"
RAW = RUN / "raw.log"
OUT_CSV = RUN / "endpoint_results.csv"
OUT_JSON = RUN / "analysis_summary.json"
OUT_PNG = RUN / "distance_multispeed_dashboard.png"
OUT_SVG = RUN / "distance_multispeed_dashboard.svg"

BACK_RE = re.compile(
    rb"SWEEP rpm=(?P<rpm>[\d.]+) target_in=(?P<target>[\d.]+) phase=back "
    rb"encoder_in=(?P<encoder>[\d.-]+) gps_in=(?P<gps>[\d.-]+) "
    rb"difference_in=(?P<difference>[\d.-]+) gps_heading_delta=(?P<gps_heading>[\d.-]+) "
    rb"gps_error_in=(?P<gps_error>[\d.-]+) imu_delta_deg=(?P<imu_heading>[\d.-]+) "
    rb"imu_max_abs_deg=(?P<imu_max>[\d.-]+) left_deg=(?P<left_deg>[\d.-]+) "
    rb"right_deg=(?P<right_deg>[\d.-]+) drive_side_diff_deg=(?P<side_diff>[\d.-]+)"
)
RETURN_RE = re.compile(
    rb"SWEEP rpm=(?P<rpm>[\d.]+) target_in=(?P<target>[\d.]+) phase=return "
    rb"encoder_residual_in=(?P<encoder_residual>[\d.-]+) "
    rb"gps_residual_in=(?P<gps_residual>[\d.-]+) "
    rb"gps_heading_delta=(?P<gps_heading_residual>[\d.-]+) "
    rb"gps_error_in=(?P<gps_error_return>[\d.-]+) "
    rb"imu_residual_deg=(?P<imu_heading_residual>[\d.-]+) "
    rb"imu_max_abs_deg=(?P<imu_max_return>[\d.-]+) "
    rb"left_deg=(?P<left_residual_deg>[\d.-]+) "
    rb"right_deg=(?P<right_residual_deg>[\d.-]+) "
    rb"drive_side_diff_deg=(?P<side_residual_diff>[\d.-]+)"
)


def parse() -> list[dict[str, float]]:
    raw = RAW.read_bytes()
    rows: dict[tuple[float, float], dict[str, float]] = {}
    for regex in (BACK_RE, RETURN_RE):
        for match in regex.finditer(raw):
            values = {
                name: float(value)
                for name, value in match.groupdict().items()
            }
            key = (values["rpm"], values["target"])
            rows.setdefault(key, {}).update(values)
    return [rows[key] for key in sorted(rows)]


def fit_scale(rows: list[dict[str, float]]) -> tuple[float, float]:
    encoder = np.array([row["encoder"] for row in rows])
    gps = np.array([row["gps"] for row in rows])
    scale = float(np.dot(encoder, gps) / np.dot(encoder, encoder))
    rmse = float(np.sqrt(np.mean((gps - scale * encoder) ** 2)))
    return scale, rmse


def main() -> None:
    rows = parse()
    if len(rows) != 9 or any("gps_residual" not in row for row in rows):
        raise SystemExit(f"expected 9 complete trials, found {len(rows)}")

    fields = list(rows[0])
    with OUT_CSV.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    overall_scale, overall_rmse = fit_scale(rows)
    speeds = sorted({row["rpm"] for row in rows})
    per_speed = {}
    for rpm in speeds:
        selected = [row for row in rows if row["rpm"] == rpm]
        scale, rmse = fit_scale(selected)
        per_speed[str(int(rpm))] = {
            "encoder_to_gps_scale": scale,
            "effective_wheel_diameter_in": 2.75 * scale,
            "fit_rmse_in": rmse,
            "max_encoder_overshoot_in": max(row["encoder"] - row["target"] for row in selected),
            "max_gps_return_residual_in": max(row["gps_residual"] for row in selected),
        }

    summary = {
        "trial_count": len(rows),
        "overall_encoder_to_gps_scale": overall_scale,
        "overall_effective_wheel_diameter_in": 2.75 * overall_scale,
        "overall_fit_rmse_in": overall_rmse,
        "per_speed_rpm": per_speed,
        "max_gps_return_residual_in": max(row["gps_residual"] for row in rows),
        "max_encoder_return_residual_in": max(row["encoder_residual"] for row in rows),
        "max_imu_heading_excursion_deg": max(row["imu_max_return"] for row in rows),
        "max_drive_side_mismatch_deg": max(abs(row["side_diff"]) for row in rows),
        "tracking_wheel_max_abs_in": 0.002,
        "interpretation": (
            "GPS consistently reports less travel than the nominal 2.75-inch encoder model. "
            "The GPS-referenced scale is relatively consistent at 10-20 RPM; 35 RPM adds "
            "braking overshoot and return error. This is not independent ground-truth calibration."
        ),
        "limitations": [
            "P7 GPS, not tape/laser metrology, is the translation reference.",
            "P7 later showed strong view-dependent position failures.",
            "The endpoint includes open-loop braking/coast and target-dependent wheel behavior.",
        ],
    }
    OUT_JSON.write_text(json.dumps(summary, indent=2) + "\n")

    plt.style.use("dark_background")
    colors = {10.0: "#59d8ff", 20.0: "#88ef9b", 35.0: "#ffb45e"}
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.5), constrained_layout=True)
    fig.suptitle("Straight-Line Calibration: GPS vs Drive Encoders", fontsize=18)

    ax = axes[0, 0]
    for rpm in speeds:
        selected = [row for row in rows if row["rpm"] == rpm]
        ax.plot([r["target"] for r in selected], [r["encoder"] for r in selected], "o-", color=colors[rpm], label=f"{rpm:.0f} RPM encoder")
        ax.plot([r["target"] for r in selected], [r["gps"] for r in selected], "s--", color=colors[rpm], alpha=0.7, label=f"{rpm:.0f} RPM GPS")
    ax.plot([0, 11.5], [0, 11.5], color="0.55", linestyle=":", label="commanded")
    ax.set(title="Endpoint travel", xlabel="target (in)", ylabel="measured travel (in)")
    ax.grid(alpha=0.2)
    ax.legend(fontsize=8, ncol=2)

    ax = axes[0, 1]
    for rpm in speeds:
        selected = [row for row in rows if row["rpm"] == rpm]
        ax.scatter([r["encoder"] for r in selected], [r["gps"] for r in selected], s=65, color=colors[rpm], label=f"{rpm:.0f} RPM")
    xx = np.linspace(0, 12, 100)
    ax.plot(xx, xx, color="0.55", linestyle=":", label="1:1")
    ax.plot(xx, overall_scale * xx, color="#d58aff", label=f"fit: GPS = {overall_scale:.4f} × encoder")
    ax.set(title=f"Scale fit (RMSE {overall_rmse:.3f} in)", xlabel="nominal encoder travel (in)", ylabel="GPS travel (in)")
    ax.grid(alpha=0.2)
    ax.legend(fontsize=9)

    ax = axes[1, 0]
    x = np.arange(len(rows))
    labels = [f"{r['target']:.0f}\"\n@{r['rpm']:.0f}" for r in rows]
    ax.bar(x - 0.18, [r["encoder_residual"] for r in rows], 0.36, label="encoder", color="#59d8ff")
    ax.bar(x + 0.18, [r["gps_residual"] for r in rows], 0.36, label="GPS", color="#ff8f91")
    ax.set_xticks(x, labels)
    ax.set(title="Return-to-start residual", xlabel="target inches / RPM", ylabel="absolute residual (in)")
    ax.grid(axis="y", alpha=0.2)
    ax.legend()

    ax = axes[1, 1]
    ax.plot(x, [abs(r["imu_heading"]) for r in rows], "o-", label="IMU at endpoint", color="#88ef9b")
    ax.plot(x, [r["imu_max_return"] for r in rows], "s--", label="max IMU excursion", color="#ffb45e")
    ax.plot(x, [abs(r["gps_heading"]) for r in rows], "^:", label="GPS at endpoint", color="#d58aff")
    ax.set_xticks(x, labels)
    ax.set(title="Unwanted yaw during straight travel", xlabel="target inches / RPM", ylabel="absolute heading change (deg)")
    ax.grid(alpha=0.2)
    ax.legend()

    fig.savefig(OUT_PNG, dpi=180)
    fig.savefig(OUT_SVG)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
