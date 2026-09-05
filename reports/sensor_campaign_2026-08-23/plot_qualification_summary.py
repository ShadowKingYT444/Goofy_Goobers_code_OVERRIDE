#!/usr/bin/env python3
"""Build a notebook-ready summary of the live localization qualification."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent


def load(name: str) -> dict:
    return json.loads((OUT / name).read_text(encoding="utf-8"))


stationary = load("stationary_04_120s/summary.json")
drive = load("distance_multispeed_live_01/analysis_summary.json")
vision = load("ai_tag4_depth_summary.json")
gps_gate = load("current_gps_gate_replay_summary.json")
p1 = load("p1_reliability_summary.json")
gps_heading_policy = load("gps_heading_policy_replay_summary.json")

inch_per_meter = 39.37007874015748
gps_x_std_in = stationary["gps_x_m_valid"]["std"] * inch_per_meter
gps_y_std_in = stationary["gps_y_m_valid"]["std"] * inch_per_meter
gps_rms_median_in = stationary["gps_error_m_valid"]["median"] * inch_per_meter
imu_std_deg = stationary["imu_deg"]["std"]
gps_heading_std_deg = stationary["gps_heading_deg_valid"]["std"]

summary = {
    "stationary_120s": {
        "gps_x_std_in": gps_x_std_in,
        "gps_y_std_in": gps_y_std_in,
        "gps_heading_std_deg": gps_heading_std_deg,
        "gps_reported_rms_median_in": gps_rms_median_in,
        "imu_heading_std_deg": imu_std_deg,
    },
    "drive_encoder_qualification": {
        "trials": drive["trial_count"],
        "encoder_scale": drive["overall_encoder_to_gps_scale"],
        "fit_rmse_in": drive["overall_fit_rmse_in"],
        "max_35rpm_overshoot_in": drive["per_speed_rpm"]["35"]["max_encoder_overshoot_in"],
        "max_return_residual_in": drive["max_gps_return_residual_in"],
    },
    "heading_motion": {
        "max_imu_commanded_return_residual_deg": 0.91,
        "fused_four_leg_final_error_deg": 0.088,
        "max_observed_gps_heading_excursion_deg": 8.55,
        "gps_false_rotation_displacement_in": 15.63,
        "old_gps_heading_policy_accumulated_bias_deg": gps_heading_policy[
            "maximum_accumulated_bias_deg"
        ],
        "production_gps_heading_gain": 0.0,
    },
    "ai_vision_tag4": {
        "samples": vision["samples"],
        "pnp_range_median_in": vision["pnp_range_in"]["median"],
        "pnp_range_std_in": vision["pnp_range_in"]["std"],
        "bearing_std_deg": vision["pnp_bearing_deg"]["std"],
        "reprojection_rmse_median_px": vision["reprojection_rmse_px"]["median"],
        "absolute_accuracy": "not measured against tape; pose correction disabled",
    },
    "fault_replay": gps_gate,
    "forward_distance": {
        "stationary_physical_return_fraction": p1[
            "stationary_physical_return_fraction"
        ],
        "stationary_precision_usable_fraction": p1[
            "stationary_precision_usable_fraction"
        ],
        "maximum_return_burst_s": p1["return_bursts"]["maximum_duration_s"],
    },
    "disabled_sensor": "P5 lateral tracker: <=0.002 in through live motion",
}
(OUT / "sensor_qualification_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.grid": True,
    "grid.alpha": 0.2,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
navy, cyan, orange, green, red, purple, gray = (
    "#172554", "#0891b2", "#f97316", "#16a34a", "#dc2626", "#7c3aed", "#64748b"
)
fig, axes = plt.subplots(2, 2, figsize=(15.5, 10.5))
fig.suptitle("Multi-Sensor Localization Qualification", fontsize=20, weight="bold", color=navy)
fig.text(
    0.5,
    0.925,
    "Live VEX V5 data • 2026-08-23 • precision, motion error, and fail-closed behavior shown separately",
    ha="center",
    color="#475569",
)

ax = axes[0, 0]
labels = ["GPS X\nstd", "GPS Y\nstd", "GPS reported\nRMS (median)"]
values = [gps_x_std_in, gps_y_std_in, gps_rms_median_in]
bars = ax.bar(labels, values, color=[cyan, cyan, gray])
ax.set_yscale("log")
ax.set_ylabel("Inches (log scale)")
ax.set_title("A. 120-second stationary GPS precision")
for bar, value in zip(bars, values):
    ax.text(bar.get_x() + bar.get_width() / 2, value * 1.18, f"{value:.4f}", ha="center", fontsize=9)

ax = axes[0, 1]
labels = ["IMU stationary\nstd", "GPS stationary\nheading std", "Fused 4-leg\nreturn", "Commanded sweep\nreturn residual*", "GPS motion\nexcursion"]
values = [imu_std_deg, gps_heading_std_deg, 0.088, 0.91, 8.55]
bars = ax.bar(labels, values, color=[green, cyan, green, orange, red])
ax.set_yscale("log")
ax.set_ylabel("Degrees (log scale)")
ax.set_title("B. Heading: quiet precision vs. motion reliability")
for bar, value in zip(bars, values):
    ax.text(bar.get_x() + bar.get_width() / 2, value * 1.2, f"{value:.3g}°", ha="center", fontsize=8)

ax = axes[1, 0]
labels = ["P7-ref encoder fit\nRMSE", "10 RPM\novershoot", "20 RPM\novershoot", "35 RPM\novershoot", "Max GPS\nreturn residual"]
values = [
    drive["overall_fit_rmse_in"],
    drive["per_speed_rpm"]["10"]["max_encoder_overshoot_in"],
    drive["per_speed_rpm"]["20"]["max_encoder_overshoot_in"],
    drive["per_speed_rpm"]["35"]["max_encoder_overshoot_in"],
    drive["max_gps_return_residual_in"],
]
bars = ax.bar(labels, values, color=[purple, green, orange, red, gray])
ax.set_ylabel("Inches")
ax.set_title("C. Nine straight trials: speed changes stopping error")
for bar, value in zip(bars, values):
    ax.text(bar.get_x() + bar.get_width() / 2, value + 0.05, f"{value:.2f}", ha="center", fontsize=9)

ax = axes[1, 1]
labels = ["Walking GPS\nraw innovation", "Walking GPS\nfused drift", "Constant wrong\nraw innovation", "Constant wrong\nfused drift"]
values = [
    gps_gate["walking solution"]["raw_gps_max_innovation_in"],
    gps_gate["walking solution"]["max_estimator_drift_in"],
    gps_gate["constant wrong fix"]["raw_gps_max_innovation_in"],
    gps_gate["constant wrong fix"]["max_estimator_drift_in"],
]
display_values = [value if value > 0 else 0.06 for value in values]
bars = ax.bar(labels, display_values, color=[red, green, red, green])
ax.set_ylabel("Inches")
ax.set_title("D. Current GPS gate fails closed")
for bar, value in zip(bars, values):
    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1.2,
            f"{value:.1f}" if value >= 1 else f"{value:.1f}", ha="center", fontsize=9, weight="bold")

fig.text(
    0.5,
    0.032,
    "Fusion policy: provisional P7-referenced encoders propagate X/Y • P6 IMU owns heading/turns • P7 GPS is bounded position-only correction\n"
    "P1 stops at 8 in • P8 tag PnP remains diagnostic until mount geometry is measured • P5 tracker disabled\n"
    "*Commanded return residual is not externally measured IMU accuracy.",
    ha="center",
    fontsize=8.5,
    color=navy,
    weight="bold",
)
fig.tight_layout(rect=(0.04, 0.095, 0.98, 0.90), h_pad=3.0, w_pad=2.0)
fig.savefig(OUT / "sensor_qualification_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "sensor_qualification_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
