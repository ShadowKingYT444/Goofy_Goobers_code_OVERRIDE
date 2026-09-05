#!/usr/bin/env python3
"""Replay real GPS noise through the firmware's bounded correction policy."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
CSV_PATH = OUT / "stationary_04_120s" / "telemetry.csv"
INCHES_PER_METER = 39.37007874015748


def normalize(angle: float) -> float:
    return angle % 360.0


def angle_error(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def robot_pose(row: dict[str, str]) -> tuple[float, float, float, float]:
    # Keep this replay in the same axes as include/gps_frame.hpp.
    sensor_x = float(row["gps_y"]) * INCHES_PER_METER
    sensor_y = -float(row["gps_x"]) * INCHES_PER_METER
    sensor_cw = float(row["gps_heading"])
    robot_cw = normalize(sensor_cw - 90.0)
    heading = normalize(-robot_cw)
    rad = math.radians(heading)
    forward = (math.cos(rad), math.sin(rad))
    right = (math.sin(rad), -math.cos(rad))
    # Sensor is 6 in right and 6 in behind the robot center.
    center_x = sensor_x - (-6.0 * forward[0] + 6.0 * right[0])
    center_y = sensor_y - (-6.0 * forward[1] + 6.0 * right[1])
    return center_x, center_y, heading, float(row["gps_error"]) * INCHES_PER_METER


with CSV_PATH.open(newline="", encoding="utf-8") as handle:
    # Every second raw frame gives an 8.33-Hz replay, close to the firmware's
    # 100-ms correction period without inventing interpolated observations.
    source = list(csv.DictReader(handle))[::2]

t = np.asarray([float(row["host_s"]) for row in source])
t -= t[0]
truth = np.asarray([robot_pose(row) for row in source])

est_x, est_y, est_heading = 30.18, 34.70, 151.65
anchored = False
consistent = 0
pending = None
last_t = t[0]
drift_velocity = np.asarray([0.65, -0.35])  # deliberately severe, in/s
heading_drift_deg_s = 0.12

records: list[dict[str, float | str | bool]] = []
for index, (now, measurement) in enumerate(zip(t, truth)):
    dt = max(0.0, now - last_t)
    last_t = now
    if anchored:
        est_x += drift_velocity[0] * dt
        est_y += drift_velocity[1] * dt
        est_heading = normalize(est_heading + heading_drift_deg_s * dt)

    gps_x, gps_y, gps_heading, gps_error = measurement
    valid = True
    injected = "normal"
    if 30.0 <= now < 50.0:
        valid = False
        injected = "visual_dropout"
    elif 60.0 <= now < 70.0:
        gps_error = 10.0
        valid = False
        injected = "reported_error_spike"
    elif 84.9 <= now < 85.1:
        gps_x += 30.0
        gps_y -= 30.0
        injected = "single_position_outlier"

    reason = "quality"
    position_step = 0.0
    heading_step = 0.0
    innovation = math.nan
    if not valid or gps_error > 1.5:
        consistent = 0
    else:
        candidate = (gps_x, gps_y, gps_heading)
        if (
            consistent == 0
            or pending is None
            or math.hypot(gps_x - pending[0], gps_y - pending[1]) > 8.0
            or abs(angle_error(gps_heading, pending[2])) > 15.0
        ):
            consistent = 1
        else:
            consistent += 1
        pending = candidate

        if consistent < 3:
            reason = "settling"
        elif not anchored:
            est_x, est_y, est_heading = gps_x, gps_y, gps_heading
            anchored = True
            reason = "anchored"
        else:
            dx, dy = gps_x - est_x, gps_y - est_y
            innovation = math.hypot(dx, dy)
            normal = innovation <= 12.0
            reacquired = innovation <= 48.0 and consistent >= 12
            if not normal and not reacquired:
                reason = "position_innovation"
            else:
                requested = 0.0 if innovation <= 0.05 else innovation * 0.20
                position_step = min(requested, 0.50)
                if innovation > 1e-9:
                    est_x += dx * position_step / innovation
                    est_y += dy * position_step / innovation
                heading_error = angle_error(gps_heading, est_heading)
                if abs(heading_error) <= 10.0:
                    heading_step = (
                        0.0
                        if abs(heading_error) <= 0.20
                        else max(-0.50, min(0.50, heading_error * 0.10))
                    )
                    est_heading = normalize(est_heading + heading_step)
                    reason = "corrected"
                else:
                    reason = "heading_innovation_position_only"

    true_x, true_y, true_heading = truth[index, :3]
    records.append({
        "t": now,
        "x_error": math.hypot(est_x - true_x, est_y - true_y),
        "heading_error": abs(angle_error(est_heading, true_heading)),
        "innovation": innovation,
        "position_step": position_step,
        "heading_step": heading_step,
        "reason": reason,
        "injected": injected,
        "valid": valid,
    })

times = np.asarray([float(row["t"]) for row in records])
position_error = np.asarray([float(row["x_error"]) for row in records])
heading_error = np.asarray([float(row["heading_error"]) for row in records])
position_steps = np.asarray([float(row["position_step"]) for row in records])
heading_steps = np.asarray([float(row["heading_step"]) for row in records])
innovations = np.asarray([float(row["innovation"]) for row in records])

summary = {
    "frames": len(records),
    "duration_s": float(times[-1]),
    "max_position_correction_step_in": float(np.max(position_steps)),
    "max_heading_correction_step_deg": float(np.max(np.abs(heading_steps))),
    "max_position_error_after_anchor_in": float(np.max(position_error[3:])),
    "final_position_error_in": float(position_error[-1]),
    "final_heading_error_deg": float(heading_error[-1]),
    "rejection_counts": dict(Counter(str(row["reason"]) for row in records)),
    "injected_fault_counts": dict(Counter(str(row["injected"]) for row in records)),
}
(OUT / "gps_fault_replay_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
navy, cyan, orange, green, red, purple = (
    "#172554", "#0891b2", "#f97316", "#16a34a", "#dc2626", "#7c3aed"
)
fig, axes = plt.subplots(2, 2, figsize=(14, 10.5))
fig.subplots_adjust(left=0.08, right=0.94, top=0.87, bottom=0.11,
                    hspace=0.34, wspace=0.28)
fig.suptitle("GPS Fusion Fault-Injection Replay", fontsize=19, color=navy, y=0.965)
fig.text(
    0.5, 0.925,
    "Real P7 noise • synthetic severe odometry drift • bounded reacquisition • no robot motion",
    ha="center", fontsize=10, color="#475569",
)

for ax in axes.flat:
    ax.axvspan(30, 50, color=red, alpha=0.08)
    ax.axvspan(60, 70, color=orange, alpha=0.10)
    ax.axvline(85, color=purple, ls="--", alpha=0.55)

ax = axes[0, 0]
ax.plot(times, position_error, color=cyan, lw=2)
ax.axhline(12, color=red, ls="--", label="Normal innovation gate")
ax.axhline(48, color="#991b1b", ls=":", label="Proven-reacquisition gate")
ax.set(xlabel="Elapsed time (s)", ylabel="Estimator position error (in)",
       title="A. Position error through dropouts")
ax.legend(fontsize=8)

ax = axes[0, 1]
ax.plot(times, heading_error, color=orange, lw=2)
ax.axhline(10, color=red, ls="--", label="Heading innovation gate")
ax.set(xlabel="Elapsed time (s)", ylabel="Absolute heading error (deg)",
       title="B. Heading continuity")
ax.legend(fontsize=8)

ax = axes[1, 0]
ax.plot(times, position_steps, color=green, lw=1.5, label="Position step")
ax.plot(times, np.abs(heading_steps), color=purple, lw=1.2, label="|Heading step|")
ax.axhline(0.5, color=red, ls="--", label="Both hard limits")
ax.set(xlabel="Elapsed time (s)", ylabel="Applied correction per update",
       title="C. Corrections remain bounded")
ax.legend(fontsize=8)

ax = axes[1, 1]
finite_innovation = np.isfinite(innovations)
ax.plot(times[finite_innovation], innovations[finite_innovation], color=navy, lw=1.5)
ax.axhline(12, color=red, ls="--", label="Normal gate")
ax.axhline(48, color="#991b1b", ls=":", label="Reacquisition gate")
ax.set(xlabel="Elapsed time (s)", ylabel="GPS innovation (in)",
       title="D. Innovation and outlier rejection")
ax.legend(fontsize=8)

fig.text(
    0.5, 0.035,
    f"Max applied steps: {summary['max_position_correction_step_in']:.2f} in / "
    f"{summary['max_heading_correction_step_deg']:.2f}° | "
    f"Final error: {summary['final_position_error_in']:.2f} in / "
    f"{summary['final_heading_error_deg']:.2f}°",
    ha="center", fontsize=10, weight="bold", color=navy,
)
fig.savefig(OUT / "gps_fault_replay_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "gps_fault_replay_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
