#!/usr/bin/env python3
"""Replay the two 2026-08-23 GPS failures through the current firmware gate."""

from __future__ import annotations

import csv
import json
import math
import re
import statistics
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt


OUT = Path(__file__).resolve().parent
CONFIG_TEXT = (OUT.parents[1] / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)
SOURCES = {
    "walking solution": (
        OUT / "post_gate_stationary_01" / "telemetry.csv", False
    ),
    # Hold the median of the actual second failure constant. This preserves its
    # observed wrong location/error while deliberately defeating the temporal
    # cluster reset, forcing the separate position-innovation gate to prove it
    # will reject a perfectly stable wrong fix.
    "constant wrong fix": (
        OUT / "fused_rotation_health_02" / "telemetry.csv", True
    ),
}
INCHES_PER_METER = 39.37007874015748
START = (30.18, 34.70, 151.65)

def config_number(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG_TEXT)
    if match is None:
        raise RuntimeError(f"missing production config constant {name}")
    return float(match.group(1))


# Read the production contract directly so the replay cannot silently retain
# stale copies of safety gates.
MAX_REPORTED_ERROR_IN = config_number("kGpsMaxReportedErrorIn")
CLUSTER_RADIUS_IN = config_number("kGpsMaxObservationStepIn")
CLUSTER_HEADING_DEG = config_number("kGpsMaxObservationHeadingStepDeg")
REQUIRED_OBSERVATIONS = int(config_number("kGpsRequiredConsistentObservations"))
NORMAL_INNOVATION_IN = config_number("kGpsMaxPositionInnovationIn")
REACQUIRE_OBSERVATIONS = int(config_number("kGpsRequiredReacquisitionObservations"))
REACQUIRE_INNOVATION_IN = config_number("kGpsMaxReacquisitionInnovationIn")
POSITION_GAIN = config_number("kGpsPositionGain")
MAX_POSITION_STEP_IN = config_number("kGpsMaxPositionStepIn")
HEADING_GATE_DEG = config_number("kGpsMaxHeadingInnovationDeg")
HEADING_GAIN = config_number("kGpsHeadingGain")
MAX_HEADING_STEP_DEG = config_number("kGpsMaxHeadingStepDeg")


def normalize(angle: float) -> float:
    return angle % 360.0


def angle_error(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def robot_pose(row: dict[str, str]) -> tuple[float, float, float, float]:
    # VEX native: +X field-right, +Y toward the 0-degree/top wall, heading
    # clockwise from top. Project: +X toward top, +Y toward red/left, heading
    # counterclockwise. Match include/gps_frame.hpp exactly.
    sensor_x = float(row["gps_y"]) * INCHES_PER_METER
    sensor_y = -float(row["gps_x"]) * INCHES_PER_METER
    sensor_cw = float(row["gps_heading"])
    robot_cw = normalize(sensor_cw - 90.0)
    heading = normalize(-robot_cw)
    heading_rad = math.radians(heading)
    forward = (math.cos(heading_rad), math.sin(heading_rad))
    right = (math.sin(heading_rad), -math.cos(heading_rad))
    center_x = sensor_x - (-6.0 * forward[0] + 6.0 * right[0])
    center_y = sensor_y - (-6.0 * forward[1] + 6.0 * right[1])
    return center_x, center_y, heading, float(row["gps_error"]) * INCHES_PER_METER


def firmware_rate_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    selected: list[dict[str, str]] = []
    last_ms = -1000
    for row in rows:
        robot_ms = int(row["robot_ms"])
        if robot_ms - last_ms >= 100:
            selected.append(row)
            last_ms = robot_ms
    return selected


def replay(rows: list[dict[str, str]], force_constant: bool) -> dict[str, object]:
    est_x, est_y, est_heading = START
    # The explicit start coordinate is an anchor from frame zero. This is the
    # regression condition that prevents a first stable-but-wrong GPS cluster
    # from teleporting the robot pose.
    anchored = True
    consistent = 0
    cluster_anchor: tuple[float, float, float] | None = None
    reasons: Counter[str] = Counter()
    records: list[dict[str, float | str]] = []

    measurements = [robot_pose(row) for row in rows]
    if force_constant:
        held = tuple(
            statistics.median(measurement[index] for measurement in measurements)
            for index in range(4)
        )
        measurements = [held] * len(measurements)

    for row, measurement in zip(rows, measurements):
        gps_x, gps_y, gps_heading, gps_error = measurement
        innovation = math.hypot(gps_x - est_x, gps_y - est_y)
        step = 0.0
        if not math.isfinite(gps_error) or gps_error > MAX_REPORTED_ERROR_IN:
            consistent = 0
            reason = "quality"
        else:
            reset = (
                consistent == 0
                or cluster_anchor is None
                or math.hypot(gps_x - cluster_anchor[0], gps_y - cluster_anchor[1])
                > CLUSTER_RADIUS_IN
                or abs(angle_error(gps_heading, cluster_anchor[2]))
                > CLUSTER_HEADING_DEG
            )
            if reset:
                consistent = 1
                cluster_anchor = (gps_x, gps_y, gps_heading)
            else:
                consistent += 1

            if consistent < REQUIRED_OBSERVATIONS:
                reason = "settling"
            elif not anchored:
                raise AssertionError("explicit start pose must already be anchored")
            else:
                normal = innovation <= NORMAL_INNOVATION_IN
                proven_reacquisition = (
                    innovation <= REACQUIRE_INNOVATION_IN
                    and consistent >= REACQUIRE_OBSERVATIONS
                )
                if not normal and not proven_reacquisition:
                    reason = "position_innovation"
                else:
                    requested = innovation * POSITION_GAIN
                    step = min(requested, MAX_POSITION_STEP_IN)
                    if innovation > 1e-9:
                        est_x += (gps_x - est_x) * step / innovation
                        est_y += (gps_y - est_y) * step / innovation
                    heading_delta = angle_error(gps_heading, est_heading)
                    if abs(heading_delta) <= HEADING_GATE_DEG:
                        est_heading = normalize(
                            est_heading
                            + max(
                                -MAX_HEADING_STEP_DEG,
                                min(MAX_HEADING_STEP_DEG,
                                    heading_delta * HEADING_GAIN),
                            )
                        )
                        reason = "corrected"
                    else:
                        reason = "heading_innovation_position_only"
        reasons[reason] += 1
        records.append({
            "t": float(row["host_s"]),
            "gps_x": gps_x,
            "gps_y": gps_y,
            "gps_error": gps_error,
            "innovation": innovation,
            "est_x": est_x,
            "est_y": est_y,
            "step": step,
            "reason": reason,
        })

    estimator_drift = [
        math.hypot(float(record["est_x"]) - START[0],
                   float(record["est_y"]) - START[1])
        for record in records
    ]
    return {
        "records": records,
        "summary": {
            "samples_at_10_hz": len(records),
            "raw_gps_max_innovation_in": max(
                float(record["innovation"]) for record in records
            ),
            "max_applied_step_in": max(float(record["step"]) for record in records),
            "max_estimator_drift_in": max(estimator_drift),
            "final_estimator_drift_in": estimator_drift[-1],
            "rejection_counts": dict(reasons),
        },
    }


results = {
    name: replay(firmware_rate_rows(path), force_constant)
    for name, (path, force_constant) in SOURCES.items()
}
summary = {name: result["summary"] for name, result in results.items()}
(OUT / "current_gps_gate_replay_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle("GPS Fail-Closed Regression Replay", fontsize=19, weight="bold")
fig.text(
    0.5,
    0.925,
    "Two live failures • explicit start pose anchored at frame zero • current 0.75 in cluster / 3 in innovation gates",
    ha="center",
    color="#475569",
)
colors = {"walking solution": "#0891b2", "constant wrong fix": "#dc2626"}
for name, result in results.items():
    records = result["records"]
    time_s = [float(record["t"]) - float(records[0]["t"]) for record in records]
    innovation = [float(record["innovation"]) for record in records]
    gps_error = [float(record["gps_error"]) for record in records]
    estimator_drift = [
        math.hypot(float(record["est_x"]) - START[0],
                   float(record["est_y"]) - START[1])
        for record in records
    ]
    steps = [float(record["step"]) for record in records]
    color = colors[name]
    axes[0, 0].plot(time_s, innovation, color=color, label=name)
    axes[0, 1].plot(time_s, gps_error, color=color, label=name)
    axes[1, 0].plot(time_s, estimator_drift, color=color, label=name)
    axes[1, 1].plot(time_s, steps, color=color, label=name)

axes[0, 0].axhline(NORMAL_INNOVATION_IN, color="#f97316", ls="--", label="normal gate")
axes[0, 0].axhline(REACQUIRE_INNOVATION_IN, color="#7c3aed", ls=":", label="reacquire gate")
axes[0, 0].set(title="A. Raw GPS innovation", xlabel="Elapsed time (s)", ylabel="Inches")
axes[0, 1].axhline(MAX_REPORTED_ERROR_IN, color="#f97316", ls="--", label="quality gate")
axes[0, 1].set(title="B. GPS self-reported RMS", xlabel="Elapsed time (s)", ylabel="Inches")
axes[1, 0].set(title="C. Fused estimator drift", xlabel="Elapsed time (s)", ylabel="Inches")
axes[1, 1].axhline(MAX_POSITION_STEP_IN, color="#f97316", ls="--", label="hard step limit")
axes[1, 1].set(title="D. Applied GPS correction", xlabel="Elapsed time (s)", ylabel="Inches/update")
for axis in axes.flat:
    axis.legend(fontsize=8)
fig.tight_layout(rect=(0.04, 0.05, 0.98, 0.90))
fig.savefig(OUT / "current_gps_gate_replay_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "current_gps_gate_replay_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
