#!/usr/bin/env python3
"""Plot live GPS quality and heading consistency versus turn orientation."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
RAW = OUT / "rotation_sweep_live_02" / "raw.log"
CONFIG_TEXT = (OUT.parents[1] / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)


def config_number(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG_TEXT)
    if match is None:
        raise RuntimeError(f"missing production config constant {name}")
    return float(match.group(1))


GPS_RMS_GATE_IN = config_number("kGpsMaxReportedErrorIn")
GPS_HEADING_GATE_DEG = config_number("kGpsMaxHeadingInnovationDeg")
TURN = re.compile(
    r"ROT_SWEEP target_deg=(?P<target>[\d.]+) phase=turn direction=(?P<direction>\w+) "
    r"encoder_deg=(?P<encoder>[\d.-]+) gps_deg=(?P<gps>[\d.-]+) "
    r"imu_deg=(?P<imu>[\d.-]+).*?gps_error_in=(?P<error>[\d.-]+)"
)
RETURN = re.compile(
    r"ROT_SWEEP target_deg=(?P<target>[\d.]+) phase=return .*?"
    r"gps_heading_residual_deg=(?P<heading_residual>[\d.-]+) "
    r"imu_heading_residual_deg=(?P<imu_residual>[\d.-]+) "
    r"gps_position_residual_in=(?P<position_residual>[\d.-]+)"
)


text = RAW.read_bytes().decode("utf-8", errors="ignore")
rows: dict[float, dict[str, float | str]] = {}
for match in TURN.finditer(text):
    values = match.groupdict()
    target = float(values["target"])
    gps = float(values["gps"])
    imu = float(values["imu"])
    rows[target] = {
        "target_deg": target,
        "direction": values["direction"],
        "encoder_delta_deg": float(values["encoder"]),
        "gps_delta_deg": gps,
        "imu_delta_deg": imu,
        "gps_reported_error_in": float(values["error"]),
        "gps_imu_disagreement_deg": abs(abs(gps) - abs(imu)),
    }
for match in RETURN.finditer(text):
    values = match.groupdict()
    target = float(values["target"])
    if target not in rows:
        continue
    rows[target].update({
        "gps_heading_return_residual_deg": float(values["heading_residual"]),
        "imu_heading_return_residual_deg": float(values["imu_residual"]),
        "gps_position_return_residual_in": float(values["position_residual"]),
    })
ordered = [rows[target] for target in sorted(rows)]
if len(ordered) != 5:
    raise SystemExit(f"expected five completed angles, found {len(ordered)}")

with (OUT / "gps_orientation_reliability.csv").open("w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(ordered[0]))
    writer.writeheader()
    writer.writerows(ordered)

summary = {
    "angles_deg_from_test_start": [row["target_deg"] for row in ordered],
    "gps_rms_gate_in": GPS_RMS_GATE_IN,
    "gps_heading_innovation_gate_deg": GPS_HEADING_GATE_DEG,
    "rms_rejected_angles_deg": [
        row["target_deg"] for row in ordered
        if row["gps_reported_error_in"] > GPS_RMS_GATE_IN
    ],
    "heading_rejected_angles_deg": [
        row["target_deg"] for row in ordered
        if row["gps_imu_disagreement_deg"] > GPS_HEADING_GATE_DEG
    ],
    "max_reported_error_in": max(row["gps_reported_error_in"] for row in ordered),
    "max_gps_imu_disagreement_deg": max(row["gps_imu_disagreement_deg"] for row in ordered),
    "interpretation": (
        "GPS reliability is orientation-dependent. RMS catches 30/45-degree "
        "failures, while the independent IMU gate catches low-RMS but wrong "
        "heading at 60/90 degrees. Active turns are rejected separately."
    ),
}
(OUT / "gps_orientation_reliability_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

angle = np.array([float(row["target_deg"]) for row in ordered])
rms = np.array([float(row["gps_reported_error_in"]) for row in ordered])
heading_delta = np.array([float(row["gps_imu_disagreement_deg"]) for row in ordered])
gps_return = np.abs([float(row["gps_heading_return_residual_deg"]) for row in ordered])
imu_return = np.abs([float(row["imu_heading_return_residual_deg"]) for row in ordered])
position_return = np.array([float(row["gps_position_return_residual_in"]) for row in ordered])

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
navy, cyan, orange, green, red, purple = (
    "#172554", "#0891b2", "#f97316", "#16a34a", "#dc2626", "#7c3aed"
)
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle("P7 GPS Orientation Reliability", fontsize=19, weight="bold", color=navy)
fig.text(
    0.5, 0.925,
    "Five live counterclockwise out/return sweeps • lens points robot-right • gates evaluated independently",
    ha="center", color="#475569",
)

ax = axes[0, 0]
ax.plot(angle, rms, "o-", lw=2, color=cyan)
ax.axhline(GPS_RMS_GATE_IN, ls="--", color=red, label="production RMS gate")
ax.set(title="A. Self-reported RMS quality", xlabel="Turn from start (deg)", ylabel="Inches")
ax.legend()

ax = axes[0, 1]
ax.plot(angle, heading_delta, "o-", lw=2, color=orange)
ax.axhline(GPS_HEADING_GATE_DEG, ls="--", color=red, label="IMU innovation gate")
ax.set(title="B. GPS vs. IMU turn magnitude", xlabel="Turn from start (deg)", ylabel="Disagreement (deg)")
ax.legend()

ax = axes[1, 0]
width = 3.2
ax.bar(angle - width / 2, gps_return, width, color=cyan, label="GPS heading")
ax.bar(angle + width / 2, imu_return, width, color=green, label="IMU heading")
ax.set(title="C. Heading residual after unwind", xlabel="Turn from start (deg)", ylabel="Absolute residual (deg)")
ax.legend()

ax = axes[1, 1]
accepted = (rms <= GPS_RMS_GATE_IN) & (heading_delta <= GPS_HEADING_GATE_DEG)
colors = [green if value else red for value in accepted]
ax.bar(angle, position_return, width=7.0, color=colors)
for x, y, ok in zip(angle, position_return, accepted):
    ax.text(x, y + 0.035, "PASS" if ok else "REJECT", ha="center", fontsize=8, weight="bold")
ax.set(title="D. Combined gate and return residual", xlabel="Turn from start (deg)", ylabel="GPS return residual (in)")

fig.text(
    0.5, 0.035,
    "Result: no single GPS quality field is sufficient; production requires reported RMS + temporal cluster + IMU heading + position innovation.",
    ha="center", color=navy, weight="bold", fontsize=9,
)
fig.tight_layout(rect=(0.05, 0.07, 0.98, 0.90), h_pad=3.0, w_pad=2.0)
fig.savefig(OUT / "gps_orientation_reliability_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "gps_orientation_reliability_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
