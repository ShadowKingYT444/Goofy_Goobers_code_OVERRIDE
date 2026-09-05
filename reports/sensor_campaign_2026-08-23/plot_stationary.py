#!/usr/bin/env python3
"""Plot the 120-second stationary GPS/IMU/Distance baseline."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
CSV_PATH = OUT / "stationary_04_120s" / "telemetry.csv"

with CSV_PATH.open(newline="", encoding="utf-8") as handle:
    rows = list(csv.DictReader(handle))

def values(name: str) -> np.ndarray:
    return np.asarray([float(row[name]) for row in rows], dtype=float)


t = values("host_s")
gps_x_in = values("gps_x") * 39.37007874015748
gps_y_in = values("gps_y") * 39.37007874015748
gps_heading = values("gps_heading")
gps_error_in = values("gps_error") * 39.37007874015748
imu = values("imu")
p1_mm = values("p1_mm")
p1_conf = values("p1_conf")
p1_valid = (p1_mm >= 20) & (p1_mm < 9999) & (p1_conf > 0)

x_delta = gps_x_in - np.median(gps_x_in)
y_delta = gps_y_in - np.median(gps_y_in)
heading_delta = gps_heading - np.median(gps_heading)
imu_delta = imu - np.median(imu)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
navy = "#172554"
cyan = "#0891b2"
orange = "#f97316"
green = "#16a34a"
red = "#dc2626"
purple = "#7c3aed"

fig, axes = plt.subplots(2, 2, figsize=(14, 10.5))
fig.subplots_adjust(left=0.08, right=0.94, top=0.87, bottom=0.11,
                    hspace=0.34, wspace=0.30)
fig.suptitle("VEX Multi-Sensor Stationary Baseline — 120 Seconds",
             fontsize=19, color=navy, y=0.965)
fig.text(
    0.5,
    0.925,
    "GPS P7 • IMU P6 • forward Distance P1 • 2,000 synchronized frames at 16.7 Hz",
    ha="center",
    fontsize=10,
    color="#475569",
)

ax = axes[0, 0]
points = ax.scatter(x_delta, y_delta, c=t, cmap="viridis", s=9, alpha=0.65)
ax.axhline(0, color="#64748b", lw=1)
ax.axvline(0, color="#64748b", lw=1)
ax.set_aspect("equal", adjustable="datalim")
ax.set(
    xlabel="GPS X deviation from median (in)",
    ylabel="GPS Y deviation from median (in)",
    title="A. Stationary position cloud",
)
fig.colorbar(points, ax=ax, label="Elapsed time (s)", fraction=0.046, pad=0.04)

ax = axes[0, 1]
ax.plot(t, x_delta, color=cyan, lw=1.2, label=f"X (σ={np.std(x_delta):.4f} in)")
ax.plot(t, y_delta, color=purple, lw=1.2, label=f"Y (σ={np.std(y_delta):.4f} in)")
ax.axhline(0, color="#334155", lw=1)
ax.set(
    xlabel="Elapsed time (s)",
    ylabel="Deviation from median (in)",
    title="B. GPS drift and jitter versus time",
)
ax.legend(fontsize=9)

ax = axes[1, 0]
ax.plot(t, heading_delta, color=orange, lw=1.2,
        label=f"GPS heading (σ={np.std(heading_delta):.4f}°)")
ax.plot(t, imu_delta, color=navy, lw=1.2,
        label=f"IMU rotation (σ={np.std(imu_delta):.4f}°)")
ax.axhline(0, color="#334155", lw=1)
ax.set(
    xlabel="Elapsed time (s)",
    ylabel="Deviation from median (deg)",
    title="C. Heading stability",
)
ax.legend(fontsize=9)
ax2 = ax.twinx()
ax2.plot(t, gps_error_in, color=green, alpha=0.35, lw=1,
         label="GPS reported RMS error")
ax2.set_ylabel("GPS reported RMS error (in)", color=green)
ax2.tick_params(axis="y", colors=green)
ax2.grid(False)

ax = axes[1, 1]
ax.scatter(t[p1_valid], p1_mm[p1_valid] / 25.4, c=p1_conf[p1_valid],
           cmap="autumn", vmin=0, vmax=63, s=16, label="Valid returns")
ax.scatter(t[~p1_valid], np.full(np.count_nonzero(~p1_valid), 43.5),
           color="#94a3b8", s=5, alpha=0.25, label="No target (9999 mm)")
ax.set(
    xlabel="Elapsed time (s)",
    ylabel="Forward range (in)",
    title="D. Forward Distance sensor availability",
)
ax.legend(fontsize=8, loc="lower right")
ax.text(
    0.03,
    0.95,
    f"Valid {100*np.mean(p1_valid):.1f}%\n"
    f"Median valid range {np.median(p1_mm[p1_valid])/25.4:.1f} in\n"
    f"Median confidence {np.median(p1_conf[p1_valid]):.0f}/63",
    transform=ax.transAxes,
    va="top",
    bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="#94a3b8"),
    fontsize=9,
)

summary = (
    f"GPS valid 100% | reported RMS error median {np.median(gps_error_in):.3f} in | "
    f"position σ = ({np.std(x_delta):.4f}, {np.std(y_delta):.4f}) in | "
    f"heading σ = {np.std(heading_delta):.4f}° | IMU σ = {np.std(imu_delta):.4f}°"
)
fig.text(0.5, 0.035, summary, ha="center", fontsize=10, weight="bold", color=navy)
fig.savefig(OUT / "stationary_sensor_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "stationary_sensor_dashboard.svg", facecolor="white")
print(summary)
