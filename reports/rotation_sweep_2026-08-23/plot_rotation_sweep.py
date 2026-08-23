#!/usr/bin/env python3
"""Generate notebook figures from the supervised rotation sweep."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
target = np.array([15., 30., 45., 60., 90.])
encoder = np.array([18.778, 30.858, 45.629, 60.628, 90.571])
gps = np.abs(np.array([-21.474, -33.937, -44.895, -59.605, -85.381]))
imu = np.abs(np.array([-20.513, -32.198, -45.790, -61.537, -92.528]))
track_gps = np.array([10.501, 10.919, 12.205, 12.215, 12.738])
gps_error_in = np.array([0.386, 0.385, 0.385, 0.386, 1.174])
return_target = np.array([15., 30., 45., 60.])
return_gps_heading = np.abs(np.array([0.220, 0.247, -0.643, -0.285]))
return_imu_heading = np.abs(np.array([0.266, 0.330, -0.063, 0.474]))
return_position = np.array([0.239, 0.167, 0.080, 0.115])
configured_track = 12.0086
track_imu = configured_track * encoder / imu

# Larger turns are less dominated by static-friction/braking transients.
stable = target >= 45
gps_scale = np.dot(encoder[stable], gps[stable]) / np.dot(encoder[stable], encoder[stable])
imu_scale = np.dot(encoder[stable], imu[stable]) / np.dot(encoder[stable], encoder[stable])

plt.rcParams.update({
    "font.family": "DejaVu Sans", "axes.titleweight": "bold",
    "axes.grid": True, "grid.alpha": 0.22,
    "axes.spines.top": False, "axes.spines.right": False,
})
navy, cyan, orange, green, red, purple = "#172554", "#0891b2", "#f97316", "#16a34a", "#dc2626", "#7c3aed"

fig, axes = plt.subplots(2, 2, figsize=(14, 11))
fig.subplots_adjust(left=0.075, right=0.94, top=0.88, bottom=0.10, hspace=0.34, wspace=0.28)
fig.suptitle("VEX In-Place Rotation Validation — Encoder, GPS, and IMU", fontsize=19, color=navy, y=0.965)
fig.text(0.5, 0.925,
         "2.75-in omni wheels • configured track width 12.0086 in • GPS port 7 • IMU port 6",
         ha="center", fontsize=10, color="#475569")

ax = axes[0, 0]
ax.plot(target, target, "--", color="#64748b", lw=2, label="Commanded angle")
ax.plot(target, encoder, "o-", color=navy, lw=2.2, label="Wheel encoders")
ax.plot(target, gps, "s-", color=cyan, lw=2.2, label="GPS heading")
ax.plot(target, imu, "D-", color=orange, lw=2.2, label="IMU heading")
ax.scatter([90], [gps[-1]], s=150, facecolors="none", edgecolors=red, lw=2,
           label="GPS uncertainty spike")
ax.set(xlabel="Commanded turn (deg)", ylabel="Measured turn magnitude (deg)",
       title="A. Three-sensor angular agreement")
ax.legend(fontsize=9)

ax = axes[0, 1]
ax.axhline(0, color="#334155", lw=1)
ax.plot(target, encoder - target, "o-", color=navy, lw=2, label="Encoder − command")
ax.plot(target, gps - encoder, "s-", color=cyan, lw=2, label="GPS − encoder")
ax.plot(target, imu - encoder, "D-", color=orange, lw=2, label="IMU − encoder")
ax.axvspan(10, 35, color=red, alpha=0.08, label="Static-friction regime")
ax.set(xlabel="Commanded turn (deg)", ylabel="Angular error (deg)",
       title="B. Error decomposition")
ax.legend(fontsize=9)

ax = axes[1, 0]
ax.axhline(configured_track, color="#334155", ls="--", lw=2,
           label=f"Configured: {configured_track:.3f} in")
ax.plot(target, track_gps, "s-", color=cyan, lw=2.2, label="Track width inferred using GPS")
ax.plot(target, track_imu, "D-", color=orange, lw=2.2, label="Track width inferred using IMU")
ax.fill_between([40, 95], [11.5, 11.5], [12.8, 12.8], color=green, alpha=0.08)
ax.set(xlabel="Commanded turn (deg)", ylabel="Effective track width (in)",
       title="C. Geometry calibration converges on larger turns")
ax.legend(fontsize=9)

ax = axes[1, 1]
width = 3.5
ax.bar(return_target - width, return_gps_heading, width, color=cyan, label="GPS heading residual (deg)")
ax.bar(return_target, return_imu_heading, width, color=orange, label="IMU heading residual (deg)")
ax2 = ax.twinx()
ax2.plot(return_target, return_position, "o-", color=purple, lw=2.2,
         label="GPS position residual (in)")
ax2.spines["right"].set_visible(True)
ax2.grid(False)
ax.set(xlabel="Out-and-return trial (deg)", ylabel="Absolute heading residual (deg)",
       title="D. Return-to-start repeatability")
ax2.set_ylabel("GPS position residual (in)", color=purple)
ax2.tick_params(axis="y", colors=purple)
lines, labels = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines + lines2, labels + labels2, fontsize=8, loc="upper center")

summary = (
    f"45–90° origin fits: GPS = {gps_scale:.3f} × encoder; IMU = {imu_scale:.3f} × encoder  |  "
    f"Mean IMU-derived track width (45–90°): {np.mean(track_imu[stable]):.2f} in"
)
fig.text(0.5, 0.035, summary, ha="center", fontsize=10, weight="bold", color=navy)
fig.savefig(OUT / "rotation_sweep_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "rotation_sweep_dashboard.svg", facecolor="white")

print(f"gps_scale_45_90={gps_scale:.6f}")
print(f"imu_scale_45_90={imu_scale:.6f}")
print(f"mean_imu_track_45_90={np.mean(track_imu[stable]):.6f}")
