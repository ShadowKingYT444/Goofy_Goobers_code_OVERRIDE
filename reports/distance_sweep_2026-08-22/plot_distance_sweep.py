#!/usr/bin/env python3
"""Generate engineering-notebook plots for the 2026-08-22 drive sweep."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
target = np.arange(2.0, 11.0)
encoder = np.array([2.652, 3.336, 4.320, 5.130, 6.306, 7.343, 8.267, 9.179, 10.253])
gps = np.array([2.650, 2.991, 3.830, 4.543, 5.627, 6.544, 7.425, 8.046, 9.076])
heading = np.array([-0.179, -0.138, -0.064, -0.082, 0.045, -0.246, -0.202, -0.357, -0.305])
encoder_return = np.array([0.048, 0.180, 0.246, 0.072, 0.138, 0.186, 0.108, 0.138, 0.174])
gps_return = np.array([0.251, 0.463, 0.220, 0.143, 0.212, 0.136, 0.135, 0.296, 0.186])
gps_error = np.array([0.385, 0.385, 0.385, 0.385, 0.385, 0.385, 0.385, 0.385, 0.383])

# Exclude the 2-inch trial from scale estimation: braking overshot its commanded
# encoder target by 33%, making it a useful control transient but a poor scale point.
fit_mask = target >= 3
x = encoder[fit_mask]
y = gps[fit_mask]
scale = np.dot(x, y) / np.dot(x, x)
prediction = scale * x
r2_origin = 1.0 - np.sum((y - prediction) ** 2) / np.sum(y**2)
effective_diameter = 2.75 * scale

rng = np.random.default_rng(20260822)
boot = []
for _ in range(20000):
    idx = rng.integers(0, len(x), len(x))
    boot.append(np.dot(x[idx], y[idx]) / np.dot(x[idx], x[idx]))
ci_low, ci_high = np.percentile(boot, [2.5, 97.5])

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.titleweight": "bold",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

navy, cyan, orange, green, red = "#172554", "#0891b2", "#f97316", "#16a34a", "#dc2626"
fig, axes = plt.subplots(2, 2, figsize=(14, 11))
fig.subplots_adjust(left=0.075, right=0.94, top=0.88, bottom=0.10, hspace=0.34, wspace=0.28)
fig.suptitle("VEX Drive Distance Validation — GPS vs. Motor Encoders", fontsize=19, color=navy, y=0.965)
fig.text(0.5, 0.925,
         "2.75-in omni-wheel assumption • 1:1 external ratio • 450-RPM configuration • GPS Smart Port 7",
         ha="center", fontsize=10, color="#475569")

ax = axes[0, 0]
limit = 11
ax.plot([0, limit], [0, limit], "--", color="#64748b", lw=1.5, label="Perfect agreement (1:1)")
ax.errorbar(encoder, gps, yerr=gps_error, fmt="o", ms=7, capsize=3,
            color=cyan, ecolor="#67e8f9", label="Trials (GPS reported error)")
fit_line = np.linspace(0, limit, 200)
ax.plot(fit_line, scale * fit_line, color=orange, lw=2.5,
        label=f"Origin fit: GPS = {scale:.3f} × encoder")
for t, ex, gy in zip(target, encoder, gps):
    ax.annotate(f"{int(t)} in", (ex, gy), xytext=(5, -13), textcoords="offset points", fontsize=8)
ax.set(xlim=(0, limit), ylim=(0, limit), xlabel="Encoder-derived travel (in)",
       ylabel="GPS displacement (in)", title="A. Measurement agreement and fitted scale")
ax.set_aspect("equal", adjustable="box")
ax.legend(loc="upper left", fontsize=9)

ax = axes[0, 1]
error = gps - encoder
percent = 100 * error / encoder
colors = [green if abs(v) < 0.1 else red for v in error]
ax.bar(target, error, color=colors, alpha=0.86, width=0.65, label="GPS − encoder")
ax.axhline(0, color="#334155", lw=1)
ax2 = ax.twinx()
ax2.plot(target, percent, "D-", color=orange, lw=2, ms=5, label="Relative error")
ax2.spines["right"].set_visible(True)
ax2.grid(False)
ax.set(xlabel="Commanded trial (in)", ylabel="Absolute disagreement (in)",
       title="B. Scale error grows with distance")
ax2.set_ylabel("Relative disagreement (%)", color=orange)
ax2.tick_params(axis="y", colors=orange)
ax.text(6.1, -0.30, "Encoder increasingly overestimates travel", color=red, fontsize=9)

ax = axes[1, 0]
width = 0.34
ax.bar(target - width / 2, encoder_return, width, color=navy, label="Encoder residual")
ax.bar(target + width / 2, gps_return, width, color=cyan, label="GPS residual")
ax.axhline(0.385, color=orange, ls="--", lw=1.5, label="Typical GPS reported error")
ax.set(xlabel="Commanded trial (in)", ylabel="Distance from trial start after return (in)",
       title="C. Return-to-start repeatability")
ax.legend(fontsize=9)

ax = axes[1, 1]
ax.plot(target, heading, "o-", color=navy, lw=2, ms=6)
ax.fill_between([1.5, 10.5], [-0.4, -0.4], [0.4, 0.4], color=green, alpha=0.10,
                label="±0.4° band")
ax.axhline(0, color="#334155", lw=1)
ax.set(xlim=(1.5, 10.5), xlabel="Commanded trial (in)", ylabel="GPS heading change (deg)",
       title="D. Straight-line heading stability")
ax.legend(fontsize=9)

summary = (
    f"Scale factor: {scale:.3f}  |  Effective diameter: {effective_diameter:.2f} in  |  "
    f"Bootstrap 95% scale CI: [{ci_low:.3f}, {ci_high:.3f}]  |  Origin-fit R²: {r2_origin:.4f}"
)
fig.text(0.5, 0.035, summary, ha="center", fontsize=10, weight="bold", color=navy)
fig.savefig(OUT / "distance_sweep_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "distance_sweep_dashboard.svg", facecolor="white")

# A second, publication-style calibration graphic.
fig2, ax = plt.subplots(figsize=(12, 6.75), constrained_layout=True)
corrected = encoder * scale
ax.plot(target, target, "--", color="#64748b", lw=2, label="Commanded distance")
ax.plot(target, encoder, "o-", color=red, lw=2.4, label="Current encoder conversion (2.75 in)")
ax.plot(target, gps, "s-", color=cyan, lw=2.4, label="GPS displacement")
ax.plot(target, corrected, "D-", color=green, lw=2.2,
        label=f"Encoder corrected by {scale:.3f} (effective Ø {effective_diameter:.2f} in)")
ax.fill_between(target, gps - gps_error, gps + gps_error, color=cyan, alpha=0.12,
                label="GPS reported-error envelope")
ax.set(xlabel="Commanded trial (in)", ylabel="Measured / inferred travel (in)",
       title="Proposed Encoder Scale Correction")
ax.legend(loc="upper left", fontsize=10)
ax.text(6.0, 2.0,
        "Interpretation:\n• error is approximately multiplicative\n• heading drift is negligible\n• verify with a tape measure before changing competition constants",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="#f8fafc", edgecolor="#94a3b8"), fontsize=10)
fig2.savefig(OUT / "distance_scale_correction.png", dpi=220, facecolor="white")
fig2.savefig(OUT / "distance_scale_correction.svg", facecolor="white")

print(f"scale={scale:.6f}")
print(f"effective_diameter_in={effective_diameter:.4f}")
print(f"ci95=[{ci_low:.6f}, {ci_high:.6f}]")
print(f"r2_origin={r2_origin:.6f}")
