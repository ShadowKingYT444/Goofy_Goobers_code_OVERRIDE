#!/usr/bin/env python3
"""Combine live scale variation with a provisional GPS-outage allowance."""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT = Path(__file__).resolve().parent
CONFIG = (OUT.parents[1] / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)


def setting(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG)
    if match is None:
        raise RuntimeError(f"missing production setting {name}")
    return float(match.group(1))


drive = json.loads(
    (OUT / "distance_multispeed_live_01" / "analysis_summary.json").read_text(
        encoding="utf-8"
    )
)

fitted_scale = float(drive["overall_encoder_to_gps_scale"])
speed_scales = {
    int(speed): float(values["encoder_to_gps_scale"])
    for speed, values in drive["per_speed_rpm"].items()
}
relative_scale_errors = {
    speed: abs(scale / fitted_scale - 1.0)
    for speed, scale in speed_scales.items()
}
worst_scale_fraction = max(relative_scale_errors.values())
worst_heading_deg = setting("kDeadReckoningHeadingEnvelopeDeg")
worst_heading_rad = math.radians(worst_heading_deg)

distance = np.linspace(0.0, 120.0, 241)
longitudinal = distance * worst_scale_fraction
lateral = distance * math.tan(worst_heading_rad)
combined = np.hypot(longitudinal, lateral)

rows: list[dict[str, float]] = []
for speed_in_s in (5.0, 10.0, 20.0):
    for outage_s in (1.0, 3.0, 5.0, 10.0):
        travel = speed_in_s * outage_s
        scale_error = travel * worst_scale_fraction
        lateral_error = travel * math.tan(worst_heading_rad)
        rows.append({
            "speed_in_s": speed_in_s,
            "outage_s": outage_s,
            "travel_in": travel,
            "scale_envelope_in": scale_error,
            "heading_cross_track_envelope_in": lateral_error,
            "combined_envelope_in": math.hypot(scale_error, lateral_error),
        })

with (OUT / "dead_reckoning_outage_table.csv").open(
    "w", newline="", encoding="utf-8"
) as handle:
    writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)

summary = {
    "encoder_scale_fit": fitted_scale,
    "per_speed_scale": speed_scales,
    "worst_relative_scale_variation_fraction": worst_scale_fraction,
    "worst_relative_scale_variation_percent": worst_scale_fraction * 100.0,
    "provisional_heading_controller_allowance_deg": worst_heading_deg,
    "examples": {
        f"{int(row['speed_in_s'])}in_s_{int(row['outage_s'])}s": row
        for row in rows
        if row["outage_s"] in (3.0, 5.0)
    },
    "important_caveat": (
        "This combines measured P7-referenced scale variation with a provisional "
        "2-degree heading/controller allowance; it is not measured IMU accuracy or "
        "a statistical confidence bound. Systematic scale-reference bias, collision, "
        "wheel slip, external pushing, and braking overshoot can produce larger errors."
    ),
}
(OUT / "dead_reckoning_outage_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

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
fig, axes = plt.subplots(1, 2, figsize=(15, 6.8))
fig.suptitle("Dead Reckoning During a GPS Outage", fontsize=19, weight="bold", color=navy)
fig.text(
    0.5, 0.91,
    "P7-referenced scale variation + provisional 2-degree heading/controller allowance; no GPS correction assumed",
    ha="center", color="#475569",
)

ax = axes[0]
ax.plot(distance, longitudinal, lw=2, color=cyan,
        label=f"encoder scale ({worst_scale_fraction * 100:.2f}%)")
ax.plot(distance, lateral, lw=2, color=orange,
        label=f"cross-track from {worst_heading_deg:.2f}°")
ax.plot(distance, combined, lw=3, color=red, label="combined envelope")
ax.set(title="A. Error grows with uncorrected travel", xlabel="Travel without absolute correction (in)", ylabel="Envelope (in)")
ax.legend()

ax = axes[1]
outages = np.array([1.0, 3.0, 5.0, 10.0])
for speed_in_s, color in ((5.0, green), (10.0, purple), (20.0, red)):
    values = []
    for outage_s in outages:
        travel = speed_in_s * outage_s
        values.append(math.hypot(
            travel * worst_scale_fraction,
            travel * math.tan(worst_heading_rad),
        ))
    ax.plot(outages, values, "o-", lw=2, color=color,
            label=f"{speed_in_s:.0f} in/s")
ax.set(title="B. Example time outages", xlabel="GPS-unavailable duration (s)", ylabel="Combined envelope (in)")
ax.legend(title="Robot speed")

fig.text(
    0.5, 0.035,
    "Not included: collision, carpet slip, external push, or endpoint overshoot (observed up to 2.03 in at 35 RPM).",
    ha="center", color=navy, weight="bold", fontsize=9,
)
fig.tight_layout(rect=(0.04, 0.08, 0.98, 0.88), w_pad=3.0)
fig.savefig(OUT / "dead_reckoning_outage_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "dead_reckoning_outage_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
