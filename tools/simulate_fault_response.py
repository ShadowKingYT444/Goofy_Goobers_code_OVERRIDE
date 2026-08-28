#!/usr/bin/env python3
"""Quantify scheduler-phase response latency for production safety gates.

These are deterministic software timing bounds plus random event phase, not
measured Smart Port transport latency or physical braking distance.
"""

from __future__ import annotations

import csv
import json
import random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reports" / "sensor_campaign_2026-08-23"
TRIALS = 100_000
SEED = 20260823
GPS_EXACT_REPEAT_FRACTION = 97 / 1200

EVENTS = (
    ("drive encoder read fails", 20.0, 0.0, "stop + pose invalid", "drive/IMU"),
    ("IMU missing/calibrating/error", 20.0, 0.0, "stop + pose invalid", "drive/IMU"),
    ("P1 API failure while forward", 20.0, 0.0, "stop", "P1"),
    ("P1 target at or inside 8 in", 20.0, 0.0, "stop", "P1"),
    ("mechanical jam; encoders stop", 20.0, 1000.0, "stop: drive_stall", "watchdog"),
    ("GPS missing/quality failure", 100.0, 0.0, "reject; continue DR", "P7"),
    ("P8 no-tag/bad/ambiguous", 100.0, 0.0, "reset chain; continue DR", "P8"),
)

rng = random.Random(SEED)
rows = []
for event, period_ms, fixed_delay_ms, action, channel in EVENTS:
    samples = np.asarray(
        [fixed_delay_ms + rng.random() * period_ms for _ in range(TRIALS)],
        dtype=float,
    )
    rows.append({
        "event": event,
        "channel": channel,
        "action": action,
        "software_poll_period_ms": period_ms,
        "fixed_gate_delay_ms": fixed_delay_ms,
        "latency_p50_ms": float(np.percentile(samples, 50)),
        "latency_p95_ms": float(np.percentile(samples, 95)),
        "latency_max_bound_ms": fixed_delay_ms + period_ms,
        "trials": TRIALS,
    })

def changed_observation_latency(required_changed: int) -> np.ndarray:
    """Random poll phase plus Bernoulli exact repeats from the live P7 rate."""
    samples = np.empty(TRIALS, dtype=float)
    for trial in range(TRIALS):
        changed = 0
        polls = 0
        while changed < required_changed:
            polls += 1
            if rng.random() >= GPS_EXACT_REPEAT_FRACTION:
                changed += 1
        samples[trial] = rng.random() * 100.0 + (polls - 1) * 100.0
    return samples


gps_return_rows = []
for disagreement, required in (("<=3 in", 12), ("3-6 in", 30)):
    samples = changed_observation_latency(required)
    gps_return_rows.append({
        "event": f"stable correct GPS, {disagreement} disagreement",
        "channel": "P7",
        "action": f"first bounded correction after {required} changed observations",
        "software_poll_period_ms": 100.0,
        "fixed_gate_delay_ms": None,
        "latency_p50_ms": float(np.percentile(samples, 50)),
        "latency_p95_ms": float(np.percentile(samples, 95)),
        # With no optical-frame timestamp, an arbitrarily long exact-repeat
        # run has no finite deterministic recovery-time bound.
        "latency_max_bound_ms": None,
        "trials": TRIALS,
    })

rows.extend((*gps_return_rows,
    {
        "event": "lateral push / wheel slip",
        "channel": "P5 disabled",
        "action": "unobservable; envelope excludes it",
        "software_poll_period_ms": None,
        "fixed_gate_delay_ms": None,
        "latency_p50_ms": None,
        "latency_p95_ms": None,
        "latency_max_bound_ms": None,
        "trials": 0,
    },
))

with (OUT / "fault_response_matrix.csv").open(
    "w", newline="", encoding="utf-8"
) as handle:
    writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)

summary = {
    "trials_per_periodic_event": TRIALS,
    "total_monte_carlo_trials": sum(row["trials"] for row in rows),
    "seed": SEED,
    "gps_exact_repeat_fraction": GPS_EXACT_REPEAT_FRACTION,
    "rows": rows,
    "timing_caveat": (
        "Scheduler-phase model only. It excludes sensor-internal latency, motor "
        "command transport, wheel coast, and physical stopping distance. GPS "
        "return timing uses independent Bernoulli repeats at the measured 8.08% "
        "rate; real repeats may be correlated, so there is no finite maximum. "
        "Live fault injection remains required after the Brain reconnects."
    ),
}
(OUT / "fault_response_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
fig, axes = plt.subplots(1, 2, figsize=(16, 7.2))
fig.suptitle("Localization Fault Response Contract",
             fontsize=19, weight="bold", color="#172554")
fig.text(
    0.5, 0.925,
    "Software scheduler-phase timing; measured physical braking and live unplug tests still pending",
    ha="center", color="#475569",
)

periodic = rows[:7]
labels = [row["event"] for row in periodic]
p95 = [row["latency_p95_ms"] for row in periodic]
bounds = [row["latency_max_bound_ms"] for row in periodic]
y = np.arange(len(periodic))
axes[0].barh(y, bounds, color="#cbd5e1", label="software upper bound")
axes[0].barh(y, p95, color="#0891b2", label="random-phase p95")
axes[0].set_yticks(y, labels)
axes[0].invert_yaxis()
axes[0].set(title="A. Detection / gate response", xlabel="Milliseconds")
axes[0].legend(fontsize=8)
for index, value in enumerate(bounds):
    axes[0].text(value + 8, index, f"{value:.0f}", va="center", fontsize=8)

axes[1].axis("off")
table_rows = [
    ["Drive/IMU loss", "stop + invalidate", "<=20 ms loop"],
    ["P1 fault/close target", "stop forward", "<=20 ms loop"],
    ["Mechanical jam", "stop", "1.00-1.02 s"],
    ["GPS missing/bad", "reject; encoder+IMU DR", "<=100 ms poll"],
    ["P8 missing/bad", "reset; DR unchanged", "<=100 ms poll"],
    ["GPS correct, <=3 in", "12 changed frames",
     f"p50 {gps_return_rows[0]['latency_p50_ms']/1000:.1f}s / p95 {gps_return_rows[0]['latency_p95_ms']/1000:.1f}s"],
    ["GPS correct, 3-6 in", "30 changed frames",
     f"p50 {gps_return_rows[1]['latency_p50_ms']/1000:.1f}s / p95 {gps_return_rows[1]['latency_p95_ms']/1000:.1f}s"],
    ["Slip/lateral push", "not observed", "P5 repair needed"],
]
table = axes[1].table(
    cellText=table_rows,
    colLabels=["Fault / return", "Response", "Timing"],
    cellLoc="left", colLoc="left", loc="center",
    colWidths=[0.36, 0.34, 0.28],
)
table.auto_set_font_size(False)
table.set_fontsize(9)
table.scale(1.0, 1.75)
for (row, _column), cell in table.get_celld().items():
    if row == 0:
        cell.set_facecolor("#172554")
        cell.set_text_props(color="white", weight="bold")
    else:
        cell.set_facecolor("#f8fafc" if row % 2 else "#e2e8f0")
axes[1].set_title("B. Fallback and recovery behavior", pad=20)

fig.text(
    0.5, 0.035,
    "GPS return uses the measured 8.08% repeat rate; no finite max. A software stop is not zero physical stopping distance.",
    ha="center", color="#172554", weight="bold", fontsize=9,
)
fig.tight_layout(rect=(0.03, 0.07, 0.98, 0.89), w_pad=3.0)
fig.savefig(OUT / "fault_response_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "fault_response_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
