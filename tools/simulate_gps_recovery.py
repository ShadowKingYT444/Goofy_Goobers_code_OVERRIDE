#!/usr/bin/env python3
"""Exercise current GPS reacquisition gates after representative outages.

This is a deterministic scalar replay of the production position gate, not a
claim that GPS itself is accurate.  Initial position errors come from the live
measured encoder/IMU outage envelope and the separate hidden-slip stress test.
"""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reports" / "sensor_campaign_2026-08-23"
CONFIG = (ROOT / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)


def setting(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG)
    if match is None:
        raise RuntimeError(f"missing production setting {name}")
    return float(match.group(1))


REQUIRED = int(setting("kGpsRequiredConsistentObservations"))
REACQUIRE_REQUIRED = int(setting("kGpsRequiredReacquisitionObservations"))
NORMAL_GATE = setting("kGpsMaxPositionInnovationIn")
REACQUIRE_GATE = setting("kGpsMaxReacquisitionInnovationIn")
GAIN = setting("kGpsPositionGain")
MAX_STEP = setting("kGpsMaxPositionStepIn")
DEADBAND = setting("kGpsPositionDeadbandIn")
PERIOD_S = setting("kGpsCorrectionPeriodMs") / 1000.0
# Resampling the authoritative 120-s stationary capture at the production
# 10-Hz cadence produced 97 adjacent exact tuple repeats among 1,200
# transitions (8.083%). poll_id is not an optical timestamp, so production no
# longer counts those repeats. Use one repeat every 12 polls as a transparent
# deterministic approximation of that measured availability cost.
MEASURED_EXACT_REPEAT_FRACTION = 97 / 1200
REPEAT_EVERY_POLLS = 12

# The first three values use the provisional 2-degree heading/controller
# allowance plus scale variation. No external heading truth was available.
# The fourth is the physical p95 from the separate hidden slip/push Monte Carlo.
PROVISIONAL_ERROR_PER_TRAVEL = math.hypot(
    setting("kDeadReckoningScaleEnvelopeFraction"),
    math.tan(math.radians(setting("kDeadReckoningHeadingEnvelopeDeg"))),
)
SCENARIOS = {
    "5 s @ 10 in/s\nprovisional allowance": 50 * PROVISIONAL_ERROR_PER_TRAVEL,
    "5 s @ 20 in/s\nprovisional allowance": 100 * PROVISIONAL_ERROR_PER_TRAVEL,
    "10 s @ 20 in/s\nprovisional allowance": 200 * PROVISIONAL_ERROR_PER_TRAVEL,
    "48 in hidden slip/push\np95 physical error": 7.1363772346,
}


def replay(name: str, initial_error: float, duration_s: float = 10.0):
    residual = initial_error
    records = []
    consistent = 0
    for sample in range(int(duration_s / PERIOD_S) + 1):
        time_s = sample * PERIOD_S
        changed_observation = (sample + 1) % REPEAT_EVERY_POLLS != 0
        step = 0.0
        if not changed_observation:
            state = "repeat"
        else:
            consistent += 1
        if changed_observation and consistent < REQUIRED:
            state = "settling"
        elif changed_observation and residual <= NORMAL_GATE:
            state = "corrected"
        elif (changed_observation and residual <= REACQUIRE_GATE
              and consistent >= REACQUIRE_REQUIRED):
            state = "reacquired"
        elif changed_observation:
            state = "position_innovation"

        if state in {"corrected", "reacquired"}:
            requested = 0.0 if residual <= DEADBAND else residual * GAIN
            step = min(requested, MAX_STEP)
            residual = max(0.0, residual - step)

        records.append({
            "scenario": name.replace("\n", " "),
            "time_after_gps_return_s": time_s,
            "consistent_observations": consistent,
            "changed_observation": changed_observation,
            "position_error_before_in": residual + step,
            "applied_step_in": step,
            "position_error_after_in": residual,
            "state": state,
        })
    return records


all_records = [
    record
    for name, error in SCENARIOS.items()
    for record in replay(name, error)
]

with (OUT / "gps_recovery_gate_trials.csv").open(
    "w", newline="", encoding="utf-8"
) as handle:
    writer = csv.DictWriter(handle, fieldnames=list(all_records[0]))
    writer.writeheader()
    writer.writerows(all_records)

summaries = []
for name, initial_error in SCENARIOS.items():
    records = replay(name, initial_error)
    accepted = [row for row in records if row["applied_step_in"] > 0.0]
    below_half = [
        row for row in records if row["position_error_after_in"] <= 0.5
    ]
    summaries.append({
        "scenario": name.replace("\n", " "),
        "initial_error_in": initial_error,
        "first_nonzero_correction_s": (
            accepted[0]["time_after_gps_return_s"] if accepted else None
        ),
        "time_to_below_0_5_in_s": (
            below_half[0]["time_after_gps_return_s"] if below_half else None
        ),
        "maximum_step_in": max(row["applied_step_in"] for row in records),
        "final_error_in": records[-1]["position_error_after_in"],
        "final_state": records[-1]["state"],
    })

summary = {
    "production_gates": {
        "normal_innovation_in": NORMAL_GATE,
        "normal_consistent_observations": REQUIRED,
        "reacquisition_innovation_in": REACQUIRE_GATE,
        "reacquisition_consistent_observations": REACQUIRE_REQUIRED,
        "poll_period_s": PERIOD_S,
        "position_gain": GAIN,
        "maximum_step_in": MAX_STEP,
        "measured_exact_repeat_fraction_at_10_hz": MEASURED_EXACT_REPEAT_FRACTION,
        "modeled_repeat_every_polls": REPEAT_EVERY_POLLS,
    },
    "scenarios": summaries,
    "interpretation": (
        "Provisional-allowance outages inside 3 inches begin bounded correction "
        "after 12 numerically changed observations; 3-6 inch recovery requires "
        "30. Exact tuples do not add evidence or correction. The modeled one-"
        "in-12 repeat cadence approximates the live stationary 10-Hz replay. Error "
        "beyond 6 inches remains rejected, demonstrating that unobserved slip "
        "cannot be repaired by blindly trusting a newly returned GPS fix."
    ),
}
(OUT / "gps_recovery_gate_summary.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8"
)

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.grid": True,
    "grid.alpha": 0.22,
    "axes.spines.top": False,
    "axes.spines.right": False,
})
colors = ("#0891b2", "#16a34a", "#f97316", "#dc2626")
fig, axes = plt.subplots(1, 2, figsize=(15, 6.6))
fig.suptitle("GPS Return: Current Temporal + Innovation Gate",
             fontsize=19, weight="bold", color="#172554")
fig.text(
    0.5, 0.91,
    "Provisional dead-reckoning allowance versus an explicitly unobservable slip/push case",
    ha="center", color="#475569",
)

for (name, initial_error), color in zip(SCENARIOS.items(), colors):
    records = replay(name, initial_error)
    times = [row["time_after_gps_return_s"] for row in records]
    residuals = [row["position_error_after_in"] for row in records]
    steps = [row["applied_step_in"] for row in records]
    axes[0].plot(times, residuals, lw=2.3, color=color, label=name)
    axes[1].plot(times, steps, lw=2.0, color=color, label=name)

axes[0].axhline(NORMAL_GATE, color="#7c3aed", ls="--", alpha=0.7,
                label="3 in normal gate")
axes[0].axhline(REACQUIRE_GATE, color="#111827", ls=":", alpha=0.7,
                label="6 in reacquire gate")
axes[0].set(title="A. Residual position error after GPS returns",
            xlabel="Time after first returned GPS poll (s)", ylabel="Error (in)")
axes[1].axhline(MAX_STEP, color="#111827", ls="--", alpha=0.7,
                label="0.5 in hard step limit")
axes[1].axvline((REQUIRED - 1) * PERIOD_S, color="#7c3aed", ls=":", alpha=0.7,
                label="12 changed observations (no-repeat lower bound)")
axes[1].axvline((REACQUIRE_REQUIRED - 1) * PERIOD_S, color="#f97316", ls=":",
                alpha=0.7, label="30 changed observations (no-repeat lower bound)")
axes[1].set(title="B. Correction applied per 10 Hz GPS poll",
            xlabel="Time after first returned GPS poll (s)", ylabel="Step (in)")
for axis in axes:
    axis.legend(fontsize=8)
fig.text(
    0.5, 0.03,
    "Deterministic gate replay, not GPS accuracy. A >6 in disagreement stays rejected by design.",
    ha="center", color="#172554", weight="bold", fontsize=9,
)
fig.tight_layout(rect=(0.04, 0.08, 0.98, 0.88), w_pad=3.0)
fig.savefig(OUT / "gps_recovery_gate_dashboard.png", dpi=220, facecolor="white")
fig.savefig(OUT / "gps_recovery_gate_dashboard.svg", facecolor="white")
print(json.dumps(summary, indent=2))
