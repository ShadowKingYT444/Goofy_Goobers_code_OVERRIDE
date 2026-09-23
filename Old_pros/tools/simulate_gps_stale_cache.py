#!/usr/bin/env python3
"""Replay a short move followed by a cached pre-move GPS tuple."""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
DT = 0.1
FRESH_AT_S = 3.0
REQUIRED = 12


@dataclass
class Gate:
    freshness_enabled: bool
    estimate_in: float = 0.0
    consistent: int = 0
    pending_in: float | None = None
    last_raw_in: float | None = None
    raw_anchor_travel_in: float = 0.0
    status: str = "not_initialized"
    correction_in: float = 0.0

    def update(self, observation_in: float, speed_in_s: float,
               encoder_travel_in: float) -> None:
        self.correction_in = 0.0
        changed = self.last_raw_in is None or abs(
            observation_in - self.last_raw_in
        ) > 1e-4
        if changed:
            self.last_raw_in = observation_in
            self.raw_anchor_travel_in = encoder_travel_in
        elif (self.freshness_enabled and
              encoder_travel_in - self.raw_anchor_travel_in > 0.5):
            self.consistent = 0
            self.status = "stale_geometry"
            return
        elif self.freshness_enabled:
            self.status = "repeat"
            return

        if abs(speed_in_s) > 0.5:
            self.consistent = 0
            self.status = "motion"
            return

        if (self.consistent == 0 or self.pending_in is None or
                abs(observation_in - self.pending_in) > 0.75):
            self.pending_in = observation_in
            self.consistent = 1
        else:
            self.consistent += 1
        if self.consistent < REQUIRED:
            self.status = "settling"
            return

        innovation = observation_in - self.estimate_in
        if abs(innovation) > 3.0:
            self.status = "position_innovation"
            return
        requested = 0.0 if abs(innovation) <= 0.05 else innovation * 0.20
        self.correction_in = max(-0.5, min(0.5, requested))
        self.estimate_in += self.correction_in
        self.status = "corrected"


def main() -> None:
    baseline = Gate(freshness_enabled=False)
    production = Gate(freshness_enabled=True)
    rows = []
    for step in range(61):
        time_s = step * DT
        physical_in = min(2.0, time_s * 2.0)
        speed_in_s = 2.0 if time_s < 1.0 else 0.0
        if time_s < FRESH_AT_S:
            observation_in = 0.0
        else:
            # A newly live solution jitters by more than the production exact-
            # tuple threshold, standing in for genuinely changed frames.
            jitter = (0.0, 0.0002, -0.0002, 0.0004, -0.0004)
            observation_in = 2.0 + jitter[(step - int(FRESH_AT_S / DT)) % len(jitter)]

        # Perfect encoders isolate the stale-absolute-observation failure.
        if step == 0:
            baseline.estimate_in = physical_in
            production.estimate_in = physical_in
        else:
            previous_physical = rows[-1]["physical_in"]
            delta = physical_in - previous_physical
            baseline.estimate_in += delta
            production.estimate_in += delta

        baseline.update(observation_in, speed_in_s, physical_in)
        production.update(observation_in, speed_in_s, physical_in)
        rows.append(
            {
                "time_s": time_s,
                "physical_in": physical_in,
                "gps_observation_in": observation_in,
                "baseline_estimate_in": baseline.estimate_in,
                "production_estimate_in": production.estimate_in,
                "baseline_correction_in": baseline.correction_in,
                "production_correction_in": production.correction_in,
                "baseline_state": baseline.status,
                "production_state": production.status,
            }
        )

    csv_path = REPORT / "gps_stale_cache_trials.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    cached_rows = [row for row in rows if 1.0 <= row["time_s"] < FRESH_AT_S]
    first_baseline_wrong = next(
        (row["time_s"] for row in cached_rows
         if abs(row["baseline_correction_in"]) > 0.0),
        None,
    )
    first_production_accept = next(
        (row["time_s"] for row in rows
         if row["production_state"] == "corrected"),
        None,
    )
    summary = {
        "scenario": "2-in encoder-perfect move; P7 frozen at pre-move coordinate until 3.0 s",
        "baseline_first_wrong_correction_s": first_baseline_wrong,
        "baseline_max_error_before_fresh_in": max(
            abs(row["baseline_estimate_in"] - row["physical_in"])
            for row in cached_rows
        ),
        "production_max_error_before_fresh_in": max(
            abs(row["production_estimate_in"] - row["physical_in"])
            for row in cached_rows
        ),
        "production_first_accept_after_fresh_s": first_production_accept,
        "interpretation": (
            "The stopped/cluster/innovation gates alone can accept a cached "
            "pre-move tuple after a short move and pull a correct encoder pose "
            "backward. The motion-anchored repeat gate holds stale_geometry until "
            "P7 emits numerically changed tuples, then requires 12 changed "
            "observations; unchanged polls preserve but do not advance the chain."
        ),
        "scope": "deterministic production-logic shadow replay, not hardware latency evidence",
    }
    (REPORT / "gps_stale_cache_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True,
                             sharex=True)
    fig.suptitle("Cached P7 tuple after a short move — temporality gate replay",
                 fontsize=16, fontweight="bold")
    time = [row["time_s"] for row in rows]
    axes[0].plot(time, [row["physical_in"] for row in rows], linewidth=3,
                 label="physical / encoder truth", color="#5cffad")
    axes[0].plot(time, [row["gps_observation_in"] for row in rows], "--",
                 label="P7 API tuple", color="#ffd166")
    axes[0].plot(time, [row["baseline_estimate_in"] for row in rows],
                 label="without repeat gate", color="#ff6b6b")
    axes[0].plot(time, [row["production_estimate_in"] for row in rows],
                 label="production repeat gate", color="#57c7ff")
    axes[0].axvline(FRESH_AT_S, color="white", alpha=0.4, linestyle=":")
    axes[0].set(ylabel="Position (in)", title="A cached but plausible 2-in innovation")
    axes[0].legend(frameon=False, ncol=2)
    axes[0].grid(alpha=0.18)

    states = sorted(set(row["baseline_state"] for row in rows) |
                    set(row["production_state"] for row in rows))
    state_index = {state: index for index, state in enumerate(states)}
    axes[1].step(time, [state_index[row["baseline_state"]] for row in rows],
                 where="post", label="without repeat gate", color="#ff6b6b")
    axes[1].step(time, [state_index[row["production_state"]] for row in rows],
                 where="post", label="production", color="#57c7ff")
    axes[1].set_yticks(range(len(states)), states)
    axes[1].set(xlabel="Time (s)", ylabel="Gate state",
                title="Production waits for changed geometry, then 12 fresh polls")
    axes[1].grid(alpha=0.18)
    axes[1].legend(frameon=False)
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"gps_stale_cache_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
