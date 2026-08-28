#!/usr/bin/env python3
"""Replay measured orientation-dependent GPS heading error at stopped fixes."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
SOURCE = REPORT / "gps_orientation_reliability.csv"
POLL_S = 0.1
POLLS_PER_STOP = 30
REQUIRED_POLLS = 12
HEADING_GATE_DEG = 5.0
GAIN = 0.10
MAX_STEP_DEG = 0.50
DEADBAND_DEG = 0.20


def signed_error(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def main() -> None:
    rows = list(csv.DictReader(SOURCE.open(encoding="utf-8")))
    records = []
    fused_bias_deg = 0.0
    position_only_bias_deg = 0.0
    time_s = 0.0
    for row in rows:
        orientation_deg = float(row["target_deg"])
        # The CSV already reports |GPS-IMU|. In these CCW captures GPS changed
        # less negative than P6, so GPS minus P6 is the positive value.
        measured_gps_error_deg = float(row["gps_imu_disagreement_deg"])
        reported_error_in = float(row["gps_reported_error_in"])
        quality_ok = reported_error_in <= 0.75
        for poll in range(1, POLLS_PER_STOP + 1):
            reason = "settling"
            applied_step_deg = 0.0
            if not quality_ok:
                reason = "quality"
            elif poll >= REQUIRED_POLLS:
                innovation = signed_error(measured_gps_error_deg, fused_bias_deg)
                if abs(innovation) <= HEADING_GATE_DEG:
                    applied_step_deg = (
                        0.0
                        if abs(innovation) <= DEADBAND_DEG
                        else float(np.clip(innovation * GAIN, -MAX_STEP_DEG, MAX_STEP_DEG))
                    )
                    fused_bias_deg += applied_step_deg
                    reason = "corrected"
                else:
                    reason = "heading_gate"
            records.append({
                "time_s": time_s,
                "orientation_deg": orientation_deg,
                "measured_gps_minus_p6_deg": measured_gps_error_deg,
                "reported_error_in": reported_error_in,
                "quality_ok": quality_ok,
                "poll": poll,
                "current_policy_bias_deg": fused_bias_deg,
                "position_only_bias_deg": position_only_bias_deg,
                "applied_step_deg": applied_step_deg,
                "reason": reason,
            })
            time_s += POLL_S

    fields = list(records[0])
    with (REPORT / "gps_heading_policy_replay.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    bias = np.asarray([row["current_policy_bias_deg"] for row in records])
    steps = np.asarray([row["applied_step_deg"] for row in records])
    accepted_orientations = sorted({
        row["orientation_deg"] for row in records if row["reason"] == "corrected"
    })
    summary = {
        "source": str(SOURCE.relative_to(ROOT)),
        "measured_orientations_deg": [float(row["target_deg"]) for row in rows],
        "polls_per_stopped_orientation": POLLS_PER_STOP,
        "production_heading_gate_deg": HEADING_GATE_DEG,
        "production_gain": GAIN,
        "production_max_step_deg": MAX_STEP_DEG,
        "accepted_orientations_deg": accepted_orientations,
        "maximum_accumulated_bias_deg": float(np.max(np.abs(bias))),
        "final_accumulated_bias_deg": float(bias[-1]),
        "maximum_single_step_deg": float(np.max(np.abs(steps))),
        "position_only_accumulated_bias_deg": 0.0,
        "interpretation": (
            "A per-update bound does not bound cumulative heading bias. Repeated stopped "
            "P7 updates inside the 5-degree innovation gate can steer the P6-referenced "
            "heading by several degrees. Position-only GPS avoids this measured failure mode."
        ),
        "limitations": [
            "P6 is used as the relative reference; no external surveyed heading truth was present.",
            "The replay uses one measured CCW orientation sweep, not every field lighting/view condition.",
            "P6 long-term drift and temperature behavior beyond two minutes remain unmeasured.",
        ],
    }
    (REPORT / "gps_heading_policy_replay_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    times = np.asarray([row["time_s"] for row in records])
    orientations = np.asarray([row["orientation_deg"] for row in records])
    gps_errors = np.asarray([row["measured_gps_minus_p6_deg"] for row in records])
    quality = np.asarray([row["quality_ok"] for row in records])
    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(16.5, 5.8), constrained_layout=True)
    fig.suptitle(
        "P7 Heading Fusion Risk — measured orientation errors replayed at stops",
        fontsize=16,
        fontweight="bold",
    )
    axes[0].plot(orientations[::POLLS_PER_STOP], gps_errors[::POLLS_PER_STOP], "o-", color="#ffb347", label="|P7 − P6|")
    axes[0].axhline(HEADING_GATE_DEG, linestyle="--", color="#ff6b6b", label="heading gate")
    for x, y, ok in zip(orientations[::POLLS_PER_STOP], gps_errors[::POLLS_PER_STOP], quality[::POLLS_PER_STOP]):
        axes[0].annotate("quality ok" if ok else "RMS reject", (x, y), xytext=(0, 8), textcoords="offset points", ha="center", fontsize=8)
    axes[0].set(xlabel="Turn orientation from start (deg)", ylabel="Heading disagreement (deg)", title="Live P7 orientation dependence")
    axes[0].legend(frameon=False)
    axes[0].grid(alpha=0.16)

    axes[1].plot(times, bias, color="#ff6b6b", linewidth=2.4, label="current gain 0.10")
    axes[1].plot(times, np.zeros_like(times), color="#5cffad", linewidth=2.4, label="GPS position-only")
    axes[1].set(xlabel="Replay time (s)", ylabel="Accumulated bias vs P6 (deg)", title="Bounded steps still accumulate")
    axes[1].legend(frameon=False)
    axes[1].grid(alpha=0.16)

    axes[2].axis("off")
    note = (
        "MEASURED INPUT\n"
        "• five stopped orientations: 15-90°\n"
        "• P7−P6 disagreement: 3.42-8.57°\n"
        "• low-RMS wrong headings exist\n\n"
        "CURRENT POLICY REPLAY\n"
        f"• max cumulative P6 bias: {np.max(np.abs(bias)):.2f}°\n"
        f"• final bias: {bias[-1]:.2f}°\n"
        f"• max step: {np.max(np.abs(steps)):.2f}°\n\n"
        "RECOMMENDATION\n"
        "Keep P7 position correction, but set heading gain to zero.\n"
        "P6 owns heading until an external-truth temperature/long-run\n"
        "test proves GPS heading adds value rather than orientation bias."
    )
    axes[2].text(
        0.02,
        0.98,
        note,
        va="top",
        fontsize=11.5,
        linespacing=1.45,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#ff6b6b"},
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"gps_heading_policy_replay_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
