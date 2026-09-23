#!/usr/bin/env python3
"""Monte Carlo shadow model for the production straight-line controller.

This is an engineering stress test, not a replacement for hardware validation.
It mirrors the controller's finish-plane, cross-track, slew, and power logic,
then separates the estimator pose from the physical pose so encoder-scale
error and unobserved pushes remain visible.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DT = 0.020
TRACK_IN = 12.0086
WHEEL_DIAMETER_IN = 2.4330552523
MAX_WHEEL_IPS = 450.0 * math.pi * WHEEL_DIAMETER_IN / 60.0
TOLERANCE_IN = 1.0
MAX_FINISH_CROSS_IN = 2.0
SETTLE_S = 0.080
DRIVE_KP = 7.5
MIN_POWER = 18.0
HEADING_KP = 1.15
FINAL_HEADING_KP = 0.35
MAX_TURN_POWER = 32.0
LOOKAHEAD_IN = 10.0
MAX_CROSS_HEADING_DEG = 20.0
FWD_SLEW_PER_S = 280.0
TURN_SLEW_PER_S = 420.0
TIMEOUT_S = 12.0
ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)


def setting(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG)
    if match is None:
        raise RuntimeError(f"missing production setting {name}")
    return float(match.group(1))


PROVISIONAL_SCALE_FRACTION = setting(
    "kDeadReckoningScaleEnvelopeFraction"
)
PROVISIONAL_HEADING_DEG = setting("kDeadReckoningHeadingEnvelopeDeg")


@dataclass(frozen=True)
class Scenario:
    name: str
    scale_limit: float
    side_mismatch_limit: float
    imu_bias_limit_deg: float
    lateral_push_limit_in: float
    slip_loss_limit: float


@dataclass
class Trial:
    scenario: str
    target_in: float
    max_power: int
    success: bool
    reason: str
    duration_s: float
    actual_x_in: float
    actual_y_in: float
    estimated_x_in: float
    estimated_y_in: float
    actual_error_in: float
    estimated_error_in: float
    actual_cross_in: float
    scale_error: float
    side_mismatch: float
    imu_bias_deg: float
    lateral_push_in: float
    slip_loss: float


SCENARIOS = (
    Scenario("nominal", 0.005, 0.003, 0.15, 0.0, 0.0),
    Scenario("provisional envelope", PROVISIONAL_SCALE_FRACTION, 0.010,
             PROVISIONAL_HEADING_DEG, 0.0, 0.0),
    Scenario("unobserved slip/push", PROVISIONAL_SCALE_FRACTION, 0.015,
             PROVISIONAL_HEADING_DEG, 4.0, 0.25),
)


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def slew(target: float, current: float, rate: float) -> float:
    return clamp(target, current - rate * DT, current + rate * DT)


def signed_angle_deg(target: float, current: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def run_trial(rng: random.Random, scenario: Scenario,
              target_in: float, max_power: int) -> Trial:
    scale_error = rng.uniform(-scenario.scale_limit, scenario.scale_limit)
    side_mismatch = rng.uniform(
        -scenario.side_mismatch_limit, scenario.side_mismatch_limit)
    left_scale = 1.0 + scale_error + side_mismatch
    right_scale = 1.0 + scale_error - side_mismatch
    imu_bias_deg = rng.uniform(
        -scenario.imu_bias_limit_deg, scenario.imu_bias_limit_deg)
    lateral_push = rng.uniform(
        -scenario.lateral_push_limit_in, scenario.lateral_push_limit_in)
    slip_loss = rng.uniform(0.0, scenario.slip_loss_limit)
    push_at = rng.uniform(0.30, 0.70) * target_in

    # go_straight_to() has already completed its initial turn. A small residual
    # remains in the physical heading, while the estimator sees IMU+bias.
    actual_x = actual_y = estimated_x = estimated_y = 0.0
    actual_heading = math.radians(rng.uniform(-1.0, 1.0))
    estimated_heading = actual_heading + math.radians(imu_bias_deg)
    left_velocity = right_velocity = 0.0
    forward_command = turn_command = 0.0
    settle_started = None
    pushed = False
    reason = "timeout"
    success = False
    elapsed = 0.0

    while elapsed < TIMEOUT_S:
        along_remaining = target_in - estimated_x
        cross_track = estimated_y
        inside_finish = (
            -TOLERANCE_IN <= along_remaining <= TOLERANCE_IN
            and abs(cross_track) <= MAX_FINISH_CROSS_IN
        )
        if inside_finish:
            forward_command = turn_command = 0.0
            if settle_started is None:
                settle_started = elapsed
            elif elapsed - settle_started >= SETTLE_S:
                success = True
                reason = "success"
                break
        else:
            settle_started = None
            if along_remaining < -TOLERANCE_IN:
                reason = "finish_plane_miss"
                break

            cross_heading = clamp(
                -math.degrees(math.atan2(cross_track, LOOKAHEAD_IN)),
                -MAX_CROSS_HEADING_DEG,
                MAX_CROSS_HEADING_DEG,
            )
            bearing_error = signed_angle_deg(
                cross_heading, math.degrees(estimated_heading))
            final_error = signed_angle_deg(
                0.0, math.degrees(estimated_heading))
            target_forward = clamp(
                max(0.0, along_remaining) * DRIVE_KP, 0.0, max_power)
            if along_remaining > TOLERANCE_IN * 1.5:
                target_forward = max(target_forward, MIN_POWER)
            heading_scale = clamp(
                math.cos(math.radians(clamp(abs(bearing_error), 0.0, 80.0))),
                0.25,
                1.0,
            )
            target_forward *= heading_scale
            if abs(bearing_error) > 55.0:
                target_forward = min(target_forward, 32.0)
            target_turn = clamp(
                bearing_error * HEADING_KP + final_error * FINAL_HEADING_KP,
                -MAX_TURN_POWER,
                MAX_TURN_POWER,
            )
            forward_command = slew(
                target_forward, forward_command, FWD_SLEW_PER_S)
            turn_command = slew(
                target_turn, turn_command, TURN_SLEW_PER_S)

        encoder_left_target = (
            (forward_command + turn_command) / 127.0 * MAX_WHEEL_IPS)
        encoder_right_target = (
            (forward_command - turn_command) / 127.0 * MAX_WHEEL_IPS)
        # Active drive responds more slowly than a brake command.
        tau = 0.040 if inside_finish else 0.120
        response = 1.0 - math.exp(-DT / tau)
        left_velocity += (encoder_left_target - left_velocity) * response
        right_velocity += (encoder_right_target - right_velocity) * response
        encoder_left_delta = left_velocity * DT
        encoder_right_delta = right_velocity * DT

        segment_fraction = clamp(estimated_x / max(target_in, 1e-6), 0.0, 1.0)
        active_slip = slip_loss if 0.25 <= segment_fraction <= 0.75 else 0.0
        actual_left_delta = encoder_left_delta * left_scale * (1.0 - active_slip)
        actual_right_delta = encoder_right_delta * right_scale * (1.0 - active_slip)
        actual_center_delta = (actual_left_delta + actual_right_delta) / 2.0
        actual_heading_delta = (
            (actual_left_delta - actual_right_delta) / TRACK_IN)
        actual_heading += actual_heading_delta
        actual_x += actual_center_delta * math.cos(actual_heading)
        actual_y += actual_center_delta * math.sin(actual_heading)

        if not pushed and estimated_x >= push_at:
            actual_y += lateral_push
            pushed = True

        # Encoders propagate distance, and the IMU owns heading. An external
        # lateral push is deliberately invisible while P5 is disabled.
        estimated_center_delta = (
            encoder_left_delta + encoder_right_delta) / 2.0
        estimated_heading = actual_heading + math.radians(imu_bias_deg)
        estimated_x += estimated_center_delta * math.cos(estimated_heading)
        estimated_y += estimated_center_delta * math.sin(estimated_heading)
        elapsed += DT

    return Trial(
        scenario=scenario.name,
        target_in=target_in,
        max_power=max_power,
        success=success,
        reason=reason,
        duration_s=elapsed,
        actual_x_in=actual_x,
        actual_y_in=actual_y,
        estimated_x_in=estimated_x,
        estimated_y_in=estimated_y,
        actual_error_in=math.hypot(target_in - actual_x, actual_y),
        estimated_error_in=math.hypot(target_in - estimated_x, estimated_y),
        actual_cross_in=actual_y,
        scale_error=scale_error,
        side_mismatch=side_mismatch,
        imu_bias_deg=imu_bias_deg,
        lateral_push_in=lateral_push,
        slip_loss=slip_loss,
    )


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q))


def summarize(trials):
    summary = []
    keys = sorted({(t.scenario, t.target_in, t.max_power) for t in trials})
    for scenario, target, power in keys:
        group = [t for t in trials if
                 (t.scenario, t.target_in, t.max_power) ==
                 (scenario, target, power)]
        summary.append({
            "scenario": scenario,
            "target_in": target,
            "max_power": power,
            "controller_success_pct": 100.0 * sum(t.success for t in group) / len(group),
            "actual_within_2in_pct": 100.0 * sum(t.actual_error_in <= 2.0 for t in group) / len(group),
            "actual_error_p50_in": percentile([t.actual_error_in for t in group], 50),
            "actual_error_p95_in": percentile([t.actual_error_in for t in group], 95),
            "actual_error_max_in": max(t.actual_error_in for t in group),
            "estimated_error_p95_in": percentile([t.estimated_error_in for t in group], 95),
        })
    return summary


def plot_dashboard(trials, summary, output_base: Path):
    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    fig.suptitle("Straight-line navigation robustness (offline Monte Carlo)",
                 fontsize=18, fontweight="bold")

    colors = {"nominal": "#55d6be", "provisional envelope": "#ffd166",
              "unobserved slip/push": "#ff6b6b"}
    ax = axes[0, 0]
    for scenario in colors:
        rows = [r for r in summary if r["scenario"] == scenario and r["max_power"] == 40]
        ax.plot([r["target_in"] for r in rows],
                [r["actual_error_p95_in"] for r in rows], "o-",
                label=scenario, color=colors[scenario], linewidth=2)
    ax.axhline(2.0, color="white", linestyle="--", alpha=.5, label="2 in target")
    ax.set(title="95th-percentile physical endpoint error @ power 40",
           xlabel="Path length (in)", ylabel="Error (in)")
    ax.grid(alpha=.2); ax.legend(fontsize=8)

    ax = axes[0, 1]
    focus = [t for t in trials if t.target_in == 48 and t.max_power == 40]
    for scenario in colors:
        group = [t for t in focus if t.scenario == scenario]
        ax.scatter([t.actual_x_in for t in group], [t.actual_y_in for t in group],
                   s=9, alpha=.35, label=scenario, color=colors[scenario])
    ax.scatter([48], [0], marker="*", s=180, color="white", label="target")
    circle = plt.Circle((48, 0), 2, fill=False, color="white", linestyle="--", alpha=.6)
    ax.add_patch(circle)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set(title="Physical endpoints: 48 in path @ power 40",
           xlabel="Forward (in)", ylabel="Cross-track (in)")
    ax.grid(alpha=.2); ax.legend(fontsize=8)

    ax = axes[1, 0]
    powers = (20, 40, 60); lengths = (6, 12, 24, 48, 84)
    envelope = [r for r in summary if r["scenario"] == "provisional envelope"]
    matrix = np.array([[next(r["actual_within_2in_pct"] for r in envelope
                              if r["target_in"] == length and r["max_power"] == power)
                        for length in lengths] for power in powers])
    image = ax.imshow(matrix, vmin=0, vmax=100, cmap="viridis", aspect="auto")
    for row in range(len(powers)):
        for col in range(len(lengths)):
            ax.text(col, row, f"{matrix[row, col]:.0f}%", ha="center", va="center")
    ax.set_xticks(range(len(lengths)), lengths); ax.set_yticks(range(len(powers)), powers)
    ax.set(title="Within 2 in: provisional error envelope",
           xlabel="Path length (in)", ylabel="Max power")
    fig.colorbar(image, ax=ax, label="Trials within 2 in (%)")

    ax = axes[1, 1]
    labels = []
    estimated = []
    physical = []
    for scenario in colors:
        group = [t for t in focus if t.scenario == scenario]
        labels.append(scenario.replace("unobserved ", ""))
        estimated.append(percentile([t.estimated_error_in for t in group], 95))
        physical.append(percentile([t.actual_error_in for t in group], 95))
    x = np.arange(len(labels)); width = .36
    ax.bar(x - width / 2, estimated, width, label="Estimator says", color="#4dabf7")
    ax.bar(x + width / 2, physical, width, label="Physical truth", color="#ff922b")
    ax.set_xticks(x, labels, rotation=10)
    ax.set(title="95th-percentile error: confidence gap",
           ylabel="Endpoint error (in)")
    ax.grid(axis="y", alpha=.2); ax.legend()

    fig.text(.5, .002,
             "Shadow model only. Validates control logic trends; live carpet/tether/obstacle testing remains required.",
             ha="center", color="#adb5bd", fontsize=9)
    fig.savefig(output_base.with_suffix(".png"), dpi=180)
    fig.savefig(output_base.with_suffix(".svg"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=150,
                        help="trials per scenario/distance/power cell")
    parser.add_argument("--seed", type=int, default=20260823)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("reports/sensor_campaign_2026-08-23"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)
    trials = [
        run_trial(rng, scenario, distance, power)
        for scenario in SCENARIOS
        for distance in (6, 12, 24, 48, 84)
        for power in (20, 40, 60)
        for _ in range(args.trials)
    ]
    summary = summarize(trials)

    csv_path = args.output_dir / "navigation_monte_carlo_trials.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=asdict(trials[0]).keys())
        writer.writeheader(); writer.writerows(asdict(t) for t in trials)
    summary_path = args.output_dir / "navigation_monte_carlo_summary.json"
    summary_path.write_text(json.dumps({
        "seed": args.seed,
        "trials_per_cell": args.trials,
        "model_limitations": [
            "offline differential-drive shadow model, not hardware evidence",
            "does not model carpet seams, tether forces, battery sag, or contact geometry",
            "unobserved push/slip scenario intentionally demonstrates estimator overconfidence",
        ],
        "summary": summary,
    }, indent=2) + "\n", encoding="utf-8")
    plot_dashboard(trials, summary,
                   args.output_dir / "navigation_robustness_dashboard")
    print(f"wrote {len(trials)} trials to {csv_path}")
    for scenario in (s.name for s in SCENARIOS):
        focus = [r for r in summary if r["scenario"] == scenario
                 and r["target_in"] == 48 and r["max_power"] == 40][0]
        print(scenario, json.dumps(focus, sort_keys=True))


if __name__ == "__main__":
    main()
