#!/usr/bin/env python3
"""Stress the public straight-segment and initial-turn prechecks offline."""

from __future__ import annotations

import csv
import json
import re
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
CONFIG = (ROOT / "include" / "localization_config.hpp").read_text(
    encoding="utf-8"
)


def setting(name: str) -> float:
    match = re.search(rf"{name}\s*=\s*([\d.]+)", CONFIG)
    if match is None:
        raise RuntimeError(f"missing production setting {name}")
    return float(match.group(1))


PHYSICAL_HALF_SPAN_IN = setting("kPhysicalWallHalfSpanIn")
WALL_CLEARANCE_IN = setting("kNavigationProvisionalWallClearanceIn")
GOAL_CLEARANCE_IN = setting("kNavigationProvisionalGoalClearanceIn")
MIN_ESCAPE_CLEARANCE_IN = setting("kNavigationMinimumGoalEscapeClearanceIn")
FIELD_ELEMENT_TOLERANCE_IN = setting("kNavigationFieldElementToleranceIn")
ERROR_GROWTH_PER_TRAVEL_IN = float(
    np.hypot(
        setting("kDeadReckoningScaleEnvelopeFraction"),
        np.tan(np.deg2rad(setting("kDeadReckoningHeadingEnvelopeDeg"))),
    )
)
GOALS = np.asarray([
    (0.0, 0.0),
    (47.10, 23.55),
    (47.10, -23.54),
    (23.55, 47.10),
    (23.55, -47.09),
    (-23.54, 47.10),
    (-23.54, -47.09),
    (-47.09, 23.55),
    (-47.09, -23.54),
])
TRIALS = 250_000


def segment_goal_distances(
    starts: np.ndarray, targets: np.ndarray
) -> np.ndarray:
    delta = targets - starts
    length_sq = np.sum(delta * delta, axis=1)
    result = np.full(len(starts), np.inf)
    for goal in GOALS:
        projection = np.divide(
            np.sum((goal - starts) * delta, axis=1),
            length_sq,
            out=np.zeros_like(length_sq),
            where=length_sq > 1e-12,
        )
        projection = np.clip(projection, 0.0, 1.0)
        closest = starts + projection[:, None] * delta
        result = np.minimum(result, np.linalg.norm(closest - goal, axis=1))
    return result


def main() -> None:
    rng = np.random.default_rng(20260823)
    starts = rng.uniform(-PHYSICAL_HALF_SPAN_IN, PHYSICAL_HALF_SPAN_IN, (TRIALS, 2))
    targets = rng.uniform(-PHYSICAL_HALF_SPAN_IN, PHYSICAL_HALF_SPAN_IN, (TRIALS, 2))
    start_headings_deg = rng.uniform(0.0, 360.0, TRIALS)
    # Exercise production's dynamic map inflation across plausible reported
    # outage envelopes. This reported envelope still excludes hidden slip,
    # pushes, collision, and start-placement error.
    starting_position_error_envelope_in = rng.uniform(0.0, 3.0, TRIALS)
    requested_path_length_in = np.linalg.norm(targets - starts, axis=1)
    position_error_envelope_in = (
        starting_position_error_envelope_in
        + requested_path_length_in * ERROR_GROWTH_PER_TRAVEL_IN
    )
    target_limit = (
        PHYSICAL_HALF_SPAN_IN
        - WALL_CLEARANCE_IN
        - FIELD_ELEMENT_TOLERANCE_IN
        - position_error_envelope_in
    )
    wall_ok = np.all(np.abs(targets) <= target_limit[:, None], axis=1)

    start_goal_distance = np.min(
        np.linalg.norm(starts[:, None, :] - GOALS[None, :, :], axis=2),
        axis=1,
    )
    required = np.minimum(
        GOAL_CLEARANCE_IN
        + FIELD_ELEMENT_TOLERANCE_IN
        + position_error_envelope_in,
        np.maximum(
            MIN_ESCAPE_CLEARANCE_IN
            + FIELD_ELEMENT_TOLERANCE_IN
            + position_error_envelope_in,
            start_goal_distance - 0.5,
        ),
    )
    minimum = segment_goal_distances(starts, targets)
    goal_ok = minimum >= required
    segment_accepted = wall_ok & goal_ok
    path_delta = targets - starts
    bearing_deg = np.degrees(np.arctan2(path_delta[:, 1], path_delta[:, 0])) % 360.0
    turn_error_deg = np.abs(
        (bearing_deg - start_headings_deg + 180.0) % 360.0 - 180.0
    )
    requires_turn = turn_error_deg > 2.0
    turn_target_limit = (
        PHYSICAL_HALF_SPAN_IN
        - WALL_CLEARANCE_IN
        - FIELD_ELEMENT_TOLERANCE_IN
        - starting_position_error_envelope_in
    )
    turn_wall_ok = np.all(
        np.abs(starts) <= turn_target_limit[:, None], axis=1
    )
    turn_goal_ok = (
        start_goal_distance
        >= GOAL_CLEARANCE_IN
        + FIELD_ELEMENT_TOLERANCE_IN
        + starting_position_error_envelope_in
    )
    turn_center_ok = turn_wall_ok & turn_goal_ok
    accepted = segment_accepted & (~requires_turn | turn_center_ok)
    reasons = np.where(
        ~wall_ok,
        "wall_clearance",
        np.where(
            ~goal_ok,
            "goal_clearance",
            np.where(
                requires_turn & ~turn_wall_ok,
                "turn_wall_clearance",
                np.where(
                    requires_turn & ~turn_goal_ok,
                    "turn_goal_clearance",
                    "accepted",
                ),
            ),
        ),
    )

    # Accepted is defined by these exact independent vectorized invariants;
    # nonzero values here would expose an analysis/implementation mismatch.
    accepted_wall_violations = int(np.sum(accepted & ~wall_ok))
    accepted_goal_violations = int(np.sum(accepted & ~goal_ok))
    accepted_turn_violations = int(
        np.sum(accepted & requires_turn & ~turn_center_ok)
    )
    summary = {
        "trials": TRIALS,
        "seed": 20260823,
        "physical_field_half_span_in": PHYSICAL_HALF_SPAN_IN,
        "endpoint_center_limit_in_at_zero_error": (
            PHYSICAL_HALF_SPAN_IN
            - WALL_CLEARANCE_IN
            - FIELD_ELEMENT_TOLERANCE_IN
        ),
        "sampled_start_pose_error_envelope_in": [0.0, 3.0],
        "dead_reckoning_error_growth_per_travel_in": ERROR_GROWTH_PER_TRAVEL_IN,
        "provisional_wall_clearance_in": WALL_CLEARANCE_IN,
        "normal_goal_center_clearance_in": GOAL_CLEARANCE_IN,
        "minimum_escape_clearance_in": MIN_ESCAPE_CLEARANCE_IN,
        "field_element_tolerance_in": FIELD_ELEMENT_TOLERANCE_IN,
        "result_counts": dict(Counter(reasons.tolist())),
        "segment_only_accepted_fraction": float(np.mean(segment_accepted)),
        "accepted_fraction": float(np.mean(accepted)),
        "accepted_wall_invariant_violations": accepted_wall_violations,
        "accepted_goal_invariant_violations": accepted_goal_violations,
        "accepted_turn_invariant_violations": accepted_turn_violations,
        "accepted_minimum_goal_clearance_in": float(np.min(minimum[accepted])),
        "interpretation": (
            "The precheck deterministically rejects every sampled target outside the "
            "endpoint wall envelope, every segment below its start-dependent Goal-center "
            "clearance, and every required turn whose center violates the provisional turn "
            "clearances. This validates gate logic, not physical collision safety."
        ),
        "not_modeled": [
            "actual robot footprint or rotation-center offset",
            "mechanism expansion",
            "other robots, cups, pins, loaders, toggles, or moved scoring objects",
            "path deviation, braking overshoot, or localization error outside the reported envelope",
            "actual swept footprint during the initial in-place turn; the gate checks only its center",
        ],
    }
    (REPORT / "route_safety_stress_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    examples = []
    ordered = (
        "accepted",
        "goal_clearance",
        "wall_clearance",
        "turn_goal_clearance",
        "turn_wall_clearance",
    )
    for reason in ordered:
        indices = np.flatnonzero(reasons == reason)[:20]
        for index in indices:
            examples.append({
                "reason": reason,
                "start_x_in": float(starts[index, 0]),
                "start_y_in": float(starts[index, 1]),
                "target_x_in": float(targets[index, 0]),
                "target_y_in": float(targets[index, 1]),
                "start_heading_deg": float(start_headings_deg[index]),
                "initial_turn_error_deg": float(turn_error_deg[index]),
                "position_error_envelope_in": float(
                    position_error_envelope_in[index]
                ),
                "starting_position_error_envelope_in": float(
                    starting_position_error_envelope_in[index]
                ),
                "requested_path_length_in": float(
                    requested_path_length_in[index]
                ),
                "minimum_goal_clearance_in": float(minimum[index]),
                "required_goal_clearance_in": float(required[index]),
            })
    with (REPORT / "route_safety_examples.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=examples[0])
        writer.writeheader()
        writer.writerows(examples)

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(17, 6.2), constrained_layout=True)
    fig.suptitle(
        "Public Route + Initial-Turn Precheck — 250,000 deterministic commands",
        fontsize=16,
        fontweight="bold",
    )
    ax = axes[0]
    colors = {
        "accepted": "#5cffad",
        "goal_clearance": "#ffb347",
        "wall_clearance": "#ff6b6b",
        "turn_goal_clearance": "#c084fc",
        "turn_wall_clearance": "#58d6ff",
    }
    for reason in ordered:
        selected = [row for row in examples if row["reason"] == reason][:8]
        for row in selected:
            ax.plot(
                [row["start_x_in"], row["target_x_in"]],
                [row["start_y_in"], row["target_y_in"]],
                color=colors[reason],
                alpha=0.55,
                linewidth=1.2,
            )
    ax.scatter(GOALS[:, 0], GOALS[:, 1], marker="s", s=55, color="#f3f4f6", label="Goal centers")
    zero_error_limit = (
        PHYSICAL_HALF_SPAN_IN
        - WALL_CLEARANCE_IN
        - FIELD_ELEMENT_TOLERANCE_IN
    )
    ax.add_patch(plt.Rectangle((-zero_error_limit, -zero_error_limit), 2 * zero_error_limit, 2 * zero_error_limit, fill=False, linestyle="--", linewidth=2, edgecolor="#58d6ff"))
    ax.set(xlim=(-72, 72), ylim=(-72, 72), aspect="equal", xlabel="Field X (in)", ylabel="Field Y (in)", title="Example accepted/rejected segments")
    ax.grid(alpha=0.16)

    ax = axes[1]
    counts = Counter(reasons.tolist())
    ax.bar(ordered, [counts[name] for name in ordered], color=[colors[name] for name in ordered])
    ax.set(ylabel="Segments", title="First rejection reason")
    ax.tick_params(axis="x", rotation=18)
    for index, name in enumerate(ordered):
        ax.text(index, counts[name], f"{counts[name]:,}\n{counts[name] / TRIALS:.1%}", ha="center", va="bottom")
    ax.grid(axis="y", alpha=0.16)

    axes[2].axis("off")
    note = (
        "LOGIC INVARIANTS\n"
        f"accepted wall violations: {accepted_wall_violations}\n"
        f"accepted Goal violations: {accepted_goal_violations}\n"
        f"accepted turn violations: {accepted_turn_violations}\n"
        f"segment-only accepted: {np.mean(segment_accepted):.1%}\n"
        f"accepted fraction: {np.mean(accepted):.1%}\n\n"
        "PROVISIONAL ENVELOPE\n"
        f"start pose envelope sampled: 0-3 in\n"
        f"projected growth: {ERROR_GROWTH_PER_TRAVEL_IN:.3%} of leg length\n"
        f"official Field Element tolerance: ±{FIELD_ELEMENT_TOLERANCE_IN:.1f} in\n"
        f"endpoint center: ±{zero_error_limit:.1f} in minus envelope\n"
        f"normal Goal radius: {GOAL_CLEARANCE_IN:.1f} + 1 in + envelope\n"
        f"escape floor: {MIN_ESCAPE_CLEARANCE_IN:.1f} + 1 in + envelope\n\n"
        "NOT A COLLISION PROOF\n"
        "No measured footprint/expansion, Pins/Loaders/Toggles,\n"
        "moved Blocks/robots, path\n"
        "deviation, braking overshoot, hidden pose error, or true swept\n"
        "footprint. Turn clearance is a center-point proxy only.\n"
        "The gate removes known-dangerous requests; live geometry\n"
        "and supervised validation are still mandatory."
    )
    axes[2].text(
        0.02,
        0.98,
        note,
        va="top",
        fontsize=11.5,
        linespacing=1.45,
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#162033", "edgecolor": "#58d6ff"},
    )
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"route_safety_stress_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
