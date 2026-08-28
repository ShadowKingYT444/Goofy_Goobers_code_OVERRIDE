#!/usr/bin/env python3
"""Stress-test duplicate Goal-tag association against pose-prior error.

The model mirrors the production candidate enumeration and gates. It does not
claim camera accuracy; it asks whether a noisy but otherwise valid observation
can be associated with the correct duplicated Goal/face from a given prior.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


GOALS = (
    ("center", 0, 0.0, 0.0),
    ("top_red_neutral", 4, 47.10, 23.55),
    ("top_blue_alliance", 3, 47.10, -23.54),
    ("upper_red_neutral", 1, 23.55, 47.10),
    ("upper_blue_alliance", 2, 23.55, -47.09),
    ("lower_red_alliance", 2, -23.54, 47.10),
    ("lower_blue_neutral", 1, -23.54, -47.09),
    ("bottom_red_alliance", 3, -47.09, 23.55),
    ("bottom_blue_neutral", 4, -47.09, -23.54),
)
FACES = (("+x", 1, 0), ("+y", 0, 1), ("-x", -1, 0), ("-y", 0, -1))
FACE_OFFSET = 5.61 / 2.0
FIELD_HALF = 70.2
MIN_RANGE = 4.0
MAX_RANGE = 96.0
MAX_BEARING_RESIDUAL = 40.0
MAX_RANGE_RESIDUAL = 8.0
MIN_GOAL_MARGIN = 8.0
RANGE_RESIDUAL_WEIGHT_DEG_PER_IN = 0.5
MIN_FACE_SCORE_MARGIN = 2.0


def wrap(value):
    return value % 360.0


def angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def face_point(goal, face):
    return goal[2] + face[1] * FACE_OFFSET, goal[3] + face[2] * FACE_OFFSET


def candidates(tag_id, pose_x, pose_y, heading, bearing, measured_range):
    found = []
    for goal in GOALS:
        if goal[1] != tag_id:
            continue
        for face in FACES:
            tag_x, tag_y = face_point(goal, face)
            if (pose_x - tag_x) * face[1] + (pose_y - tag_y) * face[2] <= 0:
                continue
            predicted_bearing = angle_diff(
                wrap(math.degrees(math.atan2(tag_y - pose_y, tag_x - pose_x))),
                heading,
            )
            bearing_residual = abs(angle_diff(bearing, predicted_bearing))
            predicted_range = math.hypot(tag_x - pose_x, tag_y - pose_y)
            found.append({
                "goal": goal[0], "face": face[0],
                "bearing_residual": bearing_residual,
                "range_residual": abs(measured_range - predicted_range),
                "score": math.hypot(
                    bearing_residual,
                    abs(measured_range - predicted_range) *
                    RANGE_RESIDUAL_WEIGHT_DEG_PER_IN,
                ),
            })
    found.sort(key=lambda item: item["score"])
    return found


def classify(found, truth_goal, truth_face, measured_range):
    if not found:
        return "no_candidate"
    best = found[0]
    other_goal = next((item for item in found[1:]
                       if item["goal"] != best["goal"]), None)
    other_face = next((item for item in found[1:]
                       if item["goal"] == best["goal"] and
                       item["face"] != best["face"]), None)
    margin = (other_goal["score"] - best["score"]
              if other_goal else math.inf)
    if best["bearing_residual"] > MAX_BEARING_RESIDUAL:
        return "bearing_reject"
    if not MIN_RANGE <= measured_range <= MAX_RANGE:
        return "range_invalid"
    if best["range_residual"] > MAX_RANGE_RESIDUAL:
        return "range_reject"
    if margin < MIN_GOAL_MARGIN:
        return "ambiguous"
    if other_face and other_face["score"] - best["score"] < MIN_FACE_SCORE_MARGIN:
        return "ambiguous_face"
    if best["goal"] != truth_goal:
        return "wrong_goal_accept"
    if best["face"] != truth_face:
        return "wrong_face_accept"
    return "correct_accept"


def make_trial(rng, prior_error_in):
    goal = rng.choice(GOALS)
    face = rng.choice(FACES)
    tag_x, tag_y = face_point(goal, face)
    distance = rng.uniform(8.0, 72.0)
    # Camera must sit in front of the printed face. Add tangential displacement
    # while preserving a plausible field location.
    tangent = rng.uniform(-0.7, 0.7) * distance
    truth_x = tag_x + face[1] * distance - face[2] * tangent
    truth_y = tag_y + face[2] * distance + face[1] * tangent
    if abs(truth_x) > FIELD_HALF or abs(truth_y) > FIELD_HALF:
        return None
    global_bearing = wrap(math.degrees(math.atan2(tag_y - truth_y, tag_x - truth_x)))
    image_bearing = rng.uniform(-32.0, 32.0)
    truth_heading = wrap(global_bearing - image_bearing)
    true_range = math.hypot(tag_x - truth_x, tag_y - truth_y)
    observed_bearing = image_bearing + rng.gauss(0.0, 0.6)
    observed_range = true_range * (1.0 + rng.gauss(0.0, 0.03))

    direction = rng.uniform(0.0, 2.0 * math.pi)
    magnitude = prior_error_in * math.sqrt(rng.random())
    estimate_x = truth_x + magnitude * math.cos(direction)
    estimate_y = truth_y + magnitude * math.sin(direction)
    estimate_heading = wrap(truth_heading + rng.gauss(0.0, 0.9))
    found = candidates(goal[1], estimate_x, estimate_y, estimate_heading,
                       observed_bearing, observed_range)
    return goal[1], classify(found, goal[0], face[0], observed_range)


def run(seed, trials_per_error):
    rng = random.Random(seed)
    results = {}
    by_id = {}
    for error in (0, 2, 6, 12, 24):
        counts = Counter()
        id_counts = {tag_id: Counter() for tag_id in range(5)}
        accepted = 0
        while accepted < trials_per_error:
            trial = make_trial(rng, error)
            if trial is None:
                continue
            tag_id, outcome = trial
            counts[outcome] += 1
            id_counts[tag_id][outcome] += 1
            accepted += 1
        results[str(error)] = dict(counts)
        by_id[str(error)] = {str(key): dict(value) for key, value in id_counts.items()}
    return results, by_id


def percentage(counts, key):
    return 100.0 * counts.get(key, 0) / max(1, sum(counts.values()))


def plot(results, output_base):
    errors = [int(value) for value in results]
    correct = [percentage(results[str(error)], "correct_accept") for error in errors]
    wrong_face = [percentage(results[str(error)], "wrong_face_accept") for error in errors]
    wrong_goal = [percentage(results[str(error)], "wrong_goal_accept") for error in errors]
    reject = [100.0 - a - b - c for a, b, c in zip(correct, wrong_face, wrong_goal)]

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5), constrained_layout=True)
    fig.suptitle("Duplicate AprilTag association vs pose-prior error",
                 fontsize=17, fontweight="bold")
    axes[0].plot(errors, correct, "o-", color="#55d6be", linewidth=2.5,
                 label="correct Goal + face")
    axes[0].plot(errors, reject, "o-", color="#ffd166", linewidth=2,
                 label="safely rejected")
    axes[0].plot(errors, wrong_face, "o-", color="#ff922b", linewidth=2,
                 label="wrong face accepted")
    axes[0].plot(errors, wrong_goal, "o-", color="#ff6b6b", linewidth=2,
                 label="wrong duplicate Goal accepted")
    axes[0].set(xlabel="Estimator position-prior error radius (in)",
                ylabel="Trials (%)", ylim=(-1, 101))
    axes[0].grid(alpha=.2); axes[0].legend(fontsize=8)

    bottoms = np.zeros(len(errors))
    for values, label, color in ((correct, "correct", "#55d6be"),
                                 (reject, "rejected", "#ffd166"),
                                 (wrong_face, "wrong face", "#ff922b"),
                                 (wrong_goal, "wrong Goal", "#ff6b6b")):
        axes[1].bar(errors, values, bottom=bottoms, width=1.4,
                    label=label, color=color)
        bottoms += np.asarray(values)
    axes[1].set(xlabel="Estimator position-prior error radius (in)",
                ylabel="Outcome (%)", ylim=(0, 100))
    axes[1].grid(axis="y", alpha=.2); axes[1].legend(fontsize=8)
    fig.text(.5, .005,
             "Synthetic valid-tag model; tests association logic, not camera/PnP accuracy.",
             ha="center", color="#adb5bd")
    fig.savefig(output_base.with_suffix(".png"), dpi=180)
    fig.savefig(output_base.with_suffix(".svg"))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=20260823)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("reports/sensor_campaign_2026-08-23"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results, by_id = run(args.seed, args.trials)
    payload = {"seed": args.seed, "trials_per_prior": args.trials,
               "model_limit": "association shadow model, not camera accuracy",
               "results": results, "by_tag_id": by_id}
    path = args.output_dir / "tag_association_observability.json"
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    plot(results, args.output_dir / "tag_association_observability")
    for error, counts in results.items():
        print(error, {key: round(percentage(counts, key), 3) for key in counts})
    print(path)


if __name__ == "__main__":
    main()
