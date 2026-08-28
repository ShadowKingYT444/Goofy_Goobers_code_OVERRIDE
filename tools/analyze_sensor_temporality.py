#!/usr/bin/env python3
"""Summarize live tuple repetition and telemetry/poll cadence."""

from __future__ import annotations

import csv
import glob
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-23"
GPS_SOURCE = REPORT / "stationary_04_120s" / "telemetry.csv"
P8_VALID_SOURCE = REPORT / "ai_depth_live_01" / "raw.log"
VISION_RE = re.compile(
    r"VISION_SHADOW t=(\d+) poll=(\d+).*?repeat=(\d+) "
    r"geometry_age=(\d+) valid=(\d+)"
)


def resample_gps_10hz() -> list[dict[str, str]]:
    rows = list(csv.DictReader(GPS_SOURCE.open(encoding="utf-8")))
    selected: list[dict[str, str]] = []
    target = int(rows[0]["robot_ms"])
    index = 0
    while index < len(rows):
        while (
            index + 1 < len(rows)
            and abs(int(rows[index + 1]["robot_ms"]) - target)
            <= abs(int(rows[index]["robot_ms"]) - target)
        ):
            index += 1
        selected.append(rows[index])
        target += 100
        index += 1
    return selected


def d4_intervals() -> np.ndarray:
    values: list[int] = []
    for filename in glob.glob(str(REPORT / "*" / "telemetry.csv")):
        rows = list(csv.DictReader(open(filename, encoding="utf-8")))
        times = [int(float(row["robot_ms"])) for row in rows if row.get("robot_ms")]
        values.extend(
            delta
            for first, second in zip(times, times[1:])
            if 0 < (delta := (second - first) & 0xFFFFFFFF) < 10_000
        )
    return np.asarray(values)


def vision_frames(filename: Path) -> list[tuple[int, int, int, int, int]]:
    frames = []
    for line in filename.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = VISION_RE.search(line)
        if match:
            frames.append(tuple(map(int, match.groups())))
    return frames


def all_vision_intervals() -> np.ndarray:
    values: list[int] = []
    for filename in REPORT.glob("*/raw.log"):
        frames = vision_frames(filename)
        values.extend(
            delta
            for first, second in zip(frames, frames[1:])
            if 0 < (delta := (second[0] - first[0]) & 0xFFFFFFFF) < 10_000
        )
    return np.asarray(values)


def true_run_lengths(values: np.ndarray) -> np.ndarray:
    runs: list[int] = []
    current = 0
    for value in values:
        if value:
            current += 1
        elif current:
            runs.append(current)
            current = 0
    if current:
        runs.append(current)
    return np.asarray(runs, dtype=int)


def main() -> None:
    gps = resample_gps_10hz()
    tuples = [
        (row["gps_x"], row["gps_y"], row["gps_heading"])
        for row in gps
    ]
    gps_repeat = np.asarray(
        [first == second for first, second in zip(tuples, tuples[1:])],
        dtype=bool,
    )
    p8 = [frame for frame in vision_frames(P8_VALID_SOURCE) if frame[4] == 1]
    p8_repeat = np.asarray([bool(frame[2]) for frame in p8], dtype=bool)
    p8_intervals = np.asarray([
        (second[0] - first[0]) & 0xFFFFFFFF
        for first, second in zip(p8, p8[1:])
    ])
    d4_dt = d4_intervals()
    vision_dt = all_vision_intervals()
    gps_repeat_runs = true_run_lengths(gps_repeat)
    p8_repeat_runs = true_run_lengths(p8_repeat)
    gps_repeat_lag1 = float(np.corrcoef(
        gps_repeat[:-1].astype(float), gps_repeat[1:].astype(float)
    )[0, 1])

    summary = {
        "gps_source": str(GPS_SOURCE.relative_to(ROOT)),
        "gps_resampled_points_10hz": len(gps),
        "gps_adjacent_transitions": len(gps_repeat),
        "gps_exact_adjacent_repeats": int(np.sum(gps_repeat)),
        "gps_exact_adjacent_repeat_fraction": float(np.mean(gps_repeat)),
        "gps_repeat_runs": {
            "count": len(gps_repeat_runs),
            "median_transitions": float(np.median(gps_repeat_runs)),
            "maximum_transitions": int(np.max(gps_repeat_runs)),
            "lag1_binary_correlation": gps_repeat_lag1,
        },
        "p8_source": str(P8_VALID_SOURCE.relative_to(ROOT)),
        "p8_valid_polls": len(p8),
        "p8_valid_repeated_geometry_polls": int(np.sum(p8_repeat)),
        "p8_valid_repeated_geometry_fraction": float(np.mean(p8_repeat)),
        "p8_repeat_runs": {
            "count": len(p8_repeat_runs),
            "median_polls": float(np.median(p8_repeat_runs)),
            "maximum_polls": int(np.max(p8_repeat_runs)),
        },
        "p8_valid_poll_interval_ms": {
            "median": float(np.median(p8_intervals)),
            "maximum": int(np.max(p8_intervals)),
        },
        "all_live_d4_intervals": {
            "count": len(d4_dt),
            "median_ms": float(np.median(d4_dt)),
            "p99_ms": float(np.quantile(d4_dt, 0.99)),
            "maximum_ms": int(np.max(d4_dt)),
        },
        "all_live_p8_intervals": {
            "count": len(vision_dt),
            "median_ms": float(np.median(vision_dt)),
            "p99_ms": float(np.quantile(vision_dt, 0.99)),
            "maximum_ms": int(np.max(vision_dt)),
        },
        "interpretation": (
            "A host/Smart-Port poll is not evidence of a new optical exposure. "
            "Exact tuples are common enough to matter, so production preserves "
            "but does not advance temporal confirmation on a repeat. Cadence "
            "statistics describe saved telemetry, not sensor acquisition latency."
        ),
    }
    (REPORT / "sensor_temporality_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    with (REPORT / "sensor_temporality_samples.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=("sensor", "transition_index", "repeated", "interval_ms"),
        )
        writer.writeheader()
        for index, repeated in enumerate(gps_repeat):
            writer.writerow({
                "sensor": "P7_GPS_10Hz_resample",
                "transition_index": index,
                "repeated": int(repeated),
                "interval_ms": 100,
            })
        for index, (repeated, interval) in enumerate(zip(p8_repeat[1:], p8_intervals)):
            writer.writerow({
                "sensor": "P8_valid_tag",
                "transition_index": index,
                "repeated": int(repeated),
                "interval_ms": int(interval),
            })

    plt.style.use("dark_background")
    fig, axes = plt.subplots(1, 3, figsize=(17, 6.3))
    fig.suptitle(
        "Live Sensor Temporality — Polls Are Not Optical Timestamps",
        fontsize=17,
        fontweight="bold",
    )
    ax = axes[0]
    window = min(300, len(gps_repeat))
    ax.eventplot(
        np.flatnonzero(gps_repeat[:window]),
        colors="#ffb347",
        lineoffsets=0.5,
        linelengths=0.8,
    )
    ax.set(
        xlim=(0, window), ylim=(0, 1), yticks=[],
        xlabel="10-Hz transition index (first 300)",
        title=(
            f"P7 exact repeats: {np.sum(gps_repeat)}/{len(gps_repeat)} "
            f"({np.mean(gps_repeat):.2%})"
        ),
    )
    ax.grid(axis="x", alpha=0.16)

    ax = axes[1]
    ax.bar(
        ("changed", "exact repeat"),
        (len(p8_repeat) - int(np.sum(p8_repeat)), int(np.sum(p8_repeat))),
        color=("#5cffad", "#ffb347"),
    )
    ax.set(
        ylabel="Valid Tag 4 polls",
        title=f"P8 valid geometry: {np.sum(p8_repeat)}/{len(p8_repeat)} repeats",
    )
    ax.grid(axis="y", alpha=0.16)

    ax = axes[2]
    bins = np.arange(40, 341, 20)
    ax.hist(d4_dt, bins=bins, alpha=0.7, label=f"D4 (median {np.median(d4_dt):.0f} ms)", color="#58d6ff")
    ax.hist(vision_dt, bins=bins, alpha=0.7, label=f"P8 polls (median {np.median(vision_dt):.0f} ms)", color="#c084fc")
    ax.set(
        xlabel="Observed interval (ms)", ylabel="Transitions (log scale)",
        yscale="log", title="Saved telemetry cadence",
    )
    ax.grid(alpha=0.16)
    ax.legend()
    fig.text(
        0.5, 0.025,
        "Exact repeats preserve a pending sequence but add no confirmation/correction. "
        "Timing is transport evidence, not physical acquisition latency.",
        ha="center", fontsize=10.5, color="#dbeafe",
    )
    fig.tight_layout(rect=(0.02, 0.11, 0.99, 0.91), w_pad=3.0)
    for suffix in ("png", "svg"):
        fig.savefig(REPORT / f"sensor_temporality_dashboard.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
