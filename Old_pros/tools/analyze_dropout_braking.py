#!/usr/bin/env python3
"""Compare live navigation dropout stopping trials."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt


COMPLETE_RE = re.compile(
    r"NAV_DROPOUT event=complete result=(?P<result>\S+) injected=(?P<injected>[01]) "
    r"abort_travel=(?P<abort_travel>[-+\d.]+) coast_in=(?P<coast>[-+\d.]+) "
    r"reinitialized=(?P<reinitialized>[01])"
)


def read_trial(path: Path, label: str) -> dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="replace")
    match = COMPLETE_RE.search(text)
    if not match:
        raise ValueError(f"no NAV_DROPOUT completion record in {path}")
    values = match.groupdict()
    return {
        "label": label,
        "result": values["result"],
        "injected": bool(int(values["injected"])),
        "abort_travel_in": float(values["abort_travel"]),
        "coast_in_250ms": float(values["coast"]),
        "reinitialized": bool(int(values["reinitialized"])),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--brake", type=Path, required=True)
    parser.add_argument("--hold", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    trials = [read_trial(args.brake, "BRAKE"), read_trial(args.hold, "HOLD")]
    brake_coast = float(trials[0]["coast_in_250ms"])
    hold_coast = float(trials[1]["coast_in_250ms"])
    reduction = 100.0 * (brake_coast - hold_coast) / brake_coast
    summary = {
        "trials": trials,
        "coast_reduction_in": brake_coast - hold_coast,
        "coast_reduction_percent": reduction,
        "hold_pass_under_quarter_inch": hold_coast <= 0.25,
        "both_fail_closed": all(
            trial["result"] == "drive_failed" and trial["injected"]
            for trial in trials
        ),
        "both_reinitialized": all(trial["reinitialized"] for trial in trials),
    }
    (args.output / "dropout_braking_comparison.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
    colors = ["#d97706", "#059669"]
    coast = [float(trial["coast_in_250ms"]) for trial in trials]
    axes[0].bar(["BRAKE", "HOLD"], coast, color=colors)
    axes[0].axhline(0.25, color="#334155", linestyle="--", label="0.25-in gate")
    for index, value in enumerate(coast):
        axes[0].text(index, value + 0.025, f"{value:.4f} in", ha="center", weight="bold")
    axes[0].set(title=f"250-ms coast reduced {reduction:.1f}%", ylabel="Coast after abort (in)")
    axes[0].set_ylim(0, max(coast) * 1.25)
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend()

    abort = [float(trial["abort_travel_in"]) for trial in trials]
    total = [abort[index] + coast[index] for index in range(2)]
    axes[1].bar(["BRAKE", "HOLD"], abort, color="#2563eb", label="travel at detection")
    axes[1].bar(["BRAKE", "HOLD"], coast, bottom=abort, color=colors, label="post-abort coast")
    for index, value in enumerate(total):
        axes[1].text(index, value + 0.035, f"total {value:.3f} in", ha="center")
    axes[1].set(title="Deterministic IMU-dropout trials", ylabel="Distance from start (in)")
    axes[1].set_ylim(0, max(total) * 1.2)
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].legend(fontsize=8)
    fig.suptitle("Live fused-navigation emergency braking", fontsize=15, weight="bold")
    fig.savefig(args.output / "dropout_braking_comparison.png", dpi=220, facecolor="white")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
