#!/usr/bin/env python3
"""Parse a complete or interrupted rotation sweep and plot sensor agreement."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
RAW = ROOT / "rotation_sweep_live_01" / "raw.log"
OUT_CSV = ROOT / "rotation_sweep_live_results.csv"
OUT_JSON = ROOT / "rotation_sweep_live_summary.json"
OUT_PNG = ROOT / "rotation_sweep_live_dashboard.png"

TURN_RE = re.compile(
    r"ROT_SWEEP target_deg=(?P<target>[\d.]+) phase=turn "
    r"(?:direction=(?P<direction>\w+) )?"
    r"encoder_deg=(?P<encoder>[\d.-]+) gps_deg=(?P<gps>[\d.-]+) "
    r"imu_deg=(?P<imu>[\d.-]+) encoder_minus_gps_deg=(?P<difference>[\d.-]+) "
    r"effective_track_in=(?P<effective_track>[\d.-]+) gps_error_in=(?P<gps_error>[\d.-]+)"
)
RETURN_RE = re.compile(
    r"ROT_SWEEP target_deg=(?P<target>[\d.]+) phase=return "
    r"encoder_residual_motor_deg=(?P<encoder_return>[\d.-]+) "
    r"gps_heading_residual_deg=(?P<gps_return>[\d.-]+) "
    r"imu_heading_residual_deg=(?P<imu_return>[\d.-]+) "
    r"gps_position_residual_in=(?P<position_return>[\d.-]+)"
)


def parse() -> list[dict[str, float | str]]:
    text = RAW.read_bytes().decode("utf-8", errors="ignore")
    rows: dict[float, dict[str, float | str]] = {}
    for match in TURN_RE.finditer(text):
        raw = match.groupdict()
        target = float(raw.pop("target"))
        direction = raw.pop("direction") or "ccw_inferred"
        rows[target] = {"target": target, "direction": direction}
        rows[target].update({key: float(value) for key, value in raw.items()})
    for match in RETURN_RE.finditer(text):
        raw = match.groupdict()
        target = float(raw.pop("target"))
        rows.setdefault(target, {"target": target, "direction": "unknown"})
        rows[target].update({key: float(value) for key, value in raw.items()})
    return [rows[target] for target in sorted(rows) if "position_return" in rows[target]]


def main() -> None:
    rows = parse()
    if not rows:
        raise SystemExit("no complete turn/return pairs found")
    fields = list(rows[0])
    with OUT_CSV.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    target = np.array([float(row["target"]) for row in rows])
    encoder = np.abs([float(row["encoder"]) for row in rows])
    gps = np.abs([float(row["gps"]) for row in rows])
    imu = np.abs([float(row["imu"]) for row in rows])
    track = np.array([float(row["effective_track"]) for row in rows])
    gps_return = np.abs([float(row["gps_return"]) for row in rows])
    imu_return = np.abs([float(row["imu_return"]) for row in rows])
    pos_return = np.array([float(row["position_return"]) for row in rows])
    gps_imu_delta = gps - imu

    # This track estimate uses the configured 2.75-inch wheel diameter. Linear
    # wheel-scale correction is intentionally reported separately because the
    # two parameters are coupled.
    summary = {
        "completed_pairs": len(rows),
        "planned_pairs": 5,
        "interrupted_for_person_in_camera": len(rows) < 5,
        "effective_track_in_mean_using_configured_wheel": float(track.mean()),
        "effective_track_in_std": float(track.std()),
        "gps_imu_heading_difference_mean_deg": float(gps_imu_delta.mean()),
        "max_abs_gps_heading_return_error_deg": float(gps_return.max()),
        "max_abs_imu_heading_return_error_deg": float(imu_return.max()),
        "max_gps_position_return_error_in": float(pos_return.max()),
        "direction": "counterclockwise outward, clockwise return",
    }
    OUT_JSON.write_text(json.dumps(summary, indent=2) + "\n")

    plt.style.use("dark_background")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle("VEX Rotation Characterization — Safety-Interrupted Sweep", fontsize=17)

    ax = axes[0, 0]
    ax.plot(target, target, "--", color="0.55", label="nominal encoder target")
    ax.plot(target, encoder, "o-", label="encoder model")
    ax.plot(target, gps, "o-", label="GPS")
    ax.plot(target, imu, "o-", label="IMU")
    ax.set(title="Outward CCW angle magnitude", xlabel="nominal target (deg)", ylabel="measured (deg)")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[0, 1]
    ax.plot(target, track, "o-", color="#68d7ff")
    ax.axhline(track.mean(), linestyle="--", color="#ffe96f", label=f"mean {track.mean():.3f} in")
    ax.axhline(12.0086, linestyle=":", color="0.6", label="configured 12.009 in")
    ax.set(title="Effective track (configured wheel scale)", xlabel="nominal target (deg)", ylabel="track width (in)")
    ax.grid(alpha=0.2)
    ax.legend()

    ax = axes[1, 0]
    width = 0.28
    ax.bar(target - width, gps_return, width, label="GPS heading")
    ax.bar(target, imu_return, width, label="IMU heading")
    ax.bar(target + width, pos_return, width, label="GPS position (in)")
    ax.set(title="Residual after unwind", xlabel="nominal target (deg)", ylabel="absolute residual")
    ax.grid(axis="y", alpha=0.2)
    ax.legend()

    ax = axes[1, 1]
    ax.plot(target, gps - encoder, "o-", label="GPS − encoder")
    ax.plot(target, imu - encoder, "o-", label="IMU − encoder")
    ax.plot(target, gps_imu_delta, "o-", label="GPS − IMU")
    ax.axhline(0, color="0.55", linewidth=1)
    ax.set(title="Angle sensor disagreement", xlabel="nominal target (deg)", ylabel="difference (deg)")
    ax.grid(alpha=0.2)
    ax.legend()

    fig.savefig(OUT_PNG, dpi=180)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
