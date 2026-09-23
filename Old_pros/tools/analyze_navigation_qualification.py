#!/usr/bin/env python3
"""Analyze a captured public-navigation qualification route."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


LEG_RE = re.compile(
    r"NAV_QUAL event=leg_done index=(?P<index>\d+) name=(?P<name>\S+) "
    r"result=(?P<result>\S+) elapsed_ms=(?P<elapsed_ms>\d+) "
    r"x=(?P<x>[-+\d.]+) y=(?P<y>[-+\d.]+) heading=(?P<heading>[-+\d.]+) "
    r"position_error=(?P<position_error>[-+\d.]+) "
    r"heading_error=(?P<heading_error>[-+\d.]+)"
)
COMPLETE_RE = re.compile(
    r"NAV_QUAL event=complete ok=(?P<ok>[01]) path_points=(?P<path_points>\d+) "
    r"loop_error=(?P<loop_error>[-+\d.]+) heading_error=(?P<heading_error>[-+\d.]+)"
)
STRAIGHT_LEG_RE = re.compile(
    r"NAV_STRAIGHT event=leg_done index=(?P<index>\d+) target=(?P<target>[-+\d.]+) "
    r"result=(?P<result>\S+) elapsed_ms=(?P<elapsed_ms>\d+) "
    r"x=(?P<x>[-+\d.]+) y=(?P<y>[-+\d.]+) heading=(?P<heading>[-+\d.]+) "
    r"error=(?P<position_error>[-+\d.]+) envelope=(?P<envelope>[-+\d.]+)"
)
STRAIGHT_COMPLETE_RE = re.compile(
    r"NAV_STRAIGHT event=complete ok=(?P<ok>[01]) "
    r"x=(?P<x>[-+\d.]+) y=(?P<y>[-+\d.]+) heading=(?P<heading>[-+\d.]+) "
    r"total_error=(?P<total_error>[-+\d.]+)"
)
CAMPAIGN_COMPLETE_RE = re.compile(
    r"NAV_(?P<campaign>BIDIR|TURN_RECOVERY|MIRROR) event=complete ok=(?P<ok>[01]) "
    r"(?P<details>[^\r\n]*)"
)
KEY_VALUE_RE = re.compile(r"(?P<key>[a-z_]+)=(?P<value>\S+)")
WHEEL_DIAMETER_IN = 2.433055


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def load_fusion(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("controller") in {"fused_drive", "fused_turn"}:
            required = ("host_s", "x", "y", "h")
            if all(finite(row.get(key)) for key in required):
                rows.append(row)
    return rows


def parse_route_events(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    legs: list[dict[str, Any]] = []
    complete = None
    if not path.exists():
        return legs, complete
    text = path.read_text(encoding="utf-8", errors="replace")
    for match in LEG_RE.finditer(text):
        item: dict[str, Any] = match.groupdict()
        for key in ("index", "elapsed_ms"):
            item[key] = int(item[key])
        for key in ("x", "y", "heading", "position_error", "heading_error"):
            item[key] = float(item[key])
        legs.append(item)
    for match in STRAIGHT_LEG_RE.finditer(text):
        item = match.groupdict()
        for key in ("index", "elapsed_ms"):
            item[key] = int(item[key])
        for key in ("target", "x", "y", "heading", "position_error", "envelope"):
            item[key] = float(item[key])
        item["name"] = f"straight_{item['target']:g}in"
        legs.append(item)
    legs.sort(key=lambda item: int(item["index"]))
    match = COMPLETE_RE.search(text)
    if match:
        complete = match.groupdict()
        complete["ok"] = bool(int(complete["ok"]))
        complete["path_points"] = int(complete["path_points"])
        complete["loop_error"] = float(complete["loop_error"])
        complete["heading_error"] = float(complete["heading_error"])
    else:
        match = STRAIGHT_COMPLETE_RE.search(text)
        if match:
            complete = match.groupdict()
            complete["ok"] = bool(int(complete["ok"]))
            for key in ("x", "y", "heading", "total_error"):
                complete[key] = float(complete[key])
        else:
            match = CAMPAIGN_COMPLETE_RE.search(text)
            if match:
                complete = {
                    "campaign": match.group("campaign").lower(),
                    "ok": bool(int(match.group("ok"))),
                }
                for item in KEY_VALUE_RE.finditer(match.group("details")):
                    key, value = item.group("key"), item.group("value")
                    try:
                        complete[key] = float(value)
                    except ValueError:
                        complete[key] = value
    return legs, complete


def unwrap_degrees(values: np.ndarray) -> np.ndarray:
    return np.rad2deg(np.unwrap(np.deg2rad(values)))


def aligned_gps(rows: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
    valid = [
        row for row in rows
        if all(finite(row.get(key)) for key in ("gps_x", "gps_y", "gps_heading"))
    ]
    if not valid:
        return np.array([]), np.array([])
    first = valid[0]
    angle = math.radians(float(first["h"]) - float(first["gps_heading"]))
    cosine, sine = math.cos(angle), math.sin(angle)
    origin_x, origin_y = float(first["gps_x"]), float(first["gps_y"])
    fused_x, fused_y = float(first["x"]), float(first["y"])
    output_x, output_y = [], []
    for row in valid:
        dx = float(row["gps_x"]) - origin_x
        dy = float(row["gps_y"]) - origin_y
        output_x.append(fused_x + cosine * dx - sine * dy)
        output_y.append(fused_y + sine * dx + cosine * dy)
    return np.asarray(output_x), np.asarray(output_y)


def build_summary(rows: list[dict[str, Any]], legs: list[dict[str, Any]],
                  complete: dict[str, Any] | None) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "controller_samples": len(rows),
        "legs": legs,
        "complete": complete,
    }
    if not rows:
        summary["status"] = "no_controller_samples"
        return summary
    first, last = rows[0], rows[-1]
    summary["observed_duration_s"] = float(last["host_s"]) - float(first["host_s"])
    summary["fused_endpoint"] = {
        "x_in": float(last["x"]), "y_in": float(last["y"]),
        "heading_deg": float(last["h"]),
    }
    if finite(last.get("dr_travel")):
        summary["dead_reckoning_distance_in"] = float(last["dr_travel"])
    gps_x, gps_y = aligned_gps(rows)
    if gps_x.size:
        summary["gps_aligned_endpoint"] = {
            "x_in": float(gps_x[-1]), "y_in": float(gps_y[-1])
        }
        summary["gps_vs_fused_endpoint_in"] = math.hypot(
            float(gps_x[-1]) - float(last["x"]),
            float(gps_y[-1]) - float(last["y"]),
        )
    encoder_rows = [
        row for row in rows
        if finite(row.get("left_deg")) and finite(row.get("right_deg"))
    ]
    if len(encoder_rows) >= 2:
        circumference = math.pi * WHEEL_DIAMETER_IN
        left_delta = float(encoder_rows[-1]["left_deg"]) - float(encoder_rows[0]["left_deg"])
        right_delta = float(encoder_rows[-1]["right_deg"]) - float(encoder_rows[0]["right_deg"])
        summary["encoder_net_center_in"] = (
            0.5 * (left_delta + right_delta) / 360.0 * circumference
        )
    heading_rows = [
        row for row in rows
        if finite(row.get("imu")) and finite(row.get("gps_heading"))
    ]
    if len(heading_rows) >= 2:
        imu = unwrap_degrees(np.asarray([float(row["imu"]) for row in heading_rows]))
        gps = unwrap_degrees(np.asarray([float(row["gps_heading"]) for row in heading_rows]))
        summary["imu_heading_change_deg"] = float(imu[-1] - imu[0])
        summary["gps_heading_change_deg"] = float(gps[-1] - gps[0])
        summary["gps_vs_imu_heading_change_deg"] = float(
            (gps[-1] - gps[0]) - (imu[-1] - imu[0])
        )
    summary["status"] = "complete" if complete and complete.get("ok") else "incomplete"
    return summary


def plot(rows: list[dict[str, Any]], output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    fig.suptitle("Fused drivetrain navigation qualification", fontsize=16, weight="bold")
    if not rows:
        for axis in axes.flat:
            axis.text(0.5, 0.5, "No controller samples", ha="center", va="center")
            axis.set_axis_off()
        fig.savefig(output, dpi=200, facecolor="white")
        return
    time_s = np.asarray([float(row["host_s"]) for row in rows])
    time_s -= time_s[0]
    fused_x = np.asarray([float(row["x"]) for row in rows])
    fused_y = np.asarray([float(row["y"]) for row in rows])
    gps_x, gps_y = aligned_gps(rows)

    axis = axes[0, 0]
    axis.plot(fused_x, fused_y, "o-", ms=2.5, lw=1.8, label="encoder + IMU fusion")
    if gps_x.size == len(rows):
        axis.plot(gps_x, gps_y, ".--", lw=1.2, alpha=0.8, label="GPS, start-frame aligned")
    axis.scatter(fused_x[0], fused_y[0], s=70, marker="s", label="start")
    axis.scatter(fused_x[-1], fused_y[-1], s=70, marker="x", label="last")
    axis.set(title="Position estimate", xlabel="X (in)", ylabel="Y (in)")
    axis.axis("equal")
    axis.grid(alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(handles, labels, fontsize=8)

    axis = axes[0, 1]
    fused_heading = unwrap_degrees(np.asarray([float(row["h"]) for row in rows]))
    axis.plot(time_s, fused_heading - fused_heading[0], label="fused heading")
    if all(finite(row.get("imu")) for row in rows):
        imu = unwrap_degrees(np.asarray([float(row["imu"]) for row in rows]))
        axis.plot(time_s, imu - imu[0], "--", label="P6 IMU")
    if all(finite(row.get("gps_heading")) for row in rows):
        gps = unwrap_degrees(np.asarray([float(row["gps_heading"]) for row in rows]))
        axis.plot(time_s, gps - gps[0], ":", label="P7 GPS heading")
    axis.set(title="Relative heading agreement", xlabel="Time (s)", ylabel="Change (deg)")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)

    axis = axes[1, 0]
    forward = np.asarray([float(row.get("fwd", 0.0)) for row in rows])
    turn = np.asarray([float(row.get("turn", 0.0)) for row in rows])
    axis.plot(time_s, forward, label="forward command")
    axis.plot(time_s, turn, label="turn command")
    if all(finite(row.get("dist")) for row in rows):
        axis.plot(time_s, [float(row["dist"]) for row in rows], label="distance error (in)")
    axis.axhline(28.0, color="0.5", ls=":", lw=1, label="drive breakaway floor")
    axis.set(title="Controller and braking", xlabel="Time (s)", ylabel="Power / inches")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)

    axis = axes[1, 1]
    if all(finite(row.get("dr_travel")) for row in rows):
        dr = np.asarray([float(row["dr_travel"]) for row in rows])
        axis.plot(time_s, dr - dr[0], label="fused dead-reckoning distance")
    if all(finite(row.get("left_deg")) and finite(row.get("right_deg")) for row in rows):
        circumference = math.pi * WHEEL_DIAMETER_IN
        left = np.asarray([float(row["left_deg"]) for row in rows])
        right = np.asarray([float(row["right_deg"]) for row in rows])
        center = 0.5 * ((left - left[0]) + (right - right[0])) / 360.0 * circumference
        axis.plot(time_s, center, "--", label="encoder net center")
    if gps_x.size == len(rows):
        gps_displacement = np.hypot(gps_x - gps_x[0], gps_y - gps_y[0])
        axis.plot(time_s, gps_displacement, ":", label="GPS displacement")
    axis.set(title="Translation estimates", xlabel="Time (s)", ylabel="Distance (in)")
    axis.grid(alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(handles, labels, fontsize=8)

    fig.savefig(output, dpi=200, facecolor="white")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    rows = load_fusion(args.run_dir / "fusion.jsonl")
    legs, complete = parse_route_events(args.run_dir / "raw.log")
    summary = build_summary(rows, legs, complete)
    summary_path = args.run_dir / "navigation_analysis.json"
    graph_path = args.run_dir / "navigation_analysis.png"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plot(rows, graph_path)
    print(json.dumps(summary, indent=2))
    print(f"wrote {summary_path}")
    print(f"wrote {graph_path}")
    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
