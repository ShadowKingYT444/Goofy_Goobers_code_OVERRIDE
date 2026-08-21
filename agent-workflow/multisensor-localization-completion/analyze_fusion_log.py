"""Constant-memory summary for MoreVex serial fusion captures."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter
from pathlib import Path


FIELD = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


def fields(line: str) -> dict[str, str]:
    return dict(FIELD.findall(line))


def number(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, ValueError):
        return None


def angle_delta(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def analyze(path: Path) -> dict:
    counts: Counter[str] = Counter()
    lidar_rejects: Counter[str] = Counter()
    ai_rejects: Counter[str] = Counter()
    vision_reasons: Counter[str] = Counter()
    last_pose: tuple[float, float, float] | None = None
    last_pose_ms: float | None = None
    last_imu: float | None = None
    last_imu_ms: float | None = None
    current_brain_ms: float | None = None
    max_pose_jump = 0.0
    max_heading_jump = 0.0
    max_pose_speed = 0.0
    max_heading_rate = 0.0
    max_imu_sample_jump = 0.0
    max_left_spread = 0.0
    max_right_spread = 0.0
    max_lidar_axis_step = 0.0
    max_ai_position_step = 0.0
    max_ai_heading_step = 0.0
    route_done: dict[str, str] | None = None
    route_start: dict[str, str] | None = None

    with path.open(encoding="utf-8", errors="replace") as source:
        for line_number, line in enumerate(source, 1):
            counts["lines"] += 1
            row = fields(line)
            if line.startswith("FUSE_TEST"):
                counts["fuse"] += 1
                reject = row.get("reject")
                if reject:
                    lidar_rejects[reject] += 1
                ai_reject = row.get("ai_reject")
                if ai_reject:
                    ai_rejects[ai_reject] += 1
                x, y = number(row, "x"), number(row, "y")
                heading = number(row, "heading")
                if heading is None:
                    heading = number(row, "h")
                if x is not None and y is not None and heading is not None:
                    if (last_pose is not None and last_pose_ms is not None and
                            current_brain_ms is not None and current_brain_ms > last_pose_ms):
                        dt = (current_brain_ms - last_pose_ms) / 1000.0
                        position_jump = math.hypot(x - last_pose[0], y - last_pose[1])
                        heading_jump = abs(angle_delta(heading, last_pose[2]))
                        max_pose_jump = max(
                            max_pose_jump, position_jump,
                        )
                        max_heading_jump = max(
                            max_heading_jump, heading_jump,
                        )
                        max_pose_speed = max(max_pose_speed, position_jump / dt)
                        max_heading_rate = max(max_heading_rate, heading_jump / dt)
                    last_pose = (x, y, heading)
                    last_pose_ms = current_brain_ms
                for key, target in (
                    ("axis_step", "lidar"),
                    ("ai_pos_step", "ai_position"),
                    ("ai_heading_step", "ai_heading"),
                ):
                    value = number(row, key)
                    if value is None:
                        continue
                    value = abs(value)
                    if target == "lidar":
                        max_lidar_axis_step = max(max_lidar_axis_step, value)
                    elif target == "ai_position":
                        max_ai_position_step = max(max_ai_position_step, value)
                    else:
                        max_ai_heading_step = max(max_ai_heading_step, value)
                if row.get("route", "").endswith("_done"):
                    route_done = row
                elif row.get("route", "").endswith("_start"):
                    route_start = row
            elif line.startswith("VISION_SHADOW"):
                counts["vision"] += 1
                vision_reasons[row.get("reason", "missing")] += 1
                if row.get("valid") == "1":
                    counts["vision_valid"] += 1
            elif line.startswith("D4"):
                counts["d4"] += 1
                brain_ms = number(row, "t")
                if (brain_ms is not None and current_brain_ms is not None and
                        brain_ms <= current_brain_ms):
                    # Program/Brain time reset: start a new continuity epoch.
                    last_pose = None
                    last_pose_ms = None
                    last_imu = None
                    last_imu_ms = None
                if brain_ms is not None:
                    current_brain_ms = brain_ms
                values = {key: number(row, key) for key in ("m17", "m18", "m11", "m13")}
                if all(value is not None for value in values.values()):
                    max_left_spread = max(max_left_spread, abs(values["m17"] - values["m18"]))
                    max_right_spread = max(max_right_spread, abs(values["m11"] - values["m13"]))
                imu = number(row, "imu")
                if (imu is not None and last_imu is not None and brain_ms is not None and
                        last_imu_ms is not None and brain_ms > last_imu_ms):
                    max_imu_sample_jump = max(
                        max_imu_sample_jump, abs(angle_delta(imu, last_imu))
                    )
                if imu is not None:
                    last_imu = imu
                    last_imu_ms = brain_ms

    return_error = number(route_done or {}, "return_error")
    heading_error = number(route_done or {}, "heading_error")
    checks = {
        "route_completed": bool(route_done and route_done.get("ok") == "1"),
        "return_error_le_2in": return_error is not None and return_error <= 2.0,
        "heading_error_le_10deg": heading_error is not None and heading_error <= 10.0,
        "no_impossible_pose_speed_over_80in_s": max_pose_speed <= 80.0,
        "no_impossible_heading_rate_over_500deg_s": max_heading_rate <= 500.0,
        "lidar_step_bounded_2in": max_lidar_axis_step <= 2.0,
        "ai_position_step_bounded_0p75in": max_ai_position_step <= 0.75,
        "ai_heading_step_bounded_0p5deg": max_ai_heading_step <= 0.5,
        "same_side_encoder_spread_le_10deg": max(max_left_spread, max_right_spread) <= 10.0,
    }
    safety_check_names = (
        "route_completed",
        "no_impossible_pose_speed_over_80in_s",
        "no_impossible_heading_rate_over_500deg_s",
        "lidar_step_bounded_2in",
        "ai_position_step_bounded_0p75in",
        "ai_heading_step_bounded_0p5deg",
        "same_side_encoder_spread_le_10deg",
    )
    corrected_return_error = None
    if route_start and last_pose:
        start_x, start_y = number(route_start, "x"), number(route_start, "y")
        if start_x is not None and start_y is not None:
            corrected_return_error = math.hypot(last_pose[0] - start_x, last_pose[1] - start_y)
    return {
        "file": str(path),
        "counts": dict(counts),
        "route": route_done,
        "route_start": route_start,
        "final_pose": last_pose,
        "return_error_in": return_error,
        "post_correction_return_error_in": corrected_return_error,
        "heading_error_deg": heading_error,
        "maxima": {
            "pose_jump_in_per_fuse_report": round(max_pose_jump, 3),
            "heading_jump_deg_per_fuse_report": round(max_heading_jump, 3),
            "pose_speed_in_s": round(max_pose_speed, 3),
            "heading_rate_deg_s": round(max_heading_rate, 3),
            "imu_jump_deg_per_d4_sample": round(max_imu_sample_jump, 3),
            "left_encoder_spread_deg": max_left_spread,
            "right_encoder_spread_deg": max_right_spread,
            "lidar_axis_step_in": max_lidar_axis_step,
            "ai_position_step_in": max_ai_position_step,
            "ai_heading_step_deg": max_ai_heading_step,
        },
        "lidar_rejects": dict(lidar_rejects),
        "ai_rejects": dict(ai_rejects),
        "vision_reasons": dict(vision_reasons),
        "checks": checks,
        "fusion_safety_passed": all(checks[name] for name in safety_check_names),
        "passed": all(checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.log)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
