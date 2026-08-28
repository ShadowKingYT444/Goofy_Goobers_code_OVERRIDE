#!/usr/bin/env python3
"""Replay a fail-closed stopped P6/P7/encoder calibration-quality gate."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT23 = ROOT / "reports" / "sensor_campaign_2026-08-23"
REPORT25 = ROOT / "reports" / "sensor_campaign_2026-08-25"
OUT = REPORT25 / "imu_stationary_gate_replay"
IN_PER_M = 39.37007874015748

# A candidate window is useful only when three independent observations say
# that the chassis remained fixed. These are replay hypotheses, not production
# constants; live motion/reposition tests must validate them first.
MAX_WHEEL_CHANGE_DEG = 5.0
MAX_GPS_POSITION_CHANGE_IN = 0.50
MAX_GPS_HEADING_CHANGE_DEG = 0.25
QUALIFY_AFTER_S = 15.0
FAULT_IMU_CHANGE_DEG = 0.50


def finite(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def angle_delta(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def load_capture(path: Path) -> list[dict[str, str]]:
    with (path / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def replay(path: Path) -> dict[str, object]:
    rows = load_capture(path)
    anchor: dict[str, float] | None = None
    qualified_at: float | None = None
    fault_at: float | None = None
    maximum_qualified_divergence = 0.0
    usable_windows = 0
    reset_counts = {"invalid": 0, "encoders": 0, "gps_position": 0,
                    "gps_heading": 0}

    for row in rows:
        values = {
            key: finite(row, key)
            for key in ("host_s", "robot_ms", "imu", "gps_x", "gps_y", "gps_heading",
                        "gps_error", "m17", "m18", "m11", "m13")
        }
        if any(values[key] is None for key in values):
            anchor = None
            reset_counts["invalid"] += 1
            continue
        sample = {key: float(value) for key, value in values.items()
                  if value is not None}
        # Match the production P7 quality ceiling. A low RMS is not sufficient
        # for correction, but it is required before P7 can corroborate P6.
        if sample["gps_error"] * IN_PER_M > 0.75:
            anchor = None
            reset_counts["invalid"] += 1
            continue
        if anchor is None:
            anchor = sample
            continue
        # A Brain program restart creates a new calibration/estimator session.
        # Never compare its fresh zero against the prior program's heading.
        if sample["robot_ms"] < anchor["robot_ms"]:
            anchor = sample
            continue

        wheel_change = max(abs(sample[key] - anchor[key])
                           for key in ("m17", "m18", "m11", "m13"))
        gps_position_change = math.hypot(
            sample["gps_x"] - anchor["gps_x"],
            sample["gps_y"] - anchor["gps_y"],
        ) * IN_PER_M
        gps_heading_change = abs(angle_delta(
            sample["gps_heading"], anchor["gps_heading"]))
        if wheel_change > MAX_WHEEL_CHANGE_DEG:
            anchor = sample
            reset_counts["encoders"] += 1
            continue
        if gps_position_change > MAX_GPS_POSITION_CHANGE_IN:
            anchor = sample
            reset_counts["gps_position"] += 1
            continue
        if gps_heading_change > MAX_GPS_HEADING_CHANGE_DEG:
            anchor = sample
            reset_counts["gps_heading"] += 1
            continue

        duration_s = sample["host_s"] - anchor["host_s"]
        imu_change = abs(angle_delta(sample["imu"], anchor["imu"]))
        disagreement = abs(imu_change - gps_heading_change)
        if duration_s >= QUALIFY_AFTER_S:
            usable_windows += 1
            maximum_qualified_divergence = max(
                maximum_qualified_divergence, disagreement)
            if qualified_at is None:
                qualified_at = sample["host_s"] - rows_host_start(rows)
            if disagreement >= FAULT_IMU_CHANGE_DEG and fault_at is None:
                fault_at = sample["host_s"] - rows_host_start(rows)
                break

    start = rows_host_start(rows)
    end = next((finite(row, "host_s") for row in reversed(rows)
                if finite(row, "host_s") is not None), start)
    return {
        "capture": str(path.relative_to(ROOT)),
        "samples": len(rows),
        "duration_s": float(end - start),
        "qualified_at_s": qualified_at,
        "fault_at_s": fault_at,
        "maximum_qualified_divergence_deg": maximum_qualified_divergence,
        "usable_samples_after_qualification": usable_windows,
        "reset_counts": reset_counts,
    }


def rows_host_start(rows: list[dict[str, str]]) -> float:
    for row in rows:
        value = finite(row, "host_s")
        if value is not None:
            return value
    return 0.0


def main() -> None:
    # Include motion captures as adversarial negatives: encoder/P7 evidence
    # must prevent ordinary turns, drives, and GPS visual walking from being
    # misclassified as a stopped IMU fault.
    captures = [path.parent for path in sorted(REPORT23.glob("*/telemetry.csv"))]
    captures += [
        REPORT25 / "post_powercycle_stationary_60s_01",
        REPORT25 / "post_powercycle_stationary_5min_01",
        REPORT25 / "post_powercycle_stationary_10min_02",
        REPORT25 / "imu_raw_rate_recalibration_12min_01",
    ]
    results = [replay(path) for path in captures]
    expected_bad = {
        "post_powercycle_stationary_5min_01",
        "post_powercycle_stationary_10min_02",
    }
    bad_detected = sum(
        result["fault_at_s"] is not None
        for result in results
        if Path(str(result["capture"])).name in expected_bad
    )
    other_faults = [
        result["capture"] for result in results
        if Path(str(result["capture"])).name not in expected_bad
        and result["fault_at_s"] is not None
    ]
    summary = {
        "gate_hypothesis": {
            "maximum_wheel_change_deg": MAX_WHEEL_CHANGE_DEG,
            "maximum_gps_position_change_in": MAX_GPS_POSITION_CHANGE_IN,
            "maximum_gps_heading_change_deg": MAX_GPS_HEADING_CHANGE_DEG,
            "qualify_after_s": QUALIFY_AFTER_S,
            "fault_imu_gps_change_disagreement_deg": FAULT_IMU_CHANGE_DEG,
        },
        "captures": len(results),
        "expected_bad_detected": bad_detected,
        "expected_bad_total": len(expected_bad),
        "other_faults": other_faults,
        "results": results,
        "verdict": (
            "offline_candidate_only" if bad_detected == len(expected_bad)
            and not other_faults else "thresholds_need_revision"
        ),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
