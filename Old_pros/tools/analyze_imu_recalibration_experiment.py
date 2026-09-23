#!/usr/bin/env python3
"""Compare the bad first P6 calibration with a stationary warm recalibration."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "sensor_campaign_2026-08-25"
CAPTURES = {
    "cold_first_minute": REPORT / "post_powercycle_stationary_60s_01",
    "cold_drift_5min": REPORT / "post_powercycle_stationary_5min_01",
    "cold_drift_next_10min": REPORT / "post_powercycle_stationary_10min_02",
    "warm_recalibration_12min": REPORT / "imu_raw_rate_recalibration_12min_01",
    "warm_extended_19min": REPORT / "warm_imu_extended_30min_01",
}
OUT = REPORT / "imu_calibration_experiment"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def telemetry(path: Path, enhanced_only: bool = False) -> tuple[np.ndarray, np.ndarray]:
    with (path / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if enhanced_only:
        rows = [row for row in rows if row.get("imu_gyro_z") not in (None, "", "nan", "NaN")]
    t = np.asarray([float(row["host_s"]) for row in rows])
    imu = np.asarray([float(row["imu"]) for row in rows])
    return t - t[0], imu


def first_calibration_threshold_onsets() -> dict[str, float | None]:
    rows: list[dict[str, str]] = []
    for name in ("cold_first_minute", "cold_drift_5min", "cold_drift_next_10min"):
        with (CAPTURES[name] / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
            rows.extend(csv.DictReader(stream))
    samples = sorted(
        ((float(row["robot_ms"]), abs(float(row["imu"]))) for row in rows),
        key=lambda sample: sample[0],
    )
    first_robot_ms = samples[0][0]
    return {
        f"{threshold:.2f}_deg": next(
            ((robot_ms - first_robot_ms) / 1000.0
             for robot_ms, magnitude in samples if magnitude >= threshold),
            None,
        )
        for threshold in (0.10, 0.25, 0.50, 1.00, 2.00, 5.00, 10.00)
    }


def enhanced_robot_telemetry(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with (path / "telemetry.csv").open(newline="", encoding="utf-8") as stream:
        rows = [
            row for row in csv.DictReader(stream)
            if row.get("imu_gyro_z") not in (None, "", "nan", "NaN")
        ]
    return (
        np.asarray([float(row["robot_ms"]) for row in rows]),
        np.asarray([float(row["imu"]) for row in rows]),
    )


def main() -> None:
    analyses = {
        name: load_json(path / "stationary_analysis.json")
        for name, path in CAPTURES.items()
    }
    cold5 = analyses["cold_drift_5min"]
    cold10 = analyses["cold_drift_next_10min"]
    warm = analyses["warm_recalibration_12min"]
    extended = analyses["warm_extended_19min"]
    warm_robot_ms, warm_robot_imu = enhanced_robot_telemetry(
        CAPTURES["warm_recalibration_12min"]
    )
    extended_robot_ms, extended_robot_imu = enhanced_robot_telemetry(
        CAPTURES["warm_extended_19min"]
    )
    combined_warm_imu = np.concatenate((warm_robot_imu, extended_robot_imu))
    result = {
        "verdict": "P6 calibration quality is not reliably represented by ready status alone",
        "first_calibration": {
            "first_minute_peak_to_peak_deg": analyses["cold_first_minute"]["imu_heading_deg"]["peak_to_peak"],
            "next_5min_peak_to_peak_deg": cold5["imu_heading_deg"]["peak_to_peak"],
            "next_5min_slope_deg_per_min": cold5["imu_linear_slope_deg_per_min"],
            "following_10min_start_deg": cold10["imu_heading_deg"]["min"],
            "following_10min_end_deg": cold10["imu_heading_deg"]["max"],
            "following_10min_slope_deg_per_min": cold10["imu_linear_slope_deg_per_min"],
            "encoder_spans_deg": cold10["drive_encoder_span_deg"],
            "gps_heading_span_deg": cold10["gps_heading_deg"]["peak_to_peak"],
            "gps_raw_gyro_z": cold10["gps_raw_gyro_z_dps"],
            "gps_endpoint_displacement_in": cold10["gps_endpoint_displacement_in"],
            "threshold_onset_s_from_first_telemetry": first_calibration_threshold_onsets(),
        },
        "warm_recalibration": {
            "duration_s": warm["elapsed_s"],
            "samples": warm["samples"],
            "imu_peak_to_peak_deg": warm["imu_heading_deg"]["peak_to_peak"],
            "imu_std_deg": warm["imu_heading_deg"]["std"],
            "imu_slope_deg_per_min": warm["imu_linear_slope_deg_per_min"],
            "raw_gyro_z_dps": warm["imu_gyro_dps"]["z"],
            "acceleration_g": warm["imu_acceleration_g"],
            "encoder_spans_deg": warm["drive_encoder_span_deg"],
            "gps_heading_span_deg": warm["gps_heading_deg"]["peak_to_peak"],
            "gps_raw_gyro_z": warm["gps_raw_gyro_z_dps"],
            "gps_endpoint_displacement_in": warm["gps_endpoint_displacement_in"],
        },
        "warm_extended_session": {
            "captured_duration_s": warm["elapsed_s"] + extended["elapsed_s"],
            "session_span_s": float(extended_robot_ms[-1] - warm_robot_ms[0]) / 1000.0,
            "samples": int(combined_warm_imu.size),
            "imu_min_deg": float(np.min(combined_warm_imu)),
            "imu_max_deg": float(np.max(combined_warm_imu)),
            "imu_peak_to_peak_deg": float(np.ptp(combined_warm_imu)),
            "imu_endpoint_change_deg": float(combined_warm_imu[-1] - combined_warm_imu[0]),
            "extended_capture": extended,
            "termination": "Brain USB disappeared after 19.28 min of the extended capture",
        },
        "cause_status": "unresolved",
        "cause_candidates": [
            "thermal/cold-start bias before the first calibration",
            "physical disturbance during the first calibration (webcam did not show robot)",
            "intermittent P6 calibration or firmware state",
        ],
        "production_implication": (
            "A finite heading and non-calibrating status are insufficient. A bad calibration can corrupt "
            "turns by roughly one degree per minute while encoders and P7 indicate no rotation."
        ),
        "required_next_gate": (
            "Perform calibration with the robot visibly stationary after warm-up, then require a stopped "
            "cross-sensor heading-change stability check before granting navigation motion authority."
        ),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    cold5_t, cold5_imu = telemetry(CAPTURES["cold_drift_5min"])
    cold10_t, cold10_imu = telemetry(CAPTURES["cold_drift_next_10min"])
    warm_t = (warm_robot_ms - warm_robot_ms[0]) / 1000.0
    extended_t = (extended_robot_ms - warm_robot_ms[0]) / 1000.0
    warm_imu = warm_robot_imu
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5), constrained_layout=True)
    fig.suptitle("P6 calibration quality: first boot calibration vs warm recalibration", fontsize=17)
    axes[0].plot(cold5_t / 60.0, cold5_imu, label="first drift capture")
    axes[0].plot((cold10_t + cold5_t[-1]) / 60.0, cold10_imu,
                 label="following 10-min capture")
    axes[0].set(title="First calibration: large false rotation",
                xlabel="sequential captured minutes", ylabel="P6 rotation (deg)")
    axes[0].grid(alpha=0.25)
    axes[0].legend()
    axes[1].plot(warm_t / 60.0, warm_imu, color="#16a34a")
    axes[1].plot(extended_t / 60.0, extended_robot_imu, color="#16a34a")
    axes[1].set(title="Warm recalibration: stable", xlabel="minutes", ylabel="P6 rotation (deg)")
    axes[1].grid(alpha=0.25)
    fig.savefig(OUT / "comparison.png", dpi=190)

    md = f"""# P6 calibration-quality experiment

## Result

The first post-power-cycle calibration was **bad despite ready status**. After an initially quiet minute, P6 drifted 3.22° over five minutes and then from 4.25° to 15.10° over the following ten-minute capture ({cold10['imu_linear_slope_deg_per_min']:.3f}°/min). It first crossed 0.10° at {first_calibration_threshold_onsets()['0.10_deg']:.1f} s and 0.50° at {first_calibration_threshold_onsets()['0.50_deg']:.1f} s after the first telemetry frame, so a short 15-second startup check would have passed this bad calibration. All four drive encoders stayed exactly fixed; P7 heading spanned only {cold10['gps_heading_deg']['peak_to_peak']:.2f}° and its XY endpoint moved {cold10['gps_endpoint_displacement_in']:.3f} in.

Recalibrating the stationary, warmed sensor changed the result completely. Across two captures, 31,166 enhanced frames cover 31.17 min within one 36.87-min Brain session and stay within 0.03° peak-to-peak with 0.00° endpoint change. The second capture ended when the complete VEX USB device disappeared after 19.28 min of observed telemetry; its P6 span was 0.02°. Exposed raw gyro X/Z were all 0.0000°/s throughout (one isolated Y sample reached 0.2441°/s). Encoders again stayed exactly fixed. In the initial 11.89-min warm capture, P7 heading spanned {warm['gps_heading_deg']['peak_to_peak']:.2f}° and P7 XY endpoint moved {warm['gps_endpoint_displacement_in']:.3f} in.

P6 acceleration medians were X={warm['imu_acceleration_g']['x']['median']:.4f} g, Y={warm['imu_acceleration_g']['y']['median']:.4f} g, and Z={warm['imu_acceleration_g']['z']['median']:.4f} g. Their vector norm is approximately 0.9994 g and the gravity vector is 2.60° from the sensor Z axis. This is a stable mount/gravity observation, not a surveyed robot-level calibration.

## Interpretation

The evidence proves calibration quality is variable and that `ready`/finite heading alone is not a sufficient navigation gate. It does **not** distinguish cold thermal bias from physical disturbance during the first calibration because the webcam did not show the robot. PROS documents an approximately two-second calibration and status completion in `include/pros/imu.hpp`; it does not provide a post-calibration accuracy guarantee.

Before competition motion authority, calibrate after warm-up with the robot visibly stationary and verify stopped P6 heading change against stopped, good-quality P7 heading change. P7 absolute heading should remain excluded from steering; this is a relative-change fault detector only.
"""
    (OUT / "report.md").write_text(md, encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
