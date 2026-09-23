#!/usr/bin/env python3
"""Build a reproducible completion audit for the multi-sensor goal."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT23 = ROOT / "reports" / "sensor_campaign_2026-08-23"
REPORT25 = ROOT / "reports" / "sensor_campaign_2026-08-25"
OUT_JSON = REPORT25 / "goal_completion_audit.json"
OUT_MD = REPORT25 / "goal_completion_audit.md"


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def main() -> None:
    sources = {
        "qualification": REPORT23 / "sensor_qualification_summary.json",
        "imu": REPORT23 / "imu_stability_summary.json",
        "p1": REPORT23 / "p1_reliability_summary.json",
        "p5": REPORT23 / "p5_tracker_summary.json",
        "vision": REPORT23 / "ai_tag4_depth_summary.json",
        "vision_sensitivity": REPORT23 / "ai_pnp_sensitivity_summary.json",
        "reconnect": REPORT23 / "live_reconnect_sensor_summary.json",
        "gps_recovery": REPORT23 / "gps_recovery_gate_summary.json",
        "gps_stale": REPORT23 / "gps_stale_cache_summary.json",
        "outage": REPORT23 / "dead_reckoning_outage_summary.json",
        "fault_response": REPORT23 / "fault_response_summary.json",
        "slot4_runtime": REPORT25 / "resumed_stationary_01" / "summary.json",
        "slot1_runtime": REPORT25 / "slot1_runtime_isolation_01" / "summary.json",
        "post_powercycle_stationary": REPORT25 / "post_powercycle_stationary_60s_01" / "summary.json",
        "imu_calibration_experiment": REPORT25 / "imu_calibration_experiment" / "summary.json",
        "warm_extended": REPORT25 / "warm_imu_extended_30min_01" / "summary.json",
    }
    missing = [str(path.relative_to(ROOT)) for path in sources.values() if not path.exists()]
    if missing:
        raise SystemExit(f"missing evidence: {missing}")

    data = {name: load(path) for name, path in sources.items()}
    q = data["qualification"]
    imu = data["imu"]
    p1 = data["p1"]
    p5 = data["p5"]
    vision = data["vision"]
    sensitivity = data["vision_sensitivity"]
    reconnect = data["reconnect"]
    recovery = data["gps_recovery"]
    stale = data["gps_stale"]
    outage = data["outage"]
    fault = data["fault_response"]
    current = data["post_powercycle_stationary"]
    imu_calibration = data["imu_calibration_experiment"]
    extended = data["warm_extended"]

    requirements = [
        {
            "requirement": "Drive encoder translation inaccuracy",
            "status": "partial",
            "evidence_kind": "live, P7-referenced",
            "result": (
                f"9 trials; fitted scale {q['drive_encoder_qualification']['encoder_scale']:.6f}; "
                f"fit RMSE {q['drive_encoder_qualification']['fit_rmse_in']:.3f} in; "
                f"speed-dependent scale spread {outage['worst_relative_scale_variation_percent']:.3f}%"
            ),
            "missing": "independent tape/laser truth, high-speed slip, and loaded-wheel tests",
        },
        {
            "requirement": "P6 IMU inaccuracy",
            "status": "partial_with_live_calibration_failure",
            "evidence_kind": "live stationary, recalibration, and reversible motion",
            "result": (
                f"first ready calibration drifted {imu_calibration['first_calibration']['following_10min_start_deg']:.2f} to "
                f"{imu_calibration['first_calibration']['following_10min_end_deg']:.2f} deg at "
                f"{imu_calibration['first_calibration']['following_10min_slope_deg_per_min']:.3f} deg/min while all drive encoders were fixed; "
                f"warm recalibration held {imu_calibration['warm_recalibration']['imu_peak_to_peak_deg']:.3f}-deg span over "
                f"{imu_calibration['warm_recalibration']['duration_s']/60.0:.2f} min; "
                f"one reversible ~80-deg turn endpoint residual "
                f"{reconnect['reversible_rotation']['imu_endpoint_residual_deg']:.3f} deg"
            ),
            "missing": "cause of calibration variability, post-calibration production gate, external angular truth, dynamic acceleration error, and temperature sweep",
        },
        {
            "requirement": "P7 GPS inaccuracy and failure behavior",
            "status": "partial_with_live_failure",
            "evidence_kind": "live stationary/turn captures plus production-logic replay",
            "result": (
                f"quiet-run X/Y std {q['stationary_120s']['gps_x_std_in']:.4f}/"
                f"{q['stationary_120s']['gps_y_std_in']:.4f} in, but a reversible turn produced "
                f"{reconnect['reversible_rotation']['gps_max_observed_displacement_in']:.2f} in apparent motion "
                f"and at least {reconnect['reversible_rotation']['gps_min_excess_over_encoder_span_in']:.2f} in disagreement; "
                f"stopped raw Z gyro median {imu_calibration['warm_recalibration']['gps_raw_gyro_z']['median']:.2f}"
            ),
            "missing": "surveyed moving-position truth and repeat tests across field-code orientations",
        },
        {
            "requirement": "P1 forward Distance Sensor inaccuracy",
            "status": "measured_unreliable_for_localization",
            "evidence_kind": "live stationary and rotation captures",
            "result": (
                f"physical return in {100*p1['stationary_physical_return_fraction']:.1f}% of stationary frames; "
                f"roughly 40-in returns had {p1['physical_return_range_in']['peak_to_peak']:.2f}-in span at confidence "
                f"{p1['physical_return_confidence']['minimum']}-{p1['physical_return_confidence']['maximum']}/63; "
                f"0/{p1['rotation_sweep_samples']} rotation frames returned a target"
            ),
            "missing": "controlled <=8-in target truth, material/angle coverage, detection latency and braking distance",
        },
        {
            "requirement": "P8 AI Vision AprilTag range/angle inaccuracy",
            "status": "partial",
            "evidence_kind": "live corners, one later tape point, and sensitivity analysis",
            "result": (
                f"0.625-in detected-square model; Tag 4 horizontal median "
                f"{vision['pnp_horizontal_in']['median']:.2f} in and bearing std "
                f"{vision['pnp_bearing_deg']['std']:.3f} deg; one separate 18-in tape view re-solved to "
                f"18.60 in (+3.3%); modeled horizontal sensitivity p95 "
                f"{sensitivity['horizontal_range_in']['abs_delta_p95']:.2f} in; strict detections "
                f"{reconnect['p8']['strict_valid_polls']}/{reconnect['p8']['polls']} polls; extended stopped run added "
                f"{extended['ai_reject_counts'].get('shadow_valid', 0)} isolated geometrically-valid unmapped Tag 12 frame "
                f"plus {extended['ai_reject_counts'].get('repeat', 0)} exact repeat"
            ),
            "missing": "multi-range/yaw/pitch tape truth, calibrated intrinsics/distortion, motion blur and robot extrinsics",
        },
        {
            "requirement": "P5 lateral tracker usability",
            "status": "failed_disabled",
            "evidence_kind": "live rotation capture",
            "result": (
                f"observed <= {p5['observed_motion_range_in'][1]:.3f} in while geometry predicted up to "
                f"{p5['expected_motion_range_in'][1]:.2f} in; production fusion disabled"
            ),
            "missing": "mechanical repair followed by CW/CCW and translation calibration",
        },
        {
            "requirement": "GPS-outage fallback to encoders plus IMU",
            "status": "logic_verified_live_end_to_end_missing",
            "evidence_kind": "production-constant replay plus live GPS rejection",
            "result": (
                f"modeled 5 s at 10 in/s error envelope {outage['examples']['10in_s_5s']['combined_envelope_in']:.2f} in; "
                f"bounded correct-GPS return begins at {recovery['scenarios'][0]['first_nonzero_correction_s']:.1f} s "
                f"and falls below 0.5 in at {recovery['scenarios'][0]['time_to_below_0_5_in_s']:.1f} s; "
                f"frozen pre-move GPS causes {stale['production_max_error_before_fresh_in']:.1f} in production pull"
            ),
            "missing": "physical moving dropout/return route with independent endpoint truth",
        },
        {
            "requirement": "P8 fallback during GPS outage",
            "status": "not_enabled",
            "evidence_kind": "live observability and association simulation",
            "result": "tag diagnostics and hypotheses are retained, but absolute P8 field correction is intentionally disabled",
            "missing": "camera extrinsics, reliable duplicate-tag face association, and multi-view physical validation",
        },
        {
            "requirement": "Fault response timing",
            "status": "software_model_only",
            "evidence_kind": "900,000 scheduler-phase simulations",
            "result": (
                f"drive/IMU/P1 software bound {max(row['latency_max_bound_ms'] or 0 for row in fault['rows'][:4]):.0f} ms; "
                f"GPS/P8 rejection bound 100 ms; jam watchdog bound 1020 ms"
            ),
            "missing": "live unplug/fault injection, transport latency, coast and physical stopping distance",
        },
        {
            "requirement": "Current live end-to-end fused route",
            "status": "runtime_recovered_then_usb_disconnected",
            "evidence_kind": "post-power-cycle CPU1 telemetry and synchronized stationary capture",
            "result": (
                f"CPU1 recovered after a real Brain power cycle; slot 4 produced {current['frames']} frames over "
                f"{current['elapsed_s']:.1f} s with P1/P7 valid fractions "
                f"{current['p1_valid_fraction']:.3f}/{current['gps_valid_fraction']:.3f} and zero drive-encoder spans; "
                f"a later soak captured {extended['frames']} frames before the complete VEX USB device disappeared"
            ),
            "missing": "reconnect VEX USB, aim webcam at the complete robot, then run a synchronized bounded moving route with physical endpoint truth",
        },
    ]

    achieved = sum(item["status"] in {"measured", "complete"} for item in requirements)
    partial = sum("partial" in item["status"] or item["status"] in {
        "logic_verified_live_end_to_end_missing", "software_model_only",
        "measured_unreliable_for_localization"
    } for item in requirements)
    audit = {
        "generated_local": datetime.now().astimezone().isoformat(timespec="seconds"),
        "objective": "Characterize every available localization/safety sensor and verify multi-sensor fallback behavior.",
        "verdict": "not_complete",
        "counts": {
            "requirements": len(requirements),
            "complete": achieved,
            "partial_or_limited": partial,
            "not_enabled_failed_or_blocked": len(requirements) - achieved - partial,
        },
        "report_tree_bytes": tree_bytes(ROOT / "reports"),
        "requirements": requirements,
        "source_sha256": {
            str(path.relative_to(ROOT)): digest(path) for path in sources.values()
        },
        "next_live_sequence": [
            "Reconnect/power the V5 Brain and verify CPU1 telemetry without motion.",
            "Aim the Brio so the complete robot and free travel area are visible.",
            "Add and validate a fail-closed post-calibration P6 stability gate while stopped.",
            "Run short 2/5/10-in and 15/45/90-deg motions with measured ground truth.",
            "Run a bounded route with P7 accepted, rejected, and restored; compare fused endpoint against physical truth.",
            "Only enable P8 correction after extrinsics and multi-angle tape calibration pass.",
        ],
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# Sensor-goal completion audit",
        "",
        f"Generated: `{audit['generated_local']}`",
        "",
        "Verdict: **not complete**. Existing evidence strongly characterizes several failure modes, but it does not prove competition-speed fused localization or every sensor's absolute accuracy.",
        "",
        "| Requirement | Status | Authoritative result | Missing proof |",
        "|---|---|---|---|",
    ]
    for item in requirements:
        lines.append(
            f"| {item['requirement']} | `{item['status']}` | {item['result']} | {item['missing']} |"
        )
    lines += ["", "## Next live sequence", ""]
    lines += [f"{index}. {step}" for index, step in enumerate(audit["next_live_sequence"], 1)]
    lines += [
        "",
        "The JSON companion contains SHA-256 hashes for every source summary used, so this audit can be reproduced and checked against later captures.",
        "",
    ]
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(audit["counts"], indent=2))
    print(f"wrote {OUT_JSON.relative_to(ROOT)}")
    print(f"wrote {OUT_MD.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
