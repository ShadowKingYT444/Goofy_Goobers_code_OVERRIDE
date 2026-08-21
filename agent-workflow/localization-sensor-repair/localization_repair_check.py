import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
AUTONS = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
UI = (ROOT / "tools" / "lidar_bar_server.py").read_text(encoding="utf-8")


def signed_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def bias_step(raw_heading, bias, wall_heading, deadband=0.35):
    error = signed_diff(wall_heading, raw_heading + bias)
    if abs(error) <= deadband:
        return bias
    gain, maximum = (0.15, 0.75) if abs(error) <= 10.0 else (0.35, 3.0)
    return max(-90.0, min(90.0, bias + max(-maximum, min(maximum, error * gain))))


def side_delta(previous, current, dt_s, diameter=2.0):
    if current == 2**31 - 1:
        return 0.0, previous, "invalid"
    inches = (((float(current) - float(previous)) / 100.0) / 360.0) * math.pi * diameter
    if abs(inches) > 100.0 * max(0.001, dt_s) + 0.5:
        return 0.0, current, "jump"
    return inches, current, "none"


def motor_side(values):
    finite = [value for value in values if math.isfinite(value)]
    spread = max(finite) - min(finite) if len(finite) > 1 else 0.0
    return {
        "count": len(finite),
        "average": sum(finite) / len(finite) if finite else math.nan,
        "trustworthy": bool(finite) and spread <= 45.0,
    }


def consistent_fit_streak(candidates, tolerance=4.0):
    pending = None
    streak = 0
    accepted = []
    for candidate in candidates:
        if pending is None or abs(signed_diff(candidate, pending)) > tolerance:
            pending, streak = candidate, 1
        else:
            pending, streak = candidate, streak + 1
        accepted.append(streak >= 3)
    return accepted


def lidar_candidates(theta_deg, distance_in, pose, heading_est_deg, forward=0.0, left=5.29):
    wall = 70.2
    headings = [(theta_deg + 90.0 * k) % 360.0 for k in range(4)]

    def offset_x(heading):
        radians = math.radians(heading)
        return forward * math.cos(radians) - left * math.sin(radians)

    def offset_y(heading):
        radians = math.radians(heading)
        return forward * math.sin(radians) + left * math.cos(radians)

    raw = [
        ("red", headings[0], "y", wall - distance_in - offset_y(headings[0])),
        ("audience", headings[1], "x", -wall + distance_in - offset_x(headings[1])),
        ("blue", headings[2], "y", -wall + distance_in - offset_y(headings[2])),
        ("zero", headings[3], "x", wall - distance_in - offset_x(headings[3])),
    ]
    candidates = []
    for wall_name, heading, axis, observed in raw:
        heading_error = abs(signed_diff(heading, heading_est_deg))
        axis_error = abs(observed - pose[axis])
        candidates.append(
            {
                "wall": wall_name,
                "heading": heading,
                "axis": axis,
                "observed": observed,
                "score": axis_error + 0.35 * heading_error,
            }
        )
    return sorted(candidates, key=lambda candidate: candidate["score"])


def main():
    assert "LidarFusionMode::kResetImu" not in AUTONS
    correction = AUTONS[AUTONS.index("void apply_lidar_fusion"):AUTONS.index("void update_pose")]
    assert "drive_imu_reset" not in correction
    for token in (
        "kRequiredConsistentLidarFits = 3",
        "kLidarBiasGain = 0.35",
        "kMaxLidarBiasStepDeg = 3.0",
        "kMaxLidarFineBiasStepDeg = 0.75",
        "kLidarCandidateFilterGain = 0.35",
        "kLidarDistanceCalibrationMm = {7.0, 0.0, -5.0, -2.0}",
        "kLidarLeftOffsetIn = 5.29",
        "kMinLidarCandidateScoreMargin",
        "pending_lidar_axis_value_in",
        "lidar_axis_correction_in",
        "consistent_lidar_fits",
        "lidar_correction wall=",
        "read_side_odom_position",
        "kMaxSideOdomSpeedInS",
        "read_motor_side",
        "left_motor_count",
    ):
        assert token in AUTONS, token
    assert "chassis.drive_sensor_reset();" in MAIN
    assert "RUN_STARTUP_LIDAR_CALIBRATION = false" in MAIN
    assert "RUN_STARTUP_FORWARD_CALIBRATION = false" in MAIN
    assert "CONTROLLER_DRIVE_DEADBAND = 5" in MAIN
    assert "localization_telemetry_reset();" in MAIN
    assert "telemetry_pose_initialized = false" in AUTONS
    assert "pros::millis() < telemetry_pose.last_update_ms" in AUTONS
    assert "drive_positions_are_zeroed()" in MAIN
    assert "if (drive_positions_are_zeroed())" in MAIN
    assert "hardware_is_zeroed && start_pose_error_in > 2.0" in AUTONS
    assert "Drive sensors ${driveHealthLine}" in UI
    assert "const leftEncoderSign = 1;" in UI and "const rightEncoderSign = -1;" in UI
    assert "std::strcmp(best.wall, pose.pending_lidar_wall)" in AUTONS

    for theta in (-30.0, -15.0, 0.0, 15.0, 30.0, 44.9, -44.9):
        candidates = lidar_candidates(theta, 16.91, {"x": -48.0, "y": 0.0}, 90.0)
        headings = sorted(candidate["heading"] for candidate in candidates)
        circular_gaps = [
            (headings[(index + 1) % 4] - headings[index]) % 360.0
            for index in range(4)
        ]
        assert all(math.isclose(gap, 90.0, abs_tol=1e-9) for gap in circular_gaps)

    anchor = lidar_candidates(0.0, 16.91, {"x": -48.0, "y": 0.0}, 90.0)
    assert anchor[0]["wall"] == "audience", anchor
    assert anchor[0]["axis"] == "x"
    assert abs(anchor[0]["observed"] + 48.0) < 0.1, anchor[0]
    assert anchor[1]["score"] - anchor[0]["score"] > 8.0

    badly_drifted_imu = lidar_candidates(
        0.0, 16.91, {"x": -48.0, "y": 0.0}, 179.0
    )
    assert badly_drifted_imu[0]["wall"] == "audience", badly_drifted_imu

    zero_offset = lidar_candidates(
        0.0, 16.91, {"x": -48.0, "y": 0.0}, 90.0, left=0.0
    )
    assert abs(zero_offset[0]["observed"] + 48.0) > 5.0

    bias = 0.0
    errors = []
    for _ in range(45):
        bias = bias_step(140.0, bias, 90.0)
        errors.append(abs(signed_diff(90.0, 140.0 + bias)))
    assert all(b <= a + 1e-9 for a, b in zip(errors, errors[1:])), errors
    assert errors[-1] < 0.5, errors[-1]

    assert consistent_fit_streak((84.0, 96.0, 84.0, 96.0)) == [False] * 4
    assert consistent_fit_streak((89.0, 90.0, 91.0)) == [False, False, True]

    missing_motor = motor_side((math.inf, 582.0))
    assert missing_motor == {"count": 1, "average": 582.0, "trustworthy": True}
    disagreeing_motors = motor_side((0.0, 100.0))
    assert disagreeing_motors["count"] == 2 and not disagreeing_motors["trustworthy"]

    delta, baseline, reject = side_delta(12345, 2**31 - 1, 0.02)
    assert (delta, baseline, reject) == (0.0, 12345, "invalid")
    delta, baseline, reject = side_delta(12345, 2_000_000, 0.02)
    assert delta == 0.0 and baseline == 2_000_000 and reject == "jump"
    delta, _, reject = side_delta(0, 36000, 1.0)
    assert math.isclose(delta, 2 * math.pi, rel_tol=1e-9) and reject == "none"

    assert math.isclose(signed_diff(-179.0, 179.0), 2.0)
    print("localization repair adversarial checks passed")


if __name__ == "__main__":
    main()
