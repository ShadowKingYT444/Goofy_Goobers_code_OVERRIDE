from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
AUTONS = ROOT / "src" / "autons.cpp"
MAIN = ROOT / "src" / "main.cpp"


def require(text, needle, label):
    assert needle in text, f"missing {label}: {needle}"


def reject(text, needle, label):
    assert needle not in text, f"old behavior still present for {label}: {needle}"


def main():
    autons = AUTONS.read_text(encoding="utf-8")
    main_cpp = MAIN.read_text(encoding="utf-8")

    require(autons, "chassis.drive_imu_get()", "IMU heading read")
    require(autons, "chassis.drive_imu_reset(0.0)", "known-start IMU reset")
    require(autons, "localization::kEnteredStartPose", "editable known-start pose")
    require(autons, "imu_zero_field_heading_deg", "raw-IMU-zero field reference")
    require(autons, "imu_bias_deg", "IMU bias state")
    require(autons, "kLidarImuCorrectionPeriodMs = 100", "bounded LiDAR observation cadence")
    require(autons, "last_lidar_bias_ms", "LiDAR correction cadence state")
    require(autons, "kMinLidarCandidateScoreMargin", "four-candidate winner-margin gate")
    require(autons, "kMaxLidarAngularRateDegS", "angular-rate LiDAR gate")
    require(autons, "get_confidence()", "distance confidence gate")
    require(autons, "apply_lidar_fusion", "LiDAR fusion function")
    require(autons, "wall_heading_deg", "wall theta to heading conversion")
    require(autons, "lidar_axis_correction_in", "observable wall-axis correction")
    require(autons, "kLidarLeftOffsetIn", "LiDAR mount translation")
    require(autons, "LidarFusionMode::kBiasOnly", "heading-only LiDAR correction during movement")
    require(autons, "kRequiredConsistentLidarFits", "multi-sample LiDAR consistency gate")
    reject(autons, "LidarFusionMode::kResetImu", "repeated hard IMU resets")
    require(main_cpp, "chassis.drive_imu_calibrate(false)", "IMU calibration in initialize")
    reject(autons, "apply_lidar_projection", "old axis-only LiDAR projection")
    reject(autons, "kMaxLidarAxisCorrectionIn", "removed wall-axis position correction")

    print("autons fusion source checks passed")


if __name__ == "__main__":
    main()
