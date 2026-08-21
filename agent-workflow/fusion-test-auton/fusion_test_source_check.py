import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def require(text, needle, label):
    if needle not in text:
        raise AssertionError(f"missing {label}: {needle}")


def main():
    autons_cpp = read("src/autons.cpp")
    main_cpp = read("src/main.cpp")
    autons_hpp = read("include/autons.hpp")
    fusion_body = autons_cpp.split("void fusion_test_auton()", 1)[1]

    require(autons_hpp, "void fusion_test_auton();", "fusion test declaration")
    require(autons_cpp, "void fusion_test_auton()", "fusion test implementation")
    require(autons_cpp, "drive_forward_test_leg", "real forward test leg")
    require(autons_cpp, "turn_clockwise_test_leg", "real clockwise turn test leg")
    require(autons_cpp, "fused_drive_to_point", "full fused drive controller")
    require(autons_cpp, "fused_turn_to_heading", "full fused turn controller")
    require(autons_cpp, "project_field_point", "field coordinate projection helper")
    require(autons_cpp, "drive_to_field_target", "coordinate-first drive command")
    require(autons_cpp, "turn_to_field_heading", "coordinate-first turn command")
    require(autons_cpp, "set_physical_drive_power", "physical-sign drive helper")
    require(autons_cpp, "forward_inches_since", "encoder-bounded forward distance")
    require(autons_cpp, "type=pros_encoder_bounded_drive", "bounded forward drive log")
    require(autons_cpp, "commanded_inches", "planned segment distance cap log")
    require(autons_cpp, "enum class LidarFusionMode", "LiDAR correction mode")
    require(autons_cpp, "update_pose(PoseEstimate& pose, LidarFusionMode lidar_mode = LidarFusionMode::kBiasOnly)", "pose update LiDAR correction mode")
    require(autons_cpp, "LidarFusionMode::kBiasOnly", "active full-system movement uses soft LiDAR heading bias")
    require(autons_cpp, "type=fused_drive_to", "fused drive command log")
    require(autons_cpp, "type=fused_turn_to", "fused turn command log")
    require(autons_cpp, "controller=fused_drive", "fused drive loop log")
    require(autons_cpp, "controller=fused_turn", "fused turn loop log")
    require(autons_cpp, "kFusionTestTurnKp", "damped turn proportional gain")
    require(autons_cpp, "kFusionTestTurnKd", "damped turn derivative gain")
    require(autons_cpp, "kFusionTestTurnSlewPowerPerSec", "turn power slew limit")
    require(autons_cpp, "command_cw", "clockwise turn command state")
    require(autons_cpp, "rate=%.2f command_cw=%.2f", "turn damping log fields")
    require(autons_cpp, "kLidarImuCorrectionPeriodMs = 100", "LiDAR observation cadence")
    require(autons_cpp, "kMaxLidarBiasStepDeg", "bounded LiDAR bias correction")
    if "LidarFusionMode::kResetImu" in autons_cpp:
        raise AssertionError("LiDAR correction must not repeatedly reset the drive IMU")
    require(autons_cpp, "FUSE_TEST lidar_correction", "accepted LiDAR pose correction log")
    require(autons_cpp, "lidar_axis_correction_in", "wall-distance axis correction")
    require(autons_cpp, "FUSE_TEST route=start", "coordinate route log")
    require(autons_cpp, "chassis.drive_sensor_reset()", "PROS drive sensor reset")
    require(autons_cpp, "chassis.drive_imu_reset(0.0)", "PROS IMU reset")
    require(autons_cpp, "horizontal_odom.reset_position()", "horizontal odom reset")
    require(autons_cpp, "FUSE_TEST phase=%s", "phase log format")
    require(autons_cpp, 'sample_fusion_for(pose, "start"', "start phase log")
    require(autons_cpp, 'sample_fusion_for(pose, "after_first"', "after first phase log")
    require(autons_cpp, 'sample_fusion_for(pose, "after_turn"', "after turn phase log")
    require(autons_cpp, 'sample_fusion_for(pose, "final"', "final phase log")
    require(main_cpp, "fusion_test_auton();", "main/opcontrol fusion test call")
    require(main_cpp, "E_CONTROLLER_DIGITAL_X", "X hotkey")
    require(main_cpp, "E_CONTROLLER_DIGITAL_DOWN", "Down hotkey")
    require(main_cpp, "start_fusion_test_auton", "opcontrol fusion test task")

    require(main_cpp, "X+Down real fusion", "clear X+Down screen label")

    if "X+Down PID tune" in main_cpp or "X+Down fusion test" in main_cpp:
        raise AssertionError("X+Down still advertises the old test")
    if (
        re.search(r"(?<![A-Za-z0-9_])drive_to_point\s*\(", fusion_body)
        or re.search(r"(?<![A-Za-z0-9_])turn_to_heading\s*\(", fusion_body)
        or "point_ahead" in fusion_body
    ):
        raise AssertionError("fusion_test_auton still uses the old fused waypoint follower")
    if "set_physical_test_drive" in autons_cpp or "stop_physical_test_drive" in autons_cpp:
        raise AssertionError("old slow direct power loop helpers are still present")
    if "apply_lidar_projection" in autons_cpp:
        raise AssertionError("obsolete unbounded LiDAR projection helper remains")
    if "motor.move_relative" in autons_cpp or "chassis.pid_turn_relative_set" in autons_cpp:
        raise AssertionError("fusion test should not depend on motor relative targets or EZ turn signs")
    if "std::fabs(error_deg) * 2.0 + kFusionTestMinTurnPower" in autons_cpp:
        raise AssertionError("old bang-bang turn controller is still present")
    if "drive_to_field_target(pose" in fusion_body or "turn_to_field_heading(pose" in fusion_body:
        raise AssertionError("fusion_test_auton should use the full fused controllers now")
    if "fused_drive_to_point(pose" not in fusion_body or "fused_turn_to_heading(pose" not in fusion_body:
        raise AssertionError("fusion_test_auton is not using the full fused movement controllers")

    print("fusion test source checks passed")


if __name__ == "__main__":
    main()
