from __future__ import annotations

import collections
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise AssertionError(f"missing {label}: {needle}")


def normalize_deg(value: float) -> float:
    return value % 360.0


def main() -> None:
    config = read("include/localization_config.hpp")
    autons = read("src/autons.cpp")
    main_cpp = read("src/main.cpp")
    dashboard = read("tools/lidar_bar_server.py")

    require(config, "inline constexpr FieldPose kEnteredStartPose", "editable start pose")
    require(config, "kTileSizeIn = 24.0", "24 inch tile scale")
    require(config, "kNominalFieldHalfSpanIn = 72.0", "center-origin nominal field span")
    require(config, "kPhysicalWallHalfSpanIn = 70.2", "physical LiDAR wall span")
    require(config, "kLidarSide = RobotSide::kLeft", "left-side LiDAR metadata")
    require(config, "kAiVisionSide = RobotSide::kRight", "opposite-side camera metadata")

    landmark_pattern = re.compile(
        r'\{"(?P<name>[^"]+)",\s*(?P<id>\d+),\s*GoalColor::(?P<color>\w+),\s*'
        r'(?P<x>-?\d+(?:\.\d+)?),\s*(?P<y>-?\d+(?:\.\d+)?)\}'
    )
    landmarks = {
        match.group("name"): (
            int(match.group("id")),
            match.group("color"),
            float(match.group("x")),
            float(match.group("y")),
        )
        for match in landmark_pattern.finditer(config)
    }
    expected = {
        "center": (0, "kNeutral", 0.0, 0.0),
        "top_red_neutral": (2, "kNeutral", 48.0, 24.0),
        "top_blue_alliance": (1, "kBlue", 48.0, -24.0),
        "upper_red_neutral": (3, "kNeutral", 24.0, 48.0),
        "upper_blue_alliance": (4, "kBlue", 24.0, -48.0),
        "lower_red_alliance": (4, "kRed", -24.0, 48.0),
        "lower_blue_neutral": (3, "kNeutral", -24.0, -48.0),
        "bottom_red_alliance": (1, "kRed", -48.0, 24.0),
        "bottom_blue_neutral": (2, "kNeutral", -48.0, -24.0),
    }
    if landmarks != expected:
        raise AssertionError(f"goal/tag map mismatch\nexpected={expected}\nactual={landmarks}")

    id_counts = collections.Counter(item[0] for item in landmarks.values())
    if id_counts != collections.Counter({0: 1, 1: 2, 2: 2, 3: 2, 4: 2}):
        raise AssertionError(f"unexpected duplicate-ID counts: {id_counts}")
    for _, _, x_in, y_in in landmarks.values():
        if not math.isclose(x_in / 24.0, round(x_in / 24.0), abs_tol=1e-9):
            raise AssertionError(f"X coordinate is not on the 24 inch tile grid: {x_in}")
        if not math.isclose(y_in / 24.0, round(y_in / 24.0), abs_tol=1e-9):
            raise AssertionError(f"Y coordinate is not on the 24 inch tile grid: {y_in}")

    require(autons, "init_pose(PoseEstimate& pose, const localization::FieldPose& start_pose)", "start-pose initializer")
    require(autons, "pose.x = start_pose.x_in", "entered start X")
    require(autons, "pose.y = start_pose.y_in", "entered start Y")
    require(autons, "pose.imu_zero_field_heading_deg = normalize_deg(start_pose.heading_deg)", "entered field heading")
    require(autons, "return normalize_deg(pose.imu_zero_field_heading_deg - imu_rotation_deg)", "clockwise IMU to field conversion")
    require(autons, "chassis.drive_imu_reset(0.0)", "localization-start raw IMU reset")
    require(main_cpp, "chassis.drive_imu_reset(0.0)", "post-calibration raw IMU reset")
    require(autons, 'normalize_deg(theta_deg)', "red wall heading candidate")
    require(autons, 'normalize_deg(90.0 + theta_deg)', "audience wall heading candidate")
    require(autons, 'normalize_deg(180.0 + theta_deg)', "blue wall heading candidate")
    require(autons, 'normalize_deg(270.0 + theta_deg)', "zero wall heading candidate")
    require(autons, 'kLidarLeftOffsetIn = 5.29', "measured LiDAR left offset")
    if "kStartHeadingDeg" in autons or "kStartXIn" in autons or "kStartYIn" in autons:
        raise AssertionError("legacy hard-coded start-pose constants remain")

    if not math.isclose(normalize_deg(135.0 - 20.0), 115.0):
        raise AssertionError("IMU field-heading contract simulation failed")
    if not math.isclose(normalize_deg(0.0 - 20.0), 340.0):
        raise AssertionError("clockwise turn from field heading zero has wrong sign")

    require(dashboard, "const fieldHalfSpanIn = fieldSizeIn / 2", "dashboard center-origin span")
    require(dashboard, "const goalTagLandmarks = [", "dashboard landmark map")
    require(dashboard, "drawGoalTagLandmarks(toScreen)", "dashboard landmark rendering")
    require(dashboard, "fieldHalfSpanIn - Math.max(-fieldHalfSpanIn", "dashboard signed coordinate transform")
    require(dashboard, 'fieldCtx.fillText("+X / 0 deg"', "dashboard positive-X label")
    require(dashboard, 'fieldCtx.fillText("+Y red"', "dashboard positive-Y label")
    require(dashboard, 'fieldCtx.fillText("-Y blue"', "dashboard negative-Y label")
    require(dashboard, "const startPose = { x: -48.1, y: 0.5, headingRad: 94.2 * Math.PI / 180 }", "known browser start pose")
    if "Origin set at bottom-left" in dashboard:
        raise AssertionError("legacy bottom-left browser coordinate frame remains")

    dashboard_landmark_pattern = re.compile(
        r'\{\s*name:\s*"(?P<name>[^"]+)",\s*id:\s*(?P<id>\d+),\s*'
        r'color:\s*"(?P<color>\w+)",\s*x:\s*(?P<x>-?\d+),\s*y:\s*(?P<y>-?\d+)\s*\}'
    )
    dashboard_landmarks = {
        (int(match.group("id")), float(match.group("x")), float(match.group("y")))
        for match in dashboard_landmark_pattern.finditer(dashboard)
    }
    firmware_landmarks = {(item[0], item[2], item[3]) for item in landmarks.values()}
    if dashboard_landmarks != firmware_landmarks:
        raise AssertionError(
            f"dashboard and firmware landmark maps differ: {dashboard_landmarks ^ firmware_landmarks}"
        )

    def normalized_screen(x_in: float, y_in: float) -> tuple[float, float]:
        return ((72.0 - y_in) / 144.0, (72.0 - x_in) / 144.0)

    if normalized_screen(0.0, 0.0) != (0.5, 0.5):
        raise AssertionError("field center does not render at canvas center")
    blue_id4_screen = normalized_screen(24.0, -48.0)
    if not (blue_id4_screen[0] > 0.5 and blue_id4_screen[1] < 0.5):
        raise AssertionError("blue-side ID 4 should render in the upper-right field quadrant")
    red_id1_screen = normalized_screen(-48.0, 24.0)
    if not (red_id1_screen[0] < 0.5 and red_id1_screen[1] > 0.5):
        raise AssertionError("red-side ID 1 should render in the lower-left field quadrant")

    print("known-start and goal-tag map checks passed")


if __name__ == "__main__":
    main()
