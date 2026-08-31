import importlib.util
import json
import math
import re
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path

from cobs import cobs


ROOT = Path(__file__).resolve().parents[1]
SERVER_PATH = ROOT / "tools" / "lidar_bar_server.py"
SPEC = importlib.util.spec_from_file_location("lidar_bar_server", SERVER_PATH)
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)
CAPTURE_PATH = ROOT / "tools" / "capture_sensor_log.py"
NAV_ANALYZER_PATH = ROOT / "tools" / "analyze_navigation_qualification.py"
DROPOUT_ANALYZER_PATH = ROOT / "tools" / "analyze_dropout_braking.py"
CAPTURE_SPEC = importlib.util.spec_from_file_location("capture_sensor_log", CAPTURE_PATH)
CAPTURE = importlib.util.module_from_spec(CAPTURE_SPEC)
CAPTURE_SPEC.loader.exec_module(CAPTURE)


class VisionTelemetryTests(unittest.TestCase):
    def test_dashboard_labels_match_current_single_forward_distance_robot(self):
        source = SERVER_PATH.read_text(encoding="utf-8")
        self.assertIn("VEX Multi-Sensor Localization", source)
        self.assertIn("Forward Distance / Port ${port}", source)
        self.assertIn("Legacy four-sensor wall fit is disabled", source)
        self.assertNotIn("LiDAR Ports 6-9", source)
        self.assertNotIn("4x LiDAR Current Line", source)

    def test_browser_fallback_matches_coupled_encoder_failure_contract(self):
        source = SERVER_PATH.read_text(encoding="utf-8")
        self.assertIn("function coupledSideReading", source)
        self.assertIn("values.length !== motorPorts.length", source)
        self.assertIn("Math.max(...values) - Math.min(...values) > 15", source)
        self.assertIn("pose.requiresReset = true", source)
        self.assertIn("if (pose.requiresReset)", source)
        self.assertIn("const inPlaceCounterRotation", source)
        self.assertIn("inPlaceCounterRotation\n          ? 0", source)
        self.assertIn("const d4Fresh = Boolean(", source)
        self.assertIn("nowSec - d4.pc_time <= 1.5", source)
        self.assertIn("ready: pose.ready && d4Fresh", source)
        self.assertIn("D4 TELEMETRY STALE/DISCONNECTED", source)

    def test_ui_refuses_fresh_but_invalid_onboard_pose(self):
        server = SERVER_PATH.read_text(encoding="utf-8")
        firmware = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("estimator_valid=%d", firmware)
        self.assertIn("pose_valid=%d", firmware)
        self.assertIn("navigation_api_initialized && pose.ready", firmware)
        self.assertIn('"estimator_valid",', server)
        self.assertIn('"pose_valid",', server)
        self.assertIn("onboard.pose_valid === 1", server)
        self.assertIn("onboard.estimator_valid === 0", server)
        self.assertIn("ONBOARD ESTIMATOR INVALID", server)
        self.assertIn("ONBOARD POSE NOT INITIALIZED", server)

        parsed = SERVER.parse_fuse_test(
            "FUSE_TEST phase=telemetry x=1.25 y=-2.5 heading=90.0 "
            "estimator_valid=1 pose_valid=0 imu=90.0"
        )
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["estimator_valid"], 1.0)
        self.assertEqual(parsed["pose_valid"], 0.0)

    def test_public_turns_fail_closed_near_walls_and_goals(self):
        firmware = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include" / "navigation.hpp").read_text(encoding="utf-8")
        self.assertIn("public_turn_center_is_safe", firmware)
        self.assertIn('reject_reason = "turn_wall_clearance"', firmware)
        self.assertIn('reject_reason = "turn_goal_clearance"', firmware)
        self.assertGreaterEqual(firmware.count("initial_turn_error_deg"), 2)
        self.assertIn("return Result::kUnsafePath", firmware)
        self.assertIn("protect the sides or rear", header)
        turn_api = firmware.index("Result turn_to(")
        turn_gate = firmware.index("public_turn_center_is_safe(", turn_api)
        turn_command = firmware.index("chassis.drive_mode_set", turn_api)
        self.assertLess(turn_gate, turn_command)
        go_to = firmware.index("Result go_straight_to(")
        go_to_turn_gate = firmware.index("public_turn_center_is_safe(", go_to)
        go_to_command = firmware.index("chassis.drive_mode_set", go_to)
        self.assertLess(go_to_turn_gate, go_to_command)
        self.assertGreaterEqual(
            firmware.count("> kFusedTurnToleranceDeg &&"), 2
        )

    def test_capture_pipeline_preserves_float_gps_gyro(self):
        frame = CAPTURE.parse_frame(
            "D4 s=10 t=200 p1=1016,6,1 "
            "m17=1.0 m18=2.0 m11=3.0 m13=4.0 h5=50 "
            "imu=-1.2 rawimu=-1.2 imust=18 "
            "gps7=-1.3804,-0.0137,299.75,0.0098,1 errno=0 gpsgyro=0.14",
            1.5,
        )
        self.assertIsNotNone(frame)
        self.assertAlmostEqual(frame["gps_gyro_z"], 0.14)
        self.assertIn("gps_gyro_z", CAPTURE.FIELDNAMES)
        fault_frame = CAPTURE.parse_frame(
            "D4 s=11 t=260 p1=2147483647,2147483647,1 "
            "m17=inf m18=2.0 m11=3.0 m13=4.0 h5=2147483647 "
            "imu=nan rawimu=nan imust=255 "
            "gps7=nan,nan,nan,nan,0 errno=19 gpsgyro=nan",
            1.6,
        )
        self.assertIsNotNone(fault_frame)
        self.assertTrue(math.isinf(fault_frame["m17"]))
        self.assertTrue(math.isnan(fault_frame["imu"]))

    def test_capture_pipeline_preserves_raw_imu_rate_and_acceleration(self):
        line = (
            "D4 s=12 t=320 p1=738,46,1 "
            "m17=0.0 m18=0.0 m11=0.0 m13=0.0 h5=0 "
            "imu=3.22 rawimu=3.22 imust=18 "
            "imugyro=0.0100,-0.0200,0.0300 imuacc=0.1000,0.2000,0.9800 "
            "gps7=-1.3354,0.2467,308.13,0.0098,1 errno=0 gpsgyro=-0.52"
        )
        frame = CAPTURE.parse_frame(line, 2.0)
        self.assertIsNotNone(frame)
        self.assertAlmostEqual(frame["imu_gyro_z"], 0.03)
        self.assertAlmostEqual(frame["imu_acc_z"], 0.98)
        self.assertAlmostEqual(frame["gps_heading"], 308.13)
        dashboard = SERVER.parse_d4(line)
        self.assertAlmostEqual(dashboard["imu"]["gyro_z_dps"], 0.03)
        self.assertAlmostEqual(dashboard["imu"]["accel_z_g"], 0.98)
        self.assertAlmostEqual(dashboard["gps"]["heading_deg"], 308.13)

    def test_capture_missing_serial_device_still_enforces_idle_timeout(self):
        source = (ROOT / "tools" / "capture_sensor_log.py").read_text(
            encoding="utf-8"
        )
        missing_device_branch = source[
            source.index("if device is None:"):
            source.index("try:", source.index("if device is None:"))
        ]
        self.assertIn("time.monotonic() - last_frame_s >= args.idle_timeout",
                      missing_device_branch)
        self.assertIn("the serial device is unavailable", missing_device_branch)

    def test_capture_fused_control_frames_refresh_idle_watchdog(self):
        source = (ROOT / "tools" / "capture_sensor_log.py").read_text(
            encoding="utf-8"
        )
        fusion_branch = source[
            source.index("if fusion_frame is not None:"):
            source.index("if now - last_frame_s", source.index("if fusion_frame is not None:"))
        ]
        self.assertIn("fusion_rows.append(fusion_frame)", fusion_branch)
        self.assertIn("last_frame_s = now", fusion_branch)
        self.assertIn("no D4 or fused-control frame", source)

    def test_navigation_qualification_analyzer_aligns_and_compares_sensors(self):
        source = NAV_ANALYZER_PATH.read_text(encoding="utf-8")
        self.assertIn("def aligned_gps(", source)
        self.assertIn("gps_vs_fused_endpoint_in", source)
        self.assertIn("gps_vs_imu_heading_change_deg", source)
        self.assertIn("encoder_net_center_in", source)
        self.assertIn("navigation_analysis.png", source)

    def test_dropout_analyzer_reports_coast_reduction_and_safety_gates(self):
        source = DROPOUT_ANALYZER_PATH.read_text(encoding="utf-8")
        self.assertIn("coast_reduction_percent", source)
        self.assertIn("hold_pass_under_quarter_inch", source)
        self.assertIn("both_fail_closed", source)
        self.assertIn("both_reinitialized", source)

    def test_pros_serial_packets_are_cobs_decoded_before_parsing(self):
        line = b"D4 s=1 t=2 p6=100,63,1\n"
        packet = cobs.encode(b"sout" + line + b"\0") + b"\0"
        chunks, remainder = SERVER.decode_serial_packets(bytearray(), packet)
        self.assertEqual(chunks, [line.decode("ascii")])
        self.assertEqual(remainder, bytearray())

    def test_no_tag_frame_is_reported_as_healthy_but_invalid(self):
        frame = SERVER.parse_vision_shadow(
            "noise VISION_SHADOW t=1200 poll=12 installed=1 configured=1 "
            "count=0 tag=-1 corners=0,0,0,0,0,0,0,0 center=0.0,0.0 "
            "area=0.0 edge_ratio=0.0 fill=0.0 bearing=0.00 repeat=0 "
            "geometry_age=0 valid=0 reason=no_tag"
        )
        self.assertTrue(frame["installed"])
        self.assertTrue(frame["configured"])
        self.assertFalse(frame["valid"])
        self.assertEqual(frame["reason"], "no_tag")

    def test_valid_tag_geometry_is_preserved(self):
        frame = SERVER.parse_vision_shadow(
            "VISION_SHADOW t=2200 poll=22 installed=1 configured=1 count=1 "
            "tag=3 corners=100,80,140,82,138,122,98,120 center=119.0,101.0 "
            "area=1600.0 mean_edge=40.02 depth=3.90 right=-0.70 up=0.38 horizontal=3.96 "
            "range=3.98 edge_ratio=1.05 fill=0.95 bearing=-10.25 "
            "elevation=5.50 roll=2.86 repeat=0 "
            "geometry_age=0 valid=1 "
            "reason=shadow_valid"
        )
        self.assertTrue(frame["valid"])
        self.assertEqual(frame["tag_id"], 3)
        self.assertEqual(frame["corners"], [100, 80, 140, 82, 138, 122, 98, 120])
        self.assertAlmostEqual(frame["bearing_deg"], -10.25)
        self.assertAlmostEqual(frame["mean_edge_px"], 40.02)
        self.assertAlmostEqual(frame["range_estimate_in"], 3.98)
        self.assertAlmostEqual(frame["forward_depth_in"], 3.90)
        self.assertAlmostEqual(frame["right_offset_in"], -0.70)
        self.assertAlmostEqual(frame["up_offset_in"], 0.38)
        self.assertAlmostEqual(frame["horizontal_range_in"], 3.96)
        self.assertAlmostEqual(frame["elevation_deg"], 5.50)
        self.assertAlmostEqual(frame["image_roll_deg"], 2.86)
        self.assertAlmostEqual(frame["edge_ratio"], 1.05)
        self.assertAlmostEqual(frame["fill_ratio"], 0.95)

    def test_fused_pose_preserves_ai_candidate_diagnostics(self):
        pose = SERVER.parse_fuse_test(
            "FUSE_TEST phase=opcontrol x=-48.1 y=0.5 heading=94.2 "
            "ai_id=3 ai_bearing=-10.25 ai_goal=upper_red_neutral "
            "ai_range=14.8 ai_pred_range=15.4 ai_range_residual=0.6 "
            "ai_innovation=0.7 ai_pos_step=0.2 ai_heading_step=-0.1 "
            "ai_face=west ai_residual=4.5 ai_margin=12.0 ai_age=80 "
            "ai_reject=shadow_only"
        )
        self.assertEqual(pose["ai_id"], 3)
        self.assertEqual(pose["ai_goal"], "upper_red_neutral")
        self.assertEqual(pose["ai_face"], "west")
        self.assertAlmostEqual(pose["ai_residual"], 4.5)
        self.assertAlmostEqual(pose["ai_range_residual"], 0.6)
        self.assertAlmostEqual(pose["ai_innovation"], 0.7)
        self.assertEqual(pose["ai_reject"], "shadow_only")

    def test_fused_pose_preserves_calibrated_kinematics(self):
        pose = SERVER.parse_fuse_test(
            "FUSE_TEST phase=telemetry x=-47.8 y=0.5 heading=88.1 "
            "track=12.0086 rear=2.5665 lidar_scale=0.926770"
        )
        self.assertAlmostEqual(pose["track"], 12.0086)
        self.assertAlmostEqual(pose["rear"], 2.5665)
        self.assertAlmostEqual(pose["lidar_scale"], 0.926770)

    def test_current_d4_schema_preserves_forward_distance_and_gps(self):
        frame = SERVER.parse_d4(
            "D4 s=10 t=200 p1=1016,6,1 "
            "m17=1.0 m18=2.0 m11=3.0 m13=4.0 h5=50 "
            "imu=-1.2 rawimu=-1.2 imust=18 "
            "gps7=-1.3804,-0.0137,299.75,0.0098,1 errno=0 gpsgyro=0.14"
        )
        self.assertEqual(frame["sensors"]["1"]["mm"], 1016)
        self.assertEqual(frame["sensors"]["1"]["confidence"], 6)
        self.assertAlmostEqual(frame["gps"]["x_m"], -1.3804)
        self.assertAlmostEqual(frame["gps"]["heading_deg"], 299.75)
        self.assertTrue(frame["gps"]["installed"])
        self.assertAlmostEqual(frame["gps"]["gyro_z"], 0.14)

    def test_fused_pose_preserves_gps_gating_diagnostics(self):
        pose = SERVER.parse_fuse_test(
            "FUSE_TEST phase=telemetry x=-52.1 y=-8.7 heading=240.2 "
            "gps_x=-52.2 gps_y=-8.8 gps_heading=240.3 gps_error=0.39 "
            "gps_innovation=0.14 gps_pos_step=0.03 gps_heading_step=0.01 "
            "gps_reject=corrected dr_travel=18.4 pos_envelope=1.27 "
            "abs_age=2300"
        )
        self.assertAlmostEqual(pose["gps_error"], 0.39)
        self.assertAlmostEqual(pose["gps_innovation"], 0.14)
        self.assertEqual(pose["gps_reject"], "corrected")
        self.assertAlmostEqual(pose["dr_travel"], 18.4)
        self.assertAlmostEqual(pose["pos_envelope"], 1.27)
        self.assertAlmostEqual(pose["abs_age"], 2300)


class TournamentUiIsolationTests(unittest.TestCase):
    def test_webcam_is_opt_in_and_never_substitutes_official_y(self):
        self.assertFalse(SERVER.DEBUG_WEBCAM_ENABLED)
        html = SERVER.render_html("field")
        self.assertIn("const displayY = onboard.y;", html)
        self.assertNotIn("const displayY = cameraFresh", html)
        self.assertIn("Webcam DEBUG ONLY", html)
        self.assertIn("Tournament fused pose", html)
        self.assertIn("AI Vision", html)
        self.assertIn("visionFrame.range_estimate_in", html)
        self.assertIn("visionFrame.horizontal_range_in", html)
        self.assertIn("visionFrame.mean_edge_px", html)
        self.assertIn("visionCandidate.ai_range_residual", html)
        self.assertIn("visionCandidate.ai_innovation", html)
        self.assertIn("drawAiVisionHypotheses", html)
        self.assertIn("const headingRad = displayPose.headingRad;", html)
        self.assertNotIn("displayPose.headingDeg", html)
        self.assertIn("aiCameraForwardOffsetIn = 0", html)
        self.assertIn("aiCameraRightOffsetIn = 0", html)
        self.assertIn("aiCameraYawRightDeg = 0", html)
        self.assertIn("aiCameraExtrinsicsQualified = false", html)
        self.assertIn('"UNMEASURED"', html)
        self.assertIn("horizontalOdometerOffsetBackIn = 5.18", html)
        self.assertIn("effectiveWheelDiameterIn = 2.4330552523", html)
        self.assertIn("horizontalOdometerEnabled = false", html)
        self.assertNotIn("const wheelDiameterIn = 2.75", html)
        self.assertIn("START 30.2,34.7", html)
        self.assertIn("Start Pose Help", html)
        self.assertIn("hold X+Y to edit the exact start pose", html)
        self.assertIn("Effective Track", html)
        self.assertIn("Rear Wheel at 15 deg", html)
        self.assertIn("L+1 / R-1 Turn", html)
        self.assertIn("const rightEncoderSign = 1", html)
        self.assertIn("const trackWidthIn = 10.6245815677", html)
        self.assertIn("DR Since Fix", html)
        self.assertIn("Error Envelope", html)
        self.assertIn("Absolute Fix Age", html)
        self.assertIn("empirical envelope", html)
        self.assertIn("gyro-z", html)
        self.assertIn("visionCandidate.gps_reject", html)

    def test_payload_has_separate_onboard_webcam_and_vision_channels(self):
        body = SERVER.payload()
        self.assertIn("onboard_pose", body)
        self.assertIn("camera_pose", body)
        self.assertIn("vision", body)
        self.assertIsNot(body["onboard_pose"], body["camera_pose"])

    def test_serial_reader_reselects_a_silent_or_wrong_port(self):
        source = SERVER_PATH.read_text(encoding="utf-8")
        self.assertIn("last_recognized_time = time.monotonic()", source)
        self.assertIn("time.monotonic() - last_recognized_time > 6.0", source)
        self.assertIn("no recognized D4/FUSE_TEST/VISION_SHADOW telemetry", source)
        self.assertIn("bytes received but no recognized robot telemetry", source)
        self.assertIn("P1 Distance, P6 IMU, P7 GPS, P8 AI Vision", source)


class FirmwareSafetyInvariantTests(unittest.TestCase):
    def test_gps_frame_numeric_cardinal_axes_and_live_sample(self):
        program = textwrap.dedent(
            r"""
            #include "gps_frame.hpp"
            #include <cmath>

            bool near(double actual, double expected, double tolerance = 1e-6) {
              return std::fabs(actual - expected) <= tolerance;
            }

            int main() {
              constexpr double meters_per_inch = 0.0254;
              // Robot center at the origin, pointing project +X/top. Its GPS
              // is 6 in right and 6 in behind and points robot-right.
              const auto north = localization::vex_gps_to_project_robot_pose(
                  6.0 * meters_per_inch, -6.0 * meters_per_inch, 90.0);
              if (!near(north.x_in, 0.0) || !near(north.y_in, 0.0) ||
                  !near(north.heading_deg, 0.0)) return 1;

              // Same center, robot pointing project +Y/red/left.
              const auto west = localization::vex_gps_to_project_robot_pose(
                  6.0 * meters_per_inch, 6.0 * meters_per_inch, 0.0);
              if (!near(west.x_in, 0.0) || !near(west.y_in, 0.0) ||
                  !near(west.heading_deg, 90.0)) return 2;

              // A real stationary frame from stationary_04_120s. The heading
              // independently agrees with the entered 151.65-deg pose.
              const auto live = localization::vex_gps_to_project_robot_pose(
                  -1.3804, -0.0150, 299.64);
              if (!near(live.x_in, -8.774, 0.02) ||
                  !near(live.y_in, 52.102, 0.02) ||
                  !near(live.heading_deg, 150.36, 0.01)) return 3;
              return 0;
            }
            """
        )
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "gps_frame_test.cpp"
            binary = Path(directory) / "gps_frame_test"
            source.write_text(program, encoding="utf-8")
            subprocess.run(
                [
                    "c++",
                    "-std=c++17",
                    "-I",
                    str(ROOT / "include"),
                    str(source),
                    "-o",
                    str(binary),
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            subprocess.run([str(binary)], check=True)

    def test_ai_bearing_sign_and_duplicate_goal_margin_are_not_mirrored(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn(
            "signed_angle_diff_deg(global_bearing_deg, camera_heading_deg);",
            source,
        )
        self.assertNotIn(
            "-signed_angle_diff_deg(global_bearing_deg, camera_heading_deg);",
            source,
        )
        self.assertIn("std::strcmp(candidate.goal, best.goal) != 0", source)
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("kAiVisionPoseCorrectionEnabled = true", config)
        self.assertIn("kAiCameraYawRightDeg = 0.0", config)
        self.assertIn("kAiMaxPositionInnovationIn = 2.0", config)
        self.assertIn("kAiMaxPositionStepIn = 0.15", config)
        self.assertIn("kAiRangeResidualScoreDegPerIn = 0.5", config)
        self.assertIn("kAiMinFaceWinnerMarginScore = 2.0", config)
        self.assertIn("kAiHeadingGain = 0.0", config)
        self.assertIn("observation.horizontal_range_in", source)
        self.assertIn("kAiRequiredConsistentObservations = 5", config)
        self.assertIn("kAiRequiredReacquisitionObservations = 20", config)
        self.assertIn("kAiMaxReacquisitionInnovationIn = 4.0", config)
        self.assertIn("kAiPositionGain = 0.10", config)
        self.assertIn("proven_reacquisition", source)
        self.assertIn('pose.ai_reject = "face_ambiguous"', source)
        self.assertIn("kSideOdomRearOffsetIn = 5.18", config)
        self.assertIn("kSideOdomEnabled = false", config)
        for official_entry in (
            '{"top_red_neutral", 4,',
            '{"top_blue_alliance", 3,',
            '{"upper_red_neutral", 1,',
            '{"upper_blue_alliance", 2,',
            '{"lower_red_alliance", 2,',
            '{"lower_blue_neutral", 1,',
            '{"bottom_red_alliance", 3,',
            '{"bottom_blue_neutral", 4,',
        ):
            self.assertIn(official_entry, config)
        self.assertIn("kFusedTurnKp = 1.8", source)
        self.assertIn("kFusedTurnKd = 0.058514", source)
        self.assertIn("kFusedTurnMinPower = 20.0", source)
        self.assertIn("kFusedTurnBreakawayPower = 28.0", source)
        self.assertIn("kFusedTurnBreakawayDelayMs = 250", source)
        self.assertIn("now - last_motion_ms >= kFusedTurnBreakawayDelayMs", source)
        self.assertIn("kFusedTurnSlewPowerPerSec = 800.0", source)
        self.assertIn("kFusedTurnStaticRateDegPerSec = 6.0", source)
        self.assertIn("const bool crossed_target", source)
        settle_window = source.index(
            "std::fabs(error_deg) <= kFusedTurnToleranceDeg"
        )
        crossing_brake = source.index("if (crossed_target &&", settle_window)
        self.assertLess(settle_window, crossing_brake)
        self.assertIn("abort=turn_stall", source)
        self.assertIn("kForwardObstacleImmediateStopIn = 4.0", config)
        self.assertIn("kForwardObstacleConfirmationMs = 70", config)
        self.assertIn("event=forward_obstacle_pending", source)
        self.assertIn("telemetry_pose = pose", source)
        self.assertIn("kMaxSameSideMotorSpreadDeg = 15.0", source)
        self.assertIn("kDriveSpreadFaultConsecutiveSamples = 3", source)
        self.assertIn("drive_encoder_missing || persistent_drive_spread", source)
        self.assertIn(
            "turn_command = clamp(turn_command, -std::fabs(forward_command)",
            source,
        )

    def test_rear_offset_is_applied_only_to_an_accepted_side_delta(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("bool side_delta_accepted = false;", source)
        self.assertIn("const double delta_side_center_in = side_delta_accepted", source)
        self.assertIn("pose.side_odom_ready = false;", source)
        self.assertIn("localization::kSideOdomEnabled &&", source)

    def test_lidar_cannot_override_exact_start_with_a_large_angle_jump(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("kMaxWallHeadingErrorDeg = 8.0", source)
        self.assertIn("kMaxLidarAxisInnovationIn = 1.5", source)

    def test_one_meter_lidar_jump_is_rejected_before_position_correction(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        innovation_gate = source.index(
            "best.axis_error_in > kMaxLidarAxisInnovationIn"
        )
        correction = source.index("pose.x += axis_step_in")
        self.assertLess(innovation_gate, correction)
        self.assertIn("kMaxLidarAxisCandidateChangeIn", source)
        self.assertIn("kRequiredConsistentLidarFits", source)
        self.assertIn("kMaxLidarAxisStepIn = 2.0", source)

    def test_port_nine_is_exclusively_the_slider_motor(self):
        subsystems = (ROOT / "include" / "subsystems.hpp").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        autons = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        cascade = (ROOT / "src" / "cascade_lift.cpp").read_text(encoding="utf-8")
        self.assertIn("slider_left(-9)", subsystems)
        self.assertIn("slider_right(2)", subsystems)
        self.assertNotIn("distance_9", subsystems)
        self.assertNotIn("pros::Distance distance_9", main)
        self.assertIn("cascade_lift::set_manual_power", main)
        self.assertIn("slider_right.move(power)", cascade)
        self.assertIn("slider_left.move(power)", cascade)
        self.assertIn("bounded_mv * 127.0 / 12000.0", cascade)
        self.assertIn("command_power(power)", cascade)
        self.assertIn("kDownPmvPerDeg = 60.0", cascade)
        self.assertIn("kDownDmvPerDegPerS = 14.0", cascade)
        self.assertIn("kPidDownwardVoltageLimitMv = 9000.0", cascade)
        self.assertIn("kMinimumUpwardPidMv = 5000.0", cascade)
        self.assertIn("kMinimumDownwardPidMv = 4000.0", cascade)
        self.assertIn("427.59, 757.62, 1140.56, 1717.20", cascade)
        self.assertIn("kStagePositionToleranceDeg = 16.0", cascade)
        self.assertIn("kZeroPositionToleranceDeg = 1.5", cascade)
        self.assertIn("LiDAR correction remains fail-closed", autons)

    def test_claw_and_wrist_controls_match_current_mapping(self):
        subsystems = (ROOT / "include" / "subsystems.hpp").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn("Drive chassis({17, 18},\n              {-11, -13},\n              12,", main)
        self.assertIn("GPS P7 / IMU P12", main)
        self.assertIn("clamp_piston('D')", subsystems)
        self.assertIn("claw_piston('E')", subsystems)
        self.assertIn("set_claw_piston(bool extended)", subsystems)
        self.assertIn("claw_piston.set_value(!extended)", subsystems)
        self.assertIn("E_CONTROLLER_DIGITAL_L1", main)
        self.assertNotIn("counter_rollers.move(claw_arm_power)", main)
        self.assertIn("claw_toggle_pressed", main)
        self.assertIn("set_claw_piston(!claw_piston_extended.load", main)
        self.assertIn("claw_arm(4, pros::v5::MotorGears::green)", subsystems)
        self.assertIn("claw_arm.set_current_limit(2500)", main)
        self.assertIn("kArmRightTargetDeg = 278.17", main)
        self.assertIn("kArmNormalTargetDeg = 324.66", main)
        self.assertIn("kArmNormalToleranceDeg = 5.0", main)
        self.assertIn("kArmPositionKp = 1.35", main)
        self.assertIn("kArmPositionKd = 0.10", main)
        self.assertIn("ArmPositionController", main)
        self.assertIn("arm_position_controller.update(arm_position_target_deg)", main)
        self.assertIn("arm_position_target_deg = kArmNormalTargetDeg", main)
        self.assertIn("bool arm_position_hold_active = false", main)
        self.assertNotIn("kWristGravityCompensationPower", main)
        self.assertIn("wrist_up_manual_negative", main)
        self.assertIn("wrist_down_manual_positive", main)
        self.assertIn("claw_arm.move(-kMechanismPower)", main)
        self.assertIn("claw_arm.move(kMechanismPower)", main)
        self.assertIn("!master.get_digital(pros::E_CONTROLLER_DIGITAL_A)", main)
        self.assertIn("piston_toggle_pressed", main)
        self.assertIn("clamp_output_high.store(next_output_high", main)
        self.assertIn("clamp_piston.set_value(next_output_high)", main)
        auton_dispatch = main.index("bool run_selected_red_auton() {")
        piston_auton = main.index("clamp_piston.set_value(false)", auton_dispatch)
        route_dispatch = main.index("localization_two_cup_red_auton()", piston_auton)
        self.assertLess(piston_auton, route_dispatch)
        startup = main[piston_auton:route_dispatch]
        self.assertIn("clamp_piston.set_value(true)", startup)
        self.assertIn("clamp_output_high.store(true", startup)
        self.assertIn("pros::Task arm_normal_hold_task", main[auton_dispatch:route_dispatch])
        self.assertIn("controller.update(kArmNormalTargetDeg)", main[auton_dispatch:route_dispatch])
        self.assertIn("pros::delay(1000);", startup)
        self.assertIn("pros::delay(200);", startup)
        self.assertIn("motor.tare_position()", startup)
        self.assertIn("AUTON_DRIVE_ANCHOR", startup)
        self.assertNotIn("kAutonArmLoweringPower", main)
        self.assertNotIn("horizontal_odom.reset_position()", main[auton_dispatch:route_dispatch])
        self.assertIn("ARM_NORMAL target", startup)

    def test_l2_b_starts_exact_far_goal_auton(self):
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        autons = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("start_toggle_far_goal_auton();", main)
        listener_start = main.index("One task owns both L2 selection")
        listener_end = main.index("auton select and launch", listener_start)
        listener = main[listener_start:listener_end]
        self.assertIn("E_CONTROLLER_DIGITAL_L2", listener)
        self.assertIn("E_CONTROLLER_DIGITAL_B", listener)
        self.assertNotIn("E_CONTROLLER_DIGITAL_A", listener)
        self.assertIn("launch_l2_b", listener)
        self.assertIn("l2_press_had_b = true", listener)
        self.assertIn("localization_toggle_far_goal_hotkey_auton()", autons)
        self.assertIn("kFarGoalCenter{23.55, -47.09}", autons)
        self.assertIn("controller=pure_pursuit", autons)
        self.assertIn("ram_and_return(1)", autons)
        self.assertIn("ram_and_return(2)", autons)
        # Regression for the failed 2026-08-26 wall-side route: leave the
        # Toggle inward and curve around the field-center side of the nearby
        # red Goal. Absolute sensors may only make bounded fused corrections;
        # encoder/IMU propagation remains the high-rate controller backbone.
        self.assertIn("kPathEnd{3.55, -47.09}", autons)
        self.assertIn("kBezierControl1{34.0, 0.25}", autons)
        self.assertIn("kBezierControl2{-8.0, -35.0}", autons)
        self.assertIn('"far_goal_launch_turn",\n                              180.0', autons)
        self.assertIn("set_physical_drive_power(\n          127", autons)
        self.assertGreaterEqual(
            autons.count("LidarFusionMode::kBiasOnly"), 10
        )

    def test_two_cup_red_selector_and_phase_one_route_are_explicit(self):
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include" / "autons.hpp").read_text(encoding="utf-8")
        route = (ROOT / "src" / "path2_auton.cpp").read_text(encoding="utf-8")
        autons = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "path2_auton_config.hpp").read_text(
            encoding="utf-8"
        )

        self.assertIn('"1 Pin Auto Red"', main)
        self.assertIn('"2 Cup Auto Red"', main)
        self.assertIn(
            "selected_red_auton{\n    static_cast<int>(RedAutonSelection::kTwoCup)}",
            main,
        )
        self.assertIn("pros::lcd::read_buttons()", main)
        self.assertIn("brain_buttons & LCD_BTN_LEFT", main)
        self.assertIn("brain_buttons & LCD_BTN_RIGHT", main)
        self.assertIn("pros::screen::touch_status()", main)
        self.assertIn("pros::screen::print(pros::E_TEXT_LARGE_CENTER", main)
        self.assertIn("TEST LIMIT: THROUGH 2ND SCORE", main)
        self.assertIn("wrist_normal_position_pressed", main)
        self.assertNotIn("bool left_press_had_x = false", main)
        self.assertIn("opcontrol_auton_running) return", main)
        self.assertIn("while (true) {", main)
        self.assertIn("auton_selection_locked.store(false", main)
        self.assertIn("(current + step + kAutonCount) % kAutonCount", main)
        self.assertIn("run_selected_red_auton();", main)
        self.assertIn("localization_two_cup_red_auton", header)

        self.assertIn("kStart{63.0, 0.0}", config)
        self.assertIn("kGoal{48.0, 24.0}", config)
        self.assertIn("kStackA{24.0, 25.0}", config)
        self.assertIn("kStackB{48.0, 48.0}", config)
        self.assertIn("kFinal{48.0, 60.0}", config)
        self.assertIn("kStartHeadingDeg = 0.0", config)
        self.assertIn("kToggleReturnDistanceIn = 6.0", config)
        self.assertIn("kTogglePower = 90", config)
        self.assertIn("kToggleReturnPower = 80", config)
        self.assertIn("kPhase1DrivePower = 90", config)
        self.assertIn("kToggleSignedDistanceIn = 7.0", config)
        self.assertIn("kFastDrivePower = 112", config)
        self.assertIn("kStackFastFraction = 0.50", config)
        self.assertIn("kStackApproachPower = 75", config)
        self.assertIn("kSlowStackPower = 35", config)
        self.assertIn("kRobotCenterToRearIn = 9.0", config)
        self.assertIn("kRearClawExtensionIn = 1.0", config)
        self.assertIn("kRearPickupReachIn", config)
        self.assertIn("kScoreLoweringDeg = 15.0", config)
        self.assertIn("kPhase2GoalClearanceIn = 13.0", config)
        self.assertIn("kStage1ScoreLoweringDeg = 100.0", config)
        self.assertIn("kStage1ExtraCaptureIn = 1.5", config)
        self.assertIn("kStage1ExtraCapturePower = 70", config)
        self.assertIn("kStage1GoalDriveIn = 24.0", config)
        self.assertIn("kStage1GoalDrivePower = 80", config)
        self.assertIn("kStage1PostLowerWaitMs = 200", config)
        self.assertIn("kStage1OuttakeLeadMs = 100", config)
        self.assertIn("kStage1ScoreRetreatPower = 95", config)
        self.assertIn("kStage1ScoreRetreatIn = 10.0", config)
        self.assertIn("kStage2ExtraCaptureIn = 4.5", config)
        self.assertIn("kStage2GoalDriveIn = 16.0", config)
        self.assertIn("kStage2ScoreLoweringDeg = 100.0", config)
        self.assertIn("kSecondStackApproachPower = 60", config)
        self.assertIn("kSecondStackSlowPower = 38", config)
        self.assertIn("kFirstCupLiftStage = 1", config)
        self.assertIn("kSecondCupLiftStage = 2", config)
        self.assertIn("kLoadedTurnClearanceDeg = 250.0", config)
        self.assertIn("kLoadedTurnClearanceTimeoutMs = 650", config)
        self.assertIn("kScoreStageReadyTimeoutMs = 1000", config)
        self.assertIn("kStage2ScoreRetreatIn = 24.0", config)
        self.assertIn("kTestStopAfterPhase = 0", config)

        self.assertIn("path2_fast_turn(-75.0, true)", route)
        self.assertIn(
            "path2_fast_drive(\n      -11.0, kPhase1DrivePower", route
        )
        self.assertIn("kPreloadPinUpperPower", route)
        self.assertIn("kPreloadPinCounterPower", route)
        self.assertIn("-kToggleReturnDistanceIn", route)
        self.assertIn("test_stop=phase_1_after_preload_drop", route)
        self.assertIn("stack A boomerang approach", route)
        self.assertIn("reverse, true, true", route)
        self.assertIn("recovered_pose.valid", route)
        self.assertIn("PATH2_CHAIN retry_turn", route)
        self.assertIn("final_error_deg <= 10.0", route)
        self.assertIn("robot_center_target", route)
        self.assertIn("pickup_reach", route)
        self.assertIn("constexpr double kPneumaticGrabLeadIn = 9.2", route)
        self.assertIn("constexpr double kPneumaticGrabLeadIn = 5.5", route)
        slow_capture = route.index(
            "const auto capture_result = navigation::drive_relative("
        )
        first_claw_close = route.index("set_claw_piston(false);", slow_capture)
        self.assertLess(slow_capture, first_claw_close)
        self.assertNotIn("path2_set_rollers", route)
        self.assertIn("final 9.2 inches based on the latest loaded test", route)
        self.assertIn("test_stop=phase_2_after_first_cup_capture", route)
        self.assertIn("lift.request(kFirstCupLiftStage)", route)
        self.assertNotIn("first_cup_lift_not_ready", route)
        self.assertIn("first_lift_clearance_timeout", route)
        first_extra = route.index(
            "auto extra_capture = path2_fast_drive("
        )
        first_raise = route.index("lift.request(kFirstCupLiftStage)")
        self.assertLess(first_extra, first_raise)
        self.assertIn("path2_fast_turn(180.0)", route)
        self.assertIn("target_imu_cw_deg", route)
        self.assertIn("settle_tolerance_deg = 2.5", route)
        self.assertIn("final_error_deg <= tolerance_deg + 1.5", route)
        self.assertIn("stage1_lowered_target", route)
        self.assertIn("lift_at_goal.position_deg", route)
        self.assertIn("continue=first_score_drop_not_reached", route)
        self.assertIn("continue=second_score_drop_not_reached", route)
        self.assertIn("remaining_capture", route)
        self.assertIn("extra_capture_total_in", route)
        self.assertIn("first_extra_capture_short", route)
        self.assertIn("first_score_stage_not_ready", route)
        self.assertIn("-kStage1GoalDriveIn", route)
        self.assertIn("progressive_goal_decel", route)
        self.assertIn("kGoalDecelZoneIn = 7.0", route)
        self.assertIn("kGoalContactPower = 32", route)
        self.assertIn(
            "-kStage1GoalDriveIn, kStage1GoalDrivePower, 3000, true, true",
            route,
        )
        self.assertIn("pros::delay(kStage1PostLowerWaitMs)", route)
        self.assertIn("pros::delay(kStage1OuttakeLeadMs)", route)
        self.assertIn("kStage1ScoreRetreatIn, kStage1ScoreRetreatPower", route)
        first_lower_wait = route.index("pros::delay(kStage1PostLowerWaitMs)")
        first_outtake = route.index(
            "set_claw_piston(true);",
            first_lower_wait,
        )
        first_retreat = route.index("const auto score_retreat", first_outtake)
        self.assertLess(first_lower_wait, first_outtake)
        self.assertLess(first_outtake, first_retreat)
        self.assertIn("test_stop=phase_3_after_first_cup_score", route)
        self.assertIn("kSecondStackPickupHeadingDeg = 225.0", route)
        self.assertIn("completed=first_stack_deposit next=second_stack", route)
        self.assertEqual(route.count("lift.start_full_down_for(500)"), 2)
        self.assertIn("cascade_lift::set_manual_power(-127)", route)
        self.assertNotIn("first_score_lift_not_home", route)
        first_home_wait = route.index("lift.start_full_down_for(500)")
        first_retreat_start = route.index("const auto score_retreat")
        first_pulse_wait = route.index("lift.wait_full_down_complete(800)")
        self.assertLess(first_home_wait, first_retreat_start)
        self.assertLess(first_retreat_start, first_pulse_wait)
        second_stack_turn = route.index(
            "path2_gps_turn(kSecondStackPickupHeadingDeg, 4.0)"
        )
        self.assertLess(first_home_wait, second_stack_turn)
        self.assertIn("kSecondStackCenterTravelIn", route)
        self.assertIn("kSecondStackApproachPower", route)
        self.assertIn("kSecondStackSlowPower", route)
        self.assertIn("lift.request(kSecondCupLiftStage)", route)
        self.assertNotIn("second_cup_lift_not_ready", route)
        self.assertIn("second_lift_clearance_timeout", route)
        second_extra = route.index(
            "auto second_extra = path2_fast_drive("
        )
        second_raise = route.index("lift.request(kSecondCupLiftStage)")
        self.assertLess(second_extra, second_raise)
        self.assertNotIn("lift_home_timeout_after_retreat", route)
        self.assertNotIn("stage_2_home_timeout_after_retreat", route)
        self.assertIn("path2_gps_turn(90.0, 4.0)", route)
        self.assertIn("cascade_lift::clear_fault()", route)
        self.assertNotIn("path2_home_lift_at_bottom", route)
        self.assertNotIn("abort=lift_bottom_home_failed", route)
        self.assertNotIn("cascade_lift::set_target_position_deg(0.0)", route)
        self.assertIn("PATH2_LIFT command=rest_lock", route)
        self.assertIn("cascade_lift::disable_pid();", route)
        self.assertIn("requested_stage_{0}", route)
        self.assertIn("rest_lock_{true}", route)
        first_stage_two_request = route.index(
            "lift.request(kFirstCupLiftStage)"
        )
        startup_stage_zero = route.index("cascade_lift::disable_pid();")
        self.assertLess(startup_stage_zero, first_stage_two_request)
        self.assertIn("slider_right.brake()", route)
        self.assertIn("slider_left.brake()", route)
        self.assertIn("stage2_lowered_target", route)
        self.assertIn("second_score_stage_not_ready", route)
        self.assertNotIn("Path2WristService", route)
        self.assertIn("continuously holds its recorded", route)
        route_start = route.index("bool localization_two_cup_red_auton()")
        self.assertNotIn("claw_arm.move(0)", route[route_start:])
        self.assertIn("if (!path2_fast_turn(180.0)) return false;", route)
        self.assertNotIn("test_stop=phase_4_after_second_stack_score", route)
        self.assertIn("explicit successful end of the complete two-cup route", route)
        self.assertIn("second_remaining", route)
        self.assertIn("-kStage2GoalDriveIn", route)
        self.assertIn("kStage2ScoreRetreatIn, kStage2ScoreRetreatPower", route)

        one_pin_start = autons.index(
            "bool localization_simple_red_goal_hotkey_auton()"
        )
        one_pin_end = autons.index(
            "bool localization_simple_red_goal_finish_correction()",
            one_pin_start,
        )
        one_pin = autons[one_pin_start:one_pin_end]
        self.assertIn("kStart{54.0, -13.6076952, 0.0}", one_pin)
        self.assertIn("kGoal{48.0, -24.0}", one_pin)
        self.assertIn("kGoalTurnDeg = 60.0", one_pin)
        self.assertIn("kGoalContactIn = 12.0", one_pin)
        self.assertIn("toggle_ram = navigation::drive_relative(\n      6.0, 78, 1800, false, true, true", one_pin)
        self.assertIn("toggle_return = navigation::drive_relative(\n      -6.0, 68, 2600, true, true, true", one_pin)
        self.assertIn("goal_contact = navigation::drive_relative(\n      -kGoalContactIn", one_pin)
        self.assertIn("kGoalExitIn = 13.5", one_pin)
        self.assertIn("kStackReverseIn = 24.0", one_pin)
        self.assertIn("kStackClampLeadIn = 1.0", one_pin)
        self.assertIn("goal_retreat = navigation::drive_relative(\n      kGoalExitIn", one_pin)
        self.assertIn("kFinalStack{24.0, 0.0}", one_pin)
        self.assertIn("0.0, 70, 1400, true, true", one_pin)
        self.assertIn("-(kStackReverseIn - kStackClampLeadIn)", one_pin)
        self.assertIn("-kStackClampLeadIn, 38, 1600", one_pin)
        self.assertIn("ONE_PIN_NEW", one_pin)
        self.assertIn("set_claw_piston(true)", one_pin)
        self.assertIn("set_claw_piston(false)", one_pin)
        self.assertNotIn("counter_rollers", one_pin)
        self.assertIn("SIMPLE_RED chain_after_boomerang", one_pin)
        self.assertIn("approach_completed && turn_usable", one_pin)
        self.assertIn("if (route_enabled && approach_completed && turn_usable)", one_pin)
        self.assertIn("const bool nudge_ok = route_enabled", one_pin)
        self.assertNotIn("if (turn_fully_settled)", one_pin)
        self.assertIn("88, 2800, true, true, true", one_pin)
        self.assertIn("kDepositHeadingDeg, 100, 1400", one_pin)
        self.assertIn("straighten_usable", one_pin)
        self.assertIn("face_start_usable", one_pin)
        self.assertIn("final_capture", one_pin)
        self.assertIn("kFinalStack{24.0, 0.0}", one_pin)
        self.assertIn("kFinalStackRobotCenter", one_pin)
        self.assertIn("retreat_extra = navigation::drive_relative(\n            10.0", one_pin)
        self.assertIn("stack_center_error_in <= 3.0", one_pin)
        self.assertIn("SIMPLE_RED final_stack", one_pin)

    def test_gps_fusion_is_absolute_bounded_and_fail_closed(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        gps_frame = (ROOT / "include" / "gps_frame.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("kGpsSensorHeadingOffsetCwDeg = 90.0", config)
        self.assertIn("kGpsRightOffsetIn = 6.0", config)
        self.assertIn("kGpsForwardOffsetIn = -6.0", config)
        self.assertIn("kGpsMaxReportedErrorIn = 0.75", config)
        self.assertIn("kGpsMaxPositionStepIn = 0.50", config)
        self.assertIn("kGpsMaxHeadingStepDeg = 0.0", config)
        self.assertIn("kGpsHeadingGain = 0.0", config)
        self.assertIn("kGpsMaxCorrectionLinearSpeedInS = 0.50", config)
        self.assertIn("kGpsMaxRepeatedObservationDriveIn = 0.50", config)
        self.assertIn("kGpsMaxRepeatedObservationHeadingDeg = 2.0", config)
        self.assertIn("kGpsRequiredConsistentObservations = 12", config)
        self.assertIn("kGpsRequiredReacquisitionObservations = 30", config)
        self.assertIn("kGpsMaxReacquisitionInnovationIn = 6.0", config)
        quality_gate = source.index("observation.error_in > localization::kGpsMaxReportedErrorIn")
        position_correction = source.index("pose.x += innovation_x * position_step_in")
        self.assertLess(quality_gate, position_correction)
        self.assertIn("gps_reject = \"position_innovation\"", source)
        self.assertIn("gps_reject = \"spin\"", source)
        self.assertIn("gps_reject = \"motion\"", source)
        self.assertIn("std::fabs(linear_speed_in_s)", source)
        self.assertIn("std::fabs(angular_rate_deg_s)", source)
        self.assertIn("const bool proven_reacquisition", source)
        self.assertIn("pose.gps_anchored = true", source)
        self.assertIn("pose.gps_frame_aligned = true", source)
        self.assertIn("pose.gps_frame_rotation_deg", source)
        self.assertIn("pose.gps_frame_translation_x_in", source)
        self.assertIn("pose.gps_frame_translation_y_in", source)
        self.assertIn('"GPS_FRAME aligned=1', source)
        self.assertNotIn("pose.x = observation.x_in", source)
        self.assertIn('gps_reject = "start_anchor_required"', source)
        self.assertIn('pose.gps_reject = "stale_geometry"', source)
        gps_start = source.index("void apply_gps_fusion(")
        gps_repeat = source.index('pose.gps_reject = "repeat";', gps_start)
        gps_increment = source.index("++pose.consistent_gps_observations", gps_start)
        self.assertLess(gps_repeat, gps_increment)
        self.assertIn("pose.last_gps_geometry_drive_distance_in", source)
        self.assertIn("kDeadReckoningScaleEnvelopeFraction", config)
        self.assertIn("pos_envelope=%.2f", source)
        self.assertIn("sensor_project_x_in = native_sensor_y_m", gps_frame)
        self.assertIn("sensor_project_y_in = -native_sensor_x_m", gps_frame)
        self.assertIn("wrap_gps_degrees(-robot_heading_cw_deg)", gps_frame)
        self.assertEqual(
            source.count("localization::vex_gps_to_project_robot_pose("), 2
        )
        for replay_name in ("replay_current_gps_gate.py", "replay_gps_faults.py"):
            replay = (
                ROOT / "reports" / "sensor_campaign_2026-08-23" / replay_name
            ).read_text(encoding="utf-8")
            self.assertIn('sensor_x = float(row["gps_y"])', replay)
            self.assertIn('sensor_y = -float(row["gps_x"])', replay)
            self.assertIn("heading = normalize(-robot_cw)", replay)
            self.assertNotIn("heading = normalize(90.0 - robot_cw)", replay)

    def test_forward_distance_is_a_fail_closed_autonomous_stop(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("kForwardObstacleStopIn = 8.0", config)
        self.assertIn("kForwardObstacleMinConfidence = 20", config)
        self.assertIn("kForwardObstacleConfidenceAvailableMm = 200", config)
        self.assertIn("kForwardObstacleMaxRangeMm = 2000", config)
        self.assertIn("abort=forward_obstacle", source)
        self.assertIn("abort=forward_sensor_%s", source)
        self.assertIn("obstacle.installed ? \"fault\" : \"missing\"", source)
        self.assertIn("observation.distance_mm != static_cast<long>(PROS_ERR)", source)
        self.assertIn("!obstacle.api_ok", source)
        self.assertIn("forward_command > 0.0", source)
        self.assertIn("motion_sign > 0.0", source)
        self.assertIn("observation.distance_mm <=", source)
        self.assertIn("if (obstacle.distance_mm == 9999) {", source)
        self.assertIn("reset_close_confirmation();", source)
        self.assertIn(
            "if (obstacle.distance_mm > localization::kForwardObstacleMaxRangeMm)",
            source,
        )
        self.assertIn("abort=forward_sensor_range", source)
        close_stop = source.index(
            "if (distance_in > localization::kForwardObstacleStopIn) {"
        )
        obstacle_stop = source.index("abort=forward_obstacle", close_stop)
        self.assertLess(close_stop, obstacle_stop)

    def test_go_to_brakes_while_settling_in_finish_window(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        stop_helper_start = source.index("void stop_drive_motors_unlocked() {")
        stop_helper_end = source.index("bool blocking_motion_abort_requested", stop_helper_start)
        stop_helper = source[stop_helper_start:stop_helper_end]
        self.assertIn("pros::E_MOTOR_BRAKE_BRAKE", stop_helper)
        self.assertIn("pros::E_MOTOR_BRAKE_HOLD", stop_helper)
        self.assertIn("motor.set_brake_mode(stop_mode);", stop_helper)
        self.assertIn("motor.brake();", stop_helper)
        self.assertNotIn("motor.move(0);", stop_helper)
        self.assertLess(stop_helper.index("stop_drive();"), stop_helper.index("motor.brake();"))
        finish_window = source.index("const bool inside_finish_window")
        settle_continue = source.index("continue;", finish_window)
        block = source[finish_window:settle_continue]
        self.assertIn("stop_drive_motors();", block)
        self.assertIn("forward_command = 0.0;", block)
        self.assertIn("turn_command = 0.0;", block)

        legacy_start = source.index("bool drive_to_point(")
        legacy_end = source.index("}  // namespace", legacy_start)
        legacy_drive = source[legacy_start:legacy_end]
        self.assertNotIn("stop_drive();", legacy_drive)
        self.assertGreaterEqual(legacy_drive.count("stop_drive_motors();"), 2)

    def test_turn_brakes_while_settling_in_heading_window(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        start = source.index(
            "std::fabs(error_deg) <= kFusedTurnToleranceDeg"
        )
        end = source.index("} else {", start)
        block = source[start:end]
        self.assertIn("stop_drive_motors();", block)
        self.assertIn("turn_command = 0.0;", block)
        self.assertIn("continue;", block)

    def test_go_to_has_bounded_no_progress_stall_abort(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("kFusedDriveStallProgressIn = 0.10", source)
        self.assertIn("kFusedDriveStallTimeoutMs = 1000", source)
        self.assertIn("const double encoder_forward_travel_in", source)
        self.assertIn("abort=drive_stall", source)
        self.assertIn('log_drive_health("fused_drive_stall_commanded")', source)
        stall = source.index("abort=drive_stall")
        self.assertIn("stop_drive_motors();", source[stall - 250:stall])

    def test_blocking_motion_aborts_and_latches_on_sensor_loss(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("count == expected_count", source)
        self.assertIn("!chassis.imu.is_installed()", source)
        self.assertIn("chassis.imu.is_calibrating()", source)
        self.assertIn("pose.ready = false;", source)
        self.assertIn('pose.gps_reject = "drive_invalid";', source)
        self.assertIn('pose.ai_reject = "drive_invalid";', source)
        self.assertIn('pose.gps_reject = "reinit_required";', source)
        self.assertIn("if (motion_pose_invalid(pose, phase)) return false;", source)
        self.assertGreaterEqual(
            source.count("if (motion_pose_invalid(pose, phase)) return false;"),
            2,
        )
        self.assertIn("abort=localization_sensor", source)
        self.assertIn("bool drive_emergency_hold_latched = false;", source)
        self.assertIn("void emergency_stop_drive_motors()", source)
        self.assertIn("pros::E_MOTOR_BRAKE_HOLD", source)
        self.assertIn("drive_emergency_hold_latched = false;", source)
        self.assertNotIn("pose.imu_ready = true;", source[source.index("void update_pose"):])

    def test_ai_fusion_counts_each_camera_poll_at_most_once(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("std::uint32_t last_ai_poll_id = 0;", source)
        self.assertIn("observation.poll_id == pose.last_ai_poll_id", source)
        self.assertLess(
            source.index("observation.poll_id == pose.last_ai_poll_id"),
            source.index("++pose.consistent_ai_observations"),
        )
        self.assertIn("kAiMaxRepeatedGeometryDriveIn = 0.50", config)
        self.assertIn("kAiMaxRepeatedGeometryHeadingDeg = 2.0", config)
        self.assertIn('pose.ai_reject = "stale_geometry";', source)
        ai_start = source.index("void update_ai_vision_shadow(")
        ai_end = source.index("double signed_angle_diff_deg(", ai_start)
        ai_block = source[ai_start:ai_end]
        self.assertIn('pose.ai_reject = "repeat";', ai_block)
        self.assertLess(
            ai_block.index('pose.ai_reject = "repeat";'),
            ai_block.index("++pose.consistent_ai_observations"),
        )
        self.assertIn("pose.total_drive_distance_in +=", source)
        invalid_start = source.index(
            "if (!observation.installed || !observation.configured"
        )
        invalid_return = source.index("return;", invalid_start)
        self.assertIn(
            "pose.consistent_ai_observations = 0;",
            source[invalid_start:invalid_return],
        )
        for rejection in (
            'pose.ai_reject = "stale";',
            'pose.ai_reject = "no_map_candidate";',
            'pose.ai_reject = "ambiguous";',
            'pose.ai_reject = "face_ambiguous";',
        ):
            location = source.index(rejection)
            self.assertIn(
                "pose.consistent_ai_observations = 0;",
                source[max(0, location - 120):location],
            )

    def test_ai_vision_retries_after_slow_device_startup(self):
        source = (ROOT / "src" / "ai_vision_localization.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(encoding="utf-8")
        self.assertIn("now - last_init_attempt_ms >= 1000", source)
        self.assertIn("ai_vision_shadow_initialize();", source)
        self.assertIn("for (std::uint8_t port = 1; port <= 21; ++port)", source)
        self.assertIn("active_ai_vision_port", source)

    def test_ai_vision_readers_get_one_complete_published_frame(self):
        source = (ROOT / "src" / "ai_vision_localization.cpp").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "include" / "ai_vision_localization.hpp").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("AiVisionShadowSnapshot published_snapshot;", source)
        self.assertIn("pros::Mutex snapshot_mutex;", source)
        self.assertIn("void publish_snapshot()", source)
        self.assertIn("published_snapshot = snapshot;", source)
        self.assertIn("AiVisionShadowSnapshot ai_vision_shadow_snapshot()", source)
        self.assertNotIn("const AiVisionShadowSnapshot& ai_vision_shadow_snapshot", header)
        self.assertIn("snapshot.configured = false;", source)
        self.assertIn('snapshot.installed ? "read_error" : "not_installed"', source)
        self.assertIn("have_last_geometry = false;", source)
        self.assertIn("last_geometry_change_ms = 0;", source)
        self.assertIn("kAiVisionPort = 8", config)
        self.assertNotIn("best.id > 4", source)
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('"AI P%u tag=%d %s"', main)
        self.assertIn('"AI P%u T%d %-3s"', main)

    def test_ai_vision_prefers_a_usable_tag_over_a_larger_clipped_tag(self):
        source = (ROOT / "src" / "ai_vision_localization.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("bool tag_geometry_usable(", source)
        self.assertIn("bool found_usable_tag = false;", source)
        self.assertIn("tag_geometry_usable(object.object.tag, area)", source)
        self.assertIn("if (found_usable_tag)", source)
        self.assertIn("best = best_usable;", source)

    def test_p8_report_scale_matches_production_detected_square(self):
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        depth = (
            ROOT
            / "reports"
            / "sensor_campaign_2026-08-23"
            / "analyze_ai_tag_depth.py"
        ).read_text(encoding="utf-8")
        reconnect = (ROOT / "tools" / "analyze_live_reconnect_campaign.py").read_text(
            encoding="utf-8"
        )
        summary = json.loads(
            (
                ROOT
                / "reports"
                / "sensor_campaign_2026-08-23"
                / "ai_tag4_depth_summary.json"
            ).read_text(encoding="utf-8")
        )
        self.assertIn("kAiTagDetectedSizeIn = 0.625", config)
        self.assertIn("TAG_SIZE_IN = 0.625", depth)
        self.assertIn("P8_HISTORICAL_RANGE_SCALE = 0.625 / 0.875", reconnect)
        self.assertEqual(summary["assumed_detected_square_size_in"], 0.625)
        self.assertAlmostEqual(summary["pnp_horizontal_in"]["median"], 18.402471, places=5)

    def test_turn_calibration_correlates_all_motion_sensors(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(encoding="utf-8")
        for field in (
            "track_width_imu_in",
            "lidar_per_imu",
            "lidar_imu_rmse_deg",
            "odom_offset_lidar_in",
            "odom_offset_imu_in",
        ):
            self.assertIn(field, source)
        self.assertIn(
            "kDriveTrackWidthAtPhysicalWheelScaleIn = 12.0086", config
        )
        self.assertIn(
            "kDriveTrackWidthAtPhysicalWheelScaleIn * kDriveEncoderDistanceScale",
            config,
        )
        self.assertIn("kRightEncoderSign = 1.0", source)
        self.assertIn("in_place_counter_rotation", source)
        self.assertIn("kLidarThetaScale = 0.926770", config)

    def test_ai_association_does_not_allocate_in_the_estimator_loop(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        start = source.index("void update_ai_vision_shadow(")
        end = source.index("double signed_angle_diff_deg(", start)
        block = source[start:end]
        self.assertNotIn("std::vector<AiLandmarkCandidate>", block)
        self.assertIn("std::array<AiLandmarkCandidate, 8>", block)
        self.assertIn('pose.ai_reject = "candidate_overflow";', block)

    def test_navigation_path_ring_is_protected_across_tasks(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("pros::Mutex navigation_path_mutex;", source)
        self.assertGreaterEqual(
            source.count("std::lock_guard<pros::Mutex> lock(navigation_path_mutex);"),
            6,
        )
        self.assertIn("reset_navigation_path_unlocked", source)
        self.assertIn("clear_navigation_path_storage(pros::millis())", source)

    def test_navigation_pose_reads_use_complete_published_snapshots(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include" / "navigation.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("pros::Mutex telemetry_snapshot_mutex;", source)
        self.assertIn("void publish_telemetry_snapshot()", source)
        self.assertIn("bool copy_telemetry_snapshot(", source)
        self.assertIn("void publish_if_telemetry_pose(", source)
        update_start = source.index("void update_pose(")
        update_end = source.index("void log_pose(", update_start)
        update_block = source[update_start:update_end]
        self.assertGreaterEqual(
            update_block.count("publish_if_telemetry_pose(pose);"), 3
        )
        pose_start = source.index("Pose current_pose()")
        health_start = source.index("SensorHealth sensor_health()", pose_start)
        turn_start = source.index("Result turn_to(", health_start)
        self.assertIn("copy_telemetry_snapshot", source[pose_start:health_start])
        self.assertIn("copy_telemetry_snapshot", source[health_start:turn_start])
        self.assertIn("calls must still have one owner", header)

    def test_public_pose_snapshot_expires_if_update_owner_stalls(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        header = (ROOT / "include" / "navigation.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("kNavigationSnapshotMaxAgeMs = 250", config)
        self.assertGreaterEqual(
            source.count("localization::kNavigationSnapshotMaxAgeMs"), 2
        )
        self.assertGreaterEqual(header.count("estimator_age_ms"), 2)
        health_start = source.index("SensorHealth sensor_health()")
        health_end = source.index("std::size_t copy_path", health_start)
        health = source[health_start:health_end]
        self.assertIn(
            "health.drive_encoders_valid && snapshot.ready", health
        )
        self.assertIn("health.imu_valid && snapshot.imu_ready", health)
        self.assertIn('"estimator_stale"', health)

    def test_navigation_stop_latches_until_blocking_motion_observes_it(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include" / "navigation.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("std::atomic_bool navigation_stop_requested{false};", source)
        self.assertIn("bool blocking_motion_abort_requested(", source)
        self.assertIn("pros::competition::is_disabled()", source)
        self.assertGreaterEqual(
            source.count("if (blocking_motion_abort_requested(phase)) return false;"),
            2,
        )
        stop_start = source.index("void stop()")
        stop_end = source.index("const char* result_name", stop_start)
        self.assertIn(
            "navigation_stop_requested.store(true", source[stop_start:stop_end]
        )
        self.assertIn("Thread-safe cancellation", header)
        output_start = source.index("void set_physical_drive_power(")
        output_end = source.index("double forward_inches_since", output_start)
        output_block = source[output_start:output_end]
        self.assertIn("navigation_stop_requested.load", output_block)
        self.assertIn("drive_output_mutex", output_block)
        self.assertIn("stop_drive_motors_unlocked();", output_block)
        stop_motor_start = source.index("void stop_drive_motors()")
        stop_motor_end = source.index(
            "bool blocking_motion_abort_requested", stop_motor_start
        )
        stop_motor = source[stop_motor_start:stop_motor_end]
        self.assertIn("drive_output_mutex", stop_motor)
        self.assertIn("stop_drive_motors_unlocked();", stop_motor)

    def test_start_pose_edit_cancels_active_motion_before_reset(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        setter_start = source.index("bool localization_set_runtime_start_pose(")
        setter_end = source.index(
            "void localization_get_runtime_start_pose", setter_start
        )
        setter = source[setter_start:setter_end]
        cancel = setter.index("navigation_stop_requested.store(true")
        brake = setter.index("stop_drive_motors();")
        revoke = setter.index("navigation_api_initialized = false;")
        self.assertLess(cancel, brake)
        self.assertLess(brake, revoke)

        abort_start = source.index("bool blocking_motion_abort_requested(")
        abort_end = source.index("bool motion_pose_invalid", abort_start)
        abort_block = source[abort_start:abort_end]
        self.assertNotIn("navigation_api_initialized &&", abort_block)

        output_start = source.index("void set_physical_drive_power(")
        output_end = source.index("double forward_inches_since", output_start)
        output_block = source[output_start:output_end]
        self.assertNotIn("navigation_api_initialized &&", output_block)

    def test_new_motion_clears_old_stop_before_fresh_preflight_only(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        turn_start = source.index("Result turn_to(")
        go_start = source.index("Result go_straight_to(", turn_start)
        relative_start = source.index("Result drive_relative(", go_start)
        go_pose_start = source.index("Result go_to_pose(", relative_start)
        stop_start = source.index("void stop()", go_pose_start)
        turn = source[turn_start:go_start]
        go = source[go_start:relative_start]
        relative = source[relative_start:go_pose_start]
        go_pose = source[go_pose_start:stop_start]
        for block, preflight_marker in (
            (turn, "public_turn_center_is_safe("),
            (go, "public_straight_segment_is_safe("),
            (relative, "public_straight_segment_is_safe("),
            (go_pose, "public_straight_segment_is_safe("),
        ):
            self.assertEqual(
                block.count("navigation_stop_requested.store(false"), 1
            )
            self.assertLess(
                block.index("navigation_stop_requested.store(false"),
                block.index("localization_telemetry_update();"),
            )
            self.assertLess(
                block.index("localization_telemetry_update();"),
                block.index(preflight_marker),
            )

    def test_every_estimator_uses_the_runtime_start_pose(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertNotIn("init_pose(pose, localization::kEnteredStartPose)", source)
        self.assertIn("init_pose(telemetry_pose, runtime_start_pose)", source)
        self.assertIn("localization_set_runtime_start_pose", source)

    def test_stationary_encoders_cannot_erase_absolute_corrections(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertNotIn("hardware_is_zeroed", source)
        self.assertNotIn("start_pose_error_in > 2.0", source)
        self.assertIn("void localization_telemetry_reset()", source)

    def test_controller_pose_editor_is_fail_closed(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn("update_runtime_pose_editor", source)
        self.assertIn("pose_editor_active ? 0", source)
        self.assertIn("A=SAVE", source)
        self.assertNotIn("poll_host_commands();", source)

    def test_all_startup_motion_is_disabled(self):
        source = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        motion_flags = re.findall(
            r"constexpr bool (RUN_(?:STARTUP|COMPETITION)_[A-Z0-9_]+) = "
            r"(true|false);",
            source,
        )
        self.assertGreaterEqual(len(motion_flags), 10)
        self.assertEqual(
            [],
            [name for name, value in motion_flags if value != "false"],
            "every startup/competition diagnostic motion flag must be false",
        )
        self.assertIn(
            "constexpr bool ENABLE_CONTROLLER_MOTION_DIAGNOSTICS = false;",
            source,
        )

    def test_diagnostics_reuse_single_gps_and_imu_hardware_owners(self):
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        autons = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        subsystems = (ROOT / "include" / "subsystems.hpp").read_text(
            encoding="utf-8"
        )
        combined = main + autons
        self.assertEqual(1, combined.count("pros::Gps gps_7("))
        self.assertNotRegex(combined, r"pros::Gps\s+(?!gps_7\b)\w+\s*\(")
        self.assertNotRegex(combined, r"pros::Imu\s+\w+\s*\(")
        self.assertIn("auto& gps = gps_7;", main)
        self.assertIn("auto& gps = gps_7;", autons)
        self.assertIn("auto& imu = chassis.imu;", main)
        self.assertIn("extern pros::Gps gps_7;", subsystems)

    def test_navigation_api_and_competition_callback_fail_closed(self):
        header = (ROOT / "include" / "navigation.hpp").read_text(encoding="utf-8")
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        for symbol in ("bool init(", "void update()", "Pose current_pose()",
                       "SensorHealth sensor_health()",
                       "std::size_t copy_path(", "std::size_t path_size()",
                       "void clear_path()",
                       "Result turn_to(", "Result go_straight_to(",
                       "Result drive_relative(", "void stop()"):
            self.assertIn(symbol, header)
        self.assertIn("double gps_gyro_z", header)
        self.assertIn("bool gps_gyro_valid", header)
        self.assertIn("bool forward_distance_api_ok", header)
        self.assertIn("double ai_horizontal_range_in", header)
        self.assertIn("double ai_3d_range_in", header)
        self.assertIn("double ai_bearing_right_deg", header)
        self.assertIn("double ai_elevation_deg", header)
        self.assertIn("std::uint32_t ai_geometry_age_ms", header)
        self.assertIn("kUnsafePath", header)
        self.assertIn("kPathCapacity = 512", header)
        self.assertIn("record_navigation_path(pose, now);", source)
        self.assertIn("reset_navigation_path(telemetry_pose, pros::millis())", source)
        self.assertIn("navigation_path_count - write_count", source)
        self.assertGreaterEqual(source.count("navigation_path_count = 0;"), 3)
        self.assertIn("gps_7.get_gyro_rate_z()", source)
        self.assertIn("health.gps_gyro_valid = std::isfinite(gps_gyro_z)", source)
        self.assertIn("kGpsHeadingGain = 0.0", config)
        self.assertIn('pose.gps_reject = "position_only";', source)
        self.assertIn(
            'std::strcmp(snapshot.gps_reject, "position_only") == 0',
            source,
        )
        self.assertIn("abort=finish_plane_miss", source)
        self.assertIn("RUN_COMPETITION_DIAGNOSTIC_ROUTE = false", main)
        self.assertIn("ENABLE_CONTROLLER_MOTION_DIAGNOSTICS = false", main)
        self.assertIn("navigation::stop();", main)

    def test_public_go_to_pose_is_curved_bounded_and_heading_complete(self):
        header = (ROOT / "include" / "navigation.hpp").read_text(encoding="utf-8")
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("Result go_to_pose(double target_x_in", header)
        self.assertIn("kNavigationCurvedPathCorridorIn = 6.0", config)
        self.assertIn('"navigation_go_to_pose_drive"', source)
        self.assertIn('"navigation_go_to_pose_heading"', source)
        self.assertIn('"navigation_go_to_pose_preturn"', source)
        self.assertIn("kNavigationGoToPosePreturnThresholdDeg = 100.0", source)
        self.assertIn("timeout_ms - predrive_elapsed_ms", source)
        self.assertIn("timeout_ms - elapsed_ms", source)
        self.assertIn("curved_corridor_envelope_in", source)
        self.assertGreaterEqual(source.count("left_deg=%.2f right_deg=%.2f"), 2)
        self.assertGreaterEqual(source.count("gps_heading=%.2f gps_error=%.2f"), 2)
        self.assertIn("NAV_DROPOUT event=injected", source)
        self.assertIn("NAV_DROPOUT event=complete", source)
        self.assertIn("NAV_OBSTACLE event=preflight_reject", source)
        self.assertIn("preflight_distance_in > localization::kForwardObstacleStopIn", source)
        self.assertIn("RUN_STARTUP_NAVIGATION_DROPOUT_TEST = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_OBSTACLE_TEST = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_STRAIGHT_QUALIFICATION = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_BIDIRECTIONAL_QUALIFICATION = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_TURN_QUALIFICATION = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_TURN_RECOVERY = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_MIRRORED_CURVE = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_OBSTACLE_APPROACH = false", main)
        self.assertIn("RUN_STARTUP_NAVIGATION_REVERSE_RECOVERY = false", main)
        self.assertIn("RUN_STARTUP_SLIDER_SLOW_TEST = false", main)
        self.assertIn("RUN_STARTUP_ALL_MECHANISMS_TEST = false", main)
        self.assertIn("RUN_STARTUP_CLAMP_PICTURE_TEST = false", main)
        self.assertIn("RUN_STARTUP_TOGGLE_GOAL_EXAMPLE = false", main)
        self.assertIn("RUN_STARTUP_TOGGLE_GOAL_CONTINUE = false", main)
        self.assertIn("NAV_STRAIGHT event=leg_done", source)
        self.assertIn("NAV_BIDIR event=leg_done", source)
        self.assertIn("kLegsIn{{6.0, -6.0, 10.0, -10.0}}", source)
        self.assertIn("NAV_TURN_QUAL event=turn_done", source)
        self.assertIn("kHeadingsDeg{{45.0, 0.0, 315.0, 0.0}}", source)
        self.assertIn("NAV_MIRROR event=curve_done", source)
        self.assertIn("NAV_OBS_APPROACH event=complete", source)
        self.assertIn("NAV_REVERSE_RECOVERY event=complete", source)
        self.assertIn("SLIDER_TEST event=complete", source)
        self.assertIn("MECH_TEST event=complete", source)
        self.assertIn("CLAMP_TEST event=complete safe_state=0", source)
        self.assertIn("TOGGLE_GOAL event=complete", source)
        self.assertIn("kCumulativeTargetsIn{{3.0, 6.0, 9.0}}", source)
        self.assertIn("abort=path_corridor", source)
        self.assertIn("Result drive_relative(double distance_in", header)
        self.assertIn('"navigation_relative_reverse"', source)
        self.assertIn("drive_direction < 0 ? -1.0 : 1.0", source)
        self.assertIn("motion_sign *\n        0.5 *", source)
        self.assertIn(
            "stop_for_forward_obstacle && motion_sign > 0.0", source
        )
        self.assertIn(
            "timeout_ms - predrive_elapsed_ms,",
            source,
        )
        self.assertIn(
            "localization::kNavigationCurvedPathCorridorIn",
            source,
        )
        self.assertGreaterEqual(source.count("max_power <= 0"), 2)
        self.assertIn("bool navigation_api_initialized = false;", source)
        self.assertIn(
            "if (!copy_telemetry_snapshot(snapshot, navigation_initialized)",
            source,
        )
        self.assertIn(
            "navigation_api_initialized = telemetry_pose_initialized", source
        )
        self.assertIn("bool stationary_for_navigation_init()", source)
        self.assertIn("kNavigationInitSettleMs = 250", source)
        self.assertIn("kNavigationInitMaxWheelMotionIn = 0.10", source)
        self.assertIn("kNavigationInitMaxImuMotionDeg = 1.0", source)
        self.assertIn("public_straight_segment_is_safe", source)
        self.assertIn("localization::kNavigationFieldElementToleranceIn", source)
        self.assertIn("position_error_envelope_in", source)
        self.assertIn('reject_reason = "pose_uncertainty";', source)
        self.assertIn("double projected_position_error_envelope_in(", source)
        self.assertIn("additional_travel_in * per_inch_growth", source)
        self.assertIn("projected_path_error_envelope_in", source)
        self.assertIn("kNavigationProvisionalWallClearanceIn", source)
        self.assertIn("kNavigationProvisionalGoalClearanceIn", source)
        self.assertIn("kNavigationFieldElementToleranceIn = 1.0", config)
        self.assertIn("return Result::kUnsafePath;", source)
        self.assertIn('case Result::kUnsafePath: return "unsafe_path";', source)
        public_go_to = source.index("Result go_straight_to(")
        go_to_safety = source.index("public_straight_segment_is_safe(", public_go_to)
        go_to_turn = source.index("fused_turn_to_heading(", go_to_safety)
        self.assertLess(go_to_safety, go_to_turn)
        go_to_end = source.index("void stop()", public_go_to)
        go_to_block = source[public_go_to:go_to_end]
        self.assertIn("command_started_ms = pros::millis()", go_to_block)
        self.assertIn("drive_timeout_ms = timeout_ms - elapsed_ms", go_to_block)
        self.assertIn("drive_timeout_ms);", go_to_block)
        init_start = source.index("bool init(double start_x_in")
        init_end = source.index("void update()", init_start)
        init_block = source[init_start:init_end]
        self.assertIn("for (int attempt = 1; attempt <= 3; ++attempt)",
                      init_block)
        self.assertIn("NAV_INIT abort=stationary_preflight_failed", init_block)
        self.assertLess(init_block.index("!std::isfinite(start_x_in)"),
                        init_block.index("stationary_for_navigation_init()"))
        health_start = source.index("SensorHealth sensor_health()")
        health_end = source.index("Result turn_to(", health_start)
        health_block = source[health_start:health_end]
        self.assertNotIn("localization_telemetry_update();", health_block)
        self.assertIn("if (!copy_telemetry_snapshot(snapshot, navigation_initialized)) return health;",
                      health_block)

    def test_navigation_init_carries_start_placement_uncertainty(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        header = (ROOT / "include" / "navigation.hpp").read_text(
            encoding="utf-8"
        )
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("kNavigationDefaultStartPositionErrorIn = 1.0", config)
        self.assertIn("double start_position_error_in);", header)
        init_start = source.index("bool init(double start_x_in")
        init_end = source.index("void update()", init_start)
        init = source[init_start:init_end]
        self.assertIn(
            "localization::kNavigationDefaultStartPositionErrorIn", init
        )
        self.assertIn(
            "telemetry_pose.absolute_position_base_error_in = "
            "start_position_error_in;",
            init,
        )
        self.assertIn("update_position_error_envelope(telemetry_pose);", init)

    def test_outage_heading_allowance_is_not_command_return_residual(self):
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("kDeadReckoningHeadingEnvelopeDeg = 2.0", config)
        self.assertRegex(config, r"no\s*\n// external heading truth")
        self.assertIn("commanded return residual", config)
        self.assertIn(
            "localization::kDeadReckoningHeadingEnvelopeDeg", source
        )
        self.assertNotIn("kDeadReckoningHeadingEnvelopeDeg = 0.91", config)

    def test_outage_analyses_read_production_constants(self):
        scripts = (
            ROOT / "tools" / "analyze_outage_distance_budget.py",
            ROOT / "tools" / "simulate_gps_recovery.py",
            ROOT / "tools" / "simulate_route_safety.py",
            ROOT / "tools" / "simulate_navigation_robustness.py",
            ROOT / "reports" / "sensor_campaign_2026-08-23"
            / "analyze_dead_reckoning_outage.py",
        )
        for script in scripts:
            source = script.read_text(encoding="utf-8")
            self.assertIn(
                'setting("kDeadReckoningHeadingEnvelopeDeg")', source,
                script.name,
            )

    def test_generated_outage_artifacts_match_production_constants(self):
        config = (ROOT / "include" / "localization_config.hpp").read_text(
            encoding="utf-8"
        )
        number = lambda name: float(
            re.search(rf"{name}\s*=\s*([\d.]+)", config).group(1)
        )
        heading = number("kDeadReckoningHeadingEnvelopeDeg")
        growth = math.hypot(
            number("kDeadReckoningScaleEnvelopeFraction"),
            math.tan(math.radians(heading)),
        )
        report = ROOT / "reports" / "sensor_campaign_2026-08-23"
        outage = json.loads(
            (report / "gps_outage_distance_budget_summary.json").read_text()
        )
        route = json.loads(
            (report / "route_safety_stress_summary.json").read_text()
        )
        dead_reckoning = json.loads(
            (report / "dead_reckoning_outage_summary.json").read_text()
        )
        self.assertAlmostEqual(
            growth, outage["combined_error_growth_in_per_in_travel"], places=12
        )
        self.assertAlmostEqual(
            growth, route["dead_reckoning_error_growth_per_travel_in"], places=12
        )
        self.assertEqual(
            heading,
            dead_reckoning["provisional_heading_controller_allowance_deg"],
        )


if __name__ == "__main__":
    unittest.main()
