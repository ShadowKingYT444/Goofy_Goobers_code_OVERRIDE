import importlib.util
import unittest
from pathlib import Path

from cobs import cobs


ROOT = Path(__file__).resolve().parents[1]
SERVER_PATH = ROOT / "tools" / "lidar_bar_server.py"
SPEC = importlib.util.spec_from_file_location("lidar_bar_server", SERVER_PATH)
SERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SERVER)


class VisionTelemetryTests(unittest.TestCase):
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
            "area=1600.0 mean_edge=40.02 range=3.98 edge_ratio=1.05 fill=0.95 bearing=-10.25 repeat=0 "
            "geometry_age=0 valid=1 "
            "reason=shadow_valid"
        )
        self.assertTrue(frame["valid"])
        self.assertEqual(frame["tag_id"], 3)
        self.assertEqual(frame["corners"], [100, 80, 140, 82, 138, 122, 98, 120])
        self.assertAlmostEqual(frame["bearing_deg"], -10.25)
        self.assertAlmostEqual(frame["mean_edge_px"], 40.02)
        self.assertAlmostEqual(frame["range_estimate_in"], 3.98)
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
        self.assertIn("visionFrame.mean_edge_px", html)
        self.assertIn("visionCandidate.ai_range_residual", html)
        self.assertIn("visionCandidate.ai_innovation", html)
        self.assertIn("drawAiVisionHypotheses", html)
        self.assertIn("const headingRad = displayPose.headingRad;", html)
        self.assertNotIn("displayPose.headingDeg", html)
        self.assertIn("aiCameraRightOffsetIn = -10.44", html)
        self.assertIn("horizontalOdometerOffsetBackIn = 5.18", html)
        self.assertIn("START 30.2,34.7", html)
        self.assertIn("Start Pose Help", html)
        self.assertIn("hold Y to edit the exact start pose", html)
        self.assertIn("Effective Track", html)
        self.assertIn("Rear Wheel at 15 deg", html)
        self.assertIn("L+1 / R-1 Turn", html)
        self.assertIn("const rightEncoderSign = 1", html)
        self.assertIn("const trackWidthIn = 12.0086", html)

    def test_payload_has_separate_onboard_webcam_and_vision_channels(self):
        body = SERVER.payload()
        self.assertIn("onboard_pose", body)
        self.assertIn("camera_pose", body)
        self.assertIn("vision", body)
        self.assertIsNot(body["onboard_pose"], body["camera_pose"])


class FirmwareSafetyInvariantTests(unittest.TestCase):
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
        self.assertIn("kAiMaxPositionInnovationIn = 8.0", config)
        self.assertIn("kAiMaxPositionStepIn = 0.75", config)
        self.assertIn("kAiHeadingGain = 0.0", config)
        self.assertIn("kAiRequiredConsistentObservations = 3", config)
        self.assertIn("kAiRequiredReacquisitionObservations = 12", config)
        self.assertIn("kAiMaxReacquisitionInnovationIn = 24.0", config)
        self.assertIn("proven_reacquisition", source)
        self.assertIn("kSideOdomRearOffsetIn = 5.18", config)
        self.assertIn("kFusedTurnMinPower = 18.0", source)
        self.assertIn(
            "kFusedTurnMinPowerErrorDeg = kFusedTurnToleranceDeg", source
        )
        self.assertIn("abort=turn_stall", source)
        self.assertIn("telemetry_pose = pose", source)
        self.assertIn("kMaxSameSideMotorSpreadDeg = 15.0", source)

    def test_rear_offset_is_applied_only_to_an_accepted_side_delta(self):
        source = (ROOT / "src" / "autons.cpp").read_text(encoding="utf-8")
        self.assertIn("bool side_delta_accepted = false;", source)
        self.assertIn("const double delta_side_center_in = side_delta_accepted", source)
        self.assertIn("pose.side_odom_ready = false;", source)

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

    def test_distance_port_nine_is_not_also_a_motor(self):
        subsystems = (ROOT / "include" / "subsystems.hpp").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertNotIn("slider_left(-9)", subsystems)

    def test_ai_vision_retries_after_slow_device_startup(self):
        source = (ROOT / "src" / "ai_vision_localization.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include" / "localization_config.hpp").read_text(encoding="utf-8")
        self.assertIn("now - last_init_attempt_ms >= 1000", source)
        self.assertIn("ai_vision_shadow_initialize();", source)
        self.assertIn("for (std::uint8_t port = 1; port <= 21; ++port)", source)
        self.assertIn("active_ai_vision_port", source)
        self.assertIn("kAiVisionPort = 20", config)
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        self.assertIn('"AI P%u tag=%d %s"', main)
        self.assertIn('"AI P%u T%d %-3s"', main)

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
        self.assertIn("kDriveTrackWidthIn = 12.0086", config)
        self.assertIn("kRightEncoderSign = 1.0", source)
        self.assertIn("kLidarThetaScale = 0.926770", config)

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
        for flag in (
            "RUN_STARTUP_LIDAR_CALIBRATION",
            "RUN_STARTUP_FORWARD_CALIBRATION",
            "RUN_STARTUP_AI_VISION_SCAN",
            "RUN_STARTUP_SCAN_RECOVERY",
        ):
            self.assertIn(f"constexpr bool {flag} = false;", source)


if __name__ == "__main__":
    unittest.main()
