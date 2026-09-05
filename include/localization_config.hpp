#pragma once

#include <array>
#include <cstdint>

namespace localization {

enum class GoalColor {
  kNeutral,
  kRed,
  kBlue,
};

enum class RobotSide {
  kLeft,
  kRight,
};

struct FieldPose {
  double x_in;
  double y_in;
  double heading_deg;
};

struct GoalTagLandmark {
  const char* name;
  std::uint8_t tag_id;
  GoalColor color;
  double x_in;
  double y_in;
};

// EDIT THESE THREE VALUES before starting an autonomous routine. Coordinates
// describe the robot's rotation center, not a bumper or wheel edge.
inline constexpr FieldPose kEnteredStartPose{
    30.18,  // Post-20-foot Tag 3 position correction at retained IMU heading.
    34.70,
    151.65,
};

inline constexpr double kTileSizeIn = 24.0;
inline constexpr double kNominalFieldHalfSpanIn = 72.0;
inline constexpr double kPhysicalWallHalfSpanIn = 70.2;
// Provisional public straight-route exclusions. These reuse the conservative
// clearances exercised by the live GPS route diagnostic. They reduce risk but
// are not a substitute for measuring the current robot's front/back/side
// extents from its rotation center, especially if a mechanism expands.
// Override v1.1 T5 permits Field Elements to vary up to +/-1 inch from nominal;
// map safety consumes that full allowance in the hazardous direction.
inline constexpr double kNavigationFieldElementToleranceIn = 1.0;
inline constexpr double kNavigationProvisionalWallClearanceIn = 14.0;
inline constexpr double kNavigationProvisionalGoalClearanceIn = 16.0;
inline constexpr double kNavigationMinimumGoalEscapeClearanceIn = 12.0;
// Live curved-route tuning reached 3.39 in of cross-track displacement before
// converging. Reserve a larger six-inch centerline corridor in public map
// checks so the curved API never inherits straight-segment clearance claims.
inline constexpr double kNavigationCurvedPathCorridorIn = 6.0;
// Until a team measures its fixture/tape placement repeatability, do not let
// the convenient three-coordinate init overload imply perfect field position.
inline constexpr double kNavigationDefaultStartPositionErrorIn = 1.0;
// Public pose/health readers consume a published estimator snapshot. If its
// owning 20-ms update task stalls, never present that frozen snapshot as live.
inline constexpr std::uint32_t kNavigationSnapshotMaxAgeMs = 250;

// Differential/tank drivetrain geometry: eight physical 2.75-inch omni wheels,
// four per side, driven by two gear-coupled motors per side. The rollers permit
// passive lateral sliding but the drivetrain cannot command strafe. Nine GPS-
// referenced 2/5/10-inch trials at 10/20/35 RPM on 2026-08-23 fitted an
// apparent encoder scale of 0.884748 (fit RMSE 0.215 in). P7 was the only
// translation reference, not tape/laser truth, so this remains provisional.
// Fold that gearing,
// carpet compression, and effective rolling radius into the diameter consumed
// by both EZ-Template and the fused odometry, while retaining the physical
// wheel size explicitly for documentation.
inline constexpr double kDrivePhysicalWheelDiameterIn = 2.75;
inline constexpr double kDriveEncoderDistanceScale = 0.8847477281;
inline constexpr double kDriveWheelDiameterIn =
    kDrivePhysicalWheelDiameterIn * kDriveEncoderDistanceScale;
inline constexpr int kDriveRpm = 450;
inline constexpr double kDriveExternalRatio = 1.0;
// The 12.0086-in angular calibration was fitted while encoder travel used the
// physical 2.75-in wheel scale. Differential-drive heading depends on the
// wheel-diameter/track-width ratio, so applying the straight-distance scale to
// the diameter alone would under-report encoder turns by 11.5%. Scale both
// linear parameters together to preserve the live-validated angular ratio.
inline constexpr double kDriveTrackWidthAtPhysicalWheelScaleIn = 12.0086;
inline constexpr double kDriveTrackWidthIn =
    kDriveTrackWidthAtPhysicalWheelScaleIn * kDriveEncoderDistanceScale;
// Empirical GPS-outage envelope from the 2026-08-23 live campaigns. Per-speed
// encoder scale varied at most 2.1554% around the fitted model. There is no
// external heading truth; use a provisional 2.0-degree heading/controller
// allowance, rounded above the 1.945-degree live straight-run excursion rather
// than mislabeling a 0.91-degree commanded return residual as IMU accuracy.
// This is a transparent travel-based engineering allowance, not covariance;
// collision/slip
// and placement error beyond the init bound are additional. Because P7
// supplied the straight-distance reference, this variation term also excludes
// any common systematic bias in the provisional wheel scale.
inline constexpr double kDeadReckoningScaleEnvelopeFraction = 0.0215542987;
inline constexpr double kDeadReckoningHeadingEnvelopeDeg = 2.0;

// Current navigation sensor geometry. The Distance sensor on port 1 points
// along the robot's negative/rear axis, not along +forward. Never use its
// reading as a forward collision or Toggle-range measurement. The GPS lens
// points robot-right, 90 degrees clockwise from forward, and is mounted 6
// inches right and 6 inches behind the robot's rotation center.
inline constexpr std::uint8_t kRearDistancePort = 1;
inline constexpr bool kDistanceSensorFacesForward = false;
inline constexpr double kForwardObstacleStopIn = 8.0;
// A dangerously close return stops on the first poll. A return in the wider
// 4-8 inch warning band must persist beyond one nominal sensor refresh so a
// single transient does not abort a full-speed route. At 36 in/s, 70 ms adds
// 2.52 in of travel and still reaches the immediate-stop band before contact.
inline constexpr double kForwardObstacleImmediateStopIn = 4.0;
inline constexpr std::uint32_t kForwardObstacleConfirmationMs = 70;
inline constexpr long kForwardObstacleMinRangeMm = 20;
inline constexpr long kForwardObstacleMaxRangeMm = 2000;
inline constexpr long kForwardObstacleConfidenceAvailableMm = 200;
inline constexpr long kForwardObstacleMinConfidence = 20;
inline constexpr std::uint8_t kGpsPort = 7;
inline constexpr double kGpsRightOffsetIn = 6.0;
inline constexpr double kGpsForwardOffsetIn = -6.0;
inline constexpr double kGpsSensorHeadingOffsetCwDeg = 90.0;
// GPS is an absolute, lower-rate correction. Wheel encoders and the IMU still
// propagate every control tick. Reject degraded visual fixes and apply bounded
// corrections so a reacquiring sensor cannot teleport a moving robot.
inline constexpr std::uint32_t kGpsCorrectionPeriodMs = 100;
// A live 15-degree out/return produced a false 15.63-inch GPS displacement as
// its reported RMS error degraded. Require a 1.2-second stationary cluster;
// unlike a per-frame step check, all samples must remain near the first member
// of the cluster, so slow visual drift cannot masquerade as consistency.
inline constexpr int kGpsRequiredConsistentObservations = 12;
inline constexpr double kGpsMaxReportedErrorIn = 0.75;
inline constexpr double kGpsMaxObservationStepIn = 0.75;
inline constexpr double kGpsMaxObservationHeadingStepDeg = 2.0;
inline constexpr double kGpsMaxPositionInnovationIn = 3.0;
inline constexpr int kGpsRequiredReacquisitionObservations = 30;
inline constexpr double kGpsMaxReacquisitionInnovationIn = 6.0;
inline constexpr double kGpsPositionGain = 0.20;
inline constexpr double kGpsMaxPositionStepIn = 0.50;
inline constexpr double kGpsPositionDeadbandIn = 0.05;
// The 2026-08-23 rotation sweep found orientation-dependent GPS heading
// excursions up to 8.55 degrees even when reported position error was 0.43 in.
// The competition-length IMU is much more repeatable, so reject a GPS heading
// that disagrees by more than 5 degrees and never steer heading corrections
// into an active turn.
inline constexpr double kGpsMaxHeadingInnovationDeg = 5.0;
// A replay of the live orientation sweep showed that repeated low-RMS P7
// headings inside the innovation gate can accumulate 2.96 degrees of bias
// against P6. Keep P7 as position-only until surveyed long-duration truth
// demonstrates that its heading improves on P6 rather than injecting view bias.
inline constexpr double kGpsHeadingGain = 0.0;
inline constexpr double kGpsMaxHeadingStepDeg = 0.0;
inline constexpr double kGpsHeadingDeadbandDeg = 0.20;
// The fixed observation cluster is a stopped-robot consistency test. VEX also
// recommends pausing for accurate GPS fixes. Make that assumption explicit so
// slow physical motion cannot be mistaken for a stable absolute location.
inline constexpr double kGpsMaxCorrectionLinearSpeedInS = 0.50;
inline constexpr double kGpsMaxCorrectionAngularRateDegS = 12.0;
// The GPS API exposes no optical-frame timestamp. Exact repeated pose tuples
// are fine while stationary, but not after independent robot motion.
inline constexpr double kGpsMaxRepeatedObservationDriveIn = 0.50;
inline constexpr double kGpsMaxRepeatedObservationHeadingDeg = 2.0;

// The 2-inch VEX tracking wheel on P15 is mounted at the robot rotation center
// and measures longitudinal/forward displacement. A live commanded-forward
// run changed P15 coherently; it must not be injected as lateral displacement.
inline constexpr double kForwardOdomWheelDiameterIn = 2.0;
// The first run resembled a 5:7 transfer, but a repeat over-read severely and
// continued spinning after the drivetrain stopped. No fixed transfer ratio is
// valid until the two-wheel coupling and ground contact are repaired.
inline constexpr double kForwardOdomSensorToGroundRatio = 1.0;
// Legacy name retained only for the disabled lateral estimator path.
inline constexpr double kSideOdomWheelDiameterIn = kForwardOdomWheelDiameterIn;
inline constexpr double kSideOdomRawToRobotRightSign = 1.0;
// The existing estimator slot is lateral-only. Keep it disabled until P15 is
// integrated through a dedicated longitudinal-odometer path. Port 5 remains
// separate claw-arm feedback; old P5 odometry trials are irrelevant.
inline constexpr bool kSideOdomEnabled = false;
inline constexpr double kSideOdomRearOffsetIn = 0.0;

// Pooled eight-segment LiDAR/IMU slope was 1.079016. Scale raw wall theta so
// its change matches the independently measured IMU change (fitted RMSE 0.42°).
inline constexpr double kLidarThetaScale = 0.926770;

inline constexpr RobotSide kLidarSide = RobotSide::kLeft;
inline constexpr RobotSide kAiVisionSide = RobotSide::kRight;

// Live inventory on 2026-09-01 verified AI Vision on Smart Port 6.
// Runtime auto-discovery remains available as a wiring-change fallback.
inline constexpr std::uint8_t kAiVisionPort = 6;
inline constexpr double kAiImageWidthPx = 320.0;
inline constexpr double kAiImageHeightPx = 240.0;
inline constexpr double kAiHorizontalFovDeg = 74.0;
inline constexpr double kAiVerticalFovDeg = 63.0;
inline constexpr double kAiFocalLengthXPx = 212.34;
inline constexpr double kAiFocalLengthYPx = 195.82;
inline constexpr double kAiGoalFaceOffsetIn = 5.61 / 2.0;
// The official Override field assembly uses VEX's medium Circle21h7 Goal tag.
// Its printed outer square is 0.875 in (seven 0.125-in cells), but the P6 API's
// returned corner quad spans the inner five-cell detection square: 0.625 in.
// A 2026-08-25 tape check at 18.0 in gave 18.60 in from these exact corners
// with the five-cell span; using the printed outer square incorrectly gave
// 26.03 in. Keep this semantic distinction explicit in the estimator.
inline constexpr double kAiTagDetectedSizeIn = 0.625;
// This replacement robot's P6 AI Vision sensor points robot-forward. Its
// lens-to-rotation-center translation has not yet been measured, so keep a
// neutral origin and accept only very small, repeatedly consistent landmark
// innovations. The tight two-inch gate and 0.15-inch step prevent an unknown
// fixture offset or duplicate-ID association from taking over odometry.
inline constexpr double kAiCameraForwardOffsetIn = 0.0;
inline constexpr double kAiCameraRightOffsetIn = 0.0;
inline constexpr double kAiCameraYawRightDeg = 0.0;
inline constexpr bool kAiVisionPoseCorrectionEnabled = true;
inline constexpr int kAiRequiredConsistentObservations = 5;
// A larger but still plausible innovation may be recovered only after a much
// longer run of the same unambiguous physical Goal/face. Corrections remain
// bounded by the normal per-update step, so relocalization cannot teleport.
inline constexpr int kAiRequiredReacquisitionObservations = 20;
inline constexpr std::uint32_t kAiCorrectionPeriodMs = 250;
inline constexpr double kAiMaxPositionInnovationIn = 2.0;
inline constexpr double kAiMaxReacquisitionInnovationIn = 4.0;
inline constexpr double kAiPositionGain = 0.10;
inline constexpr double kAiMaxPositionStepIn = 0.15;
inline constexpr double kAiPositionDeadbandIn = 0.25;
// One tag's range+bearing constrains position only when heading is supplied by
// IMU/LiDAR. Without planar tag-pose solving, simultaneous camera position and
// heading correction is underconstrained and can converge to a false point on
// the same observation circle.
inline constexpr double kAiHeadingGain = 0.0;
inline constexpr double kAiMaxHeadingStepDeg = 0.5;
inline constexpr double kAiHeadingDeadbandDeg = 0.10;
inline constexpr std::uint32_t kAiVisionPollPeriodMs = 100;
// The API has no optical-frame timestamp. If exact integer corners remain
// unchanged despite independently observed robot motion, treat the geometry as
// cached/stale rather than letting repeated polls establish consistency.
inline constexpr double kAiMaxRepeatedGeometryDriveIn = 0.50;
inline constexpr double kAiMaxRepeatedGeometryHeadingDeg = 2.0;
inline constexpr double kAiMinTagEdgePx = 5.0;
inline constexpr double kAiMaxTagEdgeRatio = 1.8;
inline constexpr double kAiMinTagFillRatio = 0.45;
inline constexpr double kAiMinCandidateWinnerMarginDeg = 8.0;
// Associate a Goal face using both bearing and horizontal range. The score is
// expressed in bearing-equivalent degrees; face ambiguity is rejected rather
// than silently treating all four faces on one Goal as interchangeable.
inline constexpr double kAiRangeResidualScoreDegPerIn = 0.5;
inline constexpr double kAiMinFaceWinnerMarginScore = 2.0;
inline constexpr double kAiMaxCandidateBearingResidualDeg = 40.0;
inline constexpr double kAiMinUsableRangeIn = 4.0;
inline constexpr double kAiMaxUsableRangeIn = 96.0;
inline constexpr double kAiMaxCandidateRangeResidualIn = 8.0;

// Goal-center coordinates from official Override v1.1 Appendix A field-element
// drawing, transformed from bottom-left drawing coordinates into this project's
// center-origin frame (+X toward the 0-degree wall, +Y toward the red side).
// IDs come from the official A16 AprilTag Numbering/Locations drawing. Each
// Goal carries four copies of its ID; IDs 1-4 each identify two possible Goals.
inline constexpr std::array<GoalTagLandmark, 9> kGoalTagLandmarks{{
    {"center", 0, GoalColor::kNeutral, 0.0, 0.0},
    {"top_red_neutral", 4, GoalColor::kNeutral, 47.10, 23.55},
    {"top_blue_alliance", 3, GoalColor::kBlue, 47.10, -23.54},
    {"upper_red_neutral", 1, GoalColor::kNeutral, 23.55, 47.10},
    {"upper_blue_alliance", 2, GoalColor::kBlue, 23.55, -47.09},
    {"lower_red_alliance", 2, GoalColor::kRed, -23.54, 47.10},
    {"lower_blue_neutral", 1, GoalColor::kNeutral, -23.54, -47.09},
    {"bottom_red_alliance", 3, GoalColor::kRed, -47.09, 23.55},
    {"bottom_blue_neutral", 4, GoalColor::kNeutral, -47.09, -23.54},
}};

}  // namespace localization
