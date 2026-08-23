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

// Differential/tank drivetrain geometry: eight 2.75-inch (220 mm travel) omni
// wheels, four per side, driven by two gear-coupled motors per side. The rollers
// permit passive lateral sliding but the drivetrain cannot command strafe.
// Track width is the effective least-squares fit from the 2026-07-10 turns.
inline constexpr double kDriveWheelDiameterIn = 2.75;
inline constexpr int kDriveRpm = 450;
inline constexpr double kDriveExternalRatio = 1.0;
inline constexpr double kDriveTrackWidthIn = 12.0086;

// Current navigation sensor geometry. The forward Distance sensor measures
// along the robot's +forward axis. The GPS lens points robot-right, 90 degrees
// clockwise from forward, and is mounted 6 inches right and 6 inches behind
// the robot's rotation center.
inline constexpr std::uint8_t kForwardDistancePort = 1;
inline constexpr std::uint8_t kGpsPort = 7;
inline constexpr double kGpsRightOffsetIn = 6.0;
inline constexpr double kGpsForwardOffsetIn = -6.0;
inline constexpr double kGpsSensorHeadingOffsetCwDeg = 90.0;

// Confirmed 2-inch VEX omni tracking wheel at the rear center, mounted like a
// centered rear license plate and oriented to roll with sideways displacement.
// It is passive: it measures lateral slide/push but does not make the robot
// strafe. Its left/right mount offset is zero; only its rearward lever arm
// contributes wheel travel during pure rotation.
inline constexpr double kSideOdomWheelDiameterIn = 2.0;
inline constexpr double kSideOdomRawToRobotRightSign = 1.0;
// 2026-07-10 paired 12 deg correlation: pooled eight-segment IMU fit using the
// now-confirmed 2-inch tracking-wheel diameter.
// Large-angle live validation supersedes the noisy micro-turn fit. The first
// 80.80-degree turn moved the fit from 2.5665 to 4.43 inches; a second
// 73.74-degree return left 0.97 inches residual, refining the lever to 5.18.
inline constexpr double kSideOdomRearOffsetIn = 5.18;

// Pooled eight-segment LiDAR/IMU slope was 1.079016. Scale raw wall theta so
// its change matches the independently measured IMU change (fitted RMSE 0.42°).
inline constexpr double kLidarThetaScale = 0.926770;

inline constexpr RobotSide kLidarSide = RobotSide::kLeft;
inline constexpr RobotSide kAiVisionSide = RobotSide::kRight;

// Current live auto-discovery on 2026-07-10 verified AI Vision on Smart Port 20.
inline constexpr std::uint8_t kAiVisionPort = 20;
inline constexpr double kAiImageWidthPx = 320.0;
inline constexpr double kAiImageHeightPx = 240.0;
inline constexpr double kAiHorizontalFovDeg = 74.0;
inline constexpr double kAiVerticalFovDeg = 63.0;
inline constexpr double kAiFocalLengthXPx = 212.34;
inline constexpr double kAiFocalLengthYPx = 195.82;
inline constexpr double kAiGoalFaceOffsetIn = 5.61 / 2.0;
// Effective outer tag size from the reversible 1.91-inch parallax trial on
// 2026-07-10. This makes corner-derived range approximately 14.9 inches at
// the calibration pose; range remains confidence/gating data, not hard truth.
inline constexpr double kAiTagOuterSizeIn = 1.05;
// Front-right camera extrinsic from the reversible 8-degree pure-rotation
// Tag 3 fit. Position correction is enabled with bounded innovation/step gates;
// heading correction remains disabled until planar tag pose is implemented.
inline constexpr double kAiCameraForwardOffsetIn = 6.75;
inline constexpr double kAiCameraRightOffsetIn = -10.44;
inline constexpr double kAiCameraYawRightDeg = 90.0;
inline constexpr bool kAiVisionPoseCorrectionEnabled = true;
inline constexpr int kAiRequiredConsistentObservations = 3;
// A larger but still plausible innovation may be recovered only after a much
// longer run of the same unambiguous physical Goal/face. Corrections remain
// bounded by the normal per-update step, so relocalization cannot teleport.
inline constexpr int kAiRequiredReacquisitionObservations = 12;
inline constexpr std::uint32_t kAiCorrectionPeriodMs = 500;
inline constexpr double kAiMaxPositionInnovationIn = 8.0;
inline constexpr double kAiMaxReacquisitionInnovationIn = 24.0;
inline constexpr double kAiPositionGain = 0.25;
inline constexpr double kAiMaxPositionStepIn = 0.75;
inline constexpr double kAiPositionDeadbandIn = 0.10;
// One tag's range+bearing constrains position only when heading is supplied by
// IMU/LiDAR. Without planar tag-pose solving, simultaneous camera position and
// heading correction is underconstrained and can converge to a false point on
// the same observation circle.
inline constexpr double kAiHeadingGain = 0.0;
inline constexpr double kAiMaxHeadingStepDeg = 0.5;
inline constexpr double kAiHeadingDeadbandDeg = 0.10;
inline constexpr std::uint32_t kAiVisionPollPeriodMs = 100;
inline constexpr double kAiMinTagEdgePx = 5.0;
inline constexpr double kAiMaxTagEdgeRatio = 1.8;
inline constexpr double kAiMinTagFillRatio = 0.45;
inline constexpr double kAiMinCandidateWinnerMarginDeg = 8.0;
inline constexpr double kAiMaxCandidateBearingResidualDeg = 40.0;
inline constexpr double kAiMinUsableRangeIn = 4.0;
inline constexpr double kAiMaxUsableRangeIn = 96.0;
inline constexpr double kAiMaxCandidateRangeResidualIn = 8.0;

// Goal-center map from FIELD_2d_VIEW.jpg. Each Goal carries four AprilTags
// with the listed ID. IDs 1-4 each identify two possible Goals.
inline constexpr std::array<GoalTagLandmark, 9> kGoalTagLandmarks{{
    {"center", 0, GoalColor::kNeutral, 0.0, 0.0},
    {"top_red_neutral", 2, GoalColor::kNeutral, 47.10, 23.55},
    {"top_blue_alliance", 1, GoalColor::kBlue, 47.10, -23.54},
    {"upper_red_neutral", 3, GoalColor::kNeutral, 23.55, 47.10},
    {"upper_blue_alliance", 4, GoalColor::kBlue, 23.55, -47.09},
    {"lower_red_alliance", 4, GoalColor::kRed, -23.54, 47.10},
    {"lower_blue_neutral", 3, GoalColor::kNeutral, -23.54, -47.09},
    {"bottom_red_alliance", 1, GoalColor::kRed, -47.09, 23.55},
    {"bottom_blue_neutral", 2, GoalColor::kNeutral, -47.09, -23.54},
}};

}  // namespace localization
