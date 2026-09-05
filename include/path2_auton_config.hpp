#pragma once

#include <limits>

// Physical measurements for Path 2. The route remains fail-closed until every
// unset value below is measured and replaced. Coordinates use the production
// navigation frame: +X right, +Y up, 0 degrees along +X, CCW positive.
namespace path2_config {

struct Point {
  double x;
  double y;
};

constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();

constexpr Point kStart{63.0, 0.0};
constexpr Point kToggle{72.0, 0.0};
constexpr Point kGoal{48.0, 24.0};
// Final first rear-claw pickup target from the latest field test.
constexpr Point kStackA{24.5, 26.1};
constexpr Point kStackB{48.0, 48.0};
constexpr Point kFinal{48.0, 60.0};  // Path coordinate (60,-48).

// Blue coordinates are explicit so later blue-side tuning cannot alter red.
// Stack A is the exact origin-opposite of the red Stack-A target.
constexpr Point kBlueStart{0.0, 63.0};
constexpr Point kBlueToggle{0.0, 72.0};
constexpr Point kBlueGoal{24.0, 48.0};
constexpr Point kBlueStackA{-24.5, -26.1};
constexpr Point kBlueStackB{48.0, 48.0};
constexpr Point kBlueFinal{60.0, 48.0};

// Starting placement and one-contact Toggle move. Toggle distance is signed:
// positive means the robot front hits it, negative means the rear hits it.
constexpr double kStartHeadingDeg = 0.0;
constexpr double kBlueStartHeadingDeg = 90.0;
constexpr double kStartPositionErrorIn = 1.0;
constexpr double kToggleSignedDistanceIn = 7.0;
constexpr double kToggleReturnDistanceIn = 6.0;
constexpr int kTogglePower = 75;
constexpr int kToggleReturnPower = 65;
constexpr int kPhase1DrivePower = 75;
// After the first -75-degree turn, reverse one inch farther into the preload
// Goal than the previous 11-inch tune so the mechanism fully reaches it.
constexpr double kPhase1PreloadReverseIn = 12.0;
// Back another 1.5 inches clear of the preload Goal before turning toward
// Stack A. The signed drive direction is selected by the route; this magnitude
// is identical for the origin-opposite blue route.
constexpr double kPhase2GoalClearanceIn = 16.5;
constexpr double kBluePhase2GoalClearanceIn = 16.5;

// Legal navigation poses outside the mapped Goal/wall exclusions.
// The scoring edge is approximately 12 inches from robot center. Stage two
// inches outside contact, then close only that remaining gap.
constexpr Point kGoalStaging{34.0, 24.0};

// Rear docking uses a measured reverse distance followed by a short signed
// open-loop seating bump. Negative bump power drives the robot backward.
constexpr double kGoalDockReverseIn = 2.0;
constexpr int kGoalDockPower = 52;
constexpr int kGoalAlignmentBumpPower = -40;
constexpr unsigned kGoalAlignmentBumpMs = 150;
constexpr double kGoalContactExitIn = 12.0;

constexpr double kStackFastFraction = 0.50;
constexpr int kFastDrivePower = 112;
constexpr int kStackApproachPower = 75;
constexpr int kSlowStackPower = 35;
// Dedicated gentler powers for the exposed stack at path (48,-48).
constexpr int kSecondStackApproachPower = 60;
constexpr int kSecondStackSlowPower = 38;
constexpr int kTurnPower = 112;

// Rear-first pickup geometry. The robot fits the 18-inch starting envelope,
// so its center is 9 inches from the rear edge; the claw reaches one inch
// beyond that edge. Navigation targets the robot center this far before the
// field coordinate so the claw tip, not the chassis center, reaches the cup.
constexpr double kRobotCenterToRearIn = 9.0;
constexpr double kRearClawExtensionIn = 1.0;
constexpr double kRearPickupReachIn =
    kRobotCenterToRearIn + kRearClawExtensionIn;

// Raw signed motor powers, intentionally unset until the desired physical
// direction is confirmed. Either motor may be zero if a sequence uses only
// one roller.
constexpr int kIntakeUpperPower = -127;
constexpr int kIntakeCounterPower = -110;
constexpr int kOuttakeUpperPower = 127;
constexpr int kOuttakeCounterPower = 110;
constexpr int kPreloadPinUpperPower = 0;
constexpr int kPreloadPinCounterPower = 110;
constexpr unsigned kPreloadDepositMs = 350;

// Match the cascade PID's validated stage stopping band. Requiring +/-5 here
// while the controller brakes at +/-16 caused a guaranteed scoring timeout.
constexpr double kLiftReadyToleranceDeg = 16.0;
constexpr unsigned kLiftReadyTimeoutMs = 5000;
constexpr int kSafeFinalLiftStage = 0;
constexpr double kScoreLoweringDeg = 15.0;
constexpr double kStage1ExtraCaptureIn = 1.0;
constexpr int kStage1ExtraCapturePower = 70;
constexpr double kStage1GoalDriveIn = 24.0;
constexpr int kStage1GoalDrivePower = 80;
constexpr int kScoreDropPower = 100;
constexpr unsigned kScoreDropPulseMs = 100;
constexpr unsigned kScoreDropWaitTimeoutMs = 180;
constexpr unsigned kStage1OuttakeLeadMs = 100;
constexpr double kStage1ScoreRetreatIn = 9.75;
constexpr int kStage1ScoreRetreatPower = 95;
constexpr unsigned kStage1HomeWaitMs = 1800;
constexpr int kFirstCupLiftStage = 1;
constexpr int kSecondCupLiftStage = 2;
constexpr double kLoadedTurnClearanceDeg = 250.0;
constexpr unsigned kLoadedTurnClearanceTimeoutMs = 650;
constexpr unsigned kScoreStageReadyTimeoutMs = 400;

constexpr double kStage2ExtraCaptureIn = 4.5;
constexpr int kStage2ExtraCapturePower = 70;
constexpr double kStage2GoalDriveIn = 24.0;
constexpr int kStage2GoalDrivePower = 80;
constexpr double kStage2ScoreRetreatIn = 24.0;
constexpr int kStage2ScoreRetreatPower = 95;

constexpr double kFirstGoalExitIn = 12.0;
constexpr double kStackExtraCaptureIn = 5.0;
constexpr double kStackBTravelIn = 33.9411255;

// Physical qualification gate: 1 stops after the preload drop, 2 after the
// next cup is captured, 3 after that cup is scored, 4 after the second stack
// is scored, and 0 runs all.
constexpr int kTestStopAfterPhase = 0;

}  // namespace path2_config
