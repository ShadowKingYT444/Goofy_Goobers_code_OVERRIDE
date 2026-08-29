# Path 2 Autonomous — PROS / Production-Navigation Template

This is a code-oriented handoff for implementing the current Path 2 sequence in the robot project.

It is intentionally **compile-adjacent, not copy-paste-ready competition code**: mechanism polarities, lift stage targets, exact goal docking distances, start heading, and a few contact powers must come from the tested robot configuration rather than being guessed here.

## 0. Safety / repository constraint

The repository's current `NAVIGATION.md` contains an active safety hold: successful `navigation::init()` alone is not yet sufficient motion authority until the delayed P6 calibration-quality gate is implemented and passes live stopped/turning tests.

Therefore an agent may build this routine behind a disabled/auton-selection gate, but must not represent it as competition-ready or automatically enable it while that hold remains unresolved.

Also note that the production navigation map deliberately rejects endpoints/segments too close to mapped Goal centers and walls. **Do not call `navigation::go_to_pose()` directly on the literal goal center or matchloader wall coordinate and assume it will succeed.** Use a safe navigation staging pose, then a short controlled docking/contact move.

## 1. Coordinate transform — critical

The existing Path 2 simulator/spec uses planning coordinates where:

- first coordinate increases toward the top of the field image;
- second coordinate increases toward the left of the field image.

The production `navigation` API uses normal field math:

- `+X` = right;
- `+Y` = up;
- `0 deg` = `+X`;
- positive heading = counterclockwise.

Therefore transform Path 2 planning coordinates `(a,b)` to production navigation coordinates as:

```cpp
nav_x = -b;
nav_y =  a;
```

Use a helper. Never scatter hand-converted coordinates through the auton.

```cpp
struct PathPt {
  double a;  // Path2 first coordinate: + toward top
  double b;  // Path2 second coordinate: + toward left
};

struct NavPt {
  double x;
  double y;
};

constexpr NavPt to_nav(PathPt p) {
  return NavPt{-p.b, p.a};
}
```

### Current anchors

| Route object | Path 2 planning coordinate | Production nav coordinate |
|---|---:|---:|
| Safe start | `(0,-63)` | `(63,0)` |
| Toggle contact | `(0,-72)` | `(72,0)` |
| Goal | `(24,-48)` | `(48,24)` |
| Stack A | `(24,-24)` | `(24,24)` |
| Stack B | `(48,-48)` | `(48,48)` |
| Matchloader | `(60,-72)` | `(72,60)` |

The matchloader value above intentionally uses the user's newer `(60,-72)` instruction rather than the older simulator spec's `(66,-66)`.

## 2. Relevant project APIs / hardware names

Production navigation already exposes:

```cpp
navigation::init(...)
navigation::update()
navigation::current_pose()
navigation::turn_to(...)
navigation::go_straight_to(...)
navigation::drive_relative(...)
navigation::go_to_pose(...)
navigation::stop()
```

Mechanism declarations currently available from `subsystems.hpp` include:

```cpp
slider_right
slider_left
slider_rotation_sensor
claw_arm
upper_intake
counter_rollers
clamp_piston
```

Do not create duplicate motor objects for those ports.

## 3. Route constants

```cpp
namespace path2 {

constexpr PathPt kStartSafePath{0.0, -63.0};
constexpr PathPt kTogglePath{0.0, -72.0};
constexpr PathPt kGoalPath{24.0, -48.0};
constexpr PathPt kStackAPath{24.0, -24.0};
constexpr PathPt kStackBPath{48.0, -48.0};
constexpr PathPt kMatchloaderPath{60.0, -72.0};

constexpr NavPt kStartSafe = to_nav(kStartSafePath);       // (63, 0)
constexpr NavPt kToggle = to_nav(kTogglePath);             // (72, 0)
constexpr NavPt kGoal = to_nav(kGoalPath);                 // (48, 24)
constexpr NavPt kStackA = to_nav(kStackAPath);             // (24, 24)
constexpr NavPt kStackB = to_nav(kStackBPath);             // (48, 48)
constexpr NavPt kMatchloader = to_nav(kMatchloaderPath);   // (72, 60)

constexpr double kFirstGoalExitIn = 12.0;
constexpr double kNominalStackDiagonalIn = 33.9411255; // sqrt(24^2 + 24^2)
constexpr double kFinalMatchloaderDriveIn = 6.0;

// TUNE / verify physically:
constexpr double kStartHeadingNavDeg = /* TUNE */ 0.0;
constexpr double kStartPositionErrorIn = /* TUNE */ 1.0;
constexpr double kToggleHitDistanceIn = /* TUNE */ 9.0;
constexpr int kTogglePower = /* TUNE */ 45;
constexpr double kStackSlowZoneIn = /* TUNE */ 8.0;
constexpr int kFastMovePower = /* TUNE */ 60;
constexpr int kSlowStackPower = /* TUNE */ 30;
constexpr int kTurnPower = /* TUNE */ 45;
constexpr double kWallStandoffIn = /* TUNE */ 8.0;

// Must be measured/tuned outside the navigation Goal keepout.
constexpr PathPt kGoalStagingPath{/* TUNE a */, /* TUNE b */};
constexpr NavPt kGoalStaging = to_nav(kGoalStagingPath);

// Likewise, choose a legal staging pose before the right-side wall/matchloader.
constexpr PathPt kMatchloaderStagingPath{/* TUNE a */, /* TUNE b */};
constexpr NavPt kMatchloaderStaging = to_nav(kMatchloaderStagingPath);

}  // namespace path2
```

Do not keep the placeholder numeric start heading `0.0` in the final routine unless it is actually measured/verified. It is present only so an agent understands that a concrete heading is required by `navigation::init()`.

## 4. Geometry helpers

Compute turn angles from the current fused pose instead of hardcoding `x deg`.

```cpp
#include <cmath>

constexpr double kPi = 3.14159265358979323846;

double normalize_deg(double deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0) deg += 360.0;
  return deg;
}

double distance_to(const navigation::Pose& p, NavPt target) {
  return std::hypot(target.x - p.x_in, target.y - p.y_in);
}

double front_heading_to(const navigation::Pose& p, NavPt target) {
  return normalize_deg(std::atan2(target.y - p.y_in,
                                  target.x - p.x_in) * 180.0 / kPi);
}

double rear_heading_to(const navigation::Pose& p, NavPt target) {
  return normalize_deg(front_heading_to(p, target) + 180.0);
}
```

## 5. Fail-closed navigation wrapper

Every blocking navigation call should be checked.

```cpp
bool nav_ok(navigation::Result result, const char* step) {
  if (result == navigation::Result::kSuccess) return true;

  std::printf("PATH2 abort step=%s result=%s\n",
              step, navigation::result_name(result));
  navigation::stop();
  upper_intake.move(0);
  counter_rollers.move(0);
  return false;
}
```

## 6. Mechanism interfaces the agent should bind to existing tested code

Do not guess motor polarity in this document. Bind these wrappers to the already-tested mechanism directions.

```cpp
enum class RollerMode {
  kOff,
  kIntake,
  kOuttake,
};

void set_rollers(RollerMode mode) {
  // TODO: map to verified upper_intake / counter_rollers powers.
  // Never infer signs from port numbers.
}

void deposit_preload_pin() {
  // TODO: existing preload release/drop sequence.
}

void score_current_piece() {
  // TODO: existing scoring sequence using claw/rollers/etc.
}

void request_lift_stage_async(int stage) {
  // IMPORTANT: set a target on the existing cascade PID/state controller and
  // RETURN IMMEDIATELY. The controller task should keep moving the lift while
  // the drivetrain continues.
  // stage 1 -> measured level-1 target
  // stage 2 -> measured level-2 target
}

bool lift_stage_ready(int stage) {
  // TODO: query existing cascade controller tolerance/state.
  return true;
}
```

Do not implement the stage raise with a long blocking `pros::delay()` or raw full-power slider command. The route explicitly depends on lift/drivetrain overlap.

## 7. Slow approach helper for stacks

The user explicitly wants the robot to slow down before contact. Use a two-segment approach rather than one high-power command to the stack.

```cpp
NavPt point_before_target(const navigation::Pose& from,
                          NavPt target,
                          double offset_in) {
  const double dx = target.x - from.x_in;
  const double dy = target.y - from.y_in;
  const double d = std::hypot(dx, dy);
  if (d <= offset_in || d < 1e-6) return NavPt{from.x_in, from.y_in};

  const double s = (d - offset_in) / d;
  return NavPt{from.x_in + dx * s, from.y_in + dy * s};
}

bool approach_and_intake_stack(NavPt stack, const char* label) {
  navigation::update();
  auto pose = navigation::current_pose();
  if (!pose.valid) return false;

  const NavPt pre = point_before_target(pose, stack, path2::kStackSlowZoneIn);
  const double pre_heading = front_heading_to(pose, stack);

  if (!nav_ok(navigation::go_to_pose(
                  pre.x, pre.y, pre_heading,
                  path2::kFastMovePower, 9000),
              label)) {
    return false;
  }

  set_rollers(RollerMode::kIntake);

  if (!nav_ok(navigation::go_straight_to(
                  stack.x, stack.y,
                  path2::kSlowStackPower, 5000),
              label)) {
    set_rollers(RollerMode::kOff);
    return false;
  }

  // Keep intake holding only if the tested mechanism requires it.
  return true;
}
```

If the stack itself lies inside a production map exclusion not yet modeled correctly, stop navigation at a safe pre-contact pose and use the same short controlled-contact helper used for physical docking. Do not disable global safety checks just to reach a game object.

## 8. Goal docking helper

The literal `kGoal = (48,24)` is a route anchor/Goal center after coordinate transform. Production navigation intentionally protects Goal centers, so the routine should navigate only to `kGoalStaging`, orient correctly, then perform a short controlled rearward contact motion.

```cpp
bool dock_rear_to_goal() {
  // 1) Navigate to legal staging point.
  navigation::update();
  auto pose = navigation::current_pose();
  if (!pose.valid) return false;

  const double staging_heading = rear_heading_to(pose, path2::kGoal);

  if (!nav_ok(navigation::go_to_pose(
                  path2::kGoalStaging.x,
                  path2::kGoalStaging.y,
                  staging_heading,
                  path2::kFastMovePower, 8000),
              "goal staging")) {
    return false;
  }

  // 2) Recompute from fused pose; turn so REAR points at goal.
  navigation::update();
  pose = navigation::current_pose();
  const double rear_heading = rear_heading_to(pose, path2::kGoal);
  if (!nav_ok(navigation::turn_to(
                  rear_heading, path2::kTurnPower, 5000),
              "rear toward goal")) {
    return false;
  }

  // 3) Controlled final reverse alignment/contact.
  // Do NOT command drive_relative all the way to the mapped Goal center if the
  // map preflight rejects that contact zone. Bind this to the project's
  // supervised low-level drive helper or a dedicated goal-dock routine.
  //
  // Example semantics only:
  // goal_dock_reverse_contact(kDockDistanceIn,
  //                           kDockCruisePower,
  //                           kFinalAlignmentBumpPower,
  //                           kFinalAlignmentBumpMs);

  return true;
}
```

The final "accelerate to align" should be a tuned mechanical seating bump, not a long full-power run.

## 9. Exit invariant — outtake is mandatory

Centralize this so the agent cannot forget the rule.

```cpp
bool exit_goal_relative(double signed_distance_in,
                        int power,
                        const char* label) {
  set_rollers(RollerMode::kOuttake);   // BEFORE drivetrain motion

  const auto result = navigation::drive_relative(
      signed_distance_in, power, 5000);

  set_rollers(RollerMode::kOff);       // only after exit motion is done
  return nav_ok(result, label);
}
```

If the goal-contact zone requires a low-level drive helper rather than `navigation::drive_relative`, preserve the same ordering: outtake on -> move -> clear goal -> outtake off.

## 10. Main route template

```cpp
void path2_auton_template() {
  using namespace path2;

  set_rollers(RollerMode::kOff);

  // ------------------------------------------------------------
  // INITIALIZE
  // ------------------------------------------------------------

  if (!navigation::init(
          kStartSafe.x,
          kStartSafe.y,
          kStartHeadingNavDeg,
          kStartPositionErrorIn)) {
    navigation::stop();
    return;
  }

  // ------------------------------------------------------------
  // A. HIT TOGGLE ONCE
  // ------------------------------------------------------------

  // Assumption from the Path 2 start geometry: the robot starts with its rear
  // oriented toward the nearby toggle. If the measured start heading differs,
  // explicitly turn/re-anchor rather than silently reversing the wrong way.
  if (!nav_ok(navigation::drive_relative(
                  -kToggleHitDistanceIn,
                  kTogglePower,
                  2500),
              "hit toggle once")) {
    return;
  }

  // ------------------------------------------------------------
  // B. BOOMERANG TO GOAL, RECORD END HEADING, DEPOSIT PRELOAD
  // ------------------------------------------------------------

  navigation::update();
  auto pose = navigation::current_pose();
  if (!pose.valid) return;

  // Navigate to a legal pre-goal staging pose with a smooth continuous curve.
  const double goal_stage_heading = front_heading_to(pose, kGoalStaging);
  if (!nav_ok(navigation::go_to_pose(
                  kGoalStaging.x,
                  kGoalStaging.y,
                  goal_stage_heading,
                  kFastMovePower,
                  9000),
              "boomerang to goal staging")) {
    return;
  }

  // Perform the tuned final goal contact/docking required to deposit preload.
  // final_preload_dock();

  navigation::update();
  const auto goal_arrival_pose = navigation::current_pose();
  const double recorded_goal_heading_deg = goal_arrival_pose.heading_deg;
  std::printf("PATH2 preload_goal_end_heading_deg=%.3f\n",
              recorded_goal_heading_deg);

  deposit_preload_pin();

  // ------------------------------------------------------------
  // C. RETURN TO SAFE START WITHOUT TOUCHING TOGGLE
  // ------------------------------------------------------------

  set_rollers(RollerMode::kOuttake);  // exiting a goal

  // First leave the physical contact zone using the tuned exit helper if the
  // navigation map still considers the robot inside a Goal keepout.
  // exit_goal_contact_zone();

  // Once in legal navigation space, return to the safe start anchor.
  const navigation::Pose start_anchor{
      kStartSafe.x, kStartSafe.y, 0.0, 0.0, 0.0, 0, true};
  const double start_setup_heading = front_heading_to(start_anchor, kStackA);

  if (!nav_ok(navigation::go_to_pose(
                  kStartSafe.x,
                  kStartSafe.y,
                  start_setup_heading,
                  kFastMovePower,
                  9000),
              "return safe start")) {
    set_rollers(RollerMode::kOff);
    return;
  }

  set_rollers(RollerMode::kOff);

  // Endpoint is START_SAFE, not TOGGLE_CONTACT.

  // ------------------------------------------------------------
  // D. STACK A -> LIFT STAGE 1 -> SCORE
  // ------------------------------------------------------------

  if (!approach_and_intake_stack(kStackA, "stack A")) return;

  navigation::update();
  pose = navigation::current_pose();
  if (!nav_ok(navigation::turn_to(
                  rear_heading_to(pose, kGoal),
                  kTurnPower,
                  5000),
              "rear to goal after stack A")) {
    return;
  }

  request_lift_stage_async(1);  // asynchronous while driving

  if (!dock_rear_to_goal()) return;

  if (!lift_stage_ready(1)) {
    // wait_for_lift_stage(1, kLiftReadyTimeoutMs);
  }

  score_current_piece();

  // Exit exactly 12 in with outtake active.
  // If front points away from goal after rear-dock score, +12 is correct.
  if (!exit_goal_relative(+kFirstGoalExitIn,
                          40,
                          "first 12in goal exit")) {
    return;
  }

  // ------------------------------------------------------------
  // E. STACK B -> LIFT STAGE 2 -> SCORE
  // ------------------------------------------------------------

  navigation::update();
  pose = navigation::current_pose();
  if (!pose.valid) return;

  if (!nav_ok(navigation::turn_to(
                  front_heading_to(pose, kStackB),
                  kTurnPower,
                  5000),
              "turn to stack B")) {
    return;
  }

  navigation::update();
  pose = navigation::current_pose();
  const double actual_stack_b_distance_in = distance_to(pose, kStackB);
  std::printf(
      "PATH2 stackB_distance_in=%.3f nominal_from_stackA=%.3f\n",
      actual_stack_b_distance_in,
      kNominalStackDiagonalIn);

  // sqrt(24^2+24^2)=33.94 in is only STACK_A->STACK_B. After the
  // score + 12-in exit, use the current fused pose instead.
  if (!approach_and_intake_stack(kStackB, "stack B")) return;

  request_lift_stage_async(2);

  navigation::update();
  pose = navigation::current_pose();
  if (!nav_ok(navigation::turn_to(
                  rear_heading_to(pose, kGoal),
                  kTurnPower,
                  5000),
              "rear to goal after stack B")) {
    return;
  }

  if (!dock_rear_to_goal()) return;

  if (!lift_stage_ready(2)) {
    // wait_for_lift_stage(2, kLiftReadyTimeoutMs);
  }

  score_current_piece();

  // ------------------------------------------------------------
  // F. EXIT TOWARD WALL WITH OUTTAKE, FACE MATCHLOADER, FORWARD 6 IN
  // ------------------------------------------------------------

  set_rollers(RollerMode::kOuttake);  // before exit begins

  // exit_goal_contact_zone();

  // Path2 wall b=-72 maps to production nav X=+72.
  // Use fused field pose, not the forward-only P1 sensor, for a wall behind.
  const double wall_stop_x = 72.0 - kWallStandoffIn;

  navigation::update();
  pose = navigation::current_pose();
  if (!pose.valid) {
    set_rollers(RollerMode::kOff);
    return;
  }

  const NavPt wall_target{72.0, pose.y_in};
  if (!nav_ok(navigation::turn_to(
                  rear_heading_to(pose, wall_target),
                  kTurnPower,
                  5000),
              "rear toward wall")) {
    set_rollers(RollerMode::kOff);
    return;
  }

  navigation::update();
  pose = navigation::current_pose();
  const double retreat_to_standoff_in =
      std::max(0.0, wall_stop_x - pose.x_in);
  (void)retreat_to_standoff_in;

  // Use a legal nav staging target or supervised reverse helper for the final
  // approach to wall standoff. Do not globally disable map checks.
  // retreat_reverse_until_nav_x(wall_stop_x);

  set_rollers(RollerMode::kOff);  // only after fully clear of goal

  navigation::update();
  pose = navigation::current_pose();
  if (!pose.valid) return;

  if (!nav_ok(navigation::turn_to(
                  front_heading_to(pose, kMatchloader),
                  kTurnPower,
                  5000),
              "face matchloader")) {
    return;
  }

  // User requested exactly 6 in forward after facing matchloader.
  if (!nav_ok(navigation::drive_relative(
                  +kFinalMatchloaderDriveIn,
                  35,
                  3000),
              "final matchloader 6in")) {
    return;
  }

  navigation::stop();
  set_rollers(RollerMode::kOff);
}
```

## 11. Why the 33.94-inch instruction is not hardcoded

The nominal coordinate difference from Stack A `(24,-24)` to Stack B `(48,-48)` is:

```text
sqrt((48-24)^2 + (-48 - -24)^2)
= sqrt(24^2 + 24^2)
= 33.94 in
```

But the actual sequence is:

1. leave Stack A;
2. drive to the goal;
3. score;
4. exit the goal by 12 in;
5. then turn toward Stack B.

So the robot's actual starting point for the Stack B leg is not Stack A. A literal 33.94-in relative drive after the 12-in exit is generally geometrically inconsistent. The production implementation should target Stack B's coordinate / use the current fused pose to compute the true distance.

## 12. Agent implementation requirements

When turning this template into real code:

1. Preserve the event order in this document.
2. Keep the Path2->navigation coordinate transform in one helper.
3. Calculate all `x deg` turns from current fused pose.
4. Do not guess lift heights, mechanism polarities, scoring powers, goal bump duration, wall standoff, or start heading.
5. Raise lift stages 1 and 2 asynchronously with the existing cascade controller.
6. Slow the drivetrain before both stack contacts.
7. Keep outtake active for **every** goal exit, starting before the drivetrain moves.
8. Do not call field-coordinate navigation directly to a mapped Goal center/wall coordinate if production safety preflight rejects it. Use legal staging + short supervised contact movement.
9. Do not use the forward-only distance sensor as if it could see the wall behind the robot during reverse retreat.
10. Log the first goal's actual fused end heading.
11. Fail closed on invalid pose or any non-success `navigation::Result`.
12. Keep the auton disabled until the repository's current navigation safety hold is explicitly cleared by the required live qualification.
