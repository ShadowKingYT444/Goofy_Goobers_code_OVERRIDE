# Path 2 Autonomous Pseudocode Template

This is the implementation-neutral source of truth for the current Path 2 sequence.

It intentionally preserves unknown/tunable values instead of inventing them. An implementation agent should tune those values on the real robot or in the simulator, but should not silently change the event order.

## 1. Path-coordinate anchors

Use the same Path 2 coordinate convention as the existing simulator spec:

- `START_SAFE = (0, -63)` — editable starting/safe-return pose from the existing Path 2 spec
- `TOGGLE_CONTACT ~= (0, -72)`
- `GOAL = (24, -48)`
- `STACK_A = (24, -24)`
- `STACK_B = (48, -48)`
- `MATCHLOADER = (60, -72)` — use the newer value from this route description

Important: these are **Path 2 planning coordinates**, not automatically the same `(x,y)` frame used by the production PROS `navigation` API. The PROS version in the companion MD handles the transform explicitly.

## 2. Tunables that must remain explicit

```text
START_SAFE                         = (0, -63)
TOGGLE_HIT_DISTANCE_IN             = TUNE
TOGGLE_HIT_POWER                   = TUNE
GOAL_EXIT_DISTANCE_1_IN            = 12.0
STACK_SLOW_ZONE_IN                 = TUNE (suggest starting around 6-10 in)
STACK_FAST_POWER                   = TUNE
STACK_SLOW_POWER                   = TUNE
GOAL_APPROACH_POWER                = TUNE
GOAL_ALIGNMENT_BUMP_POWER          = TUNE
GOAL_ALIGNMENT_BUMP_TIME_MS        = TUNE
SCORE_DWELL_MS                     = TUNE
WALL_STANDOFF_IN                   = TUNE
FINAL_MATCHLOADER_FORWARD_IN       = 6.0
LIFT_STAGE_1_TARGET                = existing cascade-controller target
LIFT_STAGE_2_TARGET                = existing cascade-controller target
```

Do not hardcode intake/outtake motor signs until they are checked against the actual robot wiring and existing mechanism code.

## 3. Required helper semantics

### `HIT_TOGGLE_ONCE()`
Drive into the toggle exactly once, then stop/transition immediately into the next route phase. Do not oscillate or make a second contact.

### `BOOMERANG_TO(target)`
Use the robot's smooth curved point/pose motion to reach `target`. The implementation may choose the arrival heading needed for the next action, but must record the actual fused pose after arrival.

### `APPROACH_AND_INTAKE(target)`
1. Start intake before the final approach.
2. Travel quickly for the long portion of the leg.
3. Inside `STACK_SLOW_ZONE_IN`, reduce drive speed.
4. Enter the stack slowly enough not to knock it over.
5. Confirm/assume possession according to the available mechanism sensor logic.

### `TURN_FRONT_TOWARD(target)`
Compute the turn from the current pose. Do not hardcode an old `x deg` value if localization is available.

### `TURN_REAR_TOWARD(target)`
Compute the heading so the **rear/scoring side** points at `target`. This is the intended interpretation of "turn x deg so back faces goal."

### `REQUEST_LIFT_STAGE_ASYNC(stage)`
Set the cascade lift target and return immediately. A background PID/state controller should continue raising the lift while the drivetrain moves. Do not block the autonomous routine waiting for the lift unless scoring requires a final readiness gate.

### `SCORE_CURRENT_GAMEPIECE()`
Execute the existing scoring mechanism sequence and hold long enough for reliable release.

### `EXIT_GOAL(...)`
**Invariant: OUTTAKE MUST BE ON WHILE EXITING A GOAL.**

The helper must:
1. turn outtake on before the drivetrain begins moving away;
2. keep outtake active for the entire disengagement;
3. stop outtake only after the robot is clearly free of the goal.

Apply this rule after the preload deposit and after both later scores.

## 4. Route pseudocode

```text
AUTON_PATH_2:
    initialize localization + mechanisms
    verify autonomous is allowed to move
    set lift to travel/home state
    intake/off; outtake/off

    # ------------------------------------------------------------
    # PHASE A — Toggle + preload score
    # ------------------------------------------------------------

    HIT_TOGGLE_ONCE()

    BOOMERANG_TO(GOAL)

    goal_arrival_pose = RECORD_CURRENT_FUSED_POSE()
    recorded_goal_heading = goal_arrival_pose.heading

    DEPOSIT_PRELOAD_PIN()

    # User requirement: return to starting position, but DO NOT touch toggles.
    # Because this is an exit from the goal, outtake must be active.
    OUTTAKE_ON()
    MOVE_TO(START_SAFE)
    STOP_BEFORE_TOGGLE_CONTACT_ZONE()
    OUTTAKE_OFF()

    # ------------------------------------------------------------
    # PHASE B — First stack at (24,-24), lift stage 1, score
    # ------------------------------------------------------------

    BOOMERANG_TOWARD(STACK_A)
    APPROACH_AND_INTAKE(STACK_A)

    TURN_REAR_TOWARD(GOAL)

    # Start lift motion before/during travel so lift time overlaps drive time.
    REQUEST_LIFT_STAGE_ASYNC(stage = 1)

    DRIVE_REARWARD_TOWARD(GOAL)
    ACCELERATE/ALIGN_INTO_SCORING_POSITION()

    WAIT_ONLY_IF_NEEDED_FOR_LIFT_STAGE_1_READY()
    SCORE_CURRENT_GAMEPIECE()

    # Explicit first exit: 12 inches.
    OUTTAKE_ON()
    DRIVE_AWAY_FROM_GOAL(distance = 12 in)
    OUTTAKE_OFF()

    # ------------------------------------------------------------
    # PHASE C — Second stack at (48,-48), lift stage 2, score
    # ------------------------------------------------------------

    TURN_FRONT_TOWARD(STACK_B)

    current_pose = RECORD_CURRENT_FUSED_POSE()
    actual_distance_to_stack_b = DISTANCE(current_pose, STACK_B)

    # The coordinate-to-coordinate nominal diagonal from STACK_A to STACK_B is:
    # sqrt(24^2 + 24^2) = 33.94 in.
    # HOWEVER, after scoring at GOAL and exiting 12 in, the robot is no longer
    # at STACK_A. Therefore do not blindly hardcode 33.94 in here if fused pose
    # is available. Use actual_distance_to_stack_b / target coordinates.

    DRIVE_TOWARD(STACK_B, distance = actual_distance_to_stack_b)
    SLOW_DOWN_INSIDE(STACK_SLOW_ZONE_IN)
    INTAKE_STACK_B()

    REQUEST_LIFT_STAGE_ASYNC(stage = 2)

    # Interpret "turn to goal" as rear/scoring side toward the goal unless the
    # physical scoring mechanism requires the opposite orientation.
    TURN_REAR_TOWARD(GOAL)

    DRIVE_REARWARD_TOWARD(GOAL)
    ACCELERATE/ALIGN_INTO_SCORING_POSITION()

    WAIT_ONLY_IF_NEEDED_FOR_LIFT_STAGE_2_READY()
    SCORE_CURRENT_GAMEPIECE()

    # ------------------------------------------------------------
    # PHASE D — Exit toward wall, face matchloader, final 6 in
    # ------------------------------------------------------------

    # Again: outtake must remain pressed for the entire goal exit.
    OUTTAKE_ON()
    RETREAT_FROM_GOAL_UNTIL_WALL_STANDOFF(WALL_STANDOFF_IN)
    OUTTAKE_OFF()

    TURN_FRONT_TOWARD(MATCHLOADER = (60, -72))

    DRIVE_FORWARD(distance = 6 in)

    STOP_DRIVETRAIN()
    STOP_INTAKE_OUTTAKE()
    HOLD_LIFT_AT_SAFE_FINAL_TARGET()
END AUTON_PATH_2
```

## 5. Geometry helpers

Use pose-derived turns rather than fixed `x deg` constants whenever possible.

```text
FRONT_HEADING_TO(target):
    return atan2(target.y - current.y, target.x - current.x)

REAR_HEADING_TO(target):
    return normalize(FRONT_HEADING_TO(target) + 180 deg)

DISTANCE(a, b):
    return sqrt((b.x-a.x)^2 + (b.y-a.y)^2)
```

The exact heading formula must use the coordinate convention of the implementation. The companion PROS MD computes headings in the production navigation frame after transforming Path 2 coordinates.

## 6. Slow-stack approach behavior

Do not simply command one high-speed leg all the way into a stack.

Recommended behavior:

```text
approach_point = point STACK_SLOW_ZONE_IN before stack along current->stack line
MOVE_FAST_TO(approach_point)
INTAKE_ON()
MOVE_SLOW_TO(stack)
```

If the drive controller already provides a strong velocity profile near the endpoint, keep an explicit lower max-power cap for the final stack capture so the physical behavior remains intentional.

## 7. Goal alignment behavior

The phrase "accelerate to align" should be treated as a controlled final docking/bump, not an uncontrolled full-power collision.

```text
MOVE_TO_GOAL_APPROACH_POSE()
VERIFY scoring side faces goal
FINAL_ALIGNMENT_BUMP(power = GOAL_ALIGNMENT_BUMP_POWER,
                     duration = GOAL_ALIGNMENT_BUMP_TIME_MS)
BRAKE/HOLD
SCORE
```

Tune the bump on the actual goal. The robot should use the goal as a repeatable mechanical alignment surface without generating enough speed to bounce or destabilize the stack.

## 8. Wall-retreat rule

`RETREAT_FROM_GOAL_UNTIL_WALL_STANDOFF(x)` must stop based on localization/field geometry unless the robot has a sensor that actually sees the relevant wall while retreating.

Do not pretend a forward-only distance sensor can measure a wall behind the robot.

## 9. Acceptance checklist

- [ ] Toggle is hit once.
- [ ] Robot boomerangs to `(24,-48)` and records the fused end heading.
- [ ] Preload pin is deposited.
- [ ] Robot returns to the safe start pose without re-touching the toggle.
- [ ] First stack `(24,-24)` is approached slowly and intaken.
- [ ] Rear/scoring side faces the goal before the first stack score.
- [ ] Lift stage 1 raises asynchronously while traveling.
- [ ] Robot scores, then exits 12 in with outtake held.
- [ ] Robot turns to `(48,-48)`, slows near the stack, and intakes it.
- [ ] Lift stage 2 raises asynchronously while traveling.
- [ ] Robot scores the second stack.
- [ ] Every goal exit has outtake active before motion begins and until clear.
- [ ] Final retreat stops `WALL_STANDOFF_IN` from the wall.
- [ ] Robot turns to face matchloader `(60,-72)`.
- [ ] Robot drives forward exactly 6 in for the final step.
- [ ] Unknown angles/distances are calculated from current pose or left as named tunables; they are not guessed.
