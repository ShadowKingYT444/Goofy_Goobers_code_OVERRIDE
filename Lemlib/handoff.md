# LemLib odometry / PID autotune handoff for Astra

## Mission

Diagnose the large LemLib pose error during a pure in-place turn before running or trusting the PID autotuner.

Current symptom:

- The robot is rotated approximately -90 degrees in place.
- Raw tracking-wheel values change, as expected for tracking wheels mounted away from the rotation center.
- LemLib's reported pose changes by approximately X = 14 in and Y = 9 in after that single turn.
- Turning the robot back can make the round-trip delta look close to zero, but that can hide errors that cancel.

Do not treat this as a PID-gain problem yet. First establish whether rotation compensation is using the correct sensor signs, wheel scale, and offsets.

The user will copy this file into Astra and delete it afterward. Do not add durable project documentation for this handoff.

---

## Project and version

Project path:

~~~text
/home/terryd/Downloads/VEX_BACKUP/Lemlib
~~~

This is a PROS VEX V5 project using LemLib 0.5.6, recorded in project.pros:

~~~text
/home/terryd/.config/pros/templates/LemLib@0.5.6
~~~

The implementation is primarily present as:

~~~text
firmware/LemLib.a
~~~

Headers are under include/lemlib/. The implementation source is not present in this project.

The project builds successfully with:

~~~bash
make -j2
~~~

The last build after changing the horizontal offset completed compilation, linking, and hot-package creation successfully.

---

## Current hardware and LemLib configuration

Relevant file: src/main.cpp

Current declarations:

~~~cpp
pros::MotorGroup left_motors({-17, -18}, pros::MotorGears::blue);
pros::MotorGroup right_motors({11, 13}, pros::MotorGears::blue);
pros::Imu imu(14);

pros::Rotation vertical_odom(-15);
pros::Rotation horizontal_odom(-1);

lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              12, lemlib::Omniwheel::NEW_275, 450, 2);

lemlib::TrackingWheel vertical_wheel(
    &vertical_odom,
    lemlib::Omniwheel::NEW_2,
    -0.5,
    1.0
);

lemlib::TrackingWheel horizontal_wheel(
    &horizontal_odom,
    lemlib::Omniwheel::NEW_2,
    5.1,
    1.0
);

lemlib::OdomSensors sensors(
    &vertical_wheel,
    nullptr,
    &horizontal_wheel,
    nullptr,
    &imu
);
~~~

Meaning:

- Vertical pod is measured approximately 0.5 in to the robot's left, so code uses -0.5.
- Horizontal pod was measured approximately 6 in in front of the robot center; user requested the temporary code value 5.1.
- Both Rotation Sensor ports are negative because raw values decreased when the user pushed toward the robot's front/right.
- OdomSensors order is vertical1, vertical2, horizontal1, horizontal2, IMU, and the current source uses that order correctly.

The Brain screen task in src/main.cpp prints every 100 ms:

~~~text
X, Y, T, raw vertical tracking distance V, raw horizontal tracking distance H, IMU rotation
~~~

---

## User's measured behavior

### Translation sign test

The user found:

- Before reversal, raw vertical tracking value decreased when pushing toward the robot's front.
- Before reversal, raw horizontal tracking value decreased when pushing toward the robot's right.
- After using negative Rotation Sensor ports, translation signs are now correct: front motion gives positive V and right motion gives positive H.

This validates the translational sign convention only. It does not prove that each wheel's rotational delta sign matches the IMU's rotational delta sign.

### Pure-turn test

Observed:

- Horizontal tracker changes by approximately 10 in during a 90-degree turn.
- Vertical tracker changes by a small amount during a 90-degree turn.
- A single approximately -90-degree turn still changed pose by roughly:

~~~text
X: 14 in
Y: 9 in
~~~

- A round trip can return near the starting values, but that is not a valid pass condition.
- Exact signed values for delta T, delta H, and delta V for the latest turn have not been supplied. These are essential.

---

## Current autonomous and tuner state

Autonomous/PID code is in src/autons.cpp.

The compact independent tuner contains:

- run_trial(kP, kD, target, turn)
  - Lateral trials use vertical_wheel.getDistanceTraveled().
  - Angular trials use imu.get_rotation().
  - Uses P and D only.
  - Commands motor groups directly.
  - Aborts on reversed sensor signs, invalid readings, unsafe excursions, or competition connection.
- search_gains(...)
  - Five nearby candidate pairs per round.
  - Three refinement rounds.
  - Positive and negative trials.
- odom_sign_test_auton()
  - Separate manual push sign test.
- pid_autotune_auton()
  - Runs lateral tuning then angular tuning.
  - Prints candidate ControllerSettings gains.

Current src/main.cpp flags:

~~~cpp
constexpr bool RUN_TWO_CUP_AUTON = true;
constexpr bool RUN_ODOM_SIGN_TEST = false;
constexpr bool RUN_PID_AUTOTUNE = true;
~~~

Because RUN_PID_AUTOTUNE is true, autonomous currently enters the tuner unless the flag is changed. Do not run it until this pure-turn issue is explained.

The ordinary auton route no longer uses minSpeed or maxSpeed. Its first motion is:

~~~cpp
chassis.setPose(0, 0, 180);

chassis.moveToPoint(
    0, 8, 2500,
    {.forwards = false},
    false
);
~~~

No PID gains have been accepted as final.

---

## Research already performed

### Official LemLib configuration documentation

Official documentation states:

- Vertical encoder should increase when pushing toward the robot's front.
- Horizontal encoder should increase when pushing toward the robot's right.
- Vertical offset sign: left negative, right positive.
- Horizontal offset sign: back negative, front positive.
- Offset is the perpendicular distance from tracking wheel to the tracking center.
- NEW_2 is defined as 2.125 in, not exactly 2.0 in.

Source:
https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html

### Similar LemLib issue

LemLib discussion #284 reports almost the same failure in v0.5.5: turning in place changed pose by approximately 10 in. The maintainer attributed it to tracking-wheel offsets and the tracking center not aligning with the robot's actual turning center. Later comments suggested checking sensor placement, testing zero offsets diagnostically, checking wheel diameter, and checking wheel traction.

Source:
https://github.com/LemLib/LemLib/discussions/284

This is evidence that the symptom exists in the same LemLib generation, not proof that this project has the same single cause.

### PID guidance

LemLib recommends tuning P and D first, leaving I disabled unless needed, and tuning slew afterward. That guidance only applies once sensor feedback and odometry are coherent.

Source:
https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html

### Local v0.5.6 binary inspection

The local firmware/LemLib.a was inspected with nm and objdump. The one-vertical, one-horizontal, IMU path in lemlib::update() uses the expected arc-compensation pattern. In simplified form, each tracking-wheel local displacement is approximately:

~~~text
(delta_wheel / delta_heading + wheel_offset)
    * 2 * sin(delta_heading / 2)
~~~

The local result is then rotated into global X/Y coordinates.

Implication:

- If wheel delta sign is correct for translation but opposite to the IMU's rotational delta sign, the offset term adds to the rotational wheel movement instead of canceling it.
- This can produce a large apparent translation during a turn, potentially near twice the expected rotational arc.
- Changing the horizontal offset from 6.0 to 5.1 cannot fix an opposite-sign error.

For a measured delta_heading of -90 degrees and current offsets H = +5.1, V = -0.5, cancellation nominally requires:

~~~text
delta_H / delta_heading_radians ~= -5.1
delta_V / delta_heading_radians ~= +0.5
~~~

If the measured IMU delta really is -90 degrees, the approximate signed expectations are:

~~~text
expected delta_H ~= +8.0 in
expected delta_V ~= -0.8 in
~~~

Use the measured IMU delta, not the commanded turn direction.

Example:

- If actual delta_H is about -10 in for measured delta_heading -90 degrees, the horizontal rotational sign is inconsistent with the IMU and compensation adds instead of cancels.
- If actual delta_H is about +10 in, the sign may be correct but scale/offset is not exact; that alone should not create a 14-by-9-inch pose jump.

---

## Ranked potential causes

### 1. Tracking-wheel rotational sign does not match IMU sign

Most suspicious because the push test validates only translation. The odometry compensation needs wheel and IMU signs to agree under the same coordinate convention.

Required evidence:

~~~text
measured delta_T for one turn
measured delta_H
measured delta_V
~~~

Do not infer signs from “left” or “right”; use the displayed IMU delta.

### 2. Tracking-wheel scale is wrong

The code uses NEW_2 = 2.125 in. If the physical tracking wheel is a different wheel or effectively rolls at 2.0 in, the reported tracking distance is scaled.

The approximately 10-inch horizontal change for a 6-inch offset and 90-degree turn is compatible with some scale error, but scale error alone is unlikely to explain 14 in X and 9 in Y.

Calibrate by rolling a known distance and comparing physical distance to TrackingWheel distance. Do not change wheel diameter only because an arc was “around 10 inches.”

### 3. Tracking center does not equal actual turning center

Measuring from a chassis or IMU center is not automatically measuring the instantaneous center of rotation. During a motor-driven turn, unequal friction, wheel loading, traction, or alignment can cause the chassis to pivot around another point.

A 5.1-inch offset cannot explain a 16.6-inch resultant displacement by itself. If the robot actually pivots away from the assumed center, LemLib compensation will be wrong even if the tape measurement is accurate.

Mark the assumed tracking center and compare a slow manual turn with a motor-driven turn.

### 4. Tracking-wheel contact or scrub

Tracking wheels may lose contact, scrub, or be mounted with a small angular misalignment. Motor-driven turns expose this more than a push test.

Rotate slowly by hand with both tracking wheels on the floor, then repeat with both tracking wheels lifted. With wheels lifted, IMU-only rotation should not produce meaningful X/Y translation.

If drift appears only with tracking-wheel contact, inspect spring pressure, free rotation, alignment, and slipping.

### 5. Sensor assignment or port issue

The current source passes vertical, horizontal, and IMU in the expected OdomSensors order. Still verify that physical port 15 is the vertical sensor and port 1 is the horizontal sensor.

The push test suggests the ports are producing expected linear signs, but does not rule out a swapped physical orientation.

### 6. LemLib v0.5.x implementation behavior

The exact symptom is documented in v0.5.5. The current project is v0.5.6. Release notes prominently mention an angleError fix, not a documented odometry-turn compensation fix.

Sources:
- https://github.com/LemLib/LemLib/releases
- https://github.com/LemLib/LemLib/discussions/284

Do not conclude “library bug” until signed deltas and wheel-contact isolation are tested.

---

## Required controlled tests

Use the existing Brain screen. Do not turn the robot back before recording the first result.

### Test A: one single turn

1. Put the robot on a flat surface with both tracking wheels making stable contact.
2. Let odometry settle while stationary.
3. Record:

~~~text
X0, Y0, T0, V0, H0
~~~

4. Perform one slow -90-degree turn in place.
5. Stop and wait approximately 0.5 seconds.
6. Record:

~~~text
X1, Y1, T1, V1, H1
~~~

7. Calculate:

~~~text
delta_X = X1 - X0
delta_Y = Y1 - Y0
delta_T = T1 - T0
delta_V = V1 - V0
delta_H = H1 - H0
~~~

8. Repeat in the opposite direction, recording before turning back.

### Test B: manual turn, motors off

Rotate the robot slowly by hand with the drive motors off. This separates LemLib/sensor geometry from motor-driven pivoting and wheel scrub.

### Test C: tracking wheels lifted

Lift both tracking wheels just enough that they cannot rotate, then perform a slow turn. With IMU active and wheel inputs stationary, X/Y should stay nearly unchanged. This is only an isolation test, not a final odometry configuration.

### Test D: temporary zero-offset diagnostic

Only after normal signed data are captured, temporarily set both offsets to 0.0 and repeat one slow turn. This is diagnostic, not a final setting.

- If drift changes substantially, offset sign/magnitude/turn-center compensation is implicated.
- If drift is unchanged, focus on sensor signs, sensor assignment, IMU behavior, or wheel contact.

---

## Interpretation guide

### IMU delta is not actually -90 degrees

Use measured delta_T in every calculation and check IMU calibration/heading behavior.

### Horizontal delta has the opposite sign from the cancellation relation

The horizontal wheel sign is inconsistent with the IMU rotational convention. Re-evaluate the horizontal Rotation Sensor reversal and physical front direction before changing offsets.

### Signed ratios are correct but too large or small

Calculate:

~~~text
effective_horizontal_offset = delta_H / delta_T_radians
effective_vertical_offset   = delta_V / delta_T_radians
~~~

Compare these through LemLib's cancellation relation to H = +5.1 and V = -0.5. This indicates scale, gear ratio, or geometric offset error.

### Manual turn stable, motor turn drifts

The robot is not physically pivoting around the assumed center, or tracking wheels are slipping/scrubbing under drivetrain torque. Inspect drive traction, motor balance, pod pressure, and wheel alignment.

### Both manual and motor turns drift with wheels down, lifted-wheel test stable

Tracking-wheel data are causing the drift. Focus on rotational sign, wheel scale, offsets, and pod contact.

### Lifted-wheel test also drifts

Investigate IMU/pose update behavior, IMU calibration, heading sign, or LemLib implementation before changing tracking-wheel offsets.

---

## Safety and tuning rules

- Do not run the PID autotuner until a single 90-degree turn leaves X/Y nearly unchanged.
- A round-trip returning to the original pose is not sufficient; errors can cancel.
- Do not invert autonomous field coordinates to compensate for sensor signs.
- Do not keep changing offsets based only on final X/Y without recording signed T, V, and H.
- The 6.0 to 5.1 offset change can alter 90-degree compensation by only about 1.4 in; it cannot by itself explain a 14-by-9-inch jump.
- Do not run the tuner while a competition-control connection would trigger its safety abort.
- Preserve unrelated user work and make only bounded changes supported by measured evidence.

---

## What Astra should return

Please return:

1. A ranked diagnosis tied to measured signed values.
2. Exact expected signs for delta_H and delta_V relative to measured delta_T with current offsets.
3. Whether evidence points to rotational sensor sign, wheel scale, offset/turn-center geometry, physical slip, sensor assignment, or LemLib behavior.
4. One safest next physical test, not many simultaneous changes.
5. Any proposed code change as a minimal patch with the evidence supporting it.
6. A clear go/no-go decision for PID autotuning.

Immediate posture: no-go for PID tuning until the single-turn odometry drift is explained.

