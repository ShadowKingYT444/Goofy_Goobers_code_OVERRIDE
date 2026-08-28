# Cascade Lift Position Tracking + PID Guide

## Scope

This document covers **only the cascade lift**:

- How to establish a reliable zero position
- How to track lift position when the sensor shaft rotates multiple times
- How to convert encoder position into physical lift height
- How to control the lift with PID
- How to add gravity feedforward
- How to prevent unsafe motion
- How to tune and validate the system

It does **not** cover wrist/claw control.

---

# 1. Core Problem

A VEX Rotation Sensor mounted to a shaft in the cascade mechanism may rotate several full revolutions as the lift travels from bottom to top.

That means the sensor's single-revolution angle alone is not enough.

For example, these positions all have the same angle within one revolution:

```text
32°
392°
752°
1112°
```

If the program only looks at angle modulo 360°, it cannot determine which physical lift position corresponds to the reading.

The solution is:

1. Establish one known physical position.
2. Define that position as zero.
3. Track the sensor's **multi-turn relative position** from that point onward.

The control system should therefore treat the lift as a **homed relative-position mechanism**, not as a purely absolute encoder mechanism.

---

# 2. Recommended Architecture

```text
                 physical cascade lift
                          │
                          ▼
                rotation sensor shaft
                          │
                          ▼
                multi-turn position
                          │
                          ▼
                 calibrated lift height
                          │
                          ▼
             target height / target position
                          │
                          ▼
                  PID + gravity FF
                          │
                          ▼
                      lift motors
```

A bottom limit switch is strongly recommended:

```text
bottom limit switch
        │
        ▼
known mechanical reference
        │
        ▼
encoder position = 0
```

---

# 3. Homing

## Why Homing Is Required

When the robot powers on, the Rotation Sensor knows how much it moves after startup, but the program does not automatically know the lift's physical location within its total multi-revolution travel.

Therefore the system must establish a known reference.

The simplest reference is:

```text
lift fully down = position 0
```

After homing, every encoder position corresponds to a unique lift position.

---

# 4. Best Homing Method: Bottom Limit Switch

Install a limit switch so it activates when the lift reaches its mechanical bottom.

Startup sequence:

```text
1. Command lift downward slowly
2. Wait until bottom switch activates
3. Stop the lift
4. Set rotation sensor position = 0
5. Mark lift as homed
```

Pseudo-code:

```cpp
void homeLift() {
    while (!bottomSwitch.pressing()) {
        liftMotor.move_voltage(-2500);
    }

    liftMotor.brake();

    liftRotation.set_position(0);

    liftHomed = true;
}
```

Use a low homing voltage. The lift should approach the bottom gently.

Do not slam it downward at full voltage.

---

# 5. Competition-Simplified Homing

If the robot always starts Skills with the cascade physically all the way down, you can simply zero the sensor at startup:

```cpp
liftRotation.set_position(0);
liftHomed = true;
```

This is simpler but less robust.

If someone starts the program with the lift slightly raised, every commanded height will be offset.

Therefore:

```text
limit-switch homing > manual starting-position assumption
```

For autonomous/Skills reliability, a physical reference switch is preferable.

---

# 6. Use Multi-Turn Position, Not Modulo-360 Angle

Do not control the lift using:

```cpp
rotation.angle()
```

if that value wraps every revolution.

Instead use the sensor's continuously tracked position:

```cpp
rotation.position()
```

or the equivalent API in your VEX/PROS wrapper.

Conceptually:

```text
bottom        0°
             │
             ▼
          360°
             │
             ▼
          720°
             │
             ▼
         1080°
             │
             ▼
top
```

The controller should work with the full multi-turn value.

---

# 7. Encoder Position to Lift Height

You can control directly in encoder degrees, but physical units are easier to understand.

Example calibration:

```text
Lift height      Encoder position
0 mm             0°
100 mm           183°
200 mm           366°
300 mm           550°
400 mm           733°
```

If the relationship is approximately linear:

```text
height = encoder_position × scale
```

where:

```text
scale = mm_per_degree
```

or equivalently:

```text
encoder_position = height × degrees_per_mm
```

Example:

```cpp
double encoderToHeight(double positionDeg) {
    return positionDeg * MM_PER_DEGREE;
}
```

and:

```cpp
double heightToEncoder(double heightMM) {
    return heightMM * DEGREE_PER_MM;
}
```

---

# 8. Empirical Calibration Is Better Than Pure Geometry

You can theoretically calculate lift travel from pulley diameter, spool diameter, string routing, and cascade ratio.

However, real mechanisms introduce:

- string stretch
- spool diameter variation
- imperfect cascade geometry
- bearing play
- elastic deformation
- wrapping effects
- mounting tolerances

So calibrate experimentally.

Recommended process:

```text
1. Home lift
2. Raise lift to measured physical positions
3. Record encoder value
4. Repeat across entire range
5. Determine whether mapping is linear
```

Collect something like:

```text
Height_mm, Encoder_deg
0,         0
100,       182
200,       365
300,       550
400,       733
500,       917
```

Fit a line:

```text
height = a * encoder + b
```

After homing, `b` should be close to zero.

If the errors are larger than expected, use a piecewise calibration table instead of assuming perfect linearity.

---

# 9. Position Control Objective

The lift controller receives a target height:

```cpp
setLiftHeight(420);
```

Internally:

```text
target height
    ↓
target encoder position
    ↓
current encoder position
    ↓
error
```

Define:

```text
error = target - current
```

If error is positive:

```text
lift must go upward
```

If error is negative:

```text
lift must go downward
```

---

# 10. PID Controller

Basic PID:

```text
output =
    Kp * error
  + Ki * integral(error)
  + Kd * derivative(error)
```

For a VEX cascade lift, start with:

```text
PD + gravity feedforward
```

before adding integral.

In many lift systems:

```text
P + D + gravity FF
```

is already enough.

---

# 11. Proportional Term

```text
P = Kp * error
```

The farther the lift is from the target, the more motor voltage is applied.

Example:

```text
target = 900°
current = 400°
error = 500°
```

Large error:

```text
large upward command
```

Near target:

```text
target = 900°
current = 890°
error = 10°
```

Small error:

```text
small correction
```

If `Kp` is too low:

- slow response
- lift may sag
- may never strongly approach target

If `Kp` is too high:

- overshoot
- oscillation
- violent corrections

---

# 12. Derivative Term

```text
D = Kd * derivative(error)
```

Derivative measures how quickly the error is changing.

Intuitively, it acts like damping.

Suppose the lift is approaching its target very quickly.

Even though position error is still positive, derivative recognizes:

```text
"we are closing on the target fast"
```

and reduces motor command before overshoot occurs.

Too little `Kd`:

- oscillation
- overshoot

Too much `Kd`:

- sluggish movement
- noisy output
- motor command may fluctuate if sensor readings are noisy

---

# 13. Integral Term

```text
I = Ki * accumulated_error
```

Integral compensates for persistent steady-state error.

For example:

```text
target = 700°
lift settles = 690°
```

Gravity or friction may cause the lift to stay slightly below target.

Integral slowly accumulates that 10° error until enough additional output is produced.

However, lift mechanisms often do better with explicit gravity feedforward instead.

Use integral only if necessary.

If used, protect against integral windup.

Example:

```cpp
if (fabs(error) < INTEGRAL_ZONE) {
    integral += error * dt;
} else {
    integral = 0;
}
```

Also clamp the integral:

```cpp
integral = std::clamp(integral, -MAX_I, MAX_I);
```

---

# 14. Gravity Feedforward

Gravity constantly pulls the cascade downward.

Without feedforward, PID must create a persistent positive error before producing enough motor torque to hold the lift.

Instead provide a small constant upward command:

```text
output = PID + kG
```

where:

```text
kG = voltage required to approximately hold the lift against gravity
```

Example:

```cpp
output =
    kP * error +
    kD * derivative +
    kG;
```

The exact `kG` must be measured experimentally.

---

# 15. Measuring kG

Raise the lift to approximately mid-height.

Then manually test increasingly large upward holding voltages.

Find the voltage where:

```text
lift approximately stops falling
```

Example:

```text
1000 mV -> falls quickly
1500 mV -> falls slowly
1900 mV -> approximately stationary
2200 mV -> rises slowly
```

Then:

```text
kG ≈ 1900 mV
```

This becomes the starting gravity feedforward.

Do not assume this number. Measure it.

---

# 16. Full Control Equation

Recommended first version:

```text
output =
    kP * error
  + kD * derivative
  + kG
```

Optional later version:

```text
output =
    kP * error
  + kI * integral
  + kD * derivative
  + kG
```

Clamp to motor limits:

```cpp
output = std::clamp(output, -12000.0, 12000.0);
```

---

# 17. Important Gravity Feedforward Detail

A constant upward `kG` is appropriate when the lift is actively holding or controlling height.

However, near the physical bottom you may not want the controller continuously pushing upward.

You can conditionally disable gravity compensation when the target is zero and the bottom switch is pressed:

```cpp
if (bottomSwitch.pressing() && targetHeight <= 0) {
    output = 0;
}
```

This prevents unnecessary motor heating.

---

# 18. Example Lift Controller

```cpp
struct LiftController {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kG = 0.0;

    double target = 0.0;

    double integral = 0.0;
    double previousError = 0.0;

    bool homed = false;
};
```

Control loop:

```cpp
void updateLift(double dt) {
    if (!lift.homed) {
        liftMotor.move_voltage(0);
        return;
    }

    double current = liftRotation.position();
    double error = lift.target - current;

    double derivative = (error - lift.previousError) / dt;

    if (fabs(error) < INTEGRAL_ZONE) {
        lift.integral += error * dt;
    } else {
        lift.integral = 0;
    }

    lift.integral =
        std::clamp(lift.integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    double output =
        lift.kP * error +
        lift.kI * lift.integral +
        lift.kD * derivative +
        lift.kG;

    output = std::clamp(output, -12000.0, 12000.0);

    if (bottomSwitch.pressing() && output < 0) {
        output = 0;
    }

    if (current >= MAX_LIFT_POSITION && output > 0) {
        output = 0;
    }

    liftMotor.move_voltage(output);

    lift.previousError = error;
}
```

---

# 19. Fixed-Rate Control Loop

Run the PID loop at a constant period.

Recommended:

```text
10–20 ms
```

Example:

```cpp
while (true) {
    updateLift(0.02);

    pros::delay(20);
}
```

A fixed timestep makes derivative and integral behavior much more predictable.

---

# 20. Target Commands

Avoid directly setting motor power from operator-control logic.

Instead change the lift target.

Example:

```cpp
if (controller.get_digital_new_press(BUTTON_HIGH)) {
    lift.target = HIGH_POSITION;
}

if (controller.get_digital_new_press(BUTTON_MID)) {
    lift.target = MID_POSITION;
}

if (controller.get_digital_new_press(BUTTON_LOW)) {
    lift.target = LOW_POSITION;
}
```

Then the lift control loop continuously drives toward that target.

---

# 21. Named Positions

Store commonly used locations:

```cpp
constexpr double LIFT_BOTTOM = 0;
constexpr double LIFT_LOW    = 350;
constexpr double LIFT_MID    = 680;
constexpr double LIFT_HIGH   = 1040;
```

Or use millimeters:

```cpp
constexpr double LIFT_BOTTOM_MM = 0;
constexpr double LIFT_LOW_MM    = 180;
constexpr double LIFT_MID_MM    = 370;
constexpr double LIFT_HIGH_MM   = 550;
```

Using physical units is usually cleaner.

---

# 22. Soft Limits

Never rely only on the driver's judgment.

Define software travel limits.

```cpp
constexpr double MIN_LIFT_POSITION = 0;
constexpr double MAX_LIFT_POSITION = 1100;
```

Then:

```cpp
if (current <= MIN_LIFT_POSITION && output < 0) {
    output = 0;
}

if (current >= MAX_LIFT_POSITION && output > 0) {
    output = 0;
}
```

This protects:

- string
- lift rails
- motors
- mechanical hard stops

---

# 23. Bottom Re-Zeroing

If you have a bottom limit switch, use it to periodically correct the encoder reference.

Whenever the switch activates:

```cpp
if (bottomSwitch.pressing()) {
    liftRotation.set_position(0);
}
```

This is useful because any small error caused by mechanical slippage can be removed whenever the mechanism returns home.

However, only re-zero when you are confident the switch represents the actual mechanical bottom.

---

# 24. Mechanical Slippage Is the Real Position-Tracking Risk

The encoder does not significantly "drift" just because time passes.

The larger risk is that:

```text
encoder shaft motion != actual lift motion
```

For example:

- string slips
- pulley slips on shaft
- shaft collar loosens
- gear skips
- spool attachment rotates independently
- cascade becomes temporarily bound

Then the encoder may report a correct shaft position while the physical lift is somewhere else.

Therefore mechanically lock all relevant components.

Preferred:

- keyed/hex shaft where possible
- properly tightened collars
- rigid sensor coupling
- no friction-fit tracking wheel
- no part that can independently rotate

---

# 25. Sensor Placement

Best-case sensor placement:

```text
sensor directly measures a shaft whose rotation is rigidly tied to lift movement
```

Bad placement:

```text
sensor driven by something that can slip independently of lift motion
```

If the sensor is on a spool shaft that directly drives cascade string and the spool cannot slip on the shaft, that is generally suitable.

---

# 26. Stall / Jam Detection

A useful safety layer is comparing:

```text
commanded motor output
```

against:

```text
actual encoder movement
```

Example:

```text
motor voltage > 7000 mV
encoder velocity ≈ 0
for > 300 ms
```

This may indicate:

- mechanical jam
- lift at hard stop
- string failure
- object blocking mechanism

Pseudo-code:

```cpp
if (fabs(output) > 7000 &&
    fabs(liftVelocity) < MIN_EXPECTED_VELOCITY) {

    stallTimer += dt;

    if (stallTimer > 0.30) {
        liftFault = true;
        liftMotor.move_voltage(0);
    }
} else {
    stallTimer = 0;
}
```

---

# 27. Target Tolerance

The lift does not need to mathematically reach exactly zero error.

Define a tolerance.

Example:

```cpp
constexpr double POSITION_TOLERANCE = 5.0;
```

Then:

```cpp
bool liftAtTarget() {
    return fabs(target - current) < POSITION_TOLERANCE;
}
```

For autonomous sequencing, use both position and velocity:

```text
position close to target
AND
velocity low
```

Example:

```cpp
bool liftSettled =
    fabs(error) < POSITION_TOLERANCE &&
    fabs(liftVelocity) < VELOCITY_TOLERANCE;
```

This prevents the autonomous routine from advancing while the lift merely passes through the target at high speed.

---

# 28. PID Tuning Procedure

Tune in this order:

```text
1. kG
2. kP
3. kD
4. kI only if needed
```

Do not tune all gains simultaneously.

---

# 29. Step 1 — Tune Gravity Feedforward

At mid-height:

1. Disable PID.
2. Apply constant upward voltage.
3. Increase until the lift approximately holds.
4. Record voltage.

That is your starting `kG`.

---

# 30. Step 2 — Tune kP

Set:

```text
kD = 0
kI = 0
```

Command moderate movements such as:

```text
bottom → middle
middle → high
high → middle
```

Increase `kP` until the lift becomes responsive.

Stop increasing once it begins to noticeably overshoot or oscillate.

Then reduce slightly.

---

# 31. Step 3 — Tune kD

Increase `kD` gradually.

Desired effect:

```text
fast approach
little overshoot
quick settling
```

If the lift starts reacting noisily or becomes sluggish, `kD` is too high.

---

# 32. Step 4 — Add kI Only If Necessary

After `kP + kD + kG` is tuned, check steady-state error.

If the lift reliably settles several degrees below or above the target, first verify:

- `kG` is correct
- friction is not excessive
- mechanism is mechanically consistent

Only then add a small `kI`.

Integral should usually be the smallest component.

---

# 33. Tune Across the Entire Range

Do not tune only at one position.

Test:

```text
bottom → low
bottom → high
high → middle
high → bottom
middle → high
middle → low
```

Cascade friction and loading may vary with height.

A controller that looks excellent around the midpoint can behave badly near the top or bottom.

---

# 34. Telemetry Dashboard

Display or log:

```text
target position
current position
error
velocity
PID output
P contribution
I contribution
D contribution
kG contribution
bottom switch state
homed state
motor current
motor temperature
```

Example console output:

```text
LIFT
Target:     720 deg
Position:   713 deg
Error:        7 deg
Velocity:     3 deg/s

P:          420 mV
I:            0 mV
D:         -110 mV
G:         1850 mV

Output:     2160 mV

Bottom:       false
Homed:        true
Current:      1.8 A
Temp:         41 C
```

This makes tuning much faster than judging visually.

---

# 35. Data Logging

For serious tuning, log:

```text
time
target
position
velocity
error
motor voltage
motor current
```

Plot:

```text
position vs time
```

and:

```text
target vs position
```

You want:

```text
rapid rise
small overshoot
minimal oscillation
short settling time
```

---

# 36. Skills Reliability Strategy

At the beginning of Skills:

```text
home lift
        ↓
confirm homed
        ↓
begin autonomous / driver routine
```

During the run:

```text
encoder continuously tracks multi-turn position
```

Whenever lift returns to bottom:

```text
bottom switch activates
        ↓
re-zero encoder
```

This provides repeated reference correction.

---

# 37. Failure Detection

Do not allow autonomous commands if homing failed.

Example:

```cpp
if (!liftHomed) {
    liftMotor.move_voltage(0);
    return;
}
```

Homing should also have a timeout.

Example:

```text
if bottom switch not reached within 2 seconds:
    stop motor
    declare lift fault
```

This prevents the robot from continuously driving downward if the switch breaks.

---

# 38. Homing Timeout Example

```cpp
bool homeLift() {
    int start = pros::millis();

    while (!bottomSwitch.pressing()) {

        if (pros::millis() - start > 2000) {
            liftMotor.move_voltage(0);
            liftHomed = false;
            return false;
        }

        liftMotor.move_voltage(-2500);

        pros::delay(10);
    }

    liftMotor.move_voltage(0);

    liftRotation.set_position(0);

    liftHomed = true;

    return true;
}
```

---

# 39. Suggested Software Interface

Keep low-level motor logic inside the lift subsystem.

Useful functions:

```cpp
void liftHome();

void liftSetHeight(double mm);

void liftSetPosition(double degrees);

double liftGetHeight();

double liftGetPosition();

bool liftAtTarget();

bool liftIsHomed();

void liftUpdate();

void liftStop();
```

Operator code should call:

```cpp
liftSetHeight(450);
```

rather than manually commanding voltage.

---

# 40. Recommended State Variables

```cpp
double targetPosition;

double currentPosition;
double previousPosition;

double error;
double previousError;

double integral;
double derivative;

double velocity;

bool homed;
bool faulted;
```

---

# 41. Optional Motion Profiling

A raw PID controller may command a very aggressive jump:

```text
target:
0° → 1000°
instantly
```

The resulting error is huge, so the controller initially saturates.

This may be acceptable, but for a tall cascade you can improve smoothness with a motion profile.

Instead of jumping:

```text
0° → 1000°
```

the commanded setpoint moves gradually:

```text
0
50
100
150
200
...
1000
```

This limits:

- acceleration
- jerk
- string shock
- frame flex
- wheel unloading caused by fast lift movement

Do this only after basic PID is working.

---

# 42. Simple Slew-Limited Target

A lightweight alternative to a full trapezoidal motion profile:

```cpp
double commandedTarget = 0;
double requestedTarget = 0;

void updateTarget() {

    double delta = requestedTarget - commandedTarget;

    delta = std::clamp(
        delta,
        -MAX_TARGET_CHANGE_PER_LOOP,
         MAX_TARGET_CHANGE_PER_LOOP
    );

    commandedTarget += delta;
}
```

PID tracks `commandedTarget`, not `requestedTarget`.

---

# 43. Brake Mode

Test which brake mode works best mechanically.

Common strategy:

```text
during active PID:
motor voltage control

when disabled:
brake or hold depending on mechanism
```

Do not rely on motor hold mode as your primary position controller if you already have a proper PID loop.

Your own controller should determine the lift behavior.

---

# 44. Practical Control Loop Structure

```cpp
void liftTask() {

    while (true) {

        // 1. Read sensors
        double position = liftRotation.position();

        // 2. Re-reference if bottom switch is active
        if (bottomSwitch.pressing()) {
            liftRotation.set_position(0);
            position = 0;
        }

        // 3. Safety check
        if (!liftHomed || liftFault) {
            liftMotor.move_voltage(0);
            pros::delay(20);
            continue;
        }

        // 4. Calculate control error
        double error = lift.target - position;

        // 5. PID calculations
        ...

        // 6. Add gravity compensation
        ...

        // 7. Enforce limits
        ...

        // 8. Command motors
        liftMotor.move_voltage(output);

        // 9. Store state / telemetry
        ...

        pros::delay(20);
    }
}
```

---

# 45. Recommended Development Order

Implement the system in this order.

## Phase 1 — Verify Sensor

Display:

```text
rotation sensor position
```

Move lift manually from bottom to top.

Confirm:

- reading increases consistently
- no wrap at 360°
- direction is correct
- return to bottom produces approximately the original reading

---

## Phase 2 — Add Homing

Implement bottom reference.

Verify:

```text
home → sensor = 0
```

Repeat several times.

Expected:

```text
home #1 = 0
home #2 = 0
home #3 = 0
```

---

## Phase 3 — Calibrate Height

Record physical height versus encoder position.

Determine conversion.

---

## Phase 4 — Add Manual Voltage Testing

Before PID, command known voltages.

Understand:

- minimum voltage required to move upward
- voltage needed to hold
- downward response
- motor current

---

## Phase 5 — Add kG

Determine holding voltage.

---

## Phase 6 — Add P

Tune responsiveness.

---

## Phase 7 — Add D

Reduce overshoot and oscillation.

---

## Phase 8 — Add Safety

Implement:

- min position
- max position
- bottom switch
- homing timeout
- stall detection

---

## Phase 9 — Add Named Heights

Example:

```text
BOTTOM
LOW
MID
HIGH
```

---

## Phase 10 — Run Full Skills-Length Test

Run the mechanism repeatedly for at least the duration of a full Skills run.

Test:

```text
bottom → high
high → middle
middle → bottom
bottom → high
...
```

Check whether the reported bottom remains aligned with physical bottom.

---

# 46. What Accuracy Should You Expect?

If the mechanism is rigid and the encoder shaft cannot slip, relative multi-turn tracking should be very repeatable across a one-minute run.

The main sources of error are usually mechanical rather than electronic:

```text
string stretch
slippage
frame flex
cascade binding
backlash
sensor coupling
hard-stop inconsistency
```

A bottom reference switch removes long-term reference error whenever the lift returns home.

---

# 47. Important Distinction

The sensor does **not** need to know the absolute multi-turn lift location directly.

It only needs:

```text
known initial physical state
+
accurate incremental rotation tracking
```

Mathematically:

```text
current_position
=
known_start_position
+
measured_rotation_since_start
```

Because:

```text
known_start_position = 0
```

after homing:

```text
current_position
=
measured_rotation_since_home
```

That is the entire basis of the lift position estimate.

---

# 48. Final Recommended System

```text
                     ┌──────────────────┐
                     │ Bottom switch    │
                     └────────┬─────────┘
                              │
                              ▼
                           HOMING
                              │
                              ▼
                     encoder position = 0
                              │
                              ▼
                  ┌───────────────────────┐
                  │ Rotation Sensor       │
                  │ multi-turn position   │
                  └───────────┬───────────┘
                              │
                              ▼
                     calibrated height
                              │
                              ▼
                       position error
                              │
                              ▼
                    ┌─────────────────┐
                    │ PID Controller  │
                    │ P + D (+ I)     │
                    └────────┬────────┘
                             │
                             │ + kG
                             ▼
                      motor voltage
                             │
                             ▼
                       cascade lift
```

Recommended controller:

```text
P + D + gravity feedforward
```

with:

```text
bottom homing
multi-turn encoder position
software travel limits
bottom re-zeroing
stall detection
fixed 10–20 ms loop
telemetry
```

This is a robust architecture for repeatable VEX cascade lift positioning during autonomous and Skills runs.
