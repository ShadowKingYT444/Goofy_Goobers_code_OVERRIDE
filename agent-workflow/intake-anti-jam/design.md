Implement the anti-jam logic directly in `src/main.cpp` because the current intake behavior is inline in `opcontrol()`.

Use a compact state struct for calibration and jam state:

- sample current, torque, and absolute rpm for both intake motors every 20 ms.
- start calibration on a rising edge of controller UP.
- during calibration, spin the intake forward for two seconds while feeding a normal cup, collect mean load, then set current and torque thresholds 50% above that mean.
- detect a jam only when command is nonzero, the anti-jam sequence is not already running, average rpm is below threshold, and current or torque is above threshold for at least 250 ms.
- when jammed, override driver intake command with alternating 50 ms reverse/forward pulses for 20 phases, which is 10 reverse pulses and 10 forward pulses, about one second total.

Use nonblocking timing based on `pros::millis()` so drivetrain and other mechanisms continue updating during the anti-jam sequence.
