# Motion control and PID journal

## Controllers (src/main.cpp)
- Drive (lateral) PID 6 / 0 / 3, turn (angular) PID 2 / 0 / 10. Hand-set.
- An automatic PID tuner suggested 42 / 0 / 93 for driving. On the robot that
  gave jerky starts and stops and overshoot, so it was reverted and the tuner
  removed (2026-10-14; it is in git history).
- kI = 0: an integral term keeps growing while the robot is blocked and then
  overshoots. Power floors and stall endings do its job instead.
- Exit conditions: within 1 in for 100 ms, or within 3 in for 500 ms. No
  slew.
- Lift: P control, power = 0.45 x error in degrees, floors 110 down and 60
  up (rubber bands pull the lift up), ends within 8 degrees. At or above 95%
  of max height the lift lock holds it at the top. A move cannot be sent to
  the very top: skills sends the top to max + 8 degrees and lets the lock
  finish it.

## LemLib 0.5.6 behaviour we rely on
- Within 7.5 in of the target, moveToPoint stops steering and moveToPose
  only holds the final heading. A sideways error left over from a curve is
  carried straight into the target.
- With `minSpeed` set, a motion ends when the robot crosses the line through
  the target. Without it, the robot must settle; if it is blocked short, it
  pushes until the timeout (this was the old "waits 1 s at the goal" bug).
- No stall detection in LemLib; `wait_done_or_stalled()` is ours: wheels
  under 12 rpm for 150 ms (after the first 300 ms) ends the move.
- Close to a target, drive power is about 6 x the distance left, so arrival
  speed is set by kP. For pickups we drop kP to 3 for the last 10 in, with a
  power floor of 24 ("gain scheduling", `set_lateral_kp`).
- Boomerang curves (moveToPose): the robot chases a "carrot" point that is
  `lead` x the distance back from the target, along the final heading.

## Chaining and smoothness
- Moves that hand over to the next one use a floor of 40 and end within 2 in;
  turns that hand over use a floor of 20 and end within 4 degrees. With a
  raised stack, turns use a floor of 10 and end within 2 degrees.
- 2026-10-17 lesson (skills): handing over while still rolling or turning
  made the next turn swing, and the robot came into goals crooked. The skills
  loader loop now uses settled moves and settled or near-settled turns, like
  the three-pin's preload drop-off. Slower, but repeatable.
- 2026-10-17 lesson (four-pin): ending a wall-stack approach on the first
  stall cut off the push that slid the claw over the pin. The stall now only
  ends the approach once odometry has the claw at the pin (margin 0).

## Simulations
- Kinematic models and time estimates made in sessions are estimates. The
  four-pin (then two-pin) gave the only measured calibration: the model was
  about 4% fast.

## 2026-10-17: calm moves for the skills loader loop
- Fast moves (127 on 24 in and 32 in runs) and turns with a power floor that
  hand over at 2 degrees overshot by several degrees and drifted a little
  more each trip. The loop now uses speed 60 for drives and 50 for turns,
  with every move settling, like the three-pin's preload drop-off (short
  moves that never reach full speed, settled turns).
- Raising the lift during the drive out of the loader meant the turn to G2
  was made with the stack high. Turns with a raised stack pivot about a
  point behind the center (known from the three-pin), so the line-up for G2
  was off. The turn now comes first, then the lift goes up.

## 2026-10-17: faster lift raises in skills
- `moveLift(target, timeout, cancel, fastUp)`: with fastUp, going up uses a
  gain of 1.0 instead of 0.45, so the lift stays at full power until about
  125 degrees from the target instead of about 280 (it still ends within 8
  degrees, motors on HOLD). Only skills' raises use it; the match routes keep
  the tuned 0.45.
