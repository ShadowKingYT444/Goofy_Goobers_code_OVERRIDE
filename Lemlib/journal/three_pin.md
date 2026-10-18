# Three-pin autonomous (slot 1, `three_pin_auton`)

## Route
Start-relative frame: start (0, 0), heading 180, facing the toggle wall.
The claw is at the rear, so every pickup and score is driven claw first.

1. **Toggle:** back to (0, 8) while the lift rises; once the lift is 80% up,
   the hitter goes out and the robot drives into the wall to (0, -2), ending
   on stall.
2. **Preload:** back out to (0, 15), turn to 270, back to (12, 15) (8 in from
   the goal at (20, 15)), lower the lift to -75 to seat the pin, open the
   claw. Pull straight out (chained).
3. **Stack 1** at (20.1, 36.79): aim the claw, fast approach (floor 45), last
   10 in at kP 3 with floor 24, clamp 8 in short of the stack. Lift to
   stage 1 while carrying on 5 in, turn to the goal (raised-stack turn:
   floor 10, 2 degrees), run in to 1 in past the goal center, end on stall,
   drop a third of a level, open, reset position from the goal.
4. Pull out 10 in (chained).
5. **Stack 2** at (43.5, 13.25): same pickup, clamping 0.3 in deeper; GPS
   checkpoint on the clamp; carry on to the goal's line; turn aiming at
   (0, 14.75); run in, score at stage 2, reset from the goal.
6. Pull out 8 in. Ends about 8 in south of G2, nose south (skills continues
   from here).

Fits in 15 s (14.4 to 14.8 s in the last runs).

## Brain report
`3 PIN took`, Grab n short (odometry distance left to the clamp spot) and
time of the slow stretch, S1/S2 aim (claw vs goal after the turn) and short,
"Goal ref moved", GPS at grab 2.

## Tuning log
| Date | Change | Why / result |
|---|---|---|
| before 2026-10-14 | Constants field-tuned; key GPS resets off | Back-to-back runs: with resets it missed stack 1, without it did not |
| 2026-10-17 | Stack 2 from 42.5 to 43.5 in out | Claw closed just short of stack 2 |
| 2026-10-17 | Score runs sent only to the contact point (center 7 in from goal) | Tried to stop it swerving into the goal at the end. Every score after the first undershot and the goal reference put odometry off. **Reverted** to 1 in past the center |
| 2026-10-17 | Stack 2 last stretch 0.3 in deeper (`kStack2DeeperIn`) | Report: Grab 2 short 0.5 in; on the field "a hair" short of the pin |
| 2026-10-17 | Last score: really stop before and after the raised-stack turn (`stopAroundTurn`, at most 0.15 s each) | The final score often missed G2. After the seat move the code braked and set coast mode on the next line, so the robot slid into the turn at the seat move's floor (45); the turn then ended 2 degrees early still rotating and coasted past its aim with the stack raised. Costs up to 0.3 s; watch "took" stays under 15 s |

## Last field report (2026-10-17, before the 0.3 in change)
`3 PIN took 14.5 s`, Grab 1 short 0.2 (0.7 s), Grab 2 short 0.5 (0.7 s),
S1 aim -7.0 short 0.4 ok, S2 aim -3.3 short 0.3 ok, goal ref moved
x+0.0 y+8.5, GPS at grab 2 moved 2.0 in.

## Open
- The final score sometimes drifts to one side of the goal ("swerving").
  Cause: the run in keeps steering at a point 1 in past the goal center
  while the robot is against the goal. Changing the target depth broke the
  route (see above), so a fix has to leave the depth alone.
