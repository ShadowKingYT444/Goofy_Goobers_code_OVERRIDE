# Four-pin autonomous (slots 2 and 3)

`four_pin_far_auton` (slot 2, "4 pin far", quadrant 1) and
`four_pin_close_auton` (slot 3, "4 pin close", quadrant 2). Renamed from
"two-pin" on 2026-10-17: the route scores 2 cups, which counts as 4 pins.
Close is the far route mirrored in X about the start (`Side` in
`four_pin_far::run`), so the robot has to start at the mirror-image spot.

## Route (far side)
1. Toggle, identical to the three-pin.
2. Preload onto the alliance goal (goal at x = -20, scored from x = -11.25 on
   the lane y = 15).
3. Out to the staging point (8, 15), then claw first through (-3.4, 11.2)
   and a boomerang curve onto the wall stack behind the goal (recorded with
   the UP curve recorder, controller-driven). Clamp, lift to stage 1, curve
   back out nose first to the staging point, back into the goal, score.
4. Part 2, the mirror image (about x = 4): out to (0, 15), half turn to the
   mirrored heading (270), the mirrored approach onto the other wall stack,
   lift to 90% of stage 2, score on the neutral goal at x = 28.

## Shared wall-stack settings (both stacks, since 2026-10-17)
| Constant | Value | Notes |
|---|---|---|
| `kWallCurveLead` | 0.4 | 0.45 and 0.5 turned into the wall too early |
| `kWallDeeperIn` | 5.0 in | sent past the recorded stop (was 4.65 / 5.0) |
| `kWallOutIn` | 1.0 in | aimed farther out; both ended about 1 in inside the pin |
| `kWallStallWithinIn` | 0 | a stall only ends the approach once odometry has the claw at the pin |
| `kNeutralGoalExtraIn` | 0.5 in | neutral goal stops farther out: same base as the alliance goal (5.9 in), but it is 5.8 in tall vs 3.25 and stays wide where the robot touches it. The manual gives no widths, so this is an estimate |

## Brain report
Times at each step; for each wall stack: approach and stalled ms, short /
right / heading at the clamp, lift time and current, score offset, and one
GPS reading at the start of the approach and at the clamp (report only).
The header says "last step at" until the route finishes, so a run cut off at
15 s is obvious.

## Tuning log
| Date | Change | Result |
|---|---|---|
| 2026-10-17 | Approach ends on the first stall | Saved time, but the claw stopped on the cup 3 to 4.5 in short |
| 2026-10-17 | Stall only within 1.5 in of the pin; stack 2 lead 0.4; slow half turn | Both still short: timed out at 1.5 s pushing; run cut off before score 2 (stack 2 at 13.7 s) |
| 2026-10-17 | Shared constants, mirrored start, 1 in outward, margin 0 | "Now it works" |
| 2026-10-17 | Neutral goal 0.5 in farther out | For alignment on the wider neutral goal; not yet confirmed |

## Open
- The close side is unmeasured: it assumes field symmetry and a mirrored
  start spot.
- It runs close to 15 s; the two wall approaches are the biggest cost.
