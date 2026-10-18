# Skills (slot 4, `skills`)

## Plan (full route, `-DSKILLS_FULL_ROUTE`)
The three-pin (unchanged, same code), then match loads onto G2, the west wall
stack onto G2, match loads onto G3, the bottom wall stack onto G3, the
bottom toggle and a park in the center. Each goal ends with a pin and five
cups. A clock check skips steps that would not leave time for the toggle and
park. The whole list was estimated at about 87 s against 60 s, so it needs a
cut list.

## Current build (2026-10-17)
The three-pin (unchanged), then match loads onto G2 until it holds a pin and
5 cups (3 loads), then 5 match loads onto G3 (a pin dropped like the
three-pin's preload, then cups at levels 1 to 4), then the bottom toggle and
a park just south of the center goal. The clock check is off for testing
(`kUseClock`), so every step runs and its time shows on the report. Wall
stacks are not in the plan (`kWallPins = false`).

### One G2 trip
1. **Line-up spot** (-48, -57.0), nose south (no turn if already within 3
   degrees). Distance-sensor reset off the south wall: field Y only. If it
   moved odometry more than 0.3 in, drive straight forward or back onto the
   spot.
2. Turn the claw square to the loader (field heading 90), back in, end on
   stall. **Loader contact reset:** field X = -58.6.
3. Wait 0.25 s, clamp, lift to -150 (just off the floor).
4. Drive out nose first to the line-up spot, turn the claw to G2 (the load
   still low), then raise the lift and arm (fast raise).
5. Back straight north into G2. 6 in short of contact the robot holds until
   the lift is within 25 degrees of its height and the arm within 20 (at
   most 2.5 s), then goes in. Goal reference only if stalled against it and
   it moves odometry 4 in or less.
6. Release; the lift goes all the way down while the robot drives straight
   back south to the line-up spot.

### One G3 trip
As for G2 up to step 3, then: drive out nose first to the line-up spot, turn
nose south and take a second wall reset (G3's run is east-west, so this Y is
its sideways place), back straight north to kG3Lane (-48, level with G3),
turn the claw east, raise, back into G3 (same hold rule), release, and come
back out nose first to kG3Lane. G3 is placed one tile (23.62 in) east and
south of G2.

### Toggle and park
From kG3Lane to the line-up spot, east along the lane to (-6, -59.5), nose
south, lift to the preload carry height, hitter out, drive into the wall and
stop on stall (as in the three-pin), then back north claw first through
(-12, -30) to (-8, -12).

Every move is capped at 60 (turns 50) and stops crisply (low power floor at
the point, then the brakes), not with LemLib's slow settle.

### Cup heights (lift sensor degrees, up is negative)
| On the goal before | Lift | Arm | Release |
|---|---|---|---|
| pin + 2 cups (3rd cup) | -1639.3 (= -1150 + (-1150 - -660.7)); the lift lock holds it at the top, -1670 | rest | drop a third of a level, open |
| 3rd cup (4th cup) | -1639.3 | Y position, 530 | like RIGHT: arm toward 100, open halfway there, 100 ms, lift down |
| 4th cup (5th cup) | full height (sent to -1662, the lock holds -1670) | 530 | like RIGHT |

G3 uses the same table from the bottom: the pin (carried at -375, set down
at -75 like the three-pin's preload), then cups 1 to 4.

The 5th was the 4th's height at first and was too low.

## Localization history (details in localization.md)
| Date | Approach | Result |
|---|---|---|
| 2026-10-17 | Loader lane at the guide's y = -59 | Reached the loader 3 in to the robot's right; lane moved to -56 |
| 2026-10-17 | GPS reset applied at the line-up; turns handed over while rotating | Reached the loader 20 degrees crooked |
| 2026-10-17 | Settled turns, moveToPose square onto the loader | Heading 85, "a bit off" |
| 2026-10-17 | GPS measure only; goal reference every trip; settled straight moves | Odometry X at the loader drifted 3.7 in over 3 trips; physical position shifted 2 in; G2 missed on some trips |
| 2026-10-17 | Loader X contact reset; goal reference only when stalled | Still drifting |
| 2026-10-17 | Two front distance sensors (ports 1, 3): heading from their slope, reset twice per trip | Slow; after the first load off, G2 missed |
| 2026-10-17 | Sensors for distance only, IMU heading, one reset per trip (see localization.md) | Resets refused "not square" after the turn overshot; trips 2 and 3 missed G2 partly |
| 2026-10-17 | Calm, settled moves (60 / 50); turn to G2 before raising the lift | Lined up on G2 well; far too slow; reached the loader about 1 in to the robot's left |
| 2026-10-17 | Speeds 100, crisp braked stops, 0.15 s wall reset; loader lane y -56 to -57 | Consistent (resets working); still too slow; too fast into G2 for the lift; on the loader's edge |
| 2026-10-17 | Speeds 60 / 50 with braked stops; lift and arm checked by position; 0.1 s reset; loader lane y -58.5 | Reached a good line-up spot, then swung while turning/backing in and ended where it did before |
| 2026-10-17 | After the wall reset, drive straight along the robot's axis back onto the line-up spot before turning; turn touch-up if more than 1.5 degrees off; three-pin last-score fix | Very consistent; claw a hair past the loader (0.75 in) |
| 2026-10-17 | Line-up 0.75 in back (-57.75); hold short of the goal until lift and arm are measured at height; fast lift raises; loader wait 0.25 s; reset samples while braking; G3 + toggle + park added | Everything consistent; loader still off by the same 0.75 in the same way |
| 2026-10-17 | Loader lane another 0.75 in (-57.0) | Not yet run |

## Report (end of the test build)
Trip times; per trip: where odometry had the robot when it stopped at the
loader and how far the loader contact moved it; the goal reset (or "skip");
then the wall reset: both sensors' readings (about 10.1 in expected), the
sample scatter, and the change to Y (or why it was refused).

## Open
- Confirm the loader contact X (-58.6). Sensor offset is about 6 in
  (`kSensorFwdIn`).
- Where the stack sits with the arm at Y (cups 4 and 5): if it moves the
  stack forward or back, those scores need a different stopping distance.
- Real heights for cups 4 and 5 (X in driver control lists lift and arm).
- The rest of the route, the park rule, and a cut list to fit 60 s.
