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

Frames: skills positions are in field coordinates (red side: +X east, +Y
north). The three-pin's start frame has x along the toggle wall and y away
from it; three-pin x = -4 - field Y and three-pin y = field X + 63. Along
the loader wall toward its corner is field -Y (three-pin +x).

### One G2 trip
1. **Line-up spot** (-48, -59.25), nose south (no turn if already within 3
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

### 2026-10-18 speed-up (PR #2 plus switches)
- **PR #2 merged** (measured "stack up" checks, the arm's time limit
  enforced inside its own task, one 4 s limit per score shared by both
  checks, stall detection from the start of the move, last G3 score lowers
  only to toggle height, timing totals on the Brain), with five changes so
  no check ends the run where the old code carried on: the goal aim never
  aborts (one slow touch-up instead), pickup carries on after 0.4 s, the
  seat drop and the arm release open the claw anyway, and an arm move that
  stops short gets one retry (arm raises get 2.5 s).
- **Lift during the goal turn** (`kRaiseDuringTurn`): the lift starts up to
  stage 1 (-660.7) while the robot turns to the goal; the rest of the climb
  and the arm start after the turn. Higher during a turn makes the robot
  pivot about a point behind its center.
- **Tighter loader** : loader wait 0.15 s (was 0.25), stall at the loader
  100 ms (was 150), drive out once the lift has risen 40 degrees (was 60).
- **Diagonal G3** (`kG3Diagonal`): from the line-up spot the robot turns to
  nose south (wall reset, fresh Y), then turns the claw straight at G3 and
  backs in on the diagonal; it comes back the same way to the line-up spot.
  No north leg, one turn and one stop fewer each way.
- Speeds unchanged (60 / 50).

### One G3 trip (older path, `kG3Diagonal = false`)
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
| 2026-10-17 | Loader lane another 0.75 in (-57.0) | Not uploaded. Wrong way: -57.75 and -57.0 moved the robot away from the loader corner (a misread "go back") |
| 2026-10-17 | Loader lane -59.25 (0.75 in past -58.5, toward the loader corner) | Not yet run |
| 2026-10-18 | PR #2 with fixes; lift starts during the goal turn; tighter loader; diagonal G3 | Consistent, but stops beside the loader (red X): ~3.9 in short toward the corner, ~2 in deeper (against the wall); waits 1 to 2 s short of goals with the stack already up |
| 2026-10-18 | Loader stop 3.9 in toward the corner (y -63.15; line-up stays at -59.25 for turn clearance, curve in with lead 0.5); lift and arm go straight up at the clamp (lift gain 2.0 going up); no hold short of goals (`kStackGates = false`) | Overcorrected: now too far toward the corner (blue X). Some line-up stops took about 3 s |
| 2026-10-18 | Loader stop halfway (y -61.2, 1.95 in toward the corner); wall reading at G2 instead of the line-up stop; turns end on a stall, touch-up is a 0.4 s nudge | Positions right; still about 3 s standing at the line-up after coming back from a goal; lift raised so fast at the clamp the load caught the top of the loader |
| 2026-10-18 | Back into the loader once the lift is above -400 and the arm within 60 of rest (they finish on the way; hold 4 in short only if still not down); no lift wait at the goal (a raise still finishing is stopped where it is, taken to be at height); at the clamp the lift only rises 60 degrees, full raise (lift and arm) after 2 in out of the loader | Every trip after the first G3 score missed the loader |
| 2026-10-18 | No goal reference at G3 (measured only, "goal m1.2" on the report): G3's place is only assumed and the empty goal gets pushed by the pin score | Loader fixed; the 4th/5th cups missed (the RIGHT macro lowers the arm first, which swings the claw out); wall resets still slow; toggle and park slow |
| 2026-10-18 | 4th/5th cups set down like the others (drop a third of a level, open, lift down; arm to rest once clear in leave_to); no wall reset before the G3 run; line-up reset only if the last trip had no wall fix (G2 trips always get one at G2, so it alternates on G3 trips); toggle and park at full speed with hand-overs, hit ends on a 60 ms stall, lift only within 40 of the toggle height | 5th cup too low, knocked the stack over; 1 to 2 s standing at each goal; resets still most of the time, more on G3 trips; everything too slow |
| 2026-10-18 | The top level was low because the raise was stopped at contact while still climbing (it starts 2 in out of the loader now); it is the lift's top already (-1670, the lock holds it), so it cannot go higher. Now a light check 6 in short: hold only while the lift is more than 25 below its height (not in the lock band) or the arm more than 60 off. Goal contact: 30 rpm for 60 ms from 0.2 s (was 12 rpm for 150 ms from 0.3 s: wheels creeping on the goal kept it pushing 1 to 2 s). Line-up wall reset only when due: after one trip without a fix, or two if the last fix moved odometry under 0.75 in; re-place only past 0.75 in (was 0.3). Drives 66, turns 55 (10% faster) | Sometimes ends the three-pin twisted on G2 and the rest goes wrong; still slow |
| 2026-10-18 | Reset right after the three-pin: stop, square nose south, GPS checkpoint (both coordinates; within 8 in, applied past 1 in), then the south wall (Y, up to 8 in); a wall fix there spares trip 1 its line-up reset. Report line "After 3 pin: GPS ... wall ...". Drives 70, turns 60. GPS here looks west at the west wall about 22 in away, and the mount has not been surveyed: whether it gets a fix there is to be seen on the report | Filmed (20261018_174801.mp4, see below) |
| 2026-10-18 | From the video: the lift must be within 15 of its height and the arm within 30 before the last 6 in (no lock-band shortcut), and at contact it waits (at most 1 s) if not; third and fourth cups at the very top (-1662, the lock holds -1670); the loader ends on the quick contact check (30 rpm, 60 ms); two loads on G2 (pin and four cups on each goal); reset after the three-pin starts the lift down first, turns only if more than 4 degrees off, GPS 0.25 s; toggle and park at calm speeds with crisp stops, passing G3 1.5 in further south | Filmed (20261018_182937.mp4): stacks good, about 11 s too long; the reset after the three-pin about 4 s; trip 2 missed the loader by about 0.5 in; stack 2 of the three-pin taken tilted |
| 2026-10-18 | No stop after the three-pin (straight on to the line-up spot, the wall read there as it stops, up to 8 in on trip 1); the wall is read at the line-up every time the robot arrives nose south (from G2: about 0.1 s, no turn), and no longer from G2 (34 in, `kWallResetAtGoal = false`); no wait for the lift before backing into the loader (the 4 in hold covers it); drives 85, turns 70 | Too hard into the goals (top-heavy, base not settled before scoring); the lift drops too far before letting go; G3 still misaligned now and then; still some 3 s stops |
| 2026-10-18 | Last 6 in into a goal at 40 (`kGoalFinalSpeed`), then 0.2 s standing (`kGoalSettleMs`) before the load goes down; release drop halved (a sixth of a level for a cup; the pin on an empty goal to -225, was -75; the three-pin's own drops unchanged); every hold for the lift or arm capped at 1 s (`kHoldCapMs`; at most one per score; at the loader no longer ends the route, and an arm stopped short gets one more command; "capped" count on the report); G3 aim reads the south wall as the nose passes south (turn eases off there, no stop, `aim_reading_wall`; second wall column on G3 trips); aim touch-up goes straight to the nudge (no second full turn) | Third cup far too high (at the very top); G3 still missed now and then, the chassis caught on the goal's edge |
| 2026-10-18 | Third cup back to its own height (-1639) and no lift lock after the three-pin (the lock pushed anything above 95% of travel to -1670 during the new settle); G3 trips: the lift only to the first cup's height (arm down) through the first half of the claw's ~150 degree turn to G3, the full raise and arm from where the turn passes south (less pivoting with a high load, which moves the robot off its odometry) | Filmed (20261018_203818.mp4): heights good; turn to G3 a few degrees past and the chassis caught on G3 (cups 2 and 3 missed); the lift already at the top before the run to G2; G2's fourth cup (arm up) fell off; about 5 s pushing on the loader on trip 5 |
| 2026-10-18 | Lift only just off the floor through the drive out and the turn; the full raise starts after the turn, during the run in (the 6 in check still holds, at most 1 s); last 6 in into a goal is a moveToPose holding the heading of the line through the goal's center (claw arrives square), and contact also counts 3 in past the contact point by odometry (`kPastContactIn`); aim nudge from 1 degree (was 1.5); loader run at most 1.5 s (was 2.5) and also ends 3 in past the loader's face by odometry; toggle hitter in after the three-pin's preload (skills only, `run_three_pin(lower_hitter)`), out again after the last load | Filmed (20261018_210231.mp4, see below): about 72 s; 2 to 3 s at the loader every trip; G2's fourth cup knocked the stack's top off; G3 cups 1 and 2 missed to the south, cups 3 and 4 then stopped on the fallen cups |
| 2026-10-18 | Turn to G3 stops nose south and reads the south wall (stopped, every G3 trip; was a reading on the fly) and aims 3 degrees short of G3 (`kG3AimTrimDeg`: "turn less"); aim nudge only past 2 degrees, ending 1 degree short; full raise starts as the turn to the goal begins; loader: no hold after the claw has closed, the hold 4 in short re-lowers a lift stopped above the line, a missing arm reading no longer blocks "down", the run ends on no progress too (and is no longer stuck in waitUntil when it meets the loader early), 1.2 s limit, pickup raise fast and at most 0.3 s; a missing arm reading no longer counts as "up" at a goal, and loads that need the arm are skipped while the arm sensor gives nothing (report line); per-trip loader time on the report ("L") | Run by the team: everything good except the turn to G3 still ends counterclockwise of where it should (team's drawing: blue = now, red = wanted, pointing at G3's south side); error still builds up on the G3 trips; height 4 a little high |
| 2026-10-18 | G3 aimed at a point beside its center, from the team's drawing (`kG3Aim`: 4 in south, 1 in west of G3's center; about 7.5 degrees further clockwise from the line-up than the center; replaces the 3 degree trim the other way); the run in goes toward it, with the stop, the hold and the past-contact line measured from where the claw meets G3's center; G3's goal reference applied from the first cup on (at most 2.5 in, from G3's center; a refused one or one over 0.75 in makes the next line-up wall reading due), the pin's own score still only measured; height 4 (third cup) 30 degrees lower (-1609) with 0.5 s more standing on the goal before it is set down | Run by the team: G3 aim still off (wants it 1.5 in further north); height 4 still way too high |
| 2026-10-18 | `kG3Aim` 1.5 in further north (2.5 in south of G3's center, 1 in west); height 4 another 150 degrees lower (-1459), still 0.5 s more on the goal | Not yet run |

### Fourth video (2026-10-18, `20261018_210231.mp4`, 105 s)
Studied at 1, 4 and 10 frames a second, with close crops of the loader, G2
and G3.
- Run from about 13.3 s to parked at about 85 s: about 72 s. Three-pin to
  about 29.5 s; trips of about 7 s each; toggle about 83.5 s, park 85 s.
- Loader, every trip: 2 to 3 s. On trip 5 the robot stopped about 4 in
  short of the loader for about 1 s (the person loaded the cup meanwhile),
  then backed on and stood against it about 2.2 s before leaving. That is
  both "down" holds running to their 1 s cap (before the run in, and after
  the claw had closed).
- G2 trip 2 (fourth cup, arm level): the lift only started up after the
  90 degree turn, the robot did not hold 6 in short, and the cup arrived
  level with the stack's top with the claw arm still down; it knocked the
  top cups off (about 41.4 s).
- Both point to the arm's rotation sensor (port 12) giving no reading:
  then the arm's moves fail at once (the arm never rises), "down" at the
  loader can never be confirmed (both holds run out), and the check before
  the goal treated a missing reading as "up". Check port 12 on the Brain's
  device screen; the report now has a line for it.
- G3: pin fine (49.5 s). Cup 1 (56.3 s) was set down on the south side of
  the pin and fell; cup 2 (64 s) stopped short on the fallen cup and fell
  beside the goal; cups 3 and 4 were stopped by the cups on the floor and
  never reached the goal. Team: the turn to G3 overturns, worse than before.
- No stopped wall reading on G3 trips was visible (the reading on the fly
  never showed on the report as applied or not).

### Third video (2026-10-18, `20261018_203818.mp4`, 60 s, starts near the end of the three-pin)
- G2: stack 2 at 3 s, trip 1 at 11 s, trip 2 (fourth cup, arm up) at 17.5 s:
  the cup came off the top toward the far side.
- On both G2 trips the lift was at the top before the run to G2 started
  (it rose during the drive out of the loader and the 90 degree turn).
- G3: pin 23 s, cup 1 30 s, cup 2 37 s and cup 3 45 s missed, the cup on
  the floor beside G3, the chassis on the goal's edge.
- 52 to 59 s: backed into the loader and stayed about 5 s (the run into the
  loader went to its time limit, likely with the wheels slipping).

### 2026-10-18: the 3 s stops
Most likely the arm: after a score with the arm up, the robot holds short
of the loader until the arm is back at rest. `moveArm` can stop short of
its target; the arm job then waits out its 1.5 s limit, the hold sends it
again (another 1.5 s). Every such hold is now capped at 1 s and the robot
goes on; the report's "capped" count shows how often that happens.

### Second video (2026-10-18, `20261018_182937.mp4`, 82 s)
- Run from about 2 s to parked at about 77 s: about 75 s.
- Three-pin to about 15.5 s. Then the robot stood still 16.0 to 17.6 s
  (stop, turn if twisted, GPS, wall reading), drove to the line-up spot
  (17.7 to 18.7 s), turned and stood again before backing into the loader
  (18.7 to 20.3 s): about 4.8 s from G2 to the loader for about 2 s of
  driving.
- Trip 1 (G2): loader 20 s, G2 23.7 s, back at the line-up 25.3 s, then
  about 1.7 s standing before backing in (waiting for the lift to pass
  -400 on its way down from the top; it comes down slowly against its
  rubber bands), loader at about 28 s.
- Trip 2 took its position from the reading at G2 (34 in from the wall) and
  missed the loader by about 0.5 in (team). The line-up reading (10 in)
  is the accurate one, and the robot arrives there nose south from G2.
- Three-pin stack 2 (about 12.2 s): the pin tipped toward the robot as the
  claw met it and was carried tilted, then sat straight in the claw and
  scored. Not changed (the three-pin's tuned pickup, shared with slot 1).

### Video of a full run (2026-10-18, `20261018_174801.mp4`, 91 s)
Studied frame by frame (ffmpeg contact sheets, 1 to 6 frames a second).
- 0 to 18 s: the three-pin (the robot takes stack 2 at about 13.75 s and
  scores it at about 17.8 s). 18.5 to 20 s: the new reset at G2 (about 1.5
  s). Trip 1 into the loader by 22.5 s, out at 25 s.
- The robot sat against the loader about 2.5 s on most trips, with the
  cup already there (the next one in the person's hand): pushed into the
  loader the wheels crept above 12 rpm, so the run in went to its 2.5 s
  limit. The goals had the same problem until their quick contact check.
- G2 trips 2 and 3 (34 s and 41 s; fourth and fifth cups, arm up): the load
  flew off forward, in the direction the robot was moving, at the height of
  the top of the goal's stack.
- G3: pin (48 s), first cup (56 s), second cup (64.5 s) good. Third cup
  (72.5 s): the load arrived level with the top of the stack, knocked the
  top cone off and dropped in front of the goal. Fourth cup (80.5 s, arm
  up): knocked a cup off beside the stack.
- These were heights, not position: the check before the last 6 in counted
  the lift as up anywhere in the lock's band (50 below the third cup) and
  the arm within 60, so the high loads came in still climbing and clipped
  the stack.
- 83 s: on the way to the toggle the robot ran into G3 and knocked the whole
  stack over (the lane passes G3's base with about 2.4 in to spare, at full
  speed with hand-overs).
- Not seen in this run: a position error after the three-pin (trip 1 went
  in and scored normally).

### 2026-10-18: the 3 s stops at the line-up spot
The wall reading itself is at most 0.2 s (six polls at 10 ms, waiting for
the wheels to stop). The time went to the turns around it:
- A calm turn ends on a power floor of 10. From rest (worse with a raised
  stack) 10 does not turn the robot, so it could stop a few degrees out and
  push until the 1.5 s limit (it only ends early within 3 degrees after
  0.5 s).
- The touch-up after it was a settled turn capped at 30 with no floor: for 2
  degrees off that is about 4 power, which moves nothing, so it waited out
  its 0.6 s.
Now every skills turn also ends once the wheels have stopped for 0.1 s, and
the touch-up (and the goal aim's) is a nudge: floor 20, ends as it crosses
the heading or within 0.5 degrees, at most 0.4 s. The Brain report now shows
the totals for the wall readings, the re-place drive, the line-up turns and
the wait for the lift and arm to come down ("down": the arm returning from a
high score can also hold the robot there, up to two 1.5 s tries).

### 2026-10-18: wall reading at G2 (team's idea)
At G2 the robot is parked against the goal nose south while it lets go, so
the south-wall reading is taken there (ten polls, 0.1 s; about 34 in from the
wall). The robot then drives straight back to the line-up spot on it, and
that trip's line-up stop has no south turn, no reading and no re-place
drive. Farther from the wall the sensor is less exact (VEX: about 5% beyond
200 mm, mostly repeatable from run to run), and the 27 in drive back is not
measured, so this trades a little accuracy for the stop; scatter is allowed
up to 0.6 in. A refused reading falls back to the line-up reset.
`kWallResetAtGoal = false` puts it back. Not for the first trip (the
three-pin ends rolling) or for G3 (approached on the diagonal). On the
report, "wall g +0.4" for a G2 trip means: no line-up reset, +0.4 at G2.


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
- Arm rotation sensor (port 12): does it read? (fourth video; the report's
  last line says "ARM SENSOR port 12: NO READING" if not.) The arm loads
  are skipped until it does.
- `kG3Aim` (2.5 in south, 1 in west of G3's center) is measured off the
  team's drawing; tune it in inches from the next run.
