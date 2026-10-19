# Solo autonomous (`solo_auton`, slot 5 "solo")

From the three-pin's start: toggle, preload onto G2, the wall pin beside the
toggle onto G2, field pin 2 onto G3 (the other alliance goal), field pin 1
onto the black goal. Laid out 2026-10-18 from the team's sketch and a video of
another team running the same route (they finish in about 16 s, starting
differently). Drawing: `Auto_paths/SOLO/solo_route.html`.

## Coordinates (three-pin start frame: x along the toggle wall, y away)
| Point | Value | Source |
|---|---|---|
| G2 | (20, 15) | three-pin `kGoal` (tuned) |
| Wall pin | (18.7, -8.6), robot heading -28 | four-pin wall stack mirrored across the start (as four-pin close) |
| Wall-pin approach | staging (-8, 15), via (3.4, 11.2) | four-pin, mirrored |
| Back-off after the pin | 14 in straight out, nose first | clears the wall and the cup 8 in beside the pin when turning |
| Along the wall | (34, 3) | between G2 and the wall; G2 is scored from here (as drawn) |
| Field pin 2 | (43.5, 13.25) | three-pin `kStack2` (tuned) |
| G3 | (43.62, 38.62) | G2 + one 23.62 in tile each way |
| Field pin 1 | (20.1, 36.79) | three-pin `kStack1` (tuned) |
| Black goal | (-28, 15) | four-pin's neutral goal on this side |

## How each step moves
- Toggle and preload: the three-pin's code, unchanged.
- Wall pin: the four-pin's approach (mirrored), clamp, lift just off the
  floor (-250), straight back out 14 in.
- Every turn is made with the load low. On each run into a goal the lift
  starts up within 24 in of the goal and the robot holds 6 in short if it is
  not up. Runs go to 1 in past the goal center (the three-pin's depth), end
  on stall, and reset position from the goal.
- Heights: wall pin on G2 stage 1 (one above the preload); pin 2 on the empty
  G3 like the preload (carry -375, set down at -75); pin 1 on the black goal
  at the four-pin's neutral first level (-1035).
- Pickups use the three-pin's `pick_up` (fast approach, gentle last 10 in).

## Chaining (2026-10-18, to save time)
Only the kinds of chaining the three-pin already uses, at full speed:
- Turns toward a goal hand over still turning (floor 20, ends within 4
  degrees) into the run, which steers out the rest. Load low in every turn.
- After each score the robot pulls straight out 8 in without stopping (the
  three-pin's retreat), which also clears the goal before the next turn.
- Pickups are the three-pin's `pick_up` (chained turn, fast approach).
- Two spots still stop fully: beside the wall pin (the turn there clears the
  cup by about an inch) and at the end of the run along the wall (the turn
  toward G2 there clears G2 by about 2.5 in).

## Field fixes (2026-10-18, first run)
- The wall-pin curve did not get all the way onto the pin: its carrot is
  now further back (lead 0.5, was the four-pin's 0.4).
- The back-off from the wall pin handed over at 40 and coasted to about G2's
  y, so the run along the wall hit G2: it is now 11 in and stops crisply.
- Preload: one backward curve from the toggle onto the preload spot instead
  of back out, turn, back in.

## Field fixes (2026-10-18, second run)
- Waiting too long at contacts: solo has its own contact check (watches
  after 0.2 s, ends after 60 ms under 30 rpm; the shared one waits 0.3 s
  then 150 ms under 12 rpm).
- Toggle not always flipped: the hit has a floor of 50 into the wall.
- Preload curve undershot and crept in: speed 110, aims 1 in deeper and
  ends on contact with G2 (power floor) instead of settling.
- Wall pin: target 0.5 in further into the wall (y -9.1).
- After the wall pin everything hands over without stopping (turn, run
  along the wall, turn to G2, run in); the short back-off still stops.
- Field pin 2 moved to (38.9, 17.45): the robot went about 4.6 in too far
  along the wall and 4.2 in too close to it (team's map).

## Tuner (2026-10-18)
The preload curve overshot (ended about 6 in to the side of G2 and 4.6 in
short), and the chained moves were not smooth. Rather than guess,
`motion_tune_auton` (slot 5 "tuner" for now; `src/motion_tune.cpp`) drives
solo's preload curve with 12 settings (lead 0.3/0.45/0.6, speed 80/110,
floor 0/24) and the after-wall-pin chain (turn, 22 in, turn, back 12 in)
with 4 hand-over settings (settled, gentle 10/2 + 25/1, solo's 20/4 + 40/2,
fast 30/6 + 60/3), twice each. The Brain lists the best curves and each
chain setting's mean time, distance off, overshoot and heading error.

### First tuner run (2026-10-18)
| Curve lead/speed/floor | s | off (in) | past (in) | heading off |
|---|---|---|---|---|
| 0.30/80/0 | 2.51 | 2.4 | -1.4 | 1 |
| 0.30/110/0 | 2.51 | 2.6 | -1.8 | 1 |
| 0.30/80/24 | 0.89 | 5.7 | -5.3 | 41 |
| 0.30/110/24 | 0.88 | 5.8 | -5.4 | 42 |
| 0.45/110/24 | 1.17 | 7.0 | -6.7 | 36 |

- With a floor, LemLib's moveToPose ends as soon as the robot is within 7.5
  in and crosses the line through the target: 5.3 in short and 41 degrees
  crooked. That is the field miss (about 6 in to the side, 4.6 in short).
- Without a floor it finishes the turn (1 degree) but stalls about 1.4 in
  short and waits out the 2.5 s limit (the 2.51 s).
- Speed 80 and 110 took the same time on this short curve.
- Applied to solo: lead 0.30 (`kPreloadLead`), speed 80, no floor, target 3
  in past where the claw meets G2 (center 5 in from G2, was 7), so the robot
  still drives when it touches and `wait_contact` ends the move.
- Chain: stopped (B or runaway) after the first setting; "settled" took
  4.86 s, 0.8 in off, but 26 degrees off in heading (odd; to recheck). The
  tuner now runs only the chain part, lists every run on its own line and
  shows why it stopped (`kRunCurves = false`). Solo keeps its chain values.

## Field fixes (2026-10-18, third run)
The tuner's curve settings still missed the preload (ended just north of G2,
the turn unfinished), and the moves after the wall pin had long pauses.
- Preload: back to the three-pin's own moves (back out to the lane, turn,
  back in, settled), which score reliably. The curve missed four runs in a
  row, and the three-pin's notes say a curve and chained moves there were
  each tried twice and failed.
- The pauses were lift waits, not the chaining:
  - each run into a goal first waited for the previous lift move to end,
    then (lift raised only 24 in from the goal) stopped 6 in short to wait
    for it again;
  - each pickup waited for the lowering to end, 0.3 s after the lift
    reaches the bottom.
  Now the lift goes to the scoring height as soon as a load is clamped (for
  G2, right after the wall pin), so it is up when the robot arrives; the
  black goal (long run, higher) raises within 36 in. Pickups start their
  run once the lift is below -100 (`kDownEnoughDeg`); it reaches the bottom
  well before the claw reaches the pin. The three-pin's pickups are
  unchanged.
- Turns into runs hand over at 30 power within 8 degrees (was 20 within 4),
  after LemLib's motion-chaining tutorial: a turn into a move should exit
  early at speed and let the move steer out the rest (the tutorial's
  example uses full power and 20 degrees). Every run after one steers for
  at least 12 in.

## Field fixes (2026-10-18, fourth run): chaining, contact preload, two resets
Feedback: still long pauses; the preload ended about 0.5 in short of G2
and missed; positions inaccurate. Team marked two reset spots.

How LemLib 0.5.6 ends motions (its source, the motion-chaining tutorial,
aadishv.dev/move2point):
- No power floor: a move ends only by settling, within 1 in for 0.1 s or
  within 3 in for 0.5 s (turns 1 / 3 degrees). Close in, power is about 6 x
  the inches left, too little to move the robot an inch out, so it stalls
  just over an inch short and waits out the 0.5 s. That was both the pause
  after each plain move and the preload stopping short (it was sent exactly
  to the contact point and settled before reaching it).
- Floor plus early-exit range: the move ends as it crosses a line that far
  before the target, still moving, and the next starts at once. The next
  motion must be able to take the leftover (a run steers until 7.5 in from
  its target).

Changes:
- Preload: the three-pin's path chained (back out to the lane at floor 30,
  exit 1.5 in; turn at floor 20, exit 3 degrees), and the run into G2 is
  `score_on` like every other score: aimed 1 in past G2's center along the
  line from wherever the robot is, ending on contact, so it cannot stop short
  and stays centered.
- (Same day, after the field run) Chaining the preload's back-out into
  its turn was wrong: it coasted past G2's lane and met G2 at an angle,
  throwing off everything after. Now the back-out stops crisply on the lane
  (floor 20 to the point, brakes; `stop_on`), the turn stops square
  (`square_turn`), and only the run in is changed: straight along the lane,
  ending on contact. The three-pin's notes had warned against chaining here.
- Runs into goals no longer wait for the lift move to end: the lift counts
  as up within 25 degrees (`kLiftUpDeg`); its slow last degrees are cut.
- Goal reference only if the run ended against the goal and it moves
  odometry under 4 in (report: ref / noref / MISS).
- Wall reading (team's mark, by the wall level with G2): the turn out of the
  wall-pin back-off stops square nose south (`square_turn`) and the front
  sensors read the south wall, about 45 in away (skills' reset, far mode,
  up to 6 in). The one full stop in the run.
- GPS at G3 (team's mark): while the pin is lowered onto G3 (free time),
  `gpsreset::checkpoint` with the goal-reference point as the expected spot
  (within 6 in, at most 8 in from odometry; under 1 in the goal reference
  stands). Solo now anchors the GPS at the start (`capture_start_as(0, 0,
  180)`, as the three-pin does). The camera sees the north wall from almost
  the same heading at the start and at G3, so a mount error mostly cancels.
- Every load goes straight to its scoring height at the clamp (black goal
  too, as the four-pin carries its stack to the middle goal).
- Report: wall reading (or why refused, with both sensor readings), GPS
  outcome and gap, and time spent in each kind of wait (lift off the floor,
  seating, dropping, holds short of a goal, the wall stop) and in each
  pickup's slow last stretch.

## Field fixes (2026-10-18, fifth run): hand-overs removed
Field: the run to the staging point ended too far, so the claw took the cup
beside the wall pin instead of the pin; after scoring that cup on G2 the
robot ran straight into pin 2, turned (claw to G3, so nose to the wall, as
planned) and the run stopped there. Team: keep the preload exactly as the
three-pin's; the hand-overs overshoot.
- Preload: exactly the three-pin's again (settled back out, turn, run to the
  spot; no goal reference).
- No hand-overs while moving between legs and turns: the staging leg, the
  run along the wall and the pull-outs end crisply (`stop_on`: floor 20 to
  the point, then brakes), and turns into runs end within 2 degrees at a
  floor of 10 (the three-pin's turn into a goal with a raised stack). The
  settle wait is still avoided; only the hand-overs at speed are gone.
- Resets may now correct up to 8 in (wall reading and goal reference):
  taking the cup instead of the pin leaves odometry about 8 in out along
  the wall, and the earlier 4 to 6 in limits would have refused exactly
  that correction.
- Unknown: why the run stopped after the turn toward G3. The report's top
  line shows "done" or the step that failed, and the time (if it was run
  under a 15 s match timer, the clock ends it).

## 2026-10-18, sixth run
"Good right now", but too slow, and backs up too much after the preload.
The drive out to the staging point is 3 in shorter (x -5, was -8:
`kStagingShorterIn`): it drove 20 in out and backed 11.4 in to the via point
before the curve onto the wall pin. The via point and curve are unchanged;
the via must stay well over 7.5 in from the staging point (LemLib stops
steering inside that) and the path must stay clear of G2, so more than a few
inches needs the approach re-fitted.

## Open
- Not run yet. Expected to be well over 15 s at first (rough estimate 22 to
  25 s); the Brain report shows the time at each step.
- The wall pin on this side has never been measured (the four-pin close side
  is the mirror of a measured approach).
- Whether a field stack set on an empty goal should drop like the preload.
