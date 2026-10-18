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

## Open
- Not run yet. Expected to be well over 15 s at first (rough estimate 22 to
  25 s); the Brain report shows the time at each step.
- The wall pin on this side has never been measured (the four-pin close side
  is the mirror of a measured approach).
- Whether a field stack set on an empty goal should drop like the preload.
