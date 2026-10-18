# Solo autonomous (`solo_auton`, no slot yet)

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

## Open
- Not run yet. Expected to be well over 15 s at first (rough estimate 22 to
  25 s); the Brain report shows the time at each step.
- The wall pin on this side has never been measured (the four-pin close side
  is the mirror of a measured approach).
- Whether a field stack set on an empty goal should drop like the preload.
