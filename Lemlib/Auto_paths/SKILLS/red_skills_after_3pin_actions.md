# Red skills: the route after the three-pin

**Updated 2026-10-14 to match the code** (`skills_route` in `src/autons.cpp`).
An earlier version of this guide had the three-pin ending at a top goal
("G1", (-24, +48)), east of it and nose east. That was wrong. The move list
built on it has been removed, and the companion files
`red_skills_after_3pin_coordinate_plan.txt` and
`red_skills_after_3pin_pathjerry.txt` still use that wrong start. Use them
only for their field-element coordinates. To see the route as coded, open
`skills_route_visualizer.html`. For where the time goes, open
`skills_time_breakdown.html`.

Nothing in this route has run on the robot yet.

## Coordinates
- Field coordinates, inches from the field's center: +X east, +Y north,
  heading 0 = north, 90 = east.
- The routes themselves use the three-pin's start-relative frame. The start
  is (0, 0) facing the toggle wall on heading 180. In code, `on_field()`
  converts field points to that frame, anchored on G2 = the three-pin's goal
  (20, 15).
- Robot: 16 x 12 in. The claw is at the rear, 4 in wide and centered. A
  cut-out at the claw end lets goals sit deeper, so the effective length into
  a goal is about 12 to 13 in.

## The three-pin (unchanged, runs first)
- **Start:** against the west wall toggle, about (-63, -4), facing west.
- **Scores onto G2 (-48, -24):** its preload pin, then the field stacks at
  (-24, -24) and (-48, -48).
- **Ends:** about 8 in south of G2, nose south, at about (-48, -40), with
  G2 three high (a pin and two cups).

## The route after it, in order
Each goal should end with one pin and five cups. Each wall pin comes after
its goal's match loads, so a failed attempt costs nothing already scored.

1. **Two match loads onto G2**, which takes it to a pin and four cups.
2. **The west wall pin (-72, -24) onto G2**, the fifth cup. The robot backs
   up the west lane, curves onto it, curves out to (-60, -42) and backs into
   G2 from the south-west.
3. **Five match loads onto G3 (-24, -48):** the pin first, then four cups.
4. **The bottom wall pin (-24, -72) onto G3**, the fifth cup.
5. **The bottom toggle**, hit nose first at about (-6, -64) with the lift
   raised.
6. **Park in the center diamond** inside the grey goals. The robot comes in
   from the south-west and stops at about (-11, -8), nose just short of the
   center pin.

Every loader pickup is the same:
1. Come to the line-up point (-48, -59), directly in front of the loader and
   due south of G2.
2. Stop nose south and take position from the GPS.
3. Quarter turn, claw to the loader.
4. Back about 14 in until the robot is against the loader (sent to x = -66;
   it meets it at about -62).
5. Wait 0.5 s, then clamp.

The trips between the loader and the goals:
- **To G2:** 14 in east to the line-up point, then straight north into G2.
  Back out is straight south.
- **To G3:** east along the south lane to (-37, -58.5), turn the claw to G3,
  back in. Back out to the same point.

The lift and arm start rising as soon as the pin is clear of the loader.
From the third item on a goal up, the arm is raised (it adds about one
stage). The release from the arm copies the RIGHT macro: the arm comes down,
the claw opens halfway, then the lift drops. Stack heights are placeholders
in `kLevels`; record the real ones with X in driver control.

## Position resets
- **Before every loader run:** at the line-up point, from the GPS. It is the
  same spot and heading every time, so any GPS error repeats and can be tuned
  out in the loader position.
- **At every goal:** from the goal itself. The claw sits on the goal's center,
  8 in from the robot's center.
- **Against the loader:** the GPS is read but not used, to show in the report
  whether it can see from there.

## Time
The whole list is about 87 s by estimate against 60 s. A clock check skips
any step that would not leave time for the toggle and park, so in practice it
gets about two loads on G2, the wall pin on G2, two loads on G3, the toggle
and the park. `skills_time_breakdown.html` has the breakdown.

## Field targets (from screenshots; check on the field)
| Target | Field (in) | Role |
|---|---:|---|
| G2 | (-48, -24) | three-pin goal, then 2 loads and wall pin 2 |
| G3 | (-24, -48) | 5 loads and wall pin 3 |
| Red loader | wall at (-72, -59) | robot center about (-62, -59) at contact |
| Wall pin 2 | (-72, -24), cups at y = -16 and -32 | |
| Wall pin 3 | (-24, -72), cups at x = -32 and -16 | |
| Bottom toggle | bar centered on the bottom wall | hit at about x = -6 |
| Center park | diamond inside the grey goals at (0, ±24), (±24, 0) | pin at (0, 0) |

## Still to check on the field
- The real stack heights: record them with X and put them in `kLevels`.
- Whether parking needs the robot fully inside the diamond, which would
  mean pushing the center pin.
- The wall-pin approaches: these are first guesses. The two-pin's wall
  stacks needed several sessions.
- The loader's exact position, and whether the GPS reset at the line-up
  point is consistent.
- Whether the goal reference distance (8 in) holds with the cut-out.
- What to cut, since the whole list does not fit in 60 s.
