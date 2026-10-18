# VEX V5 robot (PROS 4.2.2 + LemLib 0.5.6): working notes

Read this first. Last updated 2026-10-14.
`journal/` holds the localization notes and one file per autonomous route;
`journal/gps_reset_findings.md` covers the GPS correction code in depth.

## How the user works
- Team robot, V5RC "Override" season. The user tests on the field and reports
  back in short messages, often with Brain-screen photos. Messages are terse
  and typo-heavy; read for intent.
- **Uploads go through the controller** and the user is usually *not*
  plugged in. Check `ls /dev/ttyACM*` first. Never claim something is
  uploaded unless `pros upload` printed "Finished uploading". A background
  "wait for the controller, then upload" job dies when the session ends.
  When the user says not to upload, don't.
- **Results go on the Brain screen**, not the terminal: the user can't stay
  connected. Use `fieldviz::report()` (side panel) or `fieldviz::report_full()`
  (whole screen, ~13 lines of ~50 chars).
- The user gives field corrections in inches ("0.5 in deeper", "x down 1.5")
  and expects them applied exactly, without side effects elsewhere.
- Before coding a route from a guide or sketch, check it against the
  existing code and the user's drawings. A route was once laid out from the
  wrong start because a generated guide was trusted over the three-pin code.
  The HTML visualizers in `Auto_paths/SKILLS/` catch this kind of mistake.
- Nothing since commit `1217d8e` is committed. Commit only when asked.

## Hardware (src/main.cpp, include/subsystems.hpp)
| Device | Port | Notes |
|---|---|---|
| Left drive | -17, -18 | blue (600 rpm) cartridges |
| Right drive | 11, 13 | 2.75 in omnis, 450 rpm drive, 12 in track |
| IMU | 14 | only heading source |
| Distance sensors | 1, 3 | on the front face (nose), side by side; wall resets in skills (`south_wall_reset`) |
| GPS | 10 | camera looks out the robot's **right** side; lens 3.0 in fwd, 3.7 in right (mount never re-surveyed) |
| Lift motors | `slider_left` -19, `slider_right` 2 | green; **negative power = up** |
| Lift rotation sensor | 16 | **up is negative**, 0 at rest, max -1670 |
| Arm ("claw arm") | motor 20, rotation sensor 12 | 0 = rest (boot position), 530 = max (Y macro); positive motor power = down |
| `claw_piston` | ADI B | true = clamp, false = release |
| `clamp_piston` | ADI A | the toggle hitter; extended at the start of autos |
| `side_toggle` | motor 4 | L2 spins it |

- No tracking wheels. Odometry is drive-motor encoders plus the IMU, so wheel
  slip and pivots with a raised load (the robot turns about a point behind
  its center) drift position by inches. Contact references and GPS resets
  exist because of this.
- The claw is at the **rear**: every pickup and every score is driven
  claw-first (`forwards = false`). The claw is 4 in wide, centered (4 in from
  each side). Chassis is 16 x 12 in, but a rounded cut-out at the claw end
  lets goals and stacks sit several inches deeper than a 16 in box suggests.
  The effective length into a goal is about 12 to 13 in.
- Rubber bands pull the lift up, so going down needs more power.

## Build, test, upload
- Hot/cold package build (`USE_PACKAGE:=1`); a single-image build did not
  start on the Brain.
- The default build is the three-pin. Any other routine is chosen at build
  time: `touch src/main.cpp && pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=<fn>`.
  Verify with `strings bin/hot.package.bin | grep -c 'AUTON_RESULT,<fn>'`.
  Always restore the default build afterwards (`touch src/main.cpp && pros make`).
- Upload with `pros upload --slot N --name "..."`, retrying until "Finished
  uploading". The first upload right after another (or of a new cold
  package) often fails.
- Host tests: `python3 -m unittest discover -s tests` (13 tests: map math,
  GPS estimator and corrections, curve tuner model). None exercise routes.
- Slots: 1 "3 pin" (`three_pin_auton`), 2 "4 pin far"
  (`four_pin_far_auton`), 3 "4 pin close" (`four_pin_close_auton`, quadrant 2 = mirror of quadrant 1), 4 "skills" (`skills`, three-pin + first loader entry only until -DSKILLS_FULL_ROUTE). Skills
  no slot yet; ask before overwriting one. The latest two-pin changes (both
  stacks 0.5 in deeper, preload turn/entry time savings, approach logging)
  were **never confirmed uploaded**.

## Controller (driver control)
| Input | Action |
|---|---|
| Left Y / Right X | arcade drive |
| R1 / R2 | lift up / down (takes over from any lift macro) |
| L1 | claw piston toggle |
| L2 | side toggle motor spin |
| A | clamp piston (toggle hitter) |
| B | 150 ms full-power lift burst up |
| Y | arm to max (530) |
| RIGHT | release macro: arm down to 100, claw opens halfway there, 100 ms, lift lowers to rest |
| DOWN | arm down by hand (stops at 0; overrides Y) |
| LEFT | runs the built-in autonomous from driver control (longer than 15 s is fine) |
| UP | curve recorder start/stop (two-pin wall curve; result on the Brain) |
| X | notes lift + arm readings in a list on the Brain (for skills stack heights) |

- Lift lock: at or above 95% of max height the lift is held at max against
  gravity (`holdLiftAtMax`), in driver control and (via `setLiftLock`) in
  autonomous.
- Macros run in tasks with cancel flags (`take_over_arm`,
  `cancel_driver_macros`). `moveArm()` can hang just short of its target, so
  anything that waits on it needs a deadline (`ArmJob` in skills).

## Mechanism helpers (src/autons.cpp, top)
- `moveLift(deg, timeout, cancel)`: P control (kP 0.45) with power floors
  (down 110, up 60), ends within 8 deg. It cannot be sent to the exact top;
  skills uses 97% of max and lets the lock hold it.
- `lowerLiftToRest()`: full power down until stalled (it does not always
  reach 0). Returns the height it ended at.
- `LiftJob`: a lift move in a task alongside driving. **Only one at a time**
  (shared state). Wait for the previous one (`LiftJob{LiftJob::Running{}}.wait()`)
  before starting another. `Settle` = lower to rest; succeeds once the lift is
  back down past -83 (it rarely reaches exactly 0).
- Heights (`lift_position`): matchload -250, carry (preload) -375,
  preload seat -75, stage 1 -660.7, stage 2 -1150, middle goal first level
  -1035, max -1670. Release on a goal after the lift drops a third of a level.

## PID and motion settings
- Lateral 6/0/3, angular 2/0/10. Exit conditions 1 in/100 ms and 3 in/500 ms;
  no slew. Hand-set; an autotuner result (42/0/93) was far too stiff, so
  the PID autotuner was removed (it is in git history before 2026-10-14).
- Last-stretch pickups switch the lateral kP to 3 (`set_lateral_kp`) for the
  final 10 in, with a power floor of 24.
- Curve settings (`include/curve_motion.hpp`, from the curve tuner): lead
  0.45, horizontalDrift 4.
- Chaining: `minSpeed 40, earlyExitRange 2` for legs that hand over;
  turns `minSpeed 20, exit 4°`; turns with a raised stack `minSpeed 10, exit 2°`.

## LemLib 0.5.6 behaviour (checked against its source; relied on)
- Inside 7.5 in of the target, moveToPoint stops steering and moveToPose only
  holds the final heading. A sideways error left over from a curve is
  carried straight into the target. Closely spaced waypoints therefore
  under-turn.
- With `minSpeed` set, a motion ends when the robot crosses the line
  through the target, perpendicular to the approach. Without it, it must
  "settle", which needs **both** the 1 in/100 ms and 3 in/500 ms exits. If
  the robot is blocked more than about 1 in short, it pushes until the
  timeout. That was the "waits 1 s at the goal" bug.
- LemLib has no stall detection. `wait_done_or_stalled()` (async motion,
  wheels under 12 rpm for 150 ms → cancel) is ours; use it for anything
  driven into contact.
- `cancelMotion`, `isInMotion`, `waitUntil(dist)` and `setPose` are the only
  hooks. `waitUntil` measures distance travelled, not distance remaining.
- Lateral output is roughly 6 x distance, so speed falls off over the last
  ~20 in. Power floors (`minSpeed`) are what make arrivals fast.

## Routes (all start-relative: start = (0, 0) heading 180 facing the toggle wall, +Y away from it)
- **three_pin** (`run_three_pin`, slot 1, tuned, fits 15 s): starts against
  the west wall toggle. Toggle hit with the lift 80% up, preload onto G2
  (field (-48,-24); route `kGoal` (20,15)), field stack 1 (route
  (20.1,36.79) = field pin (-24,-24)), field stack 2 (route (42.5,13.25) =
  field pin (-48,-48)), each scored one level higher. After every score it
  takes its position from the goal: center = goal + 8 in along the heading.
  GPS checkpoint after clamping stack 2. The constants are field-tuned; change
  them only as the user asks.
- **four_pin_far / four_pin_close (renamed from two-pin: 2 cups = 4 pins)** (slots 2/3): toggle, preload onto the
  goal, curve claw-first onto the wall stack behind the goal (recorded with
  the UP recorder), score it; then the mirrored wall stack onto the middle
  goal at 90% of stage 2. Close is the far route mirrored in X, which
  assumes field symmetry and is unmeasured. The wall approach needed
  4.65 / 5.0 in extra depth found on the field; the Brain report shows
  per-stack approach time, stall time, offsets and lift current. It was about
  0.75 s over 15 s before the last (unuploaded) time savings.
- **skills** (`skills_route`, untested, no slot): the three-pin, then 2
  match loads → G2, west wall pin → G2, 5 loads → G3 (a pin, then 4 cups),
  bottom wall pin → G3, bottom toggle, park in the center diamond. Each goal
  should end with 1 pin + 5 cups.
  - Uses field coordinates (inches from center, +X east, +Y north) through
    `on_field()`, anchored on G2 ↔ three-pin `kGoal`.
  - Every loader run starts at `kLineUp` (-48,-59): GPS reset nose south,
    quarter turn, back 14 in until contact, 0.5 s wait, clamp.
  - A clock check skips steps that would not leave time for toggle and park.
  - Stack heights are **placeholders** in `kLevels` (lift + arm per level;
    the arm adds about one stage from level 3 up); record real ones with X.
  - Estimated about 87 s for the whole list against 60 s; see
    `Auto_paths/SKILLS/skills_time_breakdown.html`.
- Others: `gps_reset_test_auton`, `gps_mount_survey_auton` (measures the GPS
  mount; `tools/fit_gps_mount.py`), `curve_autotune_auton` (the curve
  tuner behind `curve::tuned`).

## Field facts (from screenshots; verify on the field)
- Red goals (-48,±24), (-24,±48); blue mirrored in X; grey goals at (0,±24),
  (±24,0); field pins on the diagonals (±24,±24), (±48,±48), (0,0).
- Wall pins at (±72,±24) and (±24,±72), each with a cup 8 in either side.
  Toggle bars mid-wall on all four walls. The red loader is on the west
  wall at y ≈ -59, the center park zone is the diamond inside the grey goals.
- Three-pin frame → field: the start is about (-63,-4) facing west;
  field = (start.x + route.y, start.y - route.x); field heading = route + 90.

## Resets, GPS and odometry lessons
- The most trusted reference is contact: after scoring, the claw is on the
  goal center 8 in from the robot center.
- GPS (`include/gps_reset/`): `checkpoint(x, y, radius, wait_ms,
  min_gap_in)` averages raw fixes with the robot parked (about 250 ms) and
  only corrects within 6 in of the expected point and 8 in of odometry.
  `min_gap_in` 0 always takes GPS; >8 only measures. The mount accuracy is
  unproven, so reset at the **same spot and heading** each time, so any GPS
  error repeats and can be tuned out.
- The GPS camera looks right, so a reset is useless when the robot's right
  side faces a wall about 10 in away (e.g. backing into the loader).
- The user trusts GPS resets after clamping. An earlier run was more
  accurate with no resets at all, so measure before adding more.

## Tuning lessons from the field
- Get curves from the robot, not from guesses: the UP recorder (hand-driven
  curve → waypoints, headings, wheel rpm on the Brain) fixed the wall
  approach. A hand-pushed recording disagreed with a controller-driven one by
  7 in; use the controller-driven one.
- Approaches that "stop early" have usually stalled or crossed the target
  line early. Send targets past the contact point and end on stall.
- For time: settled endings are the main waste (~0.5 to 1 s each). Chain
  with floors, end contacts on stall, start lift and arm moves while driving.
  In the skills estimate, driving, backing into contact and turning dominate.
  Mechanisms are under 10%.
- Simulations in a session (kinematic sims, time estimates) are estimates;
  say so. The two-pin gave the only measured calibration (the model was
  about 4% fast).

## Open items
- The two-pin changes still need uploading.
- Skills: needs a slot, real stack heights (X macro), the park rule (does
  the robot have to be fully inside?), wall-pin tuning, and a cut list,
  because it doesn't fit 60 s.
- Whether the goal reference distance (8 in) holds with the cut-out.
- GPS mount survey not re-run.
