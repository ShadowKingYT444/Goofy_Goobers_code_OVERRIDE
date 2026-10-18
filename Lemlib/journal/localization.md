# Localization journal

How the robot works out where it is during autonomous, what we tried, and
what the field showed us. Code references are to `src/autons.cpp` unless
noted.

## The pose and its frames

- The robot tracks a pose: x, y in inches and a heading in degrees,
  clockwise, 0 = +Y.
- Match routes use a **start-relative frame**: the start is (0, 0) at
  heading 180, facing the toggle wall, with +Y pointing away from that wall.
  The field position of the start does not matter, so the same code runs in
  any quadrant.
- Skills uses **field coordinates** (inches from the field's center, +X east,
  +Y north, red side) through `on_field()`. These are anchored so that G2
  (-48, -24) is the three-pin's goal (20, 15). Field to route:
  `route = (G2y + 20 - field.y, field.x - G2x + 15)`, and route heading =
  field heading - 90.
- Field geometry (VEX spec): about 140.5 in between the walls, so each wall
  is 70.25 in from the center. Tiles are 600 mm (23.62 in). Our field-frame
  constants use 24 in tiles. That is fine as long as every reference is taken
  relative to G2, because the offsets come out the same every time.

## Odometry (the base layer)

- No tracking wheels. LemLib integrates the drive-motor encoders (2.75 in
  omnis, 450 rpm, 12 in track), and the heading comes from the IMU on port
  14. Every 10 ms it adds the wheel travel since last time, in the direction
  the IMU reports.
- Smooth and instant, but its errors add up and never go away. On our robot
  the causes are:
  - wheel slip in fast turns and when pushing into goals or the loader;
  - turning with a raised stack, where the robot pivots about a point behind
    its center, which the drive encoders cannot see;
  - contact, where a goal or wall shoves the robot without the wheels turning
    to match.
- The IMU heading is good over one match. Over a minute of skills it drifts
  by a degree or two, which matters on 30 in runs (1 degree is about 0.5 in
  sideways).

## Contact references (the most trusted)

### Goal reference (three-pin, four-pin, skills)
- After a stack is set on a goal, the claw's grip is on the goal's center,
  and the grip is `kReach` = 8 in behind the robot's center. So:
  center = goal + 8 in along the heading. Heading stays with the IMU.
- In the three-pin, this moved odometry by about 8.5 in after score 1
  ("Goal ref moved x+0.0 y+8.5"). That is expected: the score run is sent
  1 in past the goal center, so odometry believes the center reached it.
- **2026-10-17 lesson:** the reference is only true if the claw really is on
  the goal. In skills the robot sometimes missed G2, the reference was
  applied anyway, and the error carried into every later trip. `put_on()` now
  applies it only if the run ended stalled against the goal; otherwise the
  report says "skip".
- **2026-10-17, aligner experiment (reverted):** we sent score runs only to
  the contact point (center 7 in from the goal) instead of 1 in past the
  center. Every score after the first undershot, the goal reference then put
  odometry inches off, and the rest of the route went wrong. The aligner
  cut-out is not deep enough to plan around. Back to -1.

### Loader contact (skills, 2026-10-17)
- Backed against the match loader and stalled, the robot's distance from the
  west wall is physically fixed. That one coordinate (field X) is reset to
  `kLoaderContactFieldX` = -58.6 in; the other coordinate and the heading are
  left alone.
- The -58.6 is where odometry had the robot on the first trip, when the GPS
  agreed to 0.6 in. It is in line with the field drawing (loader face about
  4.3 in off the wall, center 8 in from the claw end: -59.7 to -57.7).
- Why: across 3 trips, odometry X at the loader stop drifted -58.6, -57.7,
  -54.9 while the robot was physically against the same loader. That X
  error becomes the sideways error on the 32 in run north to G2, and the
  robot missed the goal.
- Still needed: confirm -58.6 on the field (if the robot consistently lands
  off-center on G2 sideways, this constant is what to adjust).

## GPS (VEX GPS sensor, port 10)

- The camera looks out of the robot's **right** side. The lens is 3.0 in
  forward and 3.7 in right of center (mount never re-surveyed; see
  `gps_mount_survey_auton` and `tools/fit_gps_mount.py`).
- Pipeline (`include/gps_reset/`): readings rated worse than 3 in are dropped.
  The live estimator mixes GPS with odometry (20% per 100 ms parked, 6%
  moving, 6 in gate, 3 agreeing fixes to lock, 8 agreeing outliers to
  relock). Checkpoints average about 6 raw fixes with the robot parked and
  only correct within 6 in of the expected point and 8 in of odometry.
- What the field showed:
  - Three-pin: resets at every stop made it *less* accurate (back-to-back
    runs). Only the reset after clamping stack 2 is kept.
  - Continuous correction while driving was also worse, so it is off.
  - The camera sees only a small part of the wall code from most spots.
    Close to a wall it reads nothing: backed into the loader, the camera
    faces the south wall from about 12 in, and the report said "no reading".
    At the skills line-up spot (nose south) the camera looks straight at the
    loader, which hides the code behind it.
  - Skills, with a GPS reset applied at the line-up every trip: the loader
    position was different on every trip.
- Conclusion (2026-10-17): GPS is not precise enough for skills. It is only
  read on the four-pin approaches (report only, never applied) and at
  three-pin clamp 2.

## Distance-sensor wall resets (2026-10-17, skills)

### What other teams do (researched 2026-10-17)
- On the VEX Forum, teams with distance sensors use either **wall resets**
  or **Monte Carlo localization** (a particle filter that weighs many
  candidate poses by how well each explains the sensor readings). No common
  library provides MCL.
- The common wall reset: keep the **IMU heading**, turn square to a wall,
  read the distance, and set **only that axis**: wall - (reading + sensor
  offset). Teams put sensors on two sides (for example front and right) to
  get both axes at one stop.
- Published reset code (simple-v5-lib's resetXFromWall / resetYFromWall,
  19109M's resetFromWall) refuses a reset when the robot is moving, is not
  square to the wall, the reading is under 20 mm, low confidence or too far,
  or the change is bigger than a limit.
- V5 distance sensor facts: 20 to 2000 mm; about +/-15 mm under 200 mm and
  +/-5% beyond; about 30 Hz; the field of view is 24 degrees (+/-12) at long
  range (VEX's jpearman), so about 8 in wide at 18 in. The reading is only
  reliable when the sensor faces the wall square.

### What we do now
- Two sensors on the nose (ports 1 and 3), 4.5 in ahead of the center.
- At the line-up spot (-48, -56), nose south, before every run into the
  loader: wait until the wheels are still (at most 0.1 s); refuse if the IMU
  heading is more than 5 degrees off square; take 6 samples 10 ms apart
  (about 2 fresh readings) and use each sensor's median; refuse if a sensor's samples
  scatter more than 0.35 in RMS, if the two sensors disagree by more than
  1.5 in (one is seeing a cup or a robot), or if the result would move
  odometry by more than 4 in. Otherwise set field Y =
  south wall + (sensor offset + mean of the two) x cos(angle off square).
- Expected readings there: the south wall is 46.63 in south of G2
  (70.25 - 23.62), so the center is 14.6 in from it and both sensors should
  read about 10.1 in.
- Heading stays with the IMU.
- Together with the loader contact (field X) every trip starts from two
  references at right angles: the wall sets the sideways place at the
  loader, and the loader sets the sideways place on the run north to G2.
- Check on the robot: the sensors must be high enough that the bottom of the
  cone (12 degrees down) reaches the wall before the floor: at least about
  2.5 in off the floor for a wall 10 in away.

### Things that did not work
- **Heading from the two sensors' difference** (first two versions,
  2026-10-17). Each sensor's own error (about 15 mm) does not cancel in the
  difference, and with a 24 degree cone an angled sensor reports something
  close to the perpendicular distance, so the slope says little about the
  angle. Treating "equal readings" as square could put the heading several
  degrees out (5 degrees is 2.8 in sideways at G2); the robot missed G2. A
  relative version (slope change since the first reset, capped at 3
  degrees) was replaced by plain IMU heading too: the IMU drifts less than
  a degree over the loader loop, so the slope had more to lose than gain.
- **A second reset before the run to G2.** Facing the south wall only gives
  Y, which is along that run (it ends against the goal), so it cost time for
  nothing. Removed.
- **A probe turn to learn the sensor spacing.** No longer needed. Removed.

## Field log

| Date | Run | Result |
|---|---|---|
| 2026-10-17 | Skills, GPS reset at line-up | Robot reached the loader 20 degrees crooked (odometry heading 110, should be 90); line-up reset moved 2.5 in |
| 2026-10-17 | Skills, settled turns | Heading 85; reset moved 3.5 in; still a bit off |
| 2026-10-17 | Skills, GPS measure only, goal reference every trip | Odometry at the loader stop X -58.6, -57.7, -54.9 (drift) and Y -56.4, -56.6, -56.7; the robot physically shifted about 2 in sideways over the 3 trips |
| 2026-10-17 | Skills, loader X contact + goal reference only when stalled | "All broke and kept drifting"; no report photo |
| 2026-10-17 | Skills, distance-sensor wall resets (heading taken from the slope as an absolute) | Wasted time; everything after the first load off, G2 missed |
| 2026-10-17 | Slope used only as a change from the first reset; heading corrections capped at 3 degrees | Replaced before testing, after research |
| 2026-10-17 | IMU heading; sensors for distance only (median, cross-check, gates); one reset per trip; loader contact refused beyond 4 in | Trip 1 wall 8.5/7.8 in (10.1 expected), Y +1.3. Trips 2 and 3 refused "not square". Loader stops x -58.6, -58.3, -55.1 (loader reset 0.0, -0.3, -3.5). Goal reset skipped every trip. G2 missed partly on trips 2 and 3: the line-up was off |
| 2026-10-17 | Calm loop: speed capped at 60 (drive) and 50 (turns), every move and turn settled; the turn to G2 is made before the lift goes up | G2 lined up well; too slow; loader reached about 1 in to the robot's left |
| 2026-10-17 | Speeds back up to 100 with braked stops, wall reset 0.15 s, loader lane 1 in to the robot's right (y -57) | Consistent from trip to trip: the resets work. Still on the loader's edge |
| 2026-10-17 | Loader lane 1.5 in more to the robot's right (y -58.5); 0.1 s wall reset | not yet run |

## Sources
- VEX Forum, "About the sensors used in Autonomous" (wall resets vs MCL): https://www.vexforum.com/t/about-the-sensors-used-in-autonomous/139090
- VEX Forum, "One, or two Odom Pods?" (IMU-square wall resets, one axis per wall): https://www.vexforum.com/t/one-or-two-odom-pods/137811
- VEX Forum, "V5 Distance Sensor Technical Details" (about 30 Hz, 24 degree field of view): https://www.vexforum.com/t/v5-distance-sensor-technical-details/88482
- VEX Forum, "Distance Sensor Accuracy" (inaccurate at an angle): https://www.vexforum.com/t/distance-sensor-accuracy/146714
- VEX API, Distance Sensor (range and accuracy): https://api.vex.com/v5/home/python/Sensing/distance-sensor.html
- simple-v5-lib PR #12, resetXFromWall / resetYFromWall gates: https://github.com/falconyin/simple-v5-lib/pull/12
- 19109M Override code, resetFromWall: https://github.com/AnshuPlayz17/19109m-2026-2027-override
- VEX field specification (about 140.5 in wall to wall): https://www.vexforum.com/uploads/default/original/2X/2/28396bba80b4342650960391ed778b183840abfb.pdf
