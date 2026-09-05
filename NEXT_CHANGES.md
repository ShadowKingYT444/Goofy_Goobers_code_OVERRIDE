# Master Fusion System: Next Implementation

## 2026-09-04 replacement-robot port correction

- Production drive is P17/P18 left, P11/P13 right, with IMU P12.
- P1 is rear/negative-axis Distance, P6 AI Vision, and P7 GPS.
- P5 is claw-arm Rotation feedback; P15 is the working, center-mounted
  longitudinal/forward odometry Rotation sensor. The legacy lateral fusion
  path is disabled pending a dedicated P15 longitudinal integration.
- Toggle is the active-low ADI-D piston. Autonomous and driver control now
  extend it on entry and never retract it during the game.
- P14 is empty because the upper intake has been removed; no P14 motor object
  or controller command remains in production.
- Forward route collision logic no longer consumes rear-facing P1 as though it
  observed the space ahead. A dedicated rear/reverse safety policy is still
  needed.
- D4 telemetry now identifies the odometer as `h15`; host parsers retain
  compatibility with saved `h5` logs.

## 2026-08-25 PID-tuning update

- Live paired +/-45-degree tuning selected fused turn gains `kP=1.8`,
  `kD=0.058514`, minimum power 14, and slew 800 power/s.
- The production four-leg `+45/0/-45/0` acceptance passed with 1.464-degree
  final heading residual and 0.022-in fused position loop closure.
- The first continuous relative `(12,12)` curved route completed in 1.20 s at
  `(12.730,10.488)` but missed final heading by 15.008 degrees. After tightening
  steering and requiring final-heading settle, the second run passed in 1.30 s
  at `(12.320,11.688)`, 0.447-in radial error and 0.679-degree heading error.
- Slot 4 is back on the no-startup-motion `SafeTuned` image. See
  `reports/pid_tuning_2026-08-25/report.md`.

## 2026-08-25 live-resume update (supersedes the runtime blocker below)

- A real Brain power cycle restored CPU1 and D4/FUSE/VISION telemetry, proving
  the old user-runtime blocker resolved. After 19.28 min of a later stationary
  soak, the complete VEX USB device disappeared. Reconnect it and aim the Brio
  at the robot before any physical motion.
- A ready first P6 calibration later drifted at 1.074 degrees/min with all four
  drive encoders fixed and P7 heading nearly fixed. Warm stationary
  recalibration held 0.02 degrees peak-to-peak for 11.89 min. Immediate work
  now starts with a session-scoped stopped P6/P7/encoder calibration-quality
  monitor, followed by live turn validation once the robot is visible.
- D4 telemetry now includes P6 raw gyro X/Y/Z and acceleration X/Y/Z. Host
  capture and dashboard parsers preserve those fields.
- P7 raw Z gyro is diagnostic-only; stopped medians were approximately -0.61
  to -0.40. Do not use it as an alternate heading integrator without bias and
  motion validation.
- P8 returned corners use the 0.625-in inner detected square, not the 0.875-in
  printed outer black square. Field correction remains disabled.

Authoritative current evidence is in
`reports/sensor_campaign_2026-08-25/live_resume_addendum.md` and
`reports/sensor_campaign_2026-08-25/goal_completion_audit.md`.

## 2026-08-23 historical replacement-state note (superseded above)

The historical design text below predates the replacement robot and is not a
hardware/deployment record. Current authoritative state:

- drive: P17/P18 left and P11/P13 right, differential 2.75-in omnis, provisional
  P7-referenced encoder scale `0.8847477281` (2.433-in effective rolling
  diameter), still awaiting independent tape/laser truth;
- P6 IMU is the primary heading source;
- P7 GPS lens points robot-right and is mounted 6 in right / 6 in behind;
- P1 is the only forward Distance sensor and provides an 8-in obstacle stop;
- P8 AI Vision uses Circle21h7 and reports tag-relative PnP diagnostics;
- P5 lateral tracking enumerates but moved at most 0.002 in during live tests,
  so `kSideOdomEnabled=false`;
- no four-sensor LiDAR array exists on this robot; P9 is a slider motor.

Production is deterministic bounded fusion, not an EKF. The explicit entered
start pose is the absolute anchor. GPS must pass 0.75-in RMS, spin, fixed
12-sample temporal cluster, 3-in ordinary / 6-in 30-sample reacquisition, and
5-degree heading-innovation gates; correction steps are bounded. Automatic
GPS-first anchoring is deleted. AI field correction remains disabled until the
P8 lens extrinsics and official Goal-face transforms are measured.

Immediate work, in order:

1. Add and validate the session-scoped stopped P6 calibration-quality monitor;
   do not grant navigation motion from ready status alone.
2. Live-qualify the revised `navigation::go_straight_to()` on a short out/return
   path, including the new finish-window brake, P7 dropout/reappearance, and P1
   obstacle abort. The offline 90,000-trial shadow sweep passed normal logic but
   showed that unobserved slip/push can exceed 6 in while odometry looks good.
3. Remount P7 rear-facing with a clear view of perimeter Field Code, per current
   VEX best practices, then repeat the orientation sweep.
4. Measure P8 forward/right/up/yaw/pitch, ground-truth 0.625-in detected-square PnP against
   a tape, extract/validate tag-plane normal so the physical Goal face is
   observed rather than inferred from a drifting prior, and enter official
   Goal-face transforms before enabling AI position correction.
   Existing Tag 4 PnP gives a plausible outward plane-normal mean but 21.2
   degrees circular spread, so collect multiple known faces/ranges before
   choosing its gate.
5. Repair P5 mechanically if lateral slip measurement is desired.

See `NAVIGATION.md` and
`reports/sensor_campaign_2026-08-23/qualification_report.md` for the current API,
measured errors, plots, and acceptance checklist.

Status: future covariance-estimator plan. The active firmware now implements the
bounded deterministic subset described below (prediction, health gates, four-wall
LiDAR heading and perpendicular-axis correction), but not the proposed covariance,
NIS, recovery, or AI-Vision portions.

## Executive Verdict

The current robot has useful deterministic fusion, not the final uncertainty-weighted system.

- Pose prediction runs from drivetrain encoders, the horizontal tracking wheel,
  and the IMU.
- The four distance sensors can correct the software IMU bias and the coordinate
  perpendicular to a selected wall when an observation passes the current gates.
- The pose loop runs every 20 ms, but accepted LiDAR bias corrections are
  evaluated at most every 100 ms. Three consistent fits are required; coarse
  corrections are capped at 3 degrees and fine corrections at 0.75 degrees.
- The fixed PC camera is debug-only and never changes operational X, Y, or
  heading. Production AI Vision runs on verified Smart Port 1 in shadow mode;
  tag geometry, freshness, and duplicate-ID ambiguity are reported, but pose
  correction remains disabled until camera extrinsics are calibrated.
- The estimator now tracks short LiDAR temporal consistency and explicit
  drivetrain/side-odometer health, but it still does not maintain covariance,
  full sensor age, or statistically normalized disagreement.

The proposed final behavior is:

1. Predict the pose continuously from the IMU, drivetrain encoders, and
   horizontal tracking wheel.
2. Compare every fresh LiDAR and AI Vision observation with the predicted pose.
3. Correct only when the observation is geometrically valid, fresh,
   unambiguous, temporally consistent, and plausible relative to uncertainty.
4. Change trust using estimator uncertainty and measurement uncertainty, not
   only the raw size of the difference.
5. Enter a recovery mode for large conflicts instead of immediately replacing
   the pose or heading.

## Current Geometry Calibration State

- The drivetrain has eight 2.75-inch (220 mm travel) omni wheels, four per
  side, with two gear-coupled motors per side. It is differential/tank drive:
  the rollers permit passive lateral sliding, but there is no commanded strafe.
  Drive configuration is 450 RPM with external ratio 1.0.
- The effective drive track width is still the provisional 10.0-inch value.
  The bounded four-segment turn calibration now logs all four motor encoders,
  field-frame IMU heading, LiDAR wall theta, and rear tracking-wheel rotation,
  then fits effective track width from measured motion.
- The 2-inch center-mounted tracking wheel on P15 measures longitudinal travel.
  One run resembled an exact 5:7 transfer, but the immediate repeat over-read
  8.114 in against 5.823 in of paired-encoder travel and kept spinning during
  braking. No fixed correction is valid. Repair/preload the coupled wheels and
  repeat against tape before integrating P15 into localization.
- LiDAR/IMU calibration compares angle changes at each stationary endpoint and
  reports scale plus RMS residual. Raw LiDAR absolute theta is not compared
  across different wall identities.

## Critical Correction: External Sensors Are Not Always Right

The rule must not be:

```text
if external_difference_is_large:
    trust_lidar_or_camera_more
```

A large difference can mean either:

- the encoder/IMU prediction drifted;
- the LiDAR array saw another robot or a flat field element;
- the wrong field wall was selected;
- the camera frame was stale, blurred, clipped, or partially hidden;
- a repeated AprilTag ID was matched to the wrong Goal;
- the camera or LiDAR mounting calibration is wrong;
- the measurement and current pose refer to different times.

The magnitude alone cannot identify which source is wrong. In fact, a very
large unexplained residual is often the strongest reason to reject a single
measurement.

Every fresh observation should be evaluated immediately. It should receive
high trust only when all of the following are true:

- estimator uncertainty is large enough to make the correction plausible;
- that observation's calculated measurement uncertainty is small;
- geometry, freshness, range, motion, and ambiguity gates pass;
- the residual passes a normalized innovation test;
- a large correction is repeated or independently corroborated.

## Existing Coordinate Contract

Keep `include/localization_config.hpp` as the authoritative field and starting
pose contract.

```text
origin: center Goal
+X: toward the configured 0-deg wall
-X: toward Audience View
+Y: toward the red side
-Y: toward the blue side
scale: 1 coordinate unit = 1 inch
```

The user enters the robot rotation-center start `x`, `y`, and field heading.
During initialization, the physical IMU is calibrated and its raw rotation is
zeroed. The estimator maps raw IMU zero to the entered field heading. The robot
is not permanently anchored to any particular starting tile.

The LiDAR array is mounted on the left side of the chassis. AI Vision will be
mounted on the opposite, right side. Both sensor-to-robot transforms must be
represented explicitly rather than hidden in controller code.

## Current Sensor Roles

| Source | Current role | Fundamental limitation |
| --- | --- | --- |
| IMU | Fast heading plus software bias | Short-term noise and long-term drift |
| Drive encoders | Robot-forward displacement | Driven wheels report rotation during slip |
| Horizontal wheel | Robot-sideways displacement | Offset and wheel scale require calibration |
| Four-sensor LiDAR wall | Conditional heading-bias correction | A clean line does not prove field-wall identity |
| AI Vision | Smoke tests only | Not connected to active localization |

There is no independent forward tracking wheel and no room to add one.
Therefore forward displacement cannot be continuously observed during periods
with no useful wall, tag, contact, or other field reference. The filter can
represent this honestly by growing forward-position uncertainty, but software
cannot manufacture missing physical information.

## Selected Estimator

Implement a small uncertainty-weighted error-state filter instead of a large,
opaque localization framework.

The first practical state is:

```text
state = [x_in, y_in, imu_bias_deg]
heading = wrap(start_heading + raw_imu_rotation + imu_bias_deg)
P = 3x3 state covariance
```

This keeps the IMU as the fast short-term heading source while letting absolute
wall observations estimate its slowly changing bias. The covariance matrix
`P` records how uncertain the robot is and preserves correlations between
heading error and field-position error.

Do not repeatedly call `imu.set_rotation()` during movement. That would create
discontinuous readings for controllers. Apply correction to the estimator's
software bias and publish one continuous fused heading.

An encoder-scale state can be considered later, after recorded data proves it
is observable. It should not be added initially because collision slip is
transient and should not be learned as a permanent scale factor.

## Prediction Update

Run one localization task every 10-20 ms. It should be the only task that reads
localization sensors and mutates estimator state.

For each cycle:

1. Read raw IMU rotation.
2. Read the left and right drivetrain encoder groups.
3. Read the horizontal tracking wheel.
4. Calculate incremental robot-forward and robot-sideways displacement.
5. Compensate the horizontal wheel for its offset during rotation.
6. Rotate the robot-frame displacement into the field frame using the midpoint
   fused heading.
7. Update `x`, `y`, and `P`.
8. Publish an immutable timestamped snapshot for controllers and telemetry.

Process noise `Q` must grow dynamically. Increase it when:

- commanded acceleration is high;
- left and right encoder deltas disagree unexpectedly;
- the robot turns quickly;
- drivetrain current is high but measured displacement is low;
- wheels reverse rapidly;
- the robot collides, is pushed, or is in an anti-stall state;
- time since the last absolute observation grows.

Forward process noise should be larger than horizontal-wheel process noise
because forward motion currently comes from driven wheels. The estimator must
become less certain during an external-sensor outage instead of continuing to
report a precise-looking pose.

## Measurement Update Mathematics

Every accepted external observation uses the same update contract:

```text
innovation = wrap_or_subtract(measured - predicted_measurement)
S = H * P * transpose(H) + R
NIS = transpose(innovation) * inverse(S) * innovation

if NIS passes the gate:
    K = P * transpose(H) * inverse(S)
    state = state + K * innovation
    P = joseph_covariance_update(P, K, H, R)
else:
    reject or enter recovery evidence collection
```

Where:

- `H` describes how the observation changes with `x`, `y`, and IMU bias;
- `R` is calculated measurement uncertainty for this particular observation;
- `S` is expected innovation uncertainty;
- `NIS` is normalized innovation squared;
- `K` is the resulting gain.

Use the Joseph covariance form to keep `P` symmetric and nonnegative under
floating-point error.

The important behavior is:

- Large `P` and small `R` permit a stronger correction.
- Small `P` and large `R` produce a weak correction.
- A large raw difference can still pass if the estimator was already very
  uncertain.
- A large difference is rejected when it is inconsistent with both sources'
  uncertainty.
- Gain is not increased merely because the residual is large.

Use a soft robust weight near the NIS limit and a hard reject beyond it. Exact
thresholds must come from logged distributions, not guesses disguised as final
constants.

## LiDAR Wall Observer

### Measurement

Keep the four-sensor line fit. Use the measured physical sensor positions, not
assumed indices, to fit:

```text
distance_along_beam = intercept + slope * sensor_baseline_position
theta_relative = atan(slope)
```

All four points contribute to the least-squares slope. The endpoint formula is
useful as a cross-check, but it must not replace the four-point fit.

For each valid field-wall hypothesis, combine the known wall orientation with
`theta_relative` to create an absolute heading observation. That observation
primarily measures `imu_bias_deg`.

Wall distance is retained for:

- rejecting readings beyond reliable range;
- testing whether a candidate wall is plausible from the predicted pose;
- choosing among wall hypotheses;
- detecting flat obstacles inconsistent with field geometry;
- future optional axis correction after physical tests.

In the first master-fusion release, LiDAR wall distance must not update `x` or
`y`. Its accepted correction is heading-only, as requested.

### Per-Observation Uncertainty

Calculate `R_lidar_theta` from:

- regression RMSE;
- maximum point residual;
- sensor confidence values;
- average and maximum range;
- baseline length;
- incidence angle;
- robot angular velocity;
- wall-hypothesis winner margin;
- expected-versus-measured wall distance;
- short temporal consistency across recent fresh scans.

Longer ranges should create larger angle uncertainty. VEX documents the V5
Distance Sensor for 20-2000 mm, with approximately +/-15 mm accuracy below
200 mm and approximately 5% accuracy above 200 mm. That error matters when the
sensor baseline is only a few inches.

### Acceptance Gates

Reject a LiDAR update when any of these is true:

- one or more sensors are invalid or outside configured range;
- fit RMSE or maximum residual is too high;
- confidence is too low;
- angular velocity is above the correction limit;
- no field wall is geometrically plausible;
- the best and second-best wall scores are too close;
- measured wall distance conflicts strongly with predicted field geometry;
- recent scans do not evolve consistently with robot motion;
- the frame is not fresh;
- NIS exceeds the hard gate.

The existing 80 deg/s angular-rate gate is too permissive for a precise
correction starting point. Begin physical tuning around 20-30 deg/s while
still allowing prediction during faster motion.

### Update Rate

Remove the fixed 1000 ms accepted-correction dead time in the final estimator.
Evaluate every fresh distance-array sample at the actual sensor cadence.

This does not mean applying a large correction every 20 ms. Correlated samples
must use realistic `R`, bounded per-cycle influence, and a temporal consistency
window. A stable clean wall can then converge quickly through several small
updates without jerking the heading controller.

## AI Vision AprilTag Observer

### What It Measures

The AI Vision Sensor can provide a detected tag ID and image corner positions.
From calibrated intrinsics and the corner centroid, calculate a camera-relative
bearing:

```text
bearing = atan((pixel_x - principal_x) / focal_length_x)
```

With a known camera-to-robot transform and a candidate tag position, predicted
bearing is a function of `x`, `y`, and heading. A bearing update constrains the
robot to a line from the landmark; it is not a complete 2D pose by itself.

Because the camera faces from the right side of the chassis, its bearing to a
known Goal should often constrain the robot-forward error strongly. This is the
best available way to periodically bound forward encoder drift without adding
a forward odometry wheel. It remains intermittent: no visible, trusted tag
means no new absolute forward information.

### Landmark Geometry

The existing field map correctly stores Goal centers and repeated IDs. IDs 1-4
each refer to two different Goals, and each Goal has four tags with the same ID.

The camera observer must extend that map with:

- the four physical tag-face positions around each Goal;
- each tag face normal;
- tag height and physical side length;
- visibility rules for front-facing versus back-facing tags.

Using only Goal-center coordinates introduces a systematic position error of
roughly the Goal radius and can select an impossible face.

### Duplicate-ID Association

For every detection, score all map candidates with that ID using:

- predicted bearing residual;
- candidate face visibility;
- current pose covariance;
- optional apparent-size range consistency;
- image location and clipping;
- field-of-view limits;
- recent association history.

Require both an absolute score threshold and a winner margin over the
second-best candidate. If two candidates remain plausible, reject the update.
Do not average them and do not pick the nearest Goal unconditionally.

### Camera Quality and Uncertainty

Calculate `R_camera_bearing` from:

- tag pixel area;
- corner geometry and skew;
- distance from the image edge;
- motion blur proxy or rapid robot angular velocity;
- calibrated camera residuals at known poses;
- candidate winner margin;
- frame age;
- temporal consistency.

Apparent tag size can supply a rough range gate, but it should not be the
primary position measurement until testing proves its calibration and error
distribution. Bearing is the safer first measurement.

Two independently identified tags in one frame or a short consistent window
can triangulate a much stronger `x,y` correction. That update should still pass
the same innovation gate.

### Camera Rollout

Run camera fusion in shadow mode first. For every detection, log:

- selected and alternate landmark candidates;
- predicted and measured bearing;
- innovation, `R`, `S`, NIS, and proposed gain;
- proposed `x`, `y`, and heading change;
- accept/reject decision and reason.

Shadow mode must not alter the live pose. Enable bounded corrections only after
recorded logs and measured checkpoints show that accepted updates improve error
and adversarial cases are rejected.

## Freshness, Correlation, and Latency

Polling an API every 10 ms does not prove that the hardware produced a new
measurement every 10 ms. Never apply the same cached frame repeatedly.

Each observer should produce:

```text
capture_time
receive_time
sequence_or_frame_token
measurement
quality metrics
```

Use a hardware sequence/timestamp if exposed. Otherwise create a conservative
frame token from the returned object set and enforce the measured hardware
frame cadence. Identical readings may still be fresh, so the fallback must use
both content and time rather than content alone.

Keep a short pose-history ring buffer. If sensor latency is meaningful, compare
the measurement to the predicted pose at `capture_time`, apply the correction
there, and replay stored motion increments to the present. At minimum, reject
observations older than a strict age limit and log their age.

Repeated observations from one wall or one tag are correlated. Do not let them
collapse covariance toward zero. Inflate `R`, limit information per time
window, or model the correlation conservatively.

## Estimator Modes

Use an explicit state machine:

| Mode | Meaning | Allowed behavior |
| --- | --- | --- |
| `INIT` | IMU calibration and entered start pose | No movement; establish raw zero and covariance |
| `TRACKING` | Prediction and normal gated updates | Publish fused pose for controllers |
| `PREDICT_ONLY` | No valid external observation | Continue odometry; grow uncertainty |
| `DEGRADED` | Pose uncertainty exceeds configured limits | Continue cautiously; mark telemetry clearly |
| `RECOVERY` | Large consistent conflict or startup pose loss | Collect corroborating evidence; no teleport |

Entering `RECOVERY` does not mean trusting the newest sensor. A large correction
requires one of:

- two geometrically independent tags;
- a clean wall heading plus a consistent known tag;
- repeated consistent observations while nearly stationary;
- an explicit known physical checkpoint/contact action.

When uncertainty exceeds a hard safety boundary, autonomous field-coordinate
movement should fail closed or switch to a local fallback instead of pretending
the pose is exact.

## Update Scheduling

Recommended initial schedule:

| Operation | Rate |
| --- | --- |
| IMU/encoder/horizontal-wheel prediction | Every 10-20 ms |
| LiDAR observation construction | Every fresh array sample |
| LiDAR correction decision | Every fresh valid observation |
| AI Vision observation construction | Every fresh camera frame |
| Camera correction decision | Every fresh valid observation |
| Dashboard telemetry | 10-20 Hz |
| Persistent debug log | 20-50 Hz plus every accept/reject event |

The word `decision` is important. Every fresh frame is compared, but many
frames will intentionally produce no correction.

## Trust Policy Summary

| Source | High trust when | Low or zero trust when |
| --- | --- | --- |
| IMU | Short time horizon, calibrated, normal motion | Bias grows, shock, calibration fault |
| Drive encoders | Smooth traction, encoder agreement | Collision, skid, push, rapid reversal |
| Horizontal wheel | Calibrated and in contact | Lift, obstruction, offset error |
| LiDAR theta | Clean nearby wall, unique hypothesis, low turn rate | Obstacle, far range, ambiguity, stale scan |
| Camera bearing | Clear tag, unique candidate, calibrated geometry | Duplicate ambiguity, blur, clipping, stale frame |
| Contact checkpoint | Deliberate repeatable alignment | Uncontrolled collision or uncertain contact |

No sensor owns the pose unconditionally.

## Software Structure

Do not add the master filter as more local state inside `autons.cpp`. Create a
single localization subsystem.

Proposed files:

```text
include/localization.hpp
src/localization.cpp
include/localization_config.hpp       existing authoritative configuration
src/autons.cpp                        consumes estimator snapshots only
tools/lidar_bar_server.py             displays estimator diagnostics
```

Core types:

```cpp
struct PoseEstimate {
  double x_in;
  double y_in;
  double heading_deg;
  double sigma_x_in;
  double sigma_y_in;
  double sigma_heading_deg;
  uint32_t timestamp_ms;
  EstimatorMode mode;
};

struct WallObservation;
struct TagObservation;
struct MeasurementDecision;

class PoseEstimator {
 public:
  void reset(const localization_config::StartPose& start);
  void predict(const MotionIncrement& motion);
  MeasurementDecision update_wall(const WallObservation& observation);
  MeasurementDecision update_tag(const TagObservation& observation);
  PoseEstimate snapshot() const;
};
```

One PROS task owns sensor reads and estimator mutation. Controllers read a
snapshot through a mutex, seqlock, or short critical section. They must not race
with observer updates or cache mutable references.

Existing fused drive and turn controllers should initially continue consuming
the same `x`, `y`, and fused heading outputs. Estimator migration should not
also retune movement control. Separate pose correctness from controller tuning.

## Telemetry Required for Tuning

Extend the current `D4` telemetry contract or introduce a versioned successor
containing:

- fused `x`, `y`, and heading;
- raw IMU heading and estimated IMU bias;
- `sigma_x`, `sigma_y`, and `sigma_heading`;
- estimator mode;
- age of the latest LiDAR and camera observations;
- measurement innovation, NIS, gain, and calculated `R`;
- selected wall and runner-up score;
- selected tag landmark/face and runner-up score;
- accept/reject reason;
- stale-frame and rejected-observation counters;
- encoder disagreement and current process-noise multiplier.

The web UI should display accepted corrections differently from rejected
observations and prediction-only motion. A plausible-looking line is not enough
to evaluate fusion; uncertainty and decision reasons must be visible.

## Calibration Before Enabling Corrections

Measure and store:

- drivetrain wheel travel per encoder degree;
- left/right forward scale mismatch;
- horizontal tracking-wheel diameter and rotation offset;
- all four LiDAR sensor baseline positions and beam yaw offsets;
- LiDAR array transform from robot rotation center;
- camera `x`, `y`, yaw, and height from robot rotation center;
- camera focal length and principal point from physical calibration;
- tag size, Goal radius, tag-face position, height, and face normals.

Collect stationary and known-pose datasets to estimate noise:

1. LiDAR at several wall ranges and angles.
2. LiDAR facing robots and non-wall flat objects.
3. Camera at several Goal ranges, bearings, speeds, and lighting conditions.
4. Encoder prediction during smooth travel, hard acceleration, skid, push, and
   collision.

Use these distributions to select `Q`, `R`, NIS limits, age limits, and winner
margins. Keep raw logs so changes can be replayed without repeatedly using the
physical robot.

## Implementation Phases

### Phase 1: Central Estimator and Shadow Telemetry

- Move pose ownership out of autonomous-local controller state.
- Preserve current numerical prediction and current LiDAR behavior.
- Add timestamps, covariance, modes, and decision telemetry.
- Do not change movement behavior yet.

### Phase 2: High-Rate Adaptive LiDAR Bias Fusion

- Replace the fixed 1000 ms gate with every-fresh-observation evaluation.
- Add dynamic `R_lidar_theta`, NIS gating, temporal consistency, and stale-data
  protection.
- Keep updates bias-only and bounded.
- Replay logs, then physically test stationary, straight, turning, obstruction,
  and long-duration cases.

### Phase 3: Camera Calibration and Shadow Association

- Add the camera transform and per-face Goal tag geometry.
- Poll AprilTags from the localization task.
- Score duplicate-ID candidates and log proposed bearing updates.
- Leave all camera corrections disabled.

### Phase 4: Bounded Camera Bearing Fusion

- Enable only high-confidence, low-NIS, unique-candidate bearing updates.
- Start with a low information cap per second.
- Verify that forward drift decreases without lateral jumps.

### Phase 5: Multi-Observation Recovery

- Add two-tag triangulation.
- Add wall-plus-tag corroboration.
- Add deliberate contact checkpoints where useful in autonomous routines.
- Enable recovery only after adversarial log replay passes.

### Phase 6: Controller Integration and Long Tests

- Feed only timestamped estimator snapshots to coordinate movement.
- Use uncertainty limits to decide whether global movement is allowed.
- Run repeatable 90-second routes and scoring-macro tests with ground truth.

## Required Failure Tests

The implementation is not accepted until it catches all of these:

- another robot forms a clean four-point LiDAR line;
- a field element looks planar but is not the predicted wall;
- duplicate AprilTag candidates have nearly equal scores;
- a clipped or motion-blurred tag creates a bad centroid;
- one cached camera frame is returned repeatedly;
- LiDAR and camera disagree with each other;
- a large residual arrives while the filter claims low uncertainty;
- a correction arrives during a fast oscillating turn;
- the camera mounting yaw is deliberately offset in configuration;
- no external observation exists for an extended interval;
- a delayed frame is compared against the wrong point in pose history;
- estimator and controller tasks access pose concurrently;
- a Goal center is confused with the visible tag face;
- the robot is pushed forward while drive encoders do not represent true travel.

## Initial Acceptance Criteria

These are starting targets and must be revised from measured repeatability:

- injected 5 deg IMU bias converges below 1 deg with a clean nearby wall and no
  visible oscillation;
- no normal accepted update changes heading by more than 0.5 deg or position by
  more than 1 inch in one cycle;
- zero accepted corrections in the named flat-obstacle, ambiguity, stale-frame,
  and fast-turn adversarial tests;
- forward pose error remains below 3 inches when a valid known tag is observed
  at least every 5 seconds during the test route;
- uncertainty grows during sensor outages and decreases only after accepted
  independent information;
- a 90-second measured route reports checkpoint errors, maximum error, RMS
  error, accepted/rejected counts, and outage durations;
- fused movement regression tests retain current command direction, distance,
  turn sign, timeout, and chaining behavior.

## What Not To Implement

- Do not trust LiDAR or AI Vision automatically because the difference is large.
- Do not overwrite heading or position directly from one frame.
- Do not reset the physical IMU every refresh.
- Do not count repeated cached observations as new information.
- Do not collapse uncertainty from many correlated readings of one wall.
- Do not select a duplicate tag ID without a winner margin.
- Do not treat tag-size range as precise ground truth.
- Do not hide invalid localization behind a smooth dashboard path.
- Do not claim continuous forward localization when no external reference is
  visible.

## Exact Next Coding Step

Implement Phase 1 only:

1. Add `PoseEstimator` and immutable `PoseEstimate` snapshot types.
2. Move current prediction and LiDAR-bias logic into the localization task
   without changing constants or movement behavior.
3. Add covariance growth, timestamps, estimator modes, and detailed decision
   telemetry in shadow form.
4. Replay existing fused-auton behavior and verify direction, distance, turn,
   timeout, and chaining regressions.

Only after that baseline is stable should the 1000 ms LiDAR throttle be removed
and adaptive high-rate corrections enabled.

## Reference Documentation

- PROS AI Vision tutorial and AprilTag corner data:
  <https://pros.cs.purdue.edu/v5/pros-4/aivision.html>
- PROS C++ AI Vision API:
  <https://pros.cs.purdue.edu/v5/pros-4/group__cpp-aivision.html>
- VEX V5 Distance Sensor range and accuracy guidance:
  <https://kb.vex.com/hc/en-us/articles/360050696511-Using-the-V5-Distance-Sensor>

## Final Reality Check

This architecture can make error bounded whenever valid field observations
arrive often enough. It cannot guarantee exact position for an arbitrary
90-second route if the drivetrain slips while both the wall array and camera
have no valid reference. During that interval the system should continue from
odometry, increase uncertainty, and correct smoothly when reliable information
returns.

That is the practical definition of the master fusion system: high-rate
prediction, every-fresh-sample comparison, uncertainty-weighted correction,
strict outlier rejection, explicit degraded behavior, and evidence-based
recovery.
# 2026-07-19 hardware-map update

- Smart Port 9 is now confirmed as the reversed right slider motor, not a
  distance sensor. The left slider motor is Smart Port 2. R1/R2 command both.
- Production distance telemetry now has sensors only on Ports 6-8 and reports
  Port 9 unavailable for backward schema compatibility.
- Four-point LiDAR wall correction is fail-closed until a fourth distance
  sensor is assigned a non-conflicting port and its rigid geometry recalibrated.
- The claw pneumatic is now confirmed on three-wire ADI port H. L1 opens it and
  L2 closes it. Claw-arm motor rotation temporarily uses Right/Down arrows.
