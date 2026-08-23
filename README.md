# Brain-only localization

This PROS program estimates the robot pose on the official 12 ft × 12 ft
Push Back field. Runtime localization uses only devices connected to the V5
Brain. The laptop webcam is used only to record independent test evidence.

## Hardware contract

- drive motors: left 17/18, right 11/13
- IMU: port 3
- rear sideways tracking wheel: rotation port 5, 2 in wheel
- parallel side-facing distance sensors: ports 8 and 9, separated by 2 in
- opposed AI Vision sensors: ports 19 and 20, 35 cm lens-to-lens

The application supplies its known starting pose to `brainloc::init()`. No
starting coordinate is compiled into the library.

## Estimator behavior

- The IMU propagates heading.
- Drive encoders propagate forward motion. A 100 ms encoder-versus-IMU gate
  suppresses translation when wheel motion is inconsistent with measured yaw.
- The rear tracking wheel propagates sideways motion after removing the
  measured rotational lever-arm component.
- Each AI camera independently obtains tag range and bearing from its calibrated
  intrinsics. The field map, current IMU heading, and measured camera extrinsics
  produce position candidates. Duplicate-tag/face ambiguity is resolved by the
  closest physically plausible candidate, and corrections are rate- and
  innovation-limited.
- Ambiguity resolution is explicitly temporal. A duplicate tag candidate must
  be closest to the continuously propagated prior, agree across repeated
  frames, contain fresh geometry, and pass bounded innovation/correction-rate
  gates. A single frame cannot teleport the pose to a mirrored field location.
- Port 19 has a view of an onboard ID-3 marker near 18 in and -30 degrees.
  That camera-relative signature is removed before selecting the largest tag,
  so the robot cannot mistake its own marker for a stationary field landmark.
- The two distance sensors are sampled and checked for mutually consistent wall
  geometry. They do not currently alter global pose because no reliable
  surveyed wall association has been established for this mounting.
- When no tag is visible, odometry continues from the last pose. There is no
  laptop or network dependency.
- Localization runs in its own PROS task. `initialize()` returns normally, so
  competition callbacks and the robot's eventual autonomous/driver code can run
  alongside the estimator.

## Library API

Include `brain_localization.hpp` and compile
`src/brain_localization.cpp` with the robot project.

```cpp
void initialize() {
  brainloc::init({24.0, 24.0, 90.0});
}

void autonomous() {
  brainloc::MotionOptions options;
  options.maximum_drive = 35;
  options.maximum_turn = 30;

  const auto result = brainloc::go_to(48.0, 36.0, options);
  if (result != brainloc::MotionResult::reached) {
    brainloc::cancel_motion();
  }
}
```

Public operations include:

- `init(startPose)`, `get_pose()`, `get_status()`, and `healthy()`
- `get_history()`, `get_visible_tags()`, and `get_active_path()`
- `drive_straight_to(x, y)` / `go_to(x, y)`, `turn_to(heading)`, and
  `go_to_pose(pose)`
- `follow_path(waypoints)`, `cancel_motion()`, and `motion_active()`

Localization updates in a dedicated 20 ms task. Navigation calls are blocking
and should be invoked from autonomous code or a separate task. Every motion has
a timeout, cancellation, pose-health check, and field-coordinate progress
stall detection. Tiny encoder motion or wheel spin does not count as progress.

The example `src/main.cpp` has `kRunLibraryDemo = false`; the normal slot-6
build therefore cannot command startup motion.

## Sensor integration and tagless travel

- IMU heading, four drive encoders, and the rear tracking wheel propagate pose
  continuously, including through areas with no visible tag.
- Either AI camera can correct the propagated position. If both see landmarks,
  they independently correct the same temporal prior.
- The paired side distance sensors are continuously sampled and their
  confidence/geometry is exposed in `Status`. They are not allowed to alter the
  global pose until the sensor-to-wall association is surveyed; forcing an
  unknown wall association would make localization less competition-safe.
- If encoder motion contradicts IMU yaw, translation is gated for that update
  instead of injecting wheel slip into the pose.

## Verification

Build with the local PROS toolchain, upload to slot 6, and capture serial
telemetry. Then run:

```sh
./verify_log.py ../brain_localization_final_route.log --require-route
./verify_log.py ../brain_localization_dual_cal.log --require-cameras 19,20
```

A test is not evidence of absolute field accuracy unless the initial pose and
tag-map coordinates were independently surveyed. The verifier establishes
continuous sensor operation, bounded estimator behavior, actual movement, and
accepted Brain-camera corrections.

The measured route produced an ID-3 observation on port 19 whose range and
bearing remained essentially constant while the chassis translated. This
proves it moves with the robot rather than being a field landmark. The original
12 in innovation gate rejected it, and the production build now removes this
self-observation before target selection.
