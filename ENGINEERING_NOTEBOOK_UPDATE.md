# Engineering Notebook Update — Fused Localization

**Main work date:** July 10, 2026  
**Follow-through reviewed:** early July 11, 2026  
**Project:** MoreVex VEX V5 robot

## What changed in simple English

The localization system changed from a mostly dead-reckoning system with limited wall correction into a safer multi-sensor estimator. The robot still uses its wheel encoders, rear tracking wheel, and IMU continuously. LiDAR and AI Vision now act as occasional absolute corrections only when their measurements pass several safety checks.

This is important because no single sensor is trusted blindly:

- Drive encoders estimate forward movement.
- The rear horizontal tracking wheel measures sideways movement and the movement caused by rotation at its rear mounting position.
- The IMU supplies the continuous field heading.
- The four distance sensors estimate the nearby field wall angle and distance.
- AI Vision reads AprilTags on known Goals and can gently correct global X/Y position.
- Bad, stale, ambiguous, or physically impossible readings are rejected instead of moving the pose.

The field coordinate convention remains center-origin inches: **+X right, +Y up, and 0 degrees along +X**.

## July 10 changes

### 1. The drivetrain and tracking-wheel geometry were calibrated

The drivetrain geometry was moved into `include/localization_config.hpp` so the estimator uses one shared set of values. The effective drive track width was measured as `12.0086 in`. The rear horizontal tracking wheel was confirmed to be a 2-inch wheel centered on the rear face of the robot. Its raw sign is positive and its calibrated rear offset is `5.18 in` in the current source.

The estimator now removes the sideways motion that the rear-mounted wheel naturally reports while the robot rotates:

```cpp
const double delta_side_center_in = side_delta_accepted
    ? delta_side_wheel_in - kSideOdomOffsetBackIn * delta_heading_rad
    : 0.0;
```

This compensation is applied only to a fresh, accepted tracking-wheel sample. A disconnected sensor, the first recovery sample, or an implausible jump contributes zero motion and establishes a new baseline instead of reusing stale data.

### 2. Drive encoder health checks became stricter

Both motors on each drive side are read together. A side is accepted only when enough finite readings agree within the configured spread limit. This prevents one disconnected or badly disagreeing motor from corrupting forward distance.

Live testing included a short powered forward movement where all four motors moved by nearly the same amount and the fused pose reported approximately the commanded distance. Later route testing motivated tightening the coupled-motor spread rejection threshold from 45 motor-degrees to 15.

### 3. LiDAR changed from a hard heading reset to a bounded correction

The four distance sensors on ports 6, 7, 8, and 9 are fitted to a line. The fit is rejected when a sensor is missing, too far away, low confidence, or when the line error is too large. Valid measurements are compared with all four possible field walls.

The selected wall must agree with both the current heading and the coordinate perpendicular to that wall. The estimator also requires repeated consistent fits. Even then, it changes heading bias and the observed X or Y coordinate in small bounded steps:

```cpp
if (best.axis_error_in > kMaxLidarAxisInnovationIn) {
  pose.lidar_reject = "axis";
  return;
}

bias_step_deg = clamp(filtered_heading_error_deg * gain,
                      -maximum_step, maximum_step);
axis_step_in = clamp(axis_error_in * axis_gain,
                     -maximum_axis_step, maximum_axis_step);
```

Live wall testing also corrected the wall-angle sign, calibrated a LiDAR theta scale of `0.926770`, and reduced the allowed heading/axis innovations after an obstructed but clean-looking line tried to pull the estimator toward a false wall angle.

### 4. AI Vision was added to production localization

An isolated hardware probe and later production discovery work established the current production AI Vision mapping as **Smart Port 20**. Earlier intermediate notes mention port 1, but the present source and the later tournament workflow identify port 20; port 1 was an intermediate probe result and should not be used as the final wiring record.

Production code now:

- configures tag-only 21H7 detection;
- reads tag ID and four image corners;
- rejects small, distorted, clipped, stale, or invalid detections;
- estimates camera bearing and range;
- applies the measured camera-to-robot offset;
- compares an observed tag with the known Goal map;
- handles repeated tag IDs as multiple physical hypotheses;
- rejects the observation when two different Goals are too close in score.

The repeated-ID logic is especially important because IDs 1–4 occur on more than one physical Goal. Seeing an ID is not enough to reset the robot position.

```cpp
const double margin = different_goal != candidates.end()
    ? different_goal->residual_deg - best.residual_deg
    : INFINITY;

if (margin < localization::kAiMinCandidateWinnerMarginDeg) {
  pose.ai_reject = "ambiguous";
}
```

The current camera calibration in `localization_config.hpp` uses a front/right mounting transform of `(6.75 in, -10.44 in)` and a 90-degree right-facing yaw. These values came from live reversible motion/tag trials, not from the laptop webcam.

### 5. AI Vision corrections were made gradual and fail-closed

AI Vision does not teleport the pose. A normal position innovation must be at most 8 inches and requires three consistent observations. A larger 8–24 inch reacquisition requires 12 consistent observations. Each accepted update moves the pose by no more than 0.75 inch.

```cpp
const bool normal_innovation =
    innovation_in <= localization::kAiMaxPositionInnovationIn;
const bool proven_reacquisition =
    innovation_in <= localization::kAiMaxReacquisitionInnovationIn &&
    pose.consistent_ai_observations >=
        localization::kAiRequiredReacquisitionObservations;

const double applied_step_in =
    std::min(innovation_in * localization::kAiPositionGain,
             localization::kAiMaxPositionStepIn);
```

The single-tag observation is currently used for **position correction only**. It does not independently contain enough information to correct position and heading at the same time, so camera heading gain is set to zero. Heading continues to come from the IMU with bounded LiDAR bias correction.

### 6. The estimator runs continuously and publishes its own pose

`update_pose()` propagates the pose using drive encoders, the rear wheel, and IMU. AI Vision is checked during the update, while LiDAR is sampled at its limited correction rate. Normal operator-control telemetry calls `localization_telemetry_update()` whenever an autonomous helper is not controlling the robot.

The `FUSE_TEST` telemetry line was expanded to include:

- fused X, Y, and heading;
- raw field IMU and IMU bias;
- LiDAR wall, rejection reason, fit error, and correction step;
- drive motor counts and spreads;
- rear-wheel health and calibration values;
- AI tag ID, bearing, range, innovation, selected Goal/face, age, and rejection reason.

This makes rejected sensor data visible instead of silently disappearing.

### 7. The browser UI became a fused-localization diagnostic tool

`tools/lidar_bar_server.py` now parses the onboard fused telemetry and displays pose freshness, drive health, IMU state, LiDAR gates, AI Vision candidates, and rejection reasons. It draws the selected physical tag hypothesis in green and alternate repeated-ID hypotheses in amber.

The browser can still calculate motor-plus-side-wheel fallback odometry when the onboard fused pose is stale. That fallback is clearly labeled diagnostic and is not presented as the full estimator.

The fixed Brio laptop camera was used during calibration as independent motion evidence. Its old Y correction exists only on the host UI and is **not** an onboard sensor used by autonomous driving.

### 8. Startup behavior was made safer

A leftover startup scan flag that moved the robot after boot was disabled. Tests now require every startup-motion flag to remain false. A no-motion boot check confirmed zero drive/rear-wheel movement and nearly unchanged IMU heading.

A fail-closed controller start-pose editor was also added. While Y is held, the operator can adjust X/Y/heading and press A to save. Drive and mechanism outputs are forced to zero while editing. The code and regression tests passed, but the final workflow report says live controller verification was still pending.

### 9. A hardware port conflict was resolved safely

Smart Port 9 remains the fourth distance sensor. The old left-slider motor claim on port 9 was removed. Because the actual second slider motor port was not identified, slider output is intentionally disabled instead of driving only one side and possibly racking the mechanism.

### 10. The PROS project/toolchain was repaired

The AI Vision investigation exposed mixed PROS 4.1.1 and 4.2.2 build artifacts. The project was migrated to a consistent PROS 4.2.2 hard-float kernel and the EZ-Template archive was rebuilt. This allowed the production AI Vision API and the rest of the firmware to link together consistently.

## Live tests and evidence from July 10

The workflow records the following physical or rendered checks:

- stationary drift and LiDAR stability checks;
- a short powered straight-drive check with agreeing motor encoders;
- slow CW/CCW rotation used to determine LiDAR sign/scale and rear-wheel offset;
- AI Vision hardware scans and live tag detections;
- rejection of an ambiguous repeated-ID observation;
- rejection of an obstructed false LiDAR wall angle;
- a no-motion boot lasting more than eight seconds;
- a rendered browser dashboard with fresh fused pose and sensor status;
- unit/source-contract tests and successful firmware builds.

Some workflow artifacts disagree because they capture different stages of the same investigation. The final conclusions in this note follow the latest production source and the later evidence report, not an earlier intermediate port guess or webcam-assisted acceptance claim.

## Early July 11 follow-through

The work continued shortly after midnight and is separated here to keep the July 10 record accurate.

A slow 240-inch (20-foot) route with bounded turns was completed. The route exposed two additional estimator defects:

1. Zeroed stationary drive encoders could reset a legitimate absolute position correction. This was fixed so an accepted tag correction remains in the estimator.
2. A single point-tag range/bearing observation was incorrectly being allowed to influence heading. Heading feedback from this measurement was disabled, leaving it as a position-only correction conditioned on IMU/LiDAR heading.

The completed route reported raw dead-reckoning return error of `6.42 in` and `4.32 deg`. After Tag 3 reacquisition, the bounded position correction converged to approximately `(30.18, 34.70, 151.65 deg)` with `0.09 in` remaining innovation and `0.07 in` range residual. The workflow reports 18/18 host tests passing, a successful build/upload, and restored no-startup-motion firmware.

These are strong live results, but they are not proof that every field location and every repeated tag ID has been physically validated.

## Important limitations still remaining

- Only the currently visible Tag 3 configuration received strong live absolute-correction evidence. Other Goals and duplicated IDs have structural/adversarial coverage but still need physical field testing.
- One flat wall observes heading and only the coordinate perpendicular to that wall. It cannot independently correct travel parallel to the wall.
- The rear tracking-wheel scale and mounting calibration are based on the available rotation trials; more surveyed tests could improve them.
- Objects or robots that resemble a clean flat wall may still fool the line fit, although the confidence, consistency, heading, and axis gates reduce the risk.
- The runtime controller pose editor still needs a final physical controller test.
- The newest AI Vision ambiguity overlay passed source/unit checks, but the final report says a fresh screenshot was blocked when no browser service was available.
- The current estimator is deterministic bounded fusion, not a covariance-based EKF.

## Main files changed

- `include/localization_config.hpp` — shared geometry, calibration, tag map, camera transform, and correction gates.
- `include/ai_vision_localization.hpp` — timestamped AI Vision snapshot interface.
- `src/ai_vision_localization.cpp` — sensor discovery, configuration, tag geometry, range, and bearing observations.
- `src/autons.cpp` — continuous pose propagation, LiDAR correction, AI Vision position correction, fused controllers, telemetry, and watchdogs.
- `src/main.cpp` — initialization, device inventory, safe startup flags, pose editor, and continuous localization updates.
- `include/subsystems.hpp` — hardware declarations updated to remove the conflicting slider port claim.
- `tools/lidar_bar_server.py` — fused-pose parsing, health/freshness display, tag-hypothesis overlay, and diagnostic fallback odometry.
- `tests/test_localization_ui.py` and workflow checks — regression coverage for parser, safety gates, startup motion, and UI behavior.
- `NEXT_CHANGES.md` — updated architecture/current-state handoff.
- `agent-workflow/localization-sensor-repair/`, `agent-workflow/multisensor-localization-completion/`, and `agent-workflow/tournament-localization/` — plans, live logs, captures, checks, and evidence reports.

## How to reproduce the software checks

From the repository root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m pros.cli.main make
```

The workflow notes say the direct `.venv\Scripts\pros.exe` launcher is stale, so the Python module form above is the known working build command.

For the UI:

```powershell
.\start_localization_ui.cmd
```

Then open `http://127.0.0.1:8774/localization` and verify that the onboard fused pose is fresh, both drive sides are healthy, the rear wheel and IMU are updating, and LiDAR/AI Vision show an explicit accepted or rejected state.

Do not flash firmware or run a motion route without a clear field, tether management, and explicit authorization.

## Evidence used for this update

This notebook was reconstructed from July 10 file timestamps, the current production source, `NEXT_CHANGES.md`, and the durable reports/progress logs in the three localization workflow directories. Git history could not provide a before/after diff because this checkout's `main` branch has no commits; therefore exact “changed from” claims are limited to behaviors documented by those workflow artifacts and confirmed in the current code.
