# GPS reset localization investigation

Date: 2026-10-10  
Reviewed branch: `main` at `c8d65bd80defcea065f55de49aa7557666005fc3`

## Conclusion

The Brain map looking correct does **not** prove that an autonomous correction was applied. The map displays `live_pose()`, which is an odometry-propagated and GPS-smoothed estimate. The route correction has additional gates and can return `false` without showing which gate rejected it.

The most likely immediate failure is the checkpoint handshake:

1. A checkpoint is requested immediately after a motion.
2. The filtered estimate starts close to odometry because it was propagated by the same odometry during the move.
3. At rest, one 50 ms update moves the estimate only about 10.6% toward the raw GPS fix (`1 - sqrt(0.8)`).
4. For a 5 in raw GPS-versus-odometry error, the first filtered gap is only about 0.53 in.
5. The checkpoint requires at least 1.5 in, but it clears `checkpoint_pending` after that first confident sample even when no correction was applied.

The request is therefore consumed before the filter can expose enough of the GPS disagreement. This behavior is not covered by the existing checkpoint test because that test starts with an estimate already separated from odometry.

## Findings, ranked

### 1. Display success and reset success are different paths

`live_pose()` can stay locked and move with odometry for up to two seconds without a usable GPS fix. It also rejects a raw jump while continuing to output the propagated estimate. The yellow marker can therefore look smooth and plausible when the latest optical fix was rejected.

A reset/checkpoint additionally requires a valid route anchor, a fresh accepted fix, low speed, low turn rate, a 1.5-8 in correction, and agreement with the expected waypoint. The current display does not identify which requirement failed.

### 2. Most three-pin corrections are disabled by default

`THREE_PIN_GPS_RESETS` defaults to `0`. That disables `set_auto_reset(true)` and every `fix_at()` call. The only active correction in the default build is the explicit checkpoint after stack 1.

This is a valid safety choice while the mount is unqualified, but it can be mistaken for a GPS implementation that is running and failing.

### 3. The checkpoint clears itself before the estimator converges

Inside `live_pose()`, a confident checkpoint request is always cleared after one evaluation. It is cleared even when:

- the filtered gap is still below 1.5 in;
- the correction is above 8 in;
- the filtered GPS position is outside the waypoint radius.

Only the first case can naturally become valid a few samples later, but the request no longer exists.

Recommended behavior: keep the request pending until it is applied, proven unnecessary from a fresh raw fix, rejected with a specific terminal reason, or timed out by the caller.

### 4. The relock window is longer than the checkpoint wait

A raw fix more than 6 in from the estimator is treated as an outlier. While parked, eight agreeing fixes are required to relock. The telemetry loop runs every 50 ms, so eight samples require approximately 400 ms before scheduling overhead. The three-pin checkpoint waits only 350 ms.

A 6-8 in recoverable disagreement can therefore time out immediately before the estimator is able to accept it.

### 5. The start anchor is captured twice

`initialize()` captures `(0, 0, 180)`, and each autonomous routine captures it again. A new capture first invalidates the existing anchor. If the second 250 ms acquisition fails, a previously valid anchor is lost and all route-frame corrections fail closed.

Capture once, after final field placement and immediately before the route. Do not use an initialization-time anchor as the competition anchor.

### 6. The checkpoint handshake is shared across tasks without synchronization

The autonomous task writes `checkpoint_x`, `checkpoint_y`, `checkpoint_radius`, `checkpoint_pending`, and `checkpoint_applied`. The telemetry task reads and writes the same fields. They are plain C++ objects with no mutex or atomic release/acquire protocol.

This is a data race. The implementation must use a PROS mutex or a complete atomic publication protocol before the checkpoint handshake is reliable.

### 7. The current mount numbers are not yet qualified

The handoff records `forward=3.0 in`, `right=3.7 in`, but also records about **3.3 in disagreement** across the six spin stops. That is too large to treat the fitted offset as a precise physical measurement.

The automated turn test assumes the drivetrain rotation center stayed at one fixed field point. Real in-place turns translate because of scrub, unequal traction, and controller settling. Any center movement is mathematically indistinguishable from a wrong sensor lever arm in that fit.

## Better mount-offset measurement

Use a surveyed fixed-center test instead of trusting an autonomous spin.

1. Mark the drivetrain rotation center on the chassis floor projection.
2. Put that point over a taped cross on the field.
3. At eight headings around 360 degrees, manually rotate and then re-center the same point on the cross.
4. At every heading, hold the robot still and average at least 20 confident GPS lens readings.
5. Record the robot heading and the uncorrected lens X/Y.
6. Fit all stops to:

   ```text
   lens_x = center_x + forward*sin(h) + right*cos(h)
   lens_y = center_y + forward*cos(h) - right*sin(h)
   ```

7. Reject the run if the residuals are large or if removing one stop changes the fitted offset substantially.

The new `gps_mount_survey_auton()` collects eight manually re-centered stops and prints:

```text
GPS_MOUNT_SAMPLE,index,x_in,y_in,robot_heading_deg
```

Run the offline fit with:

```bash
python3 tools/fit_gps_mount.py gps_samples.csv --port 10 --facing 90
```

The tool reports the least-squares offset, fit RMS, largest residual, leave-one-out stability, opposite-heading consistency, and a paste-ready `gpsreset::init(...)` line.

Starting acceptance criteria in the tool are:

- fit RMS at most 1.0 in;
- largest stop residual at most 2.0 in;
- leave-one-out forward/right span at most 1.0 in;
- at least two near-opposite heading pairs;
- opposite-pair forward/right span at most 1.5 in.

These are screening limits, not guarantees of match accuracy. Repeat the survey at two field locations. A physical mount offset should remain the same; a materially different answer indicates field visibility, heading, or center-placement error.

## How to run the survey

Build the survey routine:

```bash
pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=gps_mount_survey_auton
```

Upload, enter driver control, and press LEFT. For each stop:

- keep the rotation center on the same taped cross;
- rotate approximately 45 degrees from the prior stop;
- re-center the cross precisely;
- press A and hold the robot still while it averages;
- press B to abort.

Save the `GPS_MOUNT_SAMPLE` terminal lines to a CSV file and run the Python fitter.

## Production changes recommended after the survey

1. Remove the initialization-time anchor; capture once at route start.
2. Protect all GPS shared state with a PROS mutex.
3. Keep checkpoint requests pending until apply or timeout.
4. Increase the checkpoint timeout above the worst-case relock interval, or use a direct stationary raw-fix average for explicit checkpoints.
5. Return a structured status such as `APPLIED`, `ALREADY_ALIGNED`, `NO_ANCHOR`, `NO_FRESH_FIX`, `MOVING`, `OFF_TARGET`, `TOO_LARGE`, or `TIMEOUT`.
6. Print raw GPS route pose, filtered route pose, odometry pose, gap, off-target distance, speed, turn rate, RMS, and final status for every checkpoint.
7. Do not enable continuous automatic resets until the surveyed mount passes and repeated route trials show lower independent field error than odometry alone.

## Fair validation

For each candidate configuration, run at least ten alternating trials from the same physical start. Measure the final robot center against field marks; do not use LemLib or GPS as the ground truth for its own test.

Compare:

- encoders + IMU only;
- one explicit GPS checkpoint using a qualified mount;
- automatic at-rest GPS corrections.

Log completion rate, independent endpoint X/Y error, heading error, corrections applied, corrections rejected by reason, and autonomous time. Do not keep GPS enabled merely because the on-screen traces overlap.
