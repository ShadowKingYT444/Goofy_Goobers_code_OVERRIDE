# Localization sensor repair report

## Outcome

The localization loop is running on the robot and the web UI is displaying its live fused pose, with the fixed camera providing an independently tracked along-wall Y correction when its confidence/freshness gates pass. The final stationary UI reading was approximately `X=-48.1 in`, `Y=0.5 in`, `H=94.0 deg`, with both left motors and both right motors healthy. The browser-rendered dashboard explicitly said `Fused pose + fixed camera Y` and had no console warnings or errors.

The three reported failures were addressed:

- LiDAR no longer hard-resets the IMU every fit. It applies a bounded, filtered software heading correction only after three consistent observations. A 45-second stationary run had raw-IMU span `0.02 deg` and fused-heading span `1.08 deg`, eliminating the rapid oscillation.
- Left/right drive readings are synchronized deltas with per-side finite-count and spread checks. A powered `1.5 in` forward test produced motor deltas `+64,+63,+62,+63 deg`, onboard forward displacement `1.51 in`, and less than `0.4 deg` heading change. The previous huge discrepancy came from comparing unsynchronized absolute histories.
- Horizontal/side odometry rejects device errors, non-finite values, and implausible jumps. Its offset is applied during rotation, and it remains part of the prediction loop. This differential-drive robot cannot perform a powered pure lateral-slide calibration, so the side wheel's absolute scale was not independently measured by striding.

LiDAR now evaluates all four field-wall heading candidates, chooses by heading continuity plus observable-axis agreement, and corrects the coordinate perpendicular to the observed wall. The live audience-wall calibration uses a `5.29 in` left mount offset and per-sensor corrections `{+7, 0, -5, -2} mm`; these reduced the static line-fit error from about `0.11 in` to `0.051 in`. A slow rotation proved the field-heading relationship is `wall base + measured theta`, and the corrected estimator remained within roughly `1.4 deg` of the encoder/IMU turn estimate during the confirmation run.

The LiDAR fitting workload is limited to `10 Hz`. IMU and odometry prediction continue every `20 ms`. Web telemetry is capped at `50 ms`, with an observed cadence of `16.67 Hz` because it is emitted on the 20 ms control-loop boundary.

## Camera and motion evidence

The fixed `Brio 101` camera was used as independent ground truth and as a freshness/confidence-gated Y correction in the laptop UI; it is not an onboard Brain estimator input. The captured forward-motion video is `1280x720`, 660 frames, 22 seconds. Template tracking measured `34.437 px/in` for the known motion. It exposed wheel/tether slip after the encoder return, after which a camera-guided `5.52 in` reposition placed the robot within `0.61 in` of its original image anchor.

At the final anchor, camera and onboard pose differed by approximately `1.1 in` along the wall (`0.61 in` camera offset versus `+0.50 in` onboard Y). This is the best measured end-to-end translation error from this session and is well inside the requested useful web-control margin. Final heading was about `94 deg`, within the requested `10 deg` tolerance of the known `90 deg` start orientation.

The camera anchor is persisted rather than inferred again from the first frame on each launch. A live **Reset Start** test changed the stored baseline from `714` to the current `715 px` while holding field Y at `0.5 in`; a full server restart then reloaded `715 px` and continued reporting `0.5 in`. This closes the failure where restarting the UI could silently erase along-wall motion.

A final 30.5-second post-restart stationary audit collected 61 samples: Brain time advanced `30,720 ms` without resetting, X span was `0.00 in`, camera-Y span was `0.00 in`, fused-heading span was `0.94 deg`, and camera tracking was available in all samples with a minimum confidence of `85.28%`.

Evidence artifacts are in this workflow directory, including `camera_forward_test.mp4`, `camera_forward_result.json`, `camera_reposition_result.json`, and `camera_reposition_final.png`.

## Changed files

- `src/autons.cpp`
- `src/main.cpp`
- `include/autons.hpp`
- `tools/lidar_bar_server.py`
- `LOCALIZATION_GPT56_HANDOFF.md`
- `NEXT_CHANGES.md`
- `agent-workflow/localization-sensor-repair/*`
- stale localization source-contract tests under the existing `agent-workflow` suites

## Commands and checks run

- Repo-local PROS build: passed after final source changes.
- PROS upload to `COM9`, slot 1, followed by run: passed.
- `localization_repair_check.py`: passed bounded-bias, inconsistent-fit, missing/disagreeing-motor, port-5 error/jump, and angle-wrap cases.
- `localization_failure_checks.py`, `autons_fusion_source_check.py`, `fusion_test_source_check.py`, `localization_config_check.py`, `odometry_sign_check.py`, `telemetry_parser_check.py`, and `master_fusion_note_check.py`: passed.
- Python compilation of the server/check scripts: passed.
- `camera_tracker_check.py`: passed the recorded 14 px / 0.47 in movement test and camera persistence source contract.
- Live stationary, slow-rotation, and 1.5-inch forward trials: passed the criteria above.
- Post-restart rendered `/localization` browser inspection: passed with `Fused pose + fixed camera Y`, healthy drive sides, selected audience wall, fresh 86% camera tracking, and no console warnings/errors.

## Failed attempts

- Open-loop powers 14, 20, and 30 were below or inconsistent around this geared drivetrain's breakaway torque; slow closed-loop velocity was used for repeatable calibration.
- The first heading formula used `base - theta`; the live rotation showed the sign was wrong. It was corrected to `base + theta` and revalidated at 8 RPM.
- Returning the encoders to zero did not physically restore the robot because the wheels/tether slipped. The fixed camera caught this, and the robot was repositioned from camera ground truth.
- PROS hot upload preserved estimator state while hardware encoders restarted at zero. Lifecycle and zeroed-hardware invariants were added; the final upload restarted at the configured field pose.
- Headless Chrome/Edge screenshot attempts were blocked by Windows GPU/sandbox policy. The in-app browser successfully rendered and verified the same local page.
- A server restart path hit PowerShell's duplicate `Path`/`PATH` environment behavior; the UI was restored as a long-running workspace process. A separate first-frame camera re-zero defect was then reproduced and fixed with persisted calibration.
- The first final documentation regression run failed because `master_fusion_note_check.py` required obsolete wording that described all active behavior as partial and denied any camera integration. The contract was updated to distinguish active deterministic fusion, PC-only camera Y, and the still-unimplemented covariance estimator; the full suite then passed.

## Remaining risks and work not performed

- A single flat wall supplies heading and the coordinate perpendicular to that wall; it does **not** independently measure the coordinate parallel to the wall. That axis still depends on synchronized drive odometry and the side tracking wheel. The camera test demonstrated that physical wheel/tether slip can therefore create translation error.
- Only the audience-wall candidate was physically reachable within the LiDAR's useful range at this start. Red, blue, and zero candidates have adversarial/synthetic coverage but were not each driven to and live-tested.
- The fixed camera is not fused into the robot/Brain estimator. Its Y correction exists only in the laptop UI and falls back automatically when the frame is stale or confidence is below `0.62`; therefore onboard autonomous code cannot use that correction.
- No powered pure lateral slide was possible because the left front/back are geared together and the drivetrain has no striding. Side-encoder scale therefore remains the configured value, protected by error/jump guards.
- Other robots or objects forming a convincing flat line can still be mistaken for a wall. Fit quality, range, consistency, continuity, and axis-error gates reduce this risk but cannot make measurements literally infallible.
- Existing LVGL enum, linker RWX, and `/tmp` build warnings remain unrelated to localization.

## Manual use

Run `start_localization_ui.cmd`, then open `http://127.0.0.1:8774/localization`. Before each field run, ensure the configured start pose matches the robot's real placement and press **Reset Start** only when it is at that anchor. The exact build, upload, UI, and verification commands are in `runbook.md`.
