# Multisensor Localization Evidence Report

## Outcome

The supervised tether-managed route exercised approximately 120 inches (10 feet) of commanded translation. The estimator continuously propagated with drive encoders, the rear horizontal tracking wheel, and IMU; LiDAR and AI Vision remained gated absolute corrections. A failed low-power turn stopped safely, the defect was corrected, and the continuation route completed without an abort.

## Current calibrated system

- Drive: ports 17/18 left and 11/13 right, normalized encoder signs `+1/+1`, 2.75-inch wheels.
- Effective drive track width: `12.0086 in`.
- Rear horizontal tracking wheel: port 5, 2-inch wheel, sign `+1`, rear-center lever `5.18 in`.
- IMU: port 3, continuous heading source; corrections are bounded rather than replacing it.
- Four-sensor distance bar: ports 6-9, LiDAR theta scale `0.926770`; invalid fits are rejected.
- AI Vision: port 20, camera extrinsic `(forward=6.75, right=-10.44, yaw-right=90 deg)`, effective tag outer size `1.05 in`.
- Field/UI frame: center-origin inches, +X right, +Y up, 0 degrees along +X.

## Long-route evidence

`long_route_live.log` records the first 40-inch reverse/return pair and an attempted 90-degree turn. The translation returned near the starting point, but the turn stalled 14.25 degrees short because the 13.5-power command did not overcome static friction. The route aborted before subsequent translation. This was a successful watchdog/failure-detection test, not a completed route.

The fix raised minimum turn power to 18, added a one-second turn-stall watchdog, and preserved estimator state when handing the route back to normal telemetry. `recovery_route_live.log` then records:

- recovery turn: 73.74 degrees achieved;
- reverse/forward pair 1: 40 inches total;
- reverse/forward pair 2: 40 inches total;
- continuation travel: 80 inches;
- combined session travel: approximately 120 inches;
- raw return-position error: `1.54 in`;
- raw return-heading error: `4.93 deg`;
- no stall or route abort;
- same-side encoder spread remained 0-6 motor degrees in captured settled samples.

During the route, LiDAR corrections were unavailable or rejected rather than being reused. After the route, Tag 3 reacquired and bounded fusion converged from raw `(16.85,45.30,151.16 deg)` to stationary `(22.91,41.15,144.18 deg)`. At convergence, observed/predicted tag range residual was `0.09 in`, bearing residual `0.00 deg`, and the duplicate physical-goal margin was about `159 deg`.

## AprilTag ambiguity behavior

Repeated IDs are treated as multiple physical hypotheses. Range defines a circle around each matching tag face; camera bearing plus the current fused heading turns each into a candidate robot position. Field bounds, face visibility, range/bearing residuals, innovation limits, physical-goal margin, and three consistent observations gate correction. A tag never resets pose merely because it is visible.

The web UI draws the selected physical hypothesis in green and alternate paired-prop hypotheses in amber. This source change passes unit tests, but a fresh rendered screenshot was not obtained because the in-app browser service returned `No browser is available`.

## Commands run

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m pros.cli.main make
```

Results: 16/16 tests passed; firmware linked and packaged successfully.

## Failed attempts and limitations

- The first long-route turn stalled; it aborted safely and directly produced the minimum-power/watchdog fix.
- `.venv\Scripts\pros.exe make` is stale because it embeds a removed uv Python path. The module command above is the working build path.
- The controller start-pose editor is implemented and regression-tested but has not been physically exercised in this final session.
- Only the currently visible Tag 3 pose has live absolute-correction evidence. Other duplicated IDs/field locations are covered structurally, not by physical observations.
- A fresh visual verification of the new ambiguity overlay remains pending.

## Changed production surfaces

- `include/localization_config.hpp`: centralized calibration, gates, and extrinsics.
- `src/autons.cpp`: fused propagation/correction, calibrated kinematics, watchdogs, route-state handoff, duplicate-tag handling.
- `src/main.cpp`: safe startup flags and runtime pose editor.
- `src/ai_vision_localization.cpp`: AI Vision discovery and tag geometry telemetry.
- `tools/lidar_bar_server.py`: live diagnostic UI and tag hypothesis visualization.
- `tests/test_localization_ui.py`: fusion/UI safety regressions.

## Manual follow-up

Start the UI with `start_localization_ui.cmd`, open `http://127.0.0.1:8774/localization`, and visually confirm the green selected Tag 3 circle/blob and amber alternate hypotheses. With the controller connected, verify the Y-held start-pose editor before relying on an arbitrary competition start location.

## Slow 20-foot follow-up campaign

The final campaign added `slow_long_route_final_live.log` and the constant-memory `slow_long_route_final_summary.json`. The robot completed 240 inches of slow translation plus 60 degrees of bounded rotation with no abort. Raw dead reckoning ended `6.42 in` from the route start and `4.32 deg` from the start heading. Tag 3 then converged to `(30.18,34.70,151.65 deg)` with `0.09 in` remaining innovation and `0.07 in` range residual. No impossible position/heading rate or oversized fusion correction was observed. The left coupled encoder pair peaked at 9 degrees spread and the right at 0; production now rejects a side at 15 degrees rather than the previous 45.

Two fusion defects were found and fixed during this campaign. First, stationary zeroed encoders no longer reset a legitimate absolute position correction. Second, AI Vision no longer changes heading from a single point-tag range/bearing observation, because that measurement does not independently observe position and heading. Camera position correction is conditioned on the IMU/LiDAR heading. Innovations over 8 inches require 12 consistent observations and remain limited to 0.75-inch steps; anything over 24 inches is discarded. A coherent one-meter LiDAR jump is rejected by the 1.5-inch axis-innovation gate before any correction, and accepted LiDAR axis steps remain bounded.

Final verification: 18/18 host tests pass, firmware builds and uploads, startup motion flags are false, and live telemetry on COM8 reports `(30.18,34.70,151.65 deg)` with Tag 3 range residual `0.07 in`.

The fixed BRIO 101 endpoint frame is saved as `slow_20ft_endpoint_brio.png`. Compared with `recovery_route_end.png`, it independently shows that the later route ended displaced to the right with a changed heading, supporting the fusion result that the robot did not physically return to the earlier pose. The current crop does not contain enough surveyed floor control points to convert pixels into trustworthy field inches, so this remains qualitative rather than centimeter-level ground truth.
