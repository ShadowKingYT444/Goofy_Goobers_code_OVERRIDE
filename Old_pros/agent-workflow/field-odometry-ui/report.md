Changed files:

- tools/lidar_bar_server.py
- src/main.cpp
- include/subsystems.hpp
- agent-workflow/field-odometry-ui/*

Latest update, 2026-07-08:

- Added `FUSE_TEST` pose parsing to the LiDAR dashboard server.
- `/data` now includes `onboard_pose` when robot firmware prints fused pose logs.
- Added `/pose-grid` as a direct field-grid route.
- Added an always-visible field HUD for pose source, x, y, and heading.
- The grid prefers fresh onboard fused pose and falls back to browser D4 odometry when no fresh onboard pose exists.
- Browser-side LiDAR no longer projects/corrects x/y; LiDAR remains heading/wall context only.

Latest commands run:

- `.\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py`
- Embedded dashboard JavaScript syntax check through Node `new Function(...)`
- `Invoke-WebRequest http://127.0.0.1:8774/pose-grid`
- `Invoke-WebRequest http://127.0.0.1:8774/playing-field`
- `Invoke-RestMethod http://127.0.0.1:8774/data`

Latest results:

- Python syntax passed.
- Parser regression passed for D4 frames and both `FUSE_TEST ... h=` and `FUSE_TEST ... heading=` pose lines.
- Odometry sign regression passed.
- Server is running at `http://127.0.0.1:8774/`.
- `/pose-grid` and `/playing-field` serve the field-only view with the HUD markers.
- Live `/data` is streaming from `COM8` at 50 Hz.
- `onboard_pose` is currently absent until the robot prints `FUSE_TEST` lines during the fusion auton/test.
- In-app browser control was unavailable in this environment, so visual verification is limited to served HTML markers and live endpoint checks.

What changed:

- Added a 144 in x 144 in square field map to the LiDAR dashboard.
- Added browser-side differential-drive odometry from motor positions.
- Uses left ports 17/18 and right ports 11/13.
- Uses 2.75 in wheel diameter and 10 in track width.
- Starts at bottom-left, x=0 in, y=0 in, heading=90 deg.
- Fixed left joystick inversion in src/main.cpp by negating only the left joystick before commanding left motors.
- Fixed web odometry turn direction after user reported clockwise physical turns drew counterclockwise.
- Normalized the left encoder sign in web odometry because live telemetry showed left port 18 had opposite sign from right ports 11/13.
- Stabilized Distance, Angle, and RMSE styling so visual state changes no longer change font size or color.
- Added a Reset Origin button to re-baseline the field dot without reloading the page.
- Added Left Wheel and Right Wheel travel readouts using normalized encoder signs.
- Added Last Delta and Turn Delta readouts to expose per-frame wheel/heading changes while driving.
- Added horizontal odometer telemetry from port 5 as `h5=<centidegrees>` in the D4 stream.
- Added browser-side side odometry using the 2 in odometer wheel and 12 cm rear offset compensation.
- Added Side Odom and LiDAR Assist diagnostics.
- LiDAR wall context for the left-facing sensor bar is gated by RMSE <= 0.20 in, max point error <= 0.75 in, and all wall distances under 50 in.
- Added a red LiDAR field-map visualization with four wall hypotheses, RMSE-spaced probability bands, theta labels, and arrows every 12 in.
- Added dashboard tabs and a field-only route at `/playing-field`; `/playing%20field` also works for the user's wording.
- Confirmed the field view uses `fieldSizeIn = 144`, matching the 12 ft x 12 ft field.
- Added last-good LiDAR hold behavior: bad RMSE, missing points, or failed gate no longer move the red field hypotheses or big LiDAR readouts.
- Removed browser-side LiDAR x/y correction in the latest update. With or without a good LiDAR gate, the displayed fused x/y comes from onboard `FUSE_TEST` pose when fresh, otherwise browser odometry fallback.
- Wall hypothesis selection now scores both distance agreement with current odometry and theta agreement with the odometry heading, using 180-degree-periodic line-angle error to avoid 90-degree jumps.
- Added a cyan drivetrain-encoder vertical displacement overlay: a horizontal `drive Y` line across the field plus a vertical displacement arrow from the bottom edge.
- Added telemetry parser regression coverage for old motor-only frames and new h5 frames.
- Removed the stale port-5 `claw_rotate_motor` declaration and L2 command because port 5 is now the horizontal odometer.

Commands run:

- .\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py
- .\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py
- .\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py
- Embedded dashboard JavaScript syntax check through Node `new Function(...)`
- .\.venv\Scripts\pros.exe make
- .\.venv\Scripts\pros.exe upload --slot 1 --name MoreVex --after run . COM15
- Invoke-WebRequest http://127.0.0.1:8774/
- Invoke-WebRequest http://127.0.0.1:8774/playing-field
- Invoke-WebRequest http://127.0.0.1:8774/playing%20field
- Invoke-RestMethod http://127.0.0.1:8774/data
- Chrome headless screenshot for http://127.0.0.1:8774/playing-field
- Chrome headless screenshot saved to `agent-workflow/field-odometry-ui/playing-field-drive-y.png`

Results:

- Python syntax passed.
- Browser dashboard loaded and showed the square field map.
- Live LiDAR telemetry still streams on COM14.
- Rebuild and upload succeeded after permissions changed.
- Live motor telemetry is now visible in the dashboard for ports 18, 11, and 13.
- Port 17 reports inf in the raw serial line, so the parser ignores it and uses the finite left-side motor value from port 18.
- Chrome verification after the final web UI patch showed all three big readouts as class theta-value, 72px, rgb(244, 251, 248).
- Chrome verification showed live finite motor telemetry for port 18 and right ports 11/13.
- Dashboard code now uses leftEncoderSign=-1, rightEncoderSign=1, and standard deltaHeadingRad=(right-left)/trackWidth after sign normalization.
- Added agent-workflow/field-odometry-ui/odometry_sign_check.py. It passed and verifies forward/up, clockwise, and counterclockwise sign behavior for the web odometry math.
- Chrome verification showed Reset Origin, Left Wheel, and Right Wheel controls/readouts are present and the dashboard remains live on COM14.
- Chrome verification showed Last Delta and Turn Delta readouts render, with stationary values at L 0.00 / R 0.00 and 0.00 deg when the robot is not moving.
- Latest endpoint/browser comparison showed no encoder movement over a 6-second sample; endpoint and browser both reported zero deltas.
- Completion audit reran Python syntax checks, odometry sign regression, PROS make, live /data, and Chrome DOM verification successfully.
- Horizontal odometer sign regression passed for right/left side movement.
- Telemetry parser check passed for old frames and new `h5=12345` frames.
- PROS build passed after adding `pros::Rotation horizontal_odom(5)`.
- Upload to COM15 succeeded.
- Updated dashboard server is live on http://127.0.0.1:8774/.
- Live `/data` reports COM14 at 50 Hz and includes `odometer: {"5": {"position_centideg": 0}}`.
- Served HTML contains the new Side Odom, LiDAR Assist, port 5 odometer, and 50 in LiDAR gate code.
- Served HTML contains the corrected LiDAR field visualization code with four wall hypotheses, 12 in arrows, perpendicular wall distance, and wall-parallel red lines.
- `/playing-field` serves `body class="field-only"`, includes the field canvas, hides non-field sections, and keeps live `/data` polling.
- `/playing-field` served HTML contains `lastGoodLidarEstimate` and hold-status text, confirming bad LiDAR readings preserve the prior good wall estimate in the UI.
- `/playing-field` served HTML contains `drawDrivetrainVerticalEstimate`, `drive Y`, and cyan overlay color markers. Live COM14 telemetry returned at 50 Hz after restart.
- `/playing-field` served HTML now contains onboard fused pose source handling, `/pose-grid`, and a field HUD. Live COM8 telemetry remained at 50 Hz after restart.

Failed attempts:

- PROS build failed because make.exe returned WinError 5 Access is denied. This blocks compiling and uploading the C++ joystick/motor telemetry changes in-session.
- Before the parser fix, m17=inf caused the optional motor group to be dropped.
- Before the UI lookup fix, the browser looked at latest.motors instead of latest.latest.motors.
- A first server restart command matched its own PowerShell command line and stopped early. The server was restarted cleanly afterward.
- Playwright browser verification could not launch because the local Playwright Chromium binary is not installed. Chrome was opened normally with Start-Process, and verification fell back to served HTML plus live `/data`.

Remaining risks:

- The odometry assumes one motor revolution equals one wheel revolution. The user confirmed this is correct.
- Port 17 telemetry is not finite. If port 18 is enough because the side is mechanically linked, the dot can still move; otherwise the port 17 motor/encoder wiring needs attention.
- Direction still needs manual drive verification after encoder sign normalization: drive forward and turn clockwise while watching the field dot.
- The latest audit sampled stationary encoders, so it did not prove physical dot movement on the field map.
- The latest live sample had h5=0 because the robot was stationary, so side-slide dot movement still needs physical verification.

Manual verification:

- Move the robot forward from bottom-left and confirm the dot moves upward.
- Turn the robot clockwise and confirm the field heading rotates clockwise.
- If forward movement is wrong but turning is right, flip both wheel distance signs together in the web UI.
- Slide the robot right while facing up and confirm Side Odom increases and the field dot moves right.
- Slide the robot left while facing up and confirm Side Odom decreases and the field dot moves left.
- Put a clean left-side wall under 50 in and confirm LiDAR Assist shows heading/wall context only; it should not jump the displayed x/y.
