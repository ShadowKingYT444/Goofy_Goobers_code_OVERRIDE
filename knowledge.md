# MoreVex Knowledge Base

Append durable technical facts here whenever new hardware behavior, calibration, API behavior, failure mode, or operating procedure is learned. Use ISO timestamps and distinguish measured facts from provisional assumptions.

## 2026-07-11T01:44:06.8027063-07:00 - Current system map

- Drivetrain is differential/tank and cannot strafe. It uses eight 2.75-inch omni wheels: four per side, driven by two gear-coupled motors per side.
- Left drive motors are ports 17 and 18; right drive motors are ports 11 and 13. Firmware reversal signs are hardware calibration and must not be removed casually.
- Chassis IMU is Smart Port 3.
- Rear horizontal tracking wheel is a 2-inch omni wheel on rotation port 5. It is mounted rear-center like a license plate and measures side-to-side motion; forward travel should produce nearly zero rotation.
- Four aligned distance sensors use ports 6, 7, 8, and 9 with 2-inch spacing. Port 9 must not also be assigned to a motor.
- VEX AI Vision is on Smart Port 20 and is configured for Circle 21H7 tags.
- The AI camera is mounted high on the side, points approximately 90 degrees robot-right, and is modeled at +6.75 inches robot-forward and 10.44 inches robot-left from the rotation center.
- Field/UI coordinates are center-origin inches: +X right, +Y up, and 0 degrees along +X.

## 2026-07-11T01:44:06.8027063-07:00 - Measured localization calibration

- Normalized drive encoder signs are left +1 and right +1.
- Effective drive track width is 12.0086 inches.
- Rear tracking-wheel raw sign is +1 and effective rear-center lever is 5.18 inches.
- LiDAR theta scale is 0.926770.
- AI effective tag outer size is provisionally 1.05 inches from live parallax/range trials; it is not a manufacturer-supplied distance field.
- Current entered pose after the slow 20-foot campaign is `(30.18, 34.70, 151.65 deg)`.

## 2026-07-11T01:44:06.8027063-07:00 - Fusion behavior

- Continuous propagation uses averaged trustworthy drive encoders, the rear tracking wheel, and IMU heading.
- LiDAR wall corrections require a valid four-sensor line fit, adequate confidence, an unambiguous wall, three consistent fits, heading innovation within 8 degrees, and axis innovation within 1.5 inches. Invalid or implausible readings contribute no correction.
- A coherent one-meter LiDAR range jump cannot teleport pose: it fails the axis-innovation gate before position correction. Accepted LiDAR corrections are also bounded.
- AprilTag IDs are duplicated across paired physical Goals. Candidate selection compares distinct Goals rather than treating adjacent faces of one Goal as different landmarks.
- AI position correction normally rejects innovations over 8 inches. Innovations from 8 to 24 inches require 12 consistent observations of the same Goal/face and still apply no more than 0.75 inch per update. Innovations over 24 inches are discarded.
- One tag's range+bearing point does not uniquely observe both robot position and heading. Camera heading gain is therefore zero until planar tag-pose solving is implemented. AI Vision corrects X/Y conditioned on IMU/LiDAR heading.
- Missing/stale sensors never force pose to zero and old tracking-wheel samples are not reused as fresh data.
- Stationary zeroed encoders must not reset an absolute LiDAR/AprilTag correction. An earlier reset heuristic caused this failure and was removed.
- Same-side coupled-motor spread above 15 degrees makes that side untrustworthy. The slow 20-foot route observed a maximum left/right spread of 9/0 degrees.

## 2026-07-11T01:44:06.8027063-07:00 - Live route evidence

- A supervised combined route covered about 120 inches. After a minimum-turn-power fix, its continuation returned with 1.54-inch raw position error and 4.93-degree heading error.
- The final slow route covered 240 inches (20 feet) plus 60 degrees of bounded rotation at maximum drive power 35. It completed without abort.
- The 20-foot route's raw return error was 6.42 inches and 4.32 degrees. Tag 3 then localized the stationary endpoint near `(30.18, 34.70, 151.65 deg)` with about 0.09-inch innovation and 0.07-inch range residual.
- A 2-4 degree turn-controller dead zone existed because settle tolerance was below the minimum-power threshold. Making the minimum-power threshold equal the settle tolerance fixed it.
- `agent-workflow/multisensor-localization-completion/analyze_fusion_log.py` streams logs in constant memory and records route accuracy, physical discontinuities, correction bounds, rejection reasons, and encoder spread.

## 2026-07-11T01:44:06.8027063-07:00 - AprilTags and camera APIs

- Center Goal tag ID is 0. Other field tag IDs are duplicated across paired props; use the centralized landmark table in `include/localization_config.hpp`.
- The PROS AI Vision API supplies tag ID and four image corners. It does not provide a built-in physical distance-to-tag field.
- Range is derived from calibrated tag size and corner geometry. A low observed/predicted residual proves internal agreement, not independent absolute accuracy.
- The four corners could support future planar pose/homography solving, which is the preferred path to legitimate AprilTag heading correction.
- BRIO 101 is the fixed external webcam; “robot cam” means the onboard AI Vision sensor. Raw frames are available only from BRIO.
- The BRIO publisher is FFmpeg feeding MediaMTX at `rtsp://127.0.0.1:8554/cam`. A broken PATH shim may fail; use the actual WinGet FFmpeg executable from the running publisher process.

## 2026-07-11T01:44:06.8027063-07:00 - Wireless V5 behavior

- Laptop-to-controller USB plus controller-to-Brain radio supports PROS wireless status, upload, and remote run. The controller currently appears as COM10.
- Wireless upload was proven: the Brain entered download channel, received the hot image, and returned to pit channel.
- Stop `lidar_bar_server.py` before upload because it may automatically claim COM10.
- Passing explicit `COM10` to `pros terminal` treats it as a direct serial port and skips the wireless FIFO wrapper. `pros terminal v5` selects `V5WirelessPort`, but the current controller channel transfer still hangs under the high-rate telemetry stream.
- Wireless Brain framebuffer capture works but is very slow: 557,056 bytes took about 161 seconds.
- Brain LCD line 4 now permanently reports `AI P<port> tag=<id> VALID` or the current rejection reason, allowing tag verification without the host UI.
- Direct Brain USB remains the reliable method for the existing high-rate D4/localization web UI until controller-aware wireless FIFO support is added.

## 2026-07-11T01:44:06.8027063-07:00 - Safe operating commands

```powershell
# Tests and build
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe -m pros.cli.main make

# Detect V5 devices
.\.venv\Scripts\python.exe -m pros.cli.main lsusb --target v5

# Wireless status/upload/run through the controller
.\.venv\Scripts\python.exe -m pros.cli.main v5 status COM10
.\.venv\Scripts\python.exe -m pros.cli.main upload --slot 1 --name MoreVex --description MoreVex --after none . COM10
.\.venv\Scripts\python.exe -m pros.cli.main v5 run 1 COM10

# Direct-USB localization UI
.\start_localization_ui.cmd
```

- The direct `.venv\Scripts\pros.exe` launcher contains a stale interpreter path. Use `.venv\Scripts\python.exe -m pros.cli.main`.
- Before any normal upload, verify every `RUN_STARTUP_*` diagnostic-motion flag in `src/main.cpp` is false.
