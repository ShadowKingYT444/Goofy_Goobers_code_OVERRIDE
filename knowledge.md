# MoreVex Knowledge Base

Append durable technical facts here whenever new hardware behavior, calibration, API behavior, failure mode, or operating procedure is learned. Use ISO timestamps and distinguish measured facts from provisional assumptions.

## 2026-09-04T00:00:00-07:00 - Current replacement-robot wiring correction

- Current source and operator wiring report agree on P17/P18 left drive,
  P11/P13 right drive, P12 IMU, P6 AI Vision, and P7 GPS.
- Operator-confirmed P1 points along the rear/negative robot axis. It is not a
  forward collision or Toggle-range sensor.
- P5 is claw-arm Rotation feedback. The working longitudinal/forward odometry
  Rotation sensor is P15 and is mounted at the robot rotation center. Previous
  P5 odometry evidence measured the wrong mechanism and does not describe P15.
- Toggle actuation is active-low ADI-D and must stay physically extended after
  autonomous starts. Firmware now establishes that state on entry to both
  autonomous and driver control.
- The upper intake has been removed and P14 is physically empty. Production no
  longer declares or commands a P14 intake motor.

## 2026-08-25T20:46:32-07:00 - Live power-cycle recovery and P6 calibration variability

- A real Brain power cycle restored CPU1/user telemetry and proved the former
  runtime-silence diagnosis obsolete. Slot 4 streamed with all startup motion
  disabled, then the complete VEX USB device disappeared at 20:56:52 during a
  long stationary soak. The Brio also does not show the robot. Reconnect and
  visual coverage are both required before motion.
- P6 `installed` + non-calibrating + non-error/finite output does not prove a
  usable calibration. One ready calibration drifted 4.25 to 15.10 degrees over
  ten stationary minutes at 1.074 degrees/min while all four drive encoders
  stayed fixed and P7 heading spanned 0.17 degrees. A warm stationary
  recalibration then held a 0.02-degree span over 11.89 min. Treat earlier P6
  precision claims as conditional on a good calibration.
- A candidate stopped cross-sensor gate detects the recorded drift by comparing
  relative P6/P7 heading only while both encoder sides and P7 XY/heading remain
  stable. It must be estimator-session scoped and reset on every deliberate
  tare/recalibration; old diagnostics prove otherwise it can misclassify an
  intentional zero as a fault. It is not deployed pending live turn testing.
- Current P8 returned corners scale to the 0.625-in inner five-cell detection
  square, not the medium print's 0.875-in outer black square. The corrected
  separate 18-in tape view solves to 18.60 in; angular estimates are unchanged.
  Absolute field correction remains disabled.
- Current P1 view produced a stable approximately 29.3-in return at roughly
  39-46/63 confidence, whereas a prior roughly 40-in scene returned only 13.5%
  of frames at 6/63 median confidence. Availability and quality are strongly
  scene/target dependent; P1 remains a forward stop, not localization truth.

## 2026-08-23T06:25:00-07:00 - Prior replacement map (superseded 2026-09-04)

- Current drivetrain is P17/P18 left and P11/P13 right; P6 is its IMU, P7 GPS,
  P1 forward Distance, P8 AI Vision, and P5 the inactive lateral tracker. The
  July P20 camera/P6-P9 LiDAR layout is historical and must not be used.
- P7 lens is measured 6 in right and 6 in behind the rotation center, facing
  robot-right. It is position-only, stopped-only bounded correction. Live
  rotation produced a false 15.63-in displacement and corrected-frame
  innovations up to 27.85 in despite excellent stationary precision.
- The nine-trial encoder scale `0.8847477281` and paired effective wheel/track
  values 2.433055/10.624582 in are provisional because P7, not tape/laser
  truth, was the translation reference.
- P6 stationary standard deviation was 0.00463 degrees over 120 s. A
  0.91-degree commanded return residual is not externally measured accuracy;
  the production outage model uses a provisional 2.0-degree heading/controller
  allowance. Together with 2.1554% P7-referenced scale variation, its minimum
  envelope grows by 4.1037% of dead-reckoned travel, excluding systematic
  scale-reference bias, slip/push, collision, braking, and excess placement
  error.
- P8 uses official medium Circle21h7 prints with a 0.875-in outer black square;
  returned corners were later proved to span the 0.625-in inner detected square
  (see the 2026-08-25 superseding note above).
  Field correction is disabled: mount translation/attitude and Goal-face
  transforms are unmeasured, and the onboard edge/focal range approximation
  has deterministic obliquity bias (+7.7% at 30-degree tag yaw, +20.7% at 45).
- P1 stops forward public motion on detected returns at or inside 8 in and on
  device/API/malformed-range faults. A real obstacle returned as the documented
  9999-mm/no-target value remains indistinguishable from clear space; P1 is not
  certified collision perception.
- P5 enumerated but moved only 0-0.002 in through live motion, so lateral-slip
  correction is disabled. Hidden slip or pushing is unobservable.
- Public map safety includes walls and nine Goals with Override T5's one-inch
  tolerance plus the reported pose envelope. It omits official Pins, Loaders,
  Toggles, movable Blocks, other robots, and the unmeasured swept footprint;
  arbitrary navigation is therefore not competition-safe.
- The latest no-startup-motion image passes 50 host tests and builds at SHA-256
  `192c4adc4f16805a4a3fb8c26d82867ed26b546c1070fafc21e8fefd143554a1`
  and is uploaded to slot 1. The system channel is healthy, but the user
  runtime/channel emits no telemetry after run; the program is stopped. Do not
  navigate until that failure is repaired and live boot inventory is verified.

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
- Effective drive track width is 10.624582 inches at the calibrated
  2.433055-inch encoder wheel scale. This preserves the same turn ratio as the
  earlier 12.0086-inch track expressed at the physical 2.75-inch wheel scale;
  it is not a claim about tape-measured chassis width.
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
