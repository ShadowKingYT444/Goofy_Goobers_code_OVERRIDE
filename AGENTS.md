# AGENTS.md

## Scope

This file applies to the repository root and all descendants. This is a VEX V5 C++ robot project built with PROS and EZ-Template, plus host-side localization telemetry and a browser field UI.

## Repository map

- `src/`: production robot firmware and subsystem/localization implementations.
- `include/`: project headers.
  - `localization_config.hpp`: localization geometry, calibration, thresholds, transforms, and related constants. Keep localization magic numbers here rather than scattering them through tasks.
  - `subsystems.hpp`: shared hardware declarations. The chassis, distance sensors, and horizontal odometer are declared `extern`; define each exactly once in a `.cpp`. Several mechanism devices are C++17 `inline` variables; do not add duplicate definitions.
  - `autons.hpp`: autonomous declarations and shared autonomous interfaces.
  - `pid_autotune.hpp`: PID autotuning helpers/configuration.
  - `main.h`: application-level PROS entry header.
  - `api.h`, `pros/`, `liblvgl/`, `EZ-Template/`, `okapi/`: framework or vendored headers. Avoid modifying vendored code unless the task specifically requires it.
- `project.pros`, `Makefile`, `common.mk`: PROS/build configuration.
- `tools/`: host-side localization, telemetry, server, and diagnostic utilities.
- `start_localization_ui.cmd`: launches the localization UI/tooling on Windows.
- `distance_working/`, `distance_smoke/`, `ai_vision_smoke/`: focused hardware experiments and smoke tests. Use them as references and for targeted validation; do not assume they are production code.
- `FIELD_2d_VIEW.jpg`, `screenshots/`, `brain_capture*.png`: visual field/UI references and captures.
- `*.log`, `*.err.log`, `terminal.log`, `pros_terminal_capture.txt`: diagnostic output, not source files.
- `.venv/`, `.pros-toolchain*`, `.pros-appdata/`, `bin/`, `firmware/`, `compile_commands.json`: generated environment/tooling artifacts. Do not hand-edit them unless the task is explicitly about tooling generation.
- `agent-workflow/`, `.agents/`, `.codex/`: local agent/tool workflow material; inspect before changing automation behavior.

## Hardware map shown in the current workspace

| Device | Configuration |
|---|---|
| Drive left motors | Smart Ports `17` and `18`, both reversed (`-17`, `-18`) |
| Drive right motors | Smart Ports `11` and `13` |
| Chassis IMU | Chassis constructor argument `3`; verify in the vendored EZ-Template constructor, but it is expected to be Smart Port `3` |
| Drive setup | `2.75 in` wheels, `450 RPM`, ratio `1.0`, as passed to `Drive chassis(...)` |
| Distance/ToF sensor bar | Smart Ports `6`, `7`, `8`, `9` |
| Horizontal tracking odometer | Rotation sensor on Smart Port `5` |
| Slider left motor | Smart Port `9`, reversed (`-9`) |
| Slider right motor | Smart Port `2` |
| Slider rotation sensor | Smart Port `16` |
| Clamp piston | ADI port `A` |
| Claw arm motor | Smart Port `4` |
| Upper intake motor | Smart Port `14` |
| Counter-rollers motor | Smart Port `15` |
| VEX AI Vision sensor | Port not yet verified; locate it in production source and/or `ai_vision_smoke/`. Do not invent a port. |

**Known hardware-map conflict:** `distance_9` and `slider_left(-9)` both claim Smart Port `9`. A negative motor port means reversed direction, not a different physical port. These devices cannot both be active on the same V5 Smart Port. Verify the current robot wiring and current branch before touching either subsystem; do not silently choose one mapping.

Motor reversal signs are intentional calibration. Do not remove a leading `-` merely to make port numbers look uniform.

## Localization contract

The estimator is a multi-sensor fusion system. Preserve the distinction between continuous motion propagation and absolute/periodic correction.

### Continuous propagation

- Drivetrain motor encoders estimate forward/longitudinal displacement.
- The horizontal tracking wheel on rotation port `5` estimates lateral displacement.
- The IMU supplies high-rate short-term heading. It is useful for continuity but drifts and must not be treated as permanently absolute.

### Correction sources

- The four aligned distance/ToF sensors on ports `6`–`9` form a rigid sensor bar. Existing code/calibration contains the spacing between sensors. Use that calibrated geometry and the existing sensor ordering to infer wall-relative angle and wall offset from the range differences and inverse-trigonometric calculation.
- Distance-based heading is a frequent correction for IMU drift, not the primary dead-reckoning source. Apply it opportunistically at a high rate when geometry and readings are valid. About `0.1 s` may be a useful nominal cadence, but do not introduce a hard-coded timing value without checking the current task loop and sensor update rate.
- VEX AI Vision/AprilTag observations provide landmark-based/global corrections when a known goal tag is visible. The camera-to-robot extrinsic transform, field tag pose, ambiguity handling, and measurement confidence must be explicit.

### Fusion invariants

- Never average all sensors blindly. Reject stale, out-of-range, low-confidence, geometrically impossible, or discontinuous measurements before fusion.
- A missing sensor contributes no correction; it must not force pose to zero or reuse an old value as if it were fresh.
- Keep timestamps/freshness with measurements where practical. Avoid blocking the high-rate odometry/control loop on UI, logging, networking, or camera work.
- Normalize heading consistently and handle wraparound explicitly. Do not average angles across the `0/360` boundary as ordinary scalars.
- Keep units explicit. The UI reports position/range in inches and heading in degrees; internal math may use radians, but conversions should occur at clear boundaries.
- The field UI shows `+X` to the right, `+Y` upward, and `0 deg` on the `+X` axis. Verify the rotation sign and all robot/camera/field frame transforms in code before changing them.
- Corrections should be gated and bounded so one bad wall or tag observation cannot teleport the robot pose.
- Keep calibrated sensor baselines, offsets, camera extrinsics, and field landmark poses in one configuration source, preferably `include/localization_config.hpp` or an existing dedicated config structure.

## AprilTags and field UI

The numbered markers drawn at the goals are the AprilTag identifiers used by the vision localization logic. Keep the tag-to-goal/field-pose mapping in one table and use the same mapping in robot code and the browser UI.

The supplied UI image appears to show labels `1`–`4` on both field halves. Verify whether the alliance/side is part of the identifier or whether the labels are only abbreviated before assuming every displayed number is globally unique.

The browser page is a diagnostic instrument for validating localization. Preserve visibility of pose, heading, drive-sensor health, camera confidence, IMU state, and data freshness. The UI can display **browser fallback odometry** from drive motors plus the side odometer when no fresh onboard fused pose is available; that fallback is diagnostic, not ground truth and not equivalent to the full fused estimator.

When changing telemetry fields or coordinate conventions, update both firmware and host/UI consumers together, or retain backward compatibility.

## Change discipline

- Make the smallest coherent change that satisfies the task.
- Do not edit vendored libraries to work around an application bug.
- Do not create a second hardware object for a port that already has one. Reuse the shared declaration/definition pattern.
- Do not duplicate calibration constants in test code and production code; share or clearly document intentional test overrides.
- Preserve real-time behavior: no long sleeps, filesystem/network waits, or verbose printing in control/localization hot paths.
- Add concise diagnostics for rejected measurements and stale sources, but rate-limit repetitive logs.
- Update `NEXT_CHANGES.md` or the localization handoff when a change alters hardware mapping, calibration, telemetry schema, coordinate frames, or estimator behavior.

## Build and validation

- Build from the repository root with `pros make`; use `make` only when that is the established local path. Report the exact command and error output if the build fails.
- Run the narrowest relevant smoke test or host tool after a successful build.
- For localization changes, validate at minimum: stationary drift, straight forward/backward travel, pure lateral tracking-wheel movement, in-place rotation, wall-angle correction, sensor dropout/staleness, and an AprilTag correction with a known field pose.
- Compare onboard fused pose against the browser UI and logs. A visually plausible path is not enough; check sign, scale, heading wrap, latency, and freshness.
- Do not flash the robot, actuate motors/pneumatics, or change physical calibration without explicit authorization and a safe test setup.
