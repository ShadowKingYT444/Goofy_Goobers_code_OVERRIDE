Summary:

Before this pass, the fixes in `NEXT_CHANGES.md` were directionally correct but not implemented in `src/autons.cpp`. This pass moved the actual autonomous estimator to the intended fusion shape for the sensors currently present: IMU heading plus bounded wall-theta bias correction, horizontal odom for sideways movement, drivetrain encoders as forward fallback, and strict LiDAR wall gating.

Findings:

- Current code has the right sensor plumbing: distance ports 6-9, horizontal odometer port 5, drivetrain motor telemetry, and IMU port 4 through the chassis object.
- Before this implementation pass, `src/autons.cpp` did not use the IMU as the heading source.
- Before this implementation pass, `src/autons.cpp` did not keep an `imu_bias` term and did not use wall theta as a low-rate bias correction.
- Before this implementation pass, `src/autons.cpp` used LiDAR distance projection to overwrite one field axis. This could help if wall identity was correct, but could also snap to the wrong wall.
- Before this implementation pass, `src/autons.cpp` had no winner-margin check for wall identity. Ambiguous clean scans were accepted.
- Before this implementation pass, the onboard LiDAR fit ignored confidence even though the telemetry path reads it.
- The known start pose should be used more explicitly. It lets the estimator initialize real x/y/theta and reject wall hypotheses that are impossible from that start and route history.
- A vertical non-driven odom wheel remains the biggest missing piece for reliable forward displacement. Horizontal odom only solves side slip.

Implemented changes:

- `src/autons.cpp` now resets IMU rotation at autonomous start and maps clockwise-positive IMU rotation into the field heading convention.
- `src/autons.cpp` now uses IMU heading as the primary heading source; drivetrain encoder heading is fallback only if IMU reading is unavailable.
- `src/autons.cpp` now keeps `imu_bias_deg` and updates it from wall theta every 2.5 seconds with gain, step clamp, total clamp, and max heading-error gate.
- `src/autons.cpp` now gates LiDAR correction by confidence, RMSE, max point error, max distance, angular rate, wall winner margin, and heading plausibility.
- `src/autons.cpp` now applies only bounded one-axis wall-distance correction instead of direct full-axis snapping.
- `src/main.cpp::initialize()` now calibrates the IMU before resetting the horizontal odometer.

Verdict:

The old `autons.cpp` was not doing fusion correctly. The current edited version now matches the intended architecture for the sensors that are physically present: IMU heading plus bias, horizontal odom for sideways movement, drivetrain encoders as forward fallback, and LiDAR wall theta/distance as gated correction. It still cannot be called fully reliable for long scoring macros until forward displacement is handled by a vertical non-driven odom wheel or field tests prove drivetrain-forward error is tolerable.

Commands run:

- `.\.venv\Scripts\python.exe -m py_compile .\tools\lidar_bar_server.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\odometry_sign_check.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\field-odometry-ui\telemetry_parser_check.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\localization_failure_checks.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py`
- `.\.venv\Scripts\pros.exe make`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe make`
- Workspace-local `make.exe` search
- Downloaded `https://github.com/purduesigbots/toolchain/releases/download/13.3.1/pros-toolchain-windows-formatted.zip`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; $env:PROS_TOOLCHAIN=(Resolve-Path .\.pros-toolchain\usr).Path; .\.venv\Scripts\pros.exe make clean; .\.venv\Scripts\pros.exe make`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe upload . COM9 --slot 1 --after run --name MoreVex`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe v5 status COM9`

Results:

- Python syntax passed.
- Existing odometry sign regression passed.
- Existing D4 telemetry parser regression passed.
- Adversarial localization checks passed. They lock in the core conclusions: current wall choice accepts ambiguous ties, known start pose helps wall identity, wall theta can correct IMU bias but the current drivetrain-derived heading remains wrong, and forward slip needs a vertical odom wheel.
- `autons_fusion_source_check.py` passed after the implementation pass.
- PROS build did not complete because the Windows toolchain could not execute `make.exe`.
- No workspace-local `make.exe` or compiler alternative was found.
- After downloading the public `purduesigbots/toolchain` 13.3.1 archive into `.pros-toolchain`, clean PROS build passed and produced fresh hot/cold binaries.
- The fresh program uploaded to V5 slot 1 on COM9 as `MoreVex`, and the brain responded to `pros v5 status`.

Failed attempts:

- First PROS build attempt failed before compilation because the CLI tried to write `C:\Users\terry\AppData\Roaming\PROS\cli.pros` and hit PermissionError 13.
- Retrying with repo-local APPDATA reached the PROS toolchain, but `C:\Users\terry\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin\make.exe` failed with WinError 5 Access is denied.
- A second repo-local APPDATA retry after the implementation produced the same `make.exe` WinError 5 toolchain failure.
- A final retry after adding the PID autotune source still failed before compilation at the same global-storage `make.exe` WinError 5 failure.
- The successful workaround was not to use the global-storage toolchain; it used a workspace-local `PROS_TOOLCHAIN` path.

Remaining risks:

- Physical side odometer sign and scale still need live movement verification.
- Wall theta sign must be calibrated physically.
- LiDAR sensor offset from robot center is still modeled as zero in the current estimator; if the bar is offset from the center, the one-axis wall-distance correction needs that mount offset.
- No ground-truth field run was performed in this review.

Manual verification:

- Follow `eval-plan.md` and compare estimated x/y/heading against tape-measured ground truth before using this for scoring macros.
