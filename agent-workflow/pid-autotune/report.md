Summary:

Added an autonomous PID tuner for drive and turn constants. The current version uses EZ-Template's own motion APIs for each trial instead of a custom motor command loop, so the tested constants are on the same scale as the normal autonomous PID constants.

Changed files:

- `include/pid_autotune.hpp`
- `src/pid_autotune.cpp`
- `src/main.cpp`
- `agent-workflow/pid-autotune/*`

Research basis:

- Ziegler-Nichols and relay autotune are standard ways to infer ultimate gain/period from oscillation, but they can be aggressive.
- Practical PID guidance favors anti-windup, derivative damping, and avoiding derivative kick/noise problems. Reference used: https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralDerivative
- For a VEX robot in a small practice area, bounded candidate scoring is safer than intentionally driving to sustained oscillation.

Implementation:

- Drive autotune uses alternating 18 in forward/back EZ `pid_drive_set` trials at speed 85.
- Turn autotune uses alternating 45 deg EZ `pid_turn_set` trials at speed 70.
- Trials do not use blocking `pid_wait()` anymore. Each trial has a watchdog timeout, wrong-way detection, and force-cancel through `pid_targets_reset()` plus `drive_mode_set(DISABLE)`.
- The candidate constants are centered around the existing `default_constants()` scale instead of the tiny custom-loop values.
- The routine applies selected drive, turn, and derived heading constants for the current run, and `default_constants()` re-applies tuned values during the same program run if they are ready.
- Controller launch is `X + Down`.

Commands run:

- `.\.venv\Scripts\python.exe .\agent-workflow\pid-autotune\pid_autotune_source_check.py`
- `.\.venv\Scripts\python.exe .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py`
- `.\.venv\Scripts\python.exe -m py_compile .\agent-workflow\pid-autotune\pid_autotune_source_check.py .\agent-workflow\localization-architecture-review\autons_fusion_source_check.py .\agent-workflow\localization-architecture-review\localization_failure_checks.py`
- `.\.venv\Scripts\python.exe -m json.tool .\agent-workflow\pid-autotune\acceptance.json`
- `.\.venv\Scripts\python.exe -m json.tool .\agent-workflow\localization-architecture-review\acceptance.json`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe make`
- Downloaded `https://github.com/purduesigbots/toolchain/releases/download/13.3.1/pros-toolchain-windows-formatted.zip`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; $env:PROS_TOOLCHAIN=(Resolve-Path .\.pros-toolchain\usr).Path; .\.venv\Scripts\pros.exe make clean; .\.venv\Scripts\pros.exe make`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe upload . COM9 --slot 1 --after run --name MoreVex`
- `$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe v5 status COM9`
- After physical oscillation report: reran `pid_autotune_source_check.py`, rebuilt with workspace-local `PROS_TOOLCHAIN`, uploaded to COM9 slot 1, and confirmed V5 status.
- After slow/weak movement report: replaced custom command-loop tuner with EZ movement trials, reran source checks and fusion source check, rebuilt with workspace-local `PROS_TOOLCHAIN`, uploaded to COM9 slot 1, and confirmed V5 status.
- After runaway forward report: removed blocking `pid_wait()`, reran source checks, rebuilt with workspace-local `PROS_TOOLCHAIN`, uploaded to COM9 slot 1, and confirmed V5 status.

Results:

- PID autotune source check passed.
- Localization fusion source check passed.
- Python workflow scripts compiled.
- PID and localization acceptance JSON files are valid.
- PROS build did not reach compilation because the bundled VS Code PROS toolchain `make.exe` failed with WinError 5 Access is denied.
- After installing the workspace-local toolchain, clean PROS build passed and produced fresh `bin/hot.package.bin` and `bin/cold.package.bin`.
- Fresh binary uploaded to the V5 brain on COM9, slot 1, as `MoreVex`; the brain responded to `pros v5 status`.
- After the oscillation report, safer autotuner build passed and uploaded. Fresh `bin/hot.package.bin` timestamp was 2026-07-08 20:49:01.
- After the slow/weak movement report, the EZ-based autotuner build passed and uploaded. Fresh `bin/hot.package.bin` timestamp was 2026-07-08 20:54:56.
- After the runaway forward report, the watchdog EZ-command tuner build passed and uploaded. Fresh `bin/hot.package.bin` timestamp was 2026-07-08 21:03:20.

Failed attempts:

- A subagent was requested for the PID tuning track, but the spawned agent timed out and closed without output, so the autotuner was implemented in this pass.
- The current build attempt still fails at `C:\Users\terry\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin\make.exe` with WinError 5.
- The global-storage toolchain path remains inaccessible from this environment; the successful build used the workspace-local `.pros-toolchain/usr` path instead.
- The first physical run exposed unsafe oscillation in the custom autotune loop; the derivative values were too large for the 20 ms command loop and could flip the command aggressively.
- The second physical report exposed that the safer custom-loop gains were too small and not representative of EZ-Template PID constants. The custom command loop was removed.
- The third physical report exposed that blocking `pid_wait()` is unsafe when EZ's internal sensor sign/direction does not match the movement. Blocking waits were removed from the tuner.

Remaining risks:

- Constants are not persisted across restarts.
- This is not proof of best possible values; it selects the best candidate from a bounded set.
- Physical tests are still required because carpet, battery, robot load, and wheel traction affect results.
- The current safer candidate set may be conservative; if all candidates are sluggish but stable, widen upward gradually.
- Tuning values still only persist in RAM. To keep a chosen result after reboot, copy the printed `PID_TUNE apply` values into `default_constants()`.

Manual verification:

- Clear 24 in around the robot, press `X + Down`, watch trials, then read the `PID_TUNE` serial lines.
