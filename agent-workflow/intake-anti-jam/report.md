Final evidence report, 2026-07-04.

Changed files:

- `src/main.cpp`
- `agent-workflow/intake-anti-jam/goal.md`
- `agent-workflow/intake-anti-jam/assumptions.md`
- `agent-workflow/intake-anti-jam/baseline.md`
- `agent-workflow/intake-anti-jam/design.md`
- `agent-workflow/intake-anti-jam/failure-cases.md`
- `agent-workflow/intake-anti-jam/eval-plan.md`
- `agent-workflow/intake-anti-jam/progress.md`
- `agent-workflow/intake-anti-jam/runbook.md`
- `agent-workflow/intake-anti-jam/acceptance.json`
- `agent-workflow/intake-anti-jam/report.md`

Implementation summary:

- Added `IntakeStats` and `IntakeAntiJam` state in `src/main.cpp`.
- Added UP-arrow calibration. It spins the intake forward for about two seconds, samples current, torque, and rpm from both intake motors, and sets current/torque thresholds from mean normal-cup load.
- Revised calibration after driver feedback so thresholds are based on the mean normal-cup load, not peak load. Current and torque thresholds are now 1.5x the measured mean.
- Added sustained jam detection: intake must be commanded, high current or high torque must be present, and the slower intake motor must be below the rpm threshold for at least 250 ms.
- Added a nonblocking anti-jam sequence: 20 phases at 50 ms each, alternating reverse and forward, for about one second total.
- Applied the override to both `upper_intake` and `counter_rollers`.
- Replaced stale `intake_front`/`intake_back` references with the actual subsystem motors.
- Replaced stale `claw_motor` with `claw_arm`.

Commands run:

- `make`
- `.\\.venv\\Scripts\\pros.exe make`
- `$env:APPDATA = 'C:\\Users\\terry\\Downloads\\MoreVex\\.pros-appdata'; .\\.venv\\Scripts\\pros.exe make`
- static searches with `rg`
- direct file inspection with `Get-Content`

Failed attempts:

- `make` was not on PATH.
- PROS CLI initially failed trying to write to `C:\\Users\\terry\\AppData\\Roaming\\PROS\\cli.pros`.
- With `APPDATA` redirected into the workspace, PROS CLI still could not launch `C:\\Users\\terry\\AppData\\Roaming\\Code\\User\\globalStorage\\sigbots.pros\\install\\pros-toolchain-windows\\usr\\bin\\make.exe` due to `[WinError 5] Access is denied`.
- `git diff` and `git status` reported this directory was not a normal git repository, despite a `.git` directory being present.

Verification status:

- Build verification was attempted but blocked by local toolchain access.
- Robot behavior was not manually verified in this session.
- Static inspection confirmed the new anti-jam path is called from `opcontrol()` and the stale intake/claw identifiers were removed from active source.

Remaining risks:

- The threshold multipliers are starting points and need real-robot tuning.
- Calibration should be run with the intake clear; calibrating while already jammed will produce bad thresholds.
- The motor direction assumptions match the existing single-power intake control. If the upper intake and counter roller mechanically need opposite signs, `move_intake()` should be adjusted.

Manual verification steps:

1. Build from an environment where the PROS toolchain `make.exe` can run.
2. Upload to the brain.
3. Press UP with the intake clear and let calibration finish.
4. Hold A and confirm normal intaking does not pulse.
5. Stall either intake motor and confirm both motors alternate reverse/forward every 50 ms for about one second.
6. Feed a normal game object and confirm a brief current/torque spike does not trigger anti-jam.
