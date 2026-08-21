2026-07-04: Created workflow artifacts and captured build environment blockers.
2026-07-04: Implemented intake anti-jam state machine in `src/main.cpp`.
2026-07-04: Replaced stale `intake_front`/`intake_back` references with `upper_intake`/`counter_rollers` and stale `claw_motor` with `claw_arm`.
2026-07-04: Re-ran PROS build command; still blocked by `[WinError 5] Access is denied` launching VS Code globalStorage `make.exe`.
2026-07-04: Adjusted calibration to use mean normal-cup current/torque with 1.5x thresholds instead of peak-inflated thresholds.
