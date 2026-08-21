2026-07-09: Researched PID tuning approaches and chose a conservative candidate-scoring autotuner instead of an aggressive ultimate-gain oscillation routine for the first robot-safe implementation.

2026-07-09: Added `include/pid_autotune.hpp`, `src/pid_autotune.cpp`, and `X + Down` launch path in `src/main.cpp`.

2026-07-09: Added explicit `<vector>` include to the autotune and autonomous localization sources after compile-oriented inspection.

2026-07-09: Ran PID autotune source check, localization fusion source check, Python compile checks, and acceptance JSON validation; all passed. PROS build remained blocked by the global-storage `make.exe` WinError 5 issue.

2026-07-08: Downloaded the public `purduesigbots/toolchain` 13.3.1 Windows formatted archive into the workspace, extracted it to `.pros-toolchain`, set `PROS_TOOLCHAIN=.pros-toolchain/usr`, and clean-built the project successfully.

2026-07-08: Uploaded the freshly built `bin/hot.package.bin` to the connected V5 brain on COM9, slot 1, as `MoreVex` with `--after run`; `pros v5 status COM9` confirmed the brain still responded.

2026-07-08: After the first physical PID run vibrated/oscillated badly, reduced autotune gain ranges and max power, added command ramping, and added aborts for overshoot, wrong-way movement, and repeated command sign changes.

2026-07-08: Rebuilt and uploaded the safer autotuner to V5 slot 1 on COM9. Fresh `bin/hot.package.bin` timestamp was 2026-07-08 20:49:01.

2026-07-08: User reported the safer custom-loop tuner was now too slow and barely moved. Replaced the custom command-loop tuner with EZ-Template movement trials using `pid_drive_set`, `pid_turn_set`, and `pid_wait`, restored normal autonomous speeds, and tuned candidates around the existing `default_constants()` scale.

2026-07-08: Rebuilt and uploaded the EZ-based autotuner to V5 slot 1 on COM9. Fresh `bin/hot.package.bin` timestamp was 2026-07-08 20:54:56, and `pros v5 status COM9` passed.

2026-07-08: User reported the EZ-based tuner drove forward continuously and turn behavior was wrong. Removed blocking `pid_wait()` from tuner trials and replaced it with watchdog loops, wrong-way detection, `pid_targets_reset()`, and `drive_mode_set(DISABLE)` force-cancel after every trial.

2026-07-08: Rebuilt and uploaded the watchdog EZ-command tuner to V5 slot 1 on COM9. Fresh `bin/hot.package.bin` timestamp was 2026-07-08 21:03:20, and `pros v5 status COM9` passed.
