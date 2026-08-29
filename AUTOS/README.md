# AUTOS snapshot

This folder preserves the autonomous work completed through 2026-08-28.

## Current autonomous

The current slot-4 program is `FullRedAuton`.

- Competition `autonomous()` calls `localization_simple_red_goal_hotkey_auton()`.
- Driver-control **Up+A** calls the same full routine.
- The robot starts with its front intake facing the Toggle.
- Sequence: ram the Toggle, reverse six inches slowly, curve to the Goal six inches behind and eight inches right of the starting front-intake frame, finish the left turn without reversing/oscillating, then outtake at 80/127 while driving forward three inches.

## Contents

- `main.cpp`: top-level copy of the controller hotkey and competition entry point.
- `autons.cpp`: top-level copy of all autonomous implementations.
- `autons.hpp`: top-level copy of the autonomous declarations.
- `source/src/main.cpp`: controller hotkey and competition entry point.
- `source/src/autons.cpp`: autonomous implementation and tuning routines.
- `source/include/autons.hpp`: autonomous declarations.
- `bin/FullRedAuton_slot4.bin`: compiled V5 binary uploaded to slot 4.
- `reports/path1_tuning_2026-08-28/`: camera and telemetry evidence from Path 1 testing.
- `journal.md`: timestamped hardware, localization, and autonomous notes.

The files under `source/` are a snapshot. The live build continues to use the repository's normal `src/` and `include/` paths.
