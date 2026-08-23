# Runbook

## Tournament startup

1. Place the robot at the measured start and keep it still while the IMU calibrates.
2. On the V5 controller, hold `X+Y` to enter pose-edit mode. While holding `X+Y`, D-pad left/right changes X by 0.5 inch, down/up changes Y by 0.5 inch, L1/R1 changes heading by 1 degree, and L2/R2 changes heading by 15 degrees.
3. Press `A` while holding `X+Y` to save and re-anchor the encoder/IMU state. Release the buttons; the controller rumbles after a successful save. Y alone commands intake outtake.

Coordinates use field center `(0,0)` in inches, +X right, +Y up, and 0 degrees along +X.

## Localization UI

```powershell
.\start_localization_ui.cmd
```

Open `http://127.0.0.1:8774/localization`. The laptop webcam is debug-only and never contributes to tournament pose. Set `MOREVEX_DEBUG_WEBCAM=1` only for external ground-truth work.

## Test and build

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m pros.cli.main make
```

The direct `.venv\Scripts\pros.exe` launcher currently contains a stale interpreter path; use the module command above.

Before uploading normal firmware, verify every `RUN_STARTUP_*` diagnostic-motion flag in `src/main.cpp`, including `RUN_STARTUP_LONG_FUSION_ROUTE`, is false. Upload slot 1, explicitly stop/start the program, and confirm zero startup motion before enabling driver control.
