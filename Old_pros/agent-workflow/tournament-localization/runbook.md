# Runbook

## Safety

No startup motion. Keep cumulative acceptance motion below 36 inches and 720 degrees; planned limits are 30 inches and 600 degrees. Stop on sensor fault, LiDAR loss during a LiDAR-calibration leg, unexpected translation, cable/tether risk, or timeout.

## Current stationary AI Vision diagnostic

```powershell
cd C:\Users\terry\Downloads\MoreVex\ai_vision_smoke
$env:APPDATA=(Resolve-Path ..\.pros-appdata).Path
$env:PROS_TOOLCHAIN=(Resolve-Path ..\.pros-toolchain\usr).Path
$env:CXX_STANDARD='gnu++20'
$env:C_STANDARD='gnu17'
..\.venv\Scripts\pros.exe make
..\.venv\Scripts\pros.exe upload . COM9 --slot 2 --after none
```

Stop the current program, run slot 2, and read COM8 at 115200. Restore slot 1 after diagnostics.
