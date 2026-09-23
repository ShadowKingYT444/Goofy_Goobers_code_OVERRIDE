Commands:

```powershell
.\.venv\Scripts\python.exe .\agent-workflow\pid-autotune\pid_autotune_source_check.py
.\.venv\Scripts\python.exe -m py_compile .\agent-workflow\pid-autotune\pid_autotune_source_check.py
$env:APPDATA=(Resolve-Path .\.pros-appdata).Path; .\.venv\Scripts\pros.exe make
```

Manual:

- Put the robot in open space with at least 24 in clearance.
- Start opcontrol.
- Press `X + Down`.
- Watch that drive trials alternate forward/back and turn trials alternate clockwise/counterclockwise.
- Read `PID_TUNE drive_best`, `PID_TUNE turn_best`, and `PID_TUNE apply` from serial output.
- Run the fused autonomous route after tuning and confirm it is smoother than the hand constants.

Pass criteria:

- Source check passes.
- Build passes when the PROS toolchain can execute.
- The routine stops the drivetrain after every candidate.
- The routine prints selected drive and turn constants.
- The routine applies the selected constants for the current run.
