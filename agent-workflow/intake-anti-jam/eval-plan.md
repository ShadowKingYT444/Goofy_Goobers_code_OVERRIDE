Commands:

- `make`
- `$env:APPDATA = 'C:\\Users\\terry\\Downloads\\MoreVex\\.pros-appdata'; .\\.venv\\Scripts\\pros.exe make`

Robot scenarios:

- Press A with no game object. Intake should run forward and not pulse.
- Press B. Intake should run reverse and not pulse unless physically stalled.
- Press UP with intake clear. Brain/controller should show calibration status, and thresholds should update after about two seconds.
- Hold A and physically stall one intake side. After about 250 ms, both intake motors should pulse reverse/forward every 50 ms for about one second.
- Briefly touch or feed a game object without stalling. Anti-jam should not trigger from a short spike.

Pass criteria:

- Build succeeds in an environment with a usable PROS make toolchain.
- Jam detection uses sustained high load plus low rpm.
- UP calibration writes durable threshold values in memory for the current run.
- Anti-jam sequence does not block drivetrain updates.
