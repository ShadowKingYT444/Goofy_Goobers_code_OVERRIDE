Baseline checks on 2026-07-04:

- `make` could not run because `make` is not on PATH in the PowerShell environment.
- `.\\.venv\\Scripts\\pros.exe make` initially failed because PROS tried to write `C:\\Users\\terry\\AppData\\Roaming\\PROS\\cli.pros`.
- With `APPDATA` redirected to `.pros-appdata`, PROS found a VS Code globalStorage toolchain path but reported `[WinError 5] Access is denied` when invoking its bundled `make.exe`.

Static baseline:

- `src/main.cpp` opcontrol references `intake_front`, `intake_back`, and `claw_motor`.
- `include/subsystems.hpp` defines `upper_intake`, `counter_rollers`, and `claw_arm` instead.
- The intake code therefore appears stale relative to the subsystem declarations.
