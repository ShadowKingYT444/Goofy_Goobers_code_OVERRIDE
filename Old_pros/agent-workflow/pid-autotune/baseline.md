Before this change:

- `default_constants()` in `src/autons.cpp` used hand-entered drive, heading, and turn PID constants.
- There was no autonomous routine dedicated to selecting PID constants automatically.
- Controller-triggered autonomous existed only for the fused bypass route using `B + Down`.
- PROS build verification has been blocked on this machine by the global-storage `make.exe` WinError 5 issue.
