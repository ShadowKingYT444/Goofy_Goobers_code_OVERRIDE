Failure cases to guard against:

- Normal game-object compression briefly spikes torque and falsely triggers anti-jam.
- Intake startup has low rpm for a moment and falsely triggers anti-jam.
- One motor stalls while the other still spins, hiding the jam if only a group average is checked.
- Anti-jam blocks the whole opcontrol loop for one second and freezes driving.
- Calibration threshold becomes too low because the intake was not spinning or was already jammed.
- Existing stale motor names prevent the project from compiling.
