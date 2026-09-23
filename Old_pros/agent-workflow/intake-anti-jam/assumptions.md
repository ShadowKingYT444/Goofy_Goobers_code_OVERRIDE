The active robot program is the root PROS project.

The intended intake motors are `upper_intake` on port 14 and `counter_rollers` on port 15 from `include/subsystems.hpp`.

Driver controls should keep the existing A/B intake pattern: A intakes, B outtakes.

PROS `get_current_draw()` returns milliamps, `get_actual_velocity()` returns rpm, and `get_torque()` returns Nm.

Jam detection should not trigger from one sample. It should require high load and low velocity for a short sustained window.
