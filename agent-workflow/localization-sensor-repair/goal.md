# Goal

Repair the live fused localization so clean LiDAR observations stabilize IMU heading without rapid reset oscillation, drivetrain encoder faults/signs do not silently corrupt forward displacement, and the port 5 horizontal odometer cannot inject invalid or overflowed movement.

The hardware contract is the current repo contract: motors `-17/-18` and `11/13`, IMU port 3, horizontal Rotation sensor port 5, and Distance sensors 6-9.

The completed system must also generate all four modulo-90 heading hypotheses from every clean wall fit, select the geometrically consistent visible wall, and use perpendicular wall distance to correct the observable X or Y coordinate. The parallel coordinate remains odometry-derived.
