# Design Pointer

The complete design is intentionally kept in the user-requested root artifact:

```text
NEXT_CHANGES.md
```

The required design decisions are:

- predict continuously from encoders, horizontal odometry, and IMU;
- evaluate every fresh LiDAR/camera observation;
- never equate large disagreement with sensor truth;
- calculate gain from estimator uncertainty versus measurement uncertainty;
- use normalized innovation gating before correction;
- keep wall theta as a heading observation and tag bearing as a position-line observation;
- use stronger updates only with corroboration, such as two tags or a repeatable contact checkpoint;
- expose uncertainty, observation age, innovation, gain, and reject reason in telemetry;
- roll out camera corrections in shadow mode before allowing them to move the pose.
