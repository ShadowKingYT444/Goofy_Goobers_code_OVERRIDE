# Failure Cases

- Start X/Y silently remains hard-coded to a field corner.
- Raw IMU zero is incorrectly treated as global field heading zero when a nonzero start heading was entered.
- LiDAR IMU resets use the old 90-degree constant and rotate the field frame unexpectedly.
- Tag IDs are treated as globally unique even though IDs 1 through 4 each appear at two goals.
- Goal IDs are transcribed from an older diagram instead of `FIELD_2d_VIEW.jpg`.
- Goal coordinates use tile indices instead of multiplying each tile step by 24 inches.
- X/Y axes are accidentally rendered like normal screen coordinates instead of +X toward the 0-degree wall and +Y toward red.
- Future camera code assumes the camera is on the LiDAR side.
- Existing fused movement source checks regress.
