Create a durable Override field coordinate configuration that stores the nine fixed goal/AprilTag landmarks from `FIELD_2d_VIEW.jpg`, accepts an explicit robot start position and field heading, and starts each localization run with the physical IMU reset to zero.

This pass does not apply AI Vision corrections yet. It prepares authoritative field metadata and a start-pose/IMU contract that later vision fusion can consume without assuming a corner.
