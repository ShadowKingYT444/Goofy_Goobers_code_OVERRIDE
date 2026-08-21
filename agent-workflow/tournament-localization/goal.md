# Goal

Make tournament localization work from an exact, stationary entered start pose using only on-robot sensors during a match: IMU, drivetrain encoders, rear-center port-5 tracking wheel, four LiDAR sensors, and the VEX AI Vision sensor observing AprilTags on the nine static Goals. The laptop webcam is debug ground truth only and must never alter official pose or autonomous control. LiDAR blockage must degrade gracefully rather than disable localization. Live acceptance movement is capped at 36 inches total and 720 degrees accumulated rotation.
