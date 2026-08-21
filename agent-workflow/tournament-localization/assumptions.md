# Assumptions

- The entered robot-center start pose is known before each round and the robot is stationary while the IMU calibrates.
- Smart port 20 is the permanently mounted VEX AI Vision sensor; this was verified by a live 21-port device scan.
- Official Override Goal tags use Circle21h7 and IDs shown by the current official field drawing; IDs 1-4 occur on two Goals each and ID 0 is unique.
- Four faces per Goal improve visibility but do not make an ID unique.
- The webcam may record independent ground truth during development, but tournament pose must be identical with it absent.
- Camera/tag extrinsics and apparent-size range are uncalibrated until live observations prove them.
