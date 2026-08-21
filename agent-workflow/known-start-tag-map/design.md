# Design

## Field frame

Use the center-origin tile frame specified by the user:

- Origin: center Goal at `(0, 0)`.
- X: positive toward the 0-degree/top wall and negative toward Audience View.
- Y: positive toward the red side and negative toward the blue side.
- Heading: 0 degrees is +X; 90 degrees is +Y.
- Scale: one unit is one inch; one field-tile step is 24 inches.

The nine fixed goal centers are:

| Goal | Tag ID | X (in) | Y (in) |
|---|---:|---:|---:|
| center | 0 | 0 | 0 |
| top red-side neutral | 2 | 48 | 24 |
| top blue-side alliance | 1 | 48 | -24 |
| upper red-side neutral | 3 | 24 | 48 |
| upper blue-side alliance | 4 | 24 | -48 |
| lower red-side alliance | 4 | -24 | 48 |
| lower blue-side neutral | 3 | -24 | -48 |
| bottom red-side alliance | 1 | -48 | 24 |
| bottom blue-side neutral | 2 | -48 | -24 |

The fixed goal table stores goal center X/Y, tag ID, color, and a stable name. Each Goal carries four AprilTags with the listed ID. IDs 1 through 4 occur at two different Goals, so later vision code must use pose consistency and a winner margin instead of treating ID as globally unique.

## Editable start pose

Create `include/localization_config.hpp` with one obvious `kAutonStartPose` value. The user edits its X, Y, and heading before an autonomous routine. No corner is inferred.

## IMU contract

At initialization and at localization start:

1. Calibrate the IMU.
2. Reset the physical IMU reading to 0 degrees.
3. Save the entered field heading as the field direction represented by raw IMU zero.
4. Convert later clockwise-positive IMU rotation into the counterclockwise-positive field frame.

Example: entered heading 135 degrees plus raw clockwise IMU rotation 20 degrees produces field heading 115 degrees.

## Sensor-side metadata

Record LiDAR as left-side and AI Vision as right-side. This is metadata for the later camera transform; this pass does not guess camera offsets or apply vision corrections.
