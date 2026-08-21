# Research Notes

Accessed 2026-07-09.

- Local PROS 4.2.2 headers confirm `pros::AIVision` supports AprilTag families and returns tag ID plus four image corners.
- Official PROS AI Vision tutorial confirms image coordinates use a top-left origin and AprilTag detections expose ID plus four corners: https://pros.cs.purdue.edu/v5/pros-4/aivision.html
- Official VEX AI Vision material confirms 320x240 image resolution: https://kb.vex.com/hc/en-us/articles/24070971920532-Coding-with-the-AI-Vision-Sensor-in-VEXcode-V5-Blocks
- Official VEX comparison material reports a 74-degree horizontal field of view: https://kb.vex.com/hc/en-us/articles/39803871672212-Comparing-the-VEX-IQ-AI-Vision-Sensor-to-the-Vision-Sensor
- Official Override manual and Appendix A field information: https://www.vexrobotics.com/override-manual
- Override uses a six-by-six nominal 24-inch tile layout. The config follows the user's requested ideal tile coordinate frame rather than the slightly smaller physical inside-wall dimension.
- The supplied `FIELD_2d_VIEW.jpg` is authoritative for tag ID arrangement and the user-defined coordinate orientation.

Implementation implication: store goal centers now, but defer live camera correction until the right-side camera yaw and offsets are measured on the final chassis.
