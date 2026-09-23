# Research

Accessed 2026-07-10.

- Current official Override manual: https://www.vexrobotics.com/override-manual. Version 1.0 states a 12 ft x 12 ft field with nine Goals; Appendix A includes AprilTag numbering.
- Official PROS AI Vision tutorial: https://pros.cs.purdue.edu/v5/pros-4/aivision.html. AprilTag observations provide an ID and four image-corner coordinates.
- Local PROS 4.2.2 headers/library expose `pros::AIVision`, Circle21h7 selection, tag-only detection, installed state, object count, and corner data.
- Local hardware proof supersedes the old port assumption: smart port 20 reports device type 29 and a working AI Vision sensor.
