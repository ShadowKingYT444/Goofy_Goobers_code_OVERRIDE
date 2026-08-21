# Assumptions requiring verification

- Confirmed: the port-5 tracking wheel is a 2-inch omni wheel, rolls along the robot side-to-side axis, and is mounted at rear center.
- Confirmed by live paired turns: its raw sign is +1 and its pooled effective rear lever is 2.5665 inches. A measured lateral push remains a validation of contact/scale.
- Confirmed: the drivetrain has eight 2.75-inch omni wheels, four per side, powered by two gear-coupled motors per side. It is differential/tank drive and cannot command strafe; omni rollers only permit passive lateral slide.
- AI Vision was previously observed on port 1, but the current live device reports not installed there; current wiring must be inventoried before pose correction is enabled.
- The field props are static and have AprilTags on four faces. Side IDs 1-4 appear duplicated; the center ID, tag family, physical size, and face geometry remain unverified.
- The entered start pose is exact at the beginning of a round. A fail-closed controller editor is implemented, but physical button verification is pending.
- The fixed Brio camera and field photograph are debugging evidence only, not tournament inputs.
