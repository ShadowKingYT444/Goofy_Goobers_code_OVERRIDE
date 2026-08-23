# Drive distance validation — 2026-08-22

## Test configuration

- Physical/configured wheel: 2.75-inch omni wheel
- Encoder model: 2.75-inch diameter, 1:1 external ratio
- Drive configuration: 450 RPM
- Track width in software: 12.0086 inches
- Reference sensor: VEX GPS on Smart Port 7
- Procedure: drive backward for separate 2–10 inch trials, stop and sample GPS, then return to the starting encoder position

## Results and commentary

The encoder and GPS measurements show a strong linear relationship, but they do not have a 1:1 scale. Using the 3–10 inch trials and forcing the regression through the physically meaningful origin gives:

`GPS distance = 0.887 × encoder-derived distance`

Under the current 2.75-inch wheel model, the encoders therefore overestimate actual translation by approximately 11.3%. An equivalent empirically fitted rolling diameter is about **2.44 inches**. This is an effective calibration value, not a claim that the wheel's measured plastic diameter is 2.44 inches.

The 2-inch trial was excluded from the scale fit because the low-speed open-loop controller coasted to 2.65 encoder-inches before stopping. GPS nevertheless agreed with that actual encoder displacement in that one trial. At longer distances the systematic scale difference becomes much larger than the GPS's reported ~0.385-inch uncertainty.

Heading changed by no more than 0.36 degrees during any outward trial. The scale mismatch is therefore not explained by the robot curving. Return-to-start residuals were 0.05–0.25 inches by encoder and 0.14–0.46 inches by GPS, demonstrating good short-term repeatability.

## Engineering conclusion

The result supports adding an encoder distance scale near 0.887, or changing the effective wheel diameter from 2.75 to approximately 2.44 inches. Before changing the competition configuration, repeat the test in both directions and compare against a tape-measured 24–48 inch baseline. GPS noise and floor-dependent omni-wheel slip can bias a short-distance calibration.

The IMU could not be included: the Brain reported no IMU, and Smart Port 3 currently identifies as a motor.

## Figures

- `distance_sweep_dashboard.png`: four-panel engineering summary
- `distance_scale_correction.png`: clean comparison of current and proposed scaling
- SVG versions are included for lossless notebook/report export.
