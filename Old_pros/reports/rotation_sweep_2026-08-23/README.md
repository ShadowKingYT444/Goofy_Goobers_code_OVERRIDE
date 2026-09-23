# In-place rotation validation — 2026-08-23

## Configuration and method

- Drive wheels: configured as 2.75-inch omni wheels
- Configured track width: 12.0086 inches
- GPS: Smart Port 7
- IMU: Smart Port 6, confirmed installed and calibrated successfully
- Trials: clockwise 15, 30, 45, 60, and 90 degree encoder-controlled turns; each trial immediately returned counterclockwise to its initial encoder heading

GPS and IMU heading deltas use the opposite sign from the positive encoder turn convention, so the analysis compares angular magnitudes.

## Results and commentary

At 45 and 60 degrees, all three systems agreed closely. The encoder predicted 45.63 and 60.63 degrees; GPS measured 44.90 and 59.61 degrees; the IMU measured 45.79 and 61.54 degrees. This is strong evidence that the configured track width is approximately correct for medium turns.

The 15 and 30 degree trials overshot their commands. Because the test used a slow open-loop motor command, static friction had to be overcome and the drivetrain continued moving during braking. These points characterize controller behavior, not just drivetrain geometry, and should not be used alone to tune track width.

At 90 degrees, the encoder reported 90.57 degrees and the IMU reported 92.53 degrees. GPS reported 85.38 degrees, but its reported position uncertainty simultaneously increased from approximately 0.385 inch to 1.174 inches. The GPS point is therefore lower-confidence; the IMU is the better short-term angular reference for this trial.

Using the 45–90 degree trials, the IMU-derived effective track width averages approximately **11.85 inches**, close to the configured 12.0086 inches. The remaining difference is small enough that another longer, closed-loop test should be completed before changing the competition constant.

Return-to-start performance was good for the four fully logged trials: GPS and IMU heading residuals remained below 0.65 degrees, and GPS position residuals remained below 0.24 inch.

## Important interruption

The USB serial connection reset after the outward 90-degree measurement. The robot visibly returned close to its original orientation, but the final 90-degree return telemetry was not captured. The diagnostic was disabled and the normal `MoreVex` program was restored afterward.

## Engineering conclusion

The IMU on port 6 is working and agrees well with the wheel encoders. For competition localization, use the IMU as the high-rate heading source, wheel encoders for short-term translation, and GPS as a slower global correction when its reported quality is acceptable. Small-angle motion needs a closed-loop turn controller rather than the open-loop calibration pulse used here.
