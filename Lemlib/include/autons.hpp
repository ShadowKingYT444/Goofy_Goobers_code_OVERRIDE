#pragma once

#include <cstdint>
#include <atomic>

void two_pin_far_auton();  // replaces the old one_pin_auton
void two_pin_close_auton();  // the same route from the three-pin's side
void three_pin_auton();
void gps_reset_test_auton();
void gps_mount_survey_auton();
void curve_autotune_auton();  // src/curve_autotune.cpp
// Default timeout is an initial safety deadline; validate it at the largest
// lift target under the heaviest expected load.
bool moveLift(double targetDeg, std::uint32_t timeoutMs = 6000,
              std::atomic_bool* cancel = nullptr);
// Drives the lift down until it is at rest or has stopped moving (it does not
// always reach 0) and returns the height it ended at.
double lowerLiftToRest(std::atomic_bool* cancel = nullptr);
// True while moveLift() or lowerLiftToRest() is driving the lift.
bool liftMoveActive();
// At or above lift_position::lock_from_deg, powers the lift toward max height
// to hold it against gravity and returns true. Below that it leaves the
// motors alone and returns false. Call it every loop while the lock applies.
bool holdLiftAtMax();
// Background lock for routines: while enabled, holdLiftAtMax() runs whenever
// no moveLift() is in progress.
void setLiftLock(bool enabled);
void moveArm(double targetDeg, std::atomic_bool* cancel = nullptr);
// The three-pin, then match loads, wall stacks, toggle and park; see
// skills_route in src/autons.cpp.
void skills();
// Two-pin far curve recorder for driver control; see curve_capture in
// src/autons.cpp. Toggle starts or ends a recording and returns true if one
// is now running; call Sample every loop.
bool curveCaptureToggle();
void curveCaptureSample();
// Driver-control note of where the lift and arm are now, for setting the
// skills route's stacking heights: added to a list shown over the whole
// Brain screen. Returns how many are on the list.
int recordHeights();
