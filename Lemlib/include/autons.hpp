#pragma once

#include <cstdint>

void one_pin_auton();
void three_pin_auton();
void gps_reset_test_auton();
void test_shi();
//void pid_autotune_auton();
void motion_test_auton();
// Default timeout is an initial safety deadline; validate it at the largest
// lift target under the heaviest expected load.
bool moveLift(double targetDeg, std::uint32_t timeoutMs = 6000);
void moveArm(double targetDeg, volatile bool* cancel = nullptr);
void one_pin_close();
void skills();
