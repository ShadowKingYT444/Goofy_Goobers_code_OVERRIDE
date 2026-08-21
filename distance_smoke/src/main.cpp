#include "main.h"

#include <cerrno>
#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace {
constexpr std::uint8_t DISTANCE_PORT = 1;
constexpr double MM_PER_INCH = 25.4;

pros::Distance distance_sensor(DISTANCE_PORT);
std::int32_t history[120] = {};
int history_index = 0;
int history_count = 0;

void draw_distance_screen(std::int32_t mm, std::int32_t confidence) {
  history[history_index] = mm;
  history_index = (history_index + 1) % 120;
  history_count = std::min(history_count + 1, 120);

  pros::screen::set_eraser(0x00000000);
  pros::screen::erase();
  pros::screen::set_pen(0x0000ff66);
  pros::screen::print(TEXT_MEDIUM, 0, "Distance Sensor P%u", DISTANCE_PORT);
  pros::screen::print(TEXT_MEDIUM, 1, "%ld mm   %.2f in",
                      static_cast<long>(mm),
                      mm >= 0 ? static_cast<double>(mm) / MM_PER_INCH : -1.0);
  pros::screen::print(TEXT_MEDIUM, 2, "confidence %ld / 63",
                      static_cast<long>(confidence));

  const int x0 = 20;
  const int y0 = 60;
  const int x1 = 460;
  const int y1 = 220;
  pros::screen::draw_rect(x0, y0, x1, y1);
  for (int i = 1; i < 4; ++i) {
    int y = y0 + (y1 - y0) * i / 4;
    pros::screen::draw_line(x0, y, x1, y);
  }

  if (history_count < 2) {
    return;
  }

  pros::screen::set_pen(0x0000ccff);
  auto y_for_mm = [&](std::int32_t value) {
    std::int32_t clamped = std::clamp(value, static_cast<std::int32_t>(0), static_cast<std::int32_t>(1200));
    return y1 - static_cast<int>((static_cast<double>(clamped) / 1200.0) * (y1 - y0));
  };

  int prev_x = x0;
  int prev_y = y_for_mm(history[(history_index - history_count + 120) % 120]);
  for (int i = 1; i < history_count; ++i) {
    int idx = (history_index - history_count + i + 120) % 120;
    int x = x0 + (x1 - x0) * i / 119;
    int y = y_for_mm(history[idx]);
    pros::screen::draw_line(prev_x, prev_y, x, y);
    prev_x = x;
    prev_y = y;
  }
}

void print_distance() {
  printf("DIST tick port=%u installed=%d errno=%d\n",
         static_cast<unsigned>(DISTANCE_PORT),
         static_cast<int>(distance_sensor.is_installed()),
         errno);
  fflush(stdout);

  const std::int32_t mm = distance_sensor.get_distance();
  const std::int32_t confidence = distance_sensor.get_confidence();
  const double inches = mm >= 0 ? static_cast<double>(mm) / MM_PER_INCH : -1.0;
  draw_distance_screen(mm, confidence);

  printf("DIST port=%u installed=%d mm=%ld in=%.2f confidence=%ld size=-1 velocity=0.000 errno=%d\n",
         static_cast<unsigned>(DISTANCE_PORT),
         static_cast<int>(distance_sensor.is_installed()),
         static_cast<long>(mm),
         inches,
         static_cast<long>(confidence),
         errno);
  fflush(stdout);
}
}  // namespace

void initialize() {
  printf("Distance smoke initialize\n");
  fflush(stdout);
  pros::screen::set_eraser(0x00000000);
  pros::screen::erase();
  pros::screen::set_pen(0x0000ff66);
  pros::screen::print(TEXT_MEDIUM, 0, "Distance smoke boot");
  pros::delay(500);
  print_distance();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  while (true) {
    print_distance();
    pros::delay(50);
  }
}
