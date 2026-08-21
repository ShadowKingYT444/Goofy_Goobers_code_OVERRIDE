#include "main.h"

#include <cerrno>
#include <cstdint>
#include <cstdio>

namespace {
constexpr std::uint8_t DISTANCE_PORT = 1;
constexpr double MM_PER_INCH = 25.4;

pros::Distance distance_sensor(DISTANCE_PORT);

void draw_bits(std::uint16_t value, std::int16_t y, std::int16_t bit_count) {
  for (std::int16_t bit = 0; bit < bit_count; ++bit) {
    const std::int16_t x = 20 + bit * 14;
    const bool on = (value & (1u << bit)) != 0;
    pros::screen::set_pen(on ? pros::c::COLOR_WHITE : pros::c::COLOR_BLACK);
    pros::screen::fill_rect(x, y, x + 10, y + 10);
  }
}

void print_distance() {
  static std::uint32_t sample_count = 0;
  ++sample_count;

  const std::int32_t mm = distance_sensor.get_distance();
  const std::int32_t confidence = distance_sensor.get_confidence();
  const double inches = mm >= 0 ? static_cast<double>(mm) / MM_PER_INCH : -1.0;

  pros::screen::erase();
  pros::screen::set_pen(pros::c::COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 0, "Distance port %u", static_cast<unsigned>(DISTANCE_PORT));
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "installed=%d errno=%d",
                      static_cast<int>(distance_sensor.is_installed()), errno);
  pros::screen::print(pros::E_TEXT_MEDIUM, 2, "mm=%ld in=%.2f",
                      static_cast<long>(mm), inches);
  pros::screen::print(pros::E_TEXT_MEDIUM, 3, "confidence=%ld", static_cast<long>(confidence));
  pros::screen::print(pros::E_TEXT_MEDIUM, 4, "sample=%lu", static_cast<unsigned long>(sample_count));
  pros::screen::set_pen(pros::c::COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 6, "DATA mm conf sample");
  draw_bits(mm > 0 ? static_cast<std::uint16_t>(mm) : 0, 140, 16);
  draw_bits(confidence > 0 ? static_cast<std::uint16_t>(confidence) : 0, 160, 8);
  draw_bits(static_cast<std::uint16_t>(sample_count & 0xffff), 180, 16);

  printf("DIST sample=%lu port=%u installed=%d mm=%ld in=%.2f confidence=%ld size=-1 velocity=0.000 errno=%d\n",
         static_cast<unsigned long>(sample_count),
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
  pros::c::serctl(SERCTL_ACTIVATE, reinterpret_cast<void*>(0x74756f73));
  pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);

  printf("Distance smoke initialize\n");
  fflush(stdout);
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
