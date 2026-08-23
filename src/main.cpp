#include "main.h"
#include "pros/apix.h"

#include <array>
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace {

constexpr int kAiVisionDeviceType = 29;
constexpr std::uint32_t kPollPeriodMs = 100;
constexpr std::uint32_t kDriveWatchdogMs = 250;
constexpr int kMaxDriveCommand = 63;  // 50% of PROS's 127 maximum.
std::array<std::uint8_t, 2> camera_ports{};
std::size_t camera_count = 0;
pros::MotorGroup left_drive({17, 18});
pros::MotorGroup right_drive({-11, -13});

void stop_drive() {
  left_drive.move(0);
  right_drive.move(0);
}

int clamp_drive(int command) {
  return std::clamp(command, -kMaxDriveCommand, kMaxDriveCommand);
}

bool configure_camera(std::uint8_t port) {
  const int reset = pros::c::aivision_reset(port);
  pros::delay(250);
  const int family = pros::c::aivision_set_tag_family_override(
      port, pros::TAG_CIRCLE_21H7);
  const int disabled = pros::c::aivision_disable_detection_types(
      port, pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_OBJECTS |
                pros::E_AIVISION_MODE_COLOR_MERGE);
  const int enabled = pros::c::aivision_enable_detection_types(
      port, pros::E_AIVISION_MODE_TAGS);
  const int active = pros::c::aivision_get_enabled_detection_types(port);
  std::printf(
      "AIV_INIT port=%u reset=%d family=%d disable=%d enable=%d active=%d errno=%d\n",
      static_cast<unsigned>(port), reset, family, disabled, enabled, active, errno);
  std::fflush(stdout);
  return reset >= 0 && family >= 0 && disabled >= 0 && enabled >= 0;
}

void poll_camera(std::uint8_t port) {
  while (true) {
    const int count = pros::c::aivision_get_object_count(port);
    char line[768];
    int used = std::snprintf(line, sizeof(line),
                             "AIV_FRAME port=%u count=%d",
                             static_cast<unsigned>(port), count);
    int tag_count = 0;
    const int bounded_count = count > 24 ? 24 : (count < 0 ? 0 : count);
    for (int index = 0; index < bounded_count && used < 680; ++index) {
      const auto object = pros::c::aivision_get_object(port, index);
      if (object.type != pros::E_AIVISION_DETECTED_TAG) continue;
      const auto& tag = object.object.tag;
      ++tag_count;
      used += std::snprintf(
          line + used, sizeof(line) - static_cast<std::size_t>(used),
          " TAG id=%u corners=%d,%d,%d,%d,%d,%d,%d,%d",
          static_cast<unsigned>(object.id), tag.x0, tag.y0, tag.x1, tag.y1,
          tag.x2, tag.y2, tag.x3, tag.y3);
    }
    std::printf("%s tags=%d errno=%d\n", line, tag_count, errno);
    std::fflush(stdout);
    pros::delay(kPollPeriodMs);
  }
}

}  // namespace

void initialize() {
  pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
  const int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (stdin_flags >= 0) fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK);

  pros::lcd::initialize();
  left_drive.set_brake_mode(pros::MotorBrake::brake);
  right_drive.set_brake_mode(pros::MotorBrake::brake);
  stop_drive();
  pros::lcd::set_text(1, "Dual AI Vision tag scan");
  pros::delay(700);

  for (std::uint8_t port = 1; port <= 21 && camera_count < camera_ports.size(); ++port) {
    if (static_cast<int>(pros::c::get_plugged_type(port)) != kAiVisionDeviceType) {
      continue;
    }
    camera_ports[camera_count++] = port;
  }

  std::printf("AIV_SCAN found=%u port0=%u port1=%u\n",
              static_cast<unsigned>(camera_count),
              static_cast<unsigned>(camera_ports[0]),
              static_cast<unsigned>(camera_ports[1]));
  std::fflush(stdout);
  pros::lcd::print(2, "Found %u camera(s)", static_cast<unsigned>(camera_count));

  for (std::size_t index = 0; index < camera_count; ++index) {
    const std::uint8_t port = camera_ports[index];
    configure_camera(port);
    pros::lcd::print(static_cast<std::int16_t>(3 + index), "Camera %u: port %u",
                     static_cast<unsigned>(index + 1), static_cast<unsigned>(port));
    pros::Task::create([port]() { poll_camera(port); }, TASK_PRIORITY_DEFAULT,
                       TASK_STACK_DEPTH_DEFAULT, index == 0 ? "aivision_0" : "aivision_1");
  }
}

void disabled() { stop_drive(); }
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  char line[96]{};
  std::size_t line_length = 0;
  std::uint32_t last_valid_command_ms = 0;
  std::uint32_t last_status_ms = 0;
  unsigned long last_sequence = 0;
  bool have_sequence = false;

  std::printf("CTRL_READY max=%d watchdog_ms=%u mapping=arcade\n",
              kMaxDriveCommand, static_cast<unsigned>(kDriveWatchdogMs));
  std::fflush(stdout);

  while (true) {
    char input[64];
    const int bytes_read = static_cast<int>(read(STDIN_FILENO, input, sizeof(input)));
    for (int i = 0; i < std::max(bytes_read, 0); ++i) {
      const char byte = input[i];
      if (byte == '\r') continue;
      if (byte != '\n') {
        if (line_length + 1 < sizeof(line)) line[line_length++] = byte;
        continue;
      }

      line[line_length] = '\0';
      unsigned long sequence = 0;
      int left = 0;
      int right = 0;
      if (std::sscanf(line, "D %lu %d %d", &sequence, &left, &right) == 3 &&
          (!have_sequence || sequence > last_sequence)) {
        have_sequence = true;
        last_sequence = sequence;
        left_drive.move(clamp_drive(left));
        right_drive.move(clamp_drive(right));
        last_valid_command_ms = pros::millis();
      } else if (std::sscanf(line, "S %lu", &sequence) == 1 &&
                 (!have_sequence || sequence > last_sequence)) {
        have_sequence = true;
        last_sequence = sequence;
        stop_drive();
        last_valid_command_ms = 0;
      }
      line_length = 0;
    }

    const std::uint32_t now = pros::millis();
    if (last_valid_command_ms != 0 && now - last_valid_command_ms > kDriveWatchdogMs) {
      stop_drive();
      last_valid_command_ms = 0;
      std::printf("CTRL_WATCHDOG_STOP seq=%lu\n", last_sequence);
      std::fflush(stdout);
    }
    if (now - last_status_ms >= 500) {
      last_status_ms = now;
      std::printf("CTRL_STATUS seq=%lu fresh=%d\n", last_sequence,
                  last_valid_command_ms != 0 ? 1 : 0);
      std::fflush(stdout);
    }
    pros::delay(10);
  }
}
