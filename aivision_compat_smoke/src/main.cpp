#include "main.h"

#include <cerrno>
#include <cstdio>

namespace {
constexpr std::uint8_t kAiVisionPort = 1;
}

void initialize() {
  pros::delay(700);
  const int type = static_cast<int>(pros::c::get_plugged_type(kAiVisionPort));
  const int reset = pros::c::aivision_reset(kAiVisionPort);
  const int family = pros::c::aivision_set_tag_family_override(
      kAiVisionPort, pros::TAG_CIRCLE_21H7);
  const int disable = pros::c::aivision_disable_detection_types(
      kAiVisionPort,
      pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_OBJECTS |
          pros::E_AIVISION_MODE_COLOR_MERGE);
  const int enable = pros::c::aivision_enable_detection_types(
      kAiVisionPort, pros::E_AIVISION_MODE_TAGS);
  std::printf("COMPAT_INIT port=%u type=%d reset=%d family=%d disable=%d enable=%d enabled=%d errno=%d\n",
              static_cast<unsigned>(kAiVisionPort), type, reset, family,
              disable, enable,
              pros::c::aivision_get_enabled_detection_types(kAiVisionPort),
              errno);
  std::fflush(stdout);
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  while (true) {
    const int count = pros::c::aivision_get_object_count(kAiVisionPort);
    std::printf("COMPAT_FRAME count=%d errno=%d", count, errno);
    const int bounded_count = count > 24 ? 24 : count;
    for (int i = 0; i < bounded_count; ++i) {
      const auto object = pros::c::aivision_get_object(kAiVisionPort, i);
      if (object.type == pros::E_AIVISION_DETECTED_TAG) {
        const auto& tag = object.object.tag;
        std::printf(" TAG id=%u corners=%d,%d,%d,%d,%d,%d,%d,%d",
                    static_cast<unsigned>(object.id), tag.x0, tag.y0, tag.x1,
                    tag.y1, tag.x2, tag.y2, tag.x3, tag.y3);
      }
    }
    std::printf("\n");
    std::fflush(stdout);
    pros::delay(100);
  }
}
