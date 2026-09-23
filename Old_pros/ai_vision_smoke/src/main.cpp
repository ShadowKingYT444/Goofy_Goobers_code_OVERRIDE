#include "main.h"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
constexpr std::uint8_t AIVISION_PORT = 1;
constexpr double IMAGE_WIDTH_PX = 320.0;
constexpr double IMAGE_HEIGHT_PX = 240.0;
constexpr double HORIZONTAL_FOV_DEG = 74.0;
constexpr double VERTICAL_FOV_DEG = 63.0;
constexpr double TAG_SIZE_IN = 0.75;
constexpr double PI = 3.14159265358979323846;

pros::AIVision ai_vision(AIVISION_PORT);

double deg_to_rad(double deg) {
  return deg * PI / 180.0;
}

double rad_to_deg(double rad) {
  return rad * 180.0 / PI;
}

double distance_px(double x0, double y0, double x1, double y1) {
  double dx = x1 - x0;
  double dy = y1 - y0;
  return std::sqrt(dx * dx + dy * dy);
}

void ai_vision_init() {
  pros::delay(700);

  for (std::uint8_t port = 1; port <= 21; ++port) {
    printf("PORT_SCAN port=%u type=%d\n",
           static_cast<unsigned>(port),
           static_cast<int>(pros::Device::get_plugged_type(port)));
  }
  fflush(stdout);

  int32_t reset_result = ai_vision.reset();
  int32_t family_result =
      ai_vision.set_tag_family(pros::AivisionTagFamily::tag_21H7, true);
  int32_t disable_result =
      ai_vision.disable_detection_types(pros::AivisionModeType::colors,
                                        pros::AivisionModeType::color_merge);
  int32_t enable_result =
      ai_vision.enable_detection_types(pros::AivisionModeType::tags,
                                       pros::AivisionModeType::objects);
  int32_t overlay_result =
      pros::c::aivision_set_usb_bounding_box_overlay(AIVISION_PORT, true);
  int32_t awb_result = ai_vision.start_awb();

  printf("AI Vision init. port=%u installed=%d reset=%ld family=%ld disable=%ld "
         "enable=%ld overlay=%ld awb=%ld enabled=%ld temp=%.1f errno=%d\n",
         static_cast<unsigned>(AIVISION_PORT),
         static_cast<int>(ai_vision.is_installed()),
         static_cast<long>(reset_result),
         static_cast<long>(family_result),
         static_cast<long>(disable_result),
         static_cast<long>(enable_result),
         static_cast<long>(overlay_result),
         static_cast<long>(awb_result),
         static_cast<long>(ai_vision.get_enabled_detection_types()),
         pros::c::aivision_get_temperature(AIVISION_PORT),
         errno);
  fflush(stdout);
}

void apriltag_status_log() {
  auto objects = ai_vision.get_all_objects();
  printf("AI Vision installed=%d enabled=%ld count=%ld objects=%d temp=%.1f errno=%d\n",
         static_cast<int>(ai_vision.is_installed()),
         static_cast<long>(ai_vision.get_enabled_detection_types()),
         static_cast<long>(ai_vision.get_object_count()),
         static_cast<int>(objects.size()),
         pros::c::aivision_get_temperature(AIVISION_PORT),
         errno);

  for (std::size_t i = 0; i < objects.size(); ++i) {
    const auto& object = objects[i];
    if (pros::AIVision::is_type(object, pros::AivisionDetectType::object)) {
      const auto& element = object.object.element;
      char class_name[21] = {};
      const int32_t class_result = pros::c::aivision_get_class_name(
          AIVISION_PORT, object.id, reinterpret_cast<std::uint8_t*>(class_name));
      const double center_x = element.xoffset + 0.5 * element.width;
      const double fx = (IMAGE_WIDTH_PX * 0.5) /
                        std::tan(deg_to_rad(HORIZONTAL_FOV_DEG * 0.5));
      const double bearing_deg =
          rad_to_deg(std::atan((center_x - IMAGE_WIDTH_PX * 0.5) / fx));
      printf("DETECT type=object id=%d class=%s class_ok=%ld x=%u y=%u w=%u h=%u score=%u bearing=%.2f\n",
             static_cast<int>(object.id),
             class_result == 1 ? class_name : "unknown",
             static_cast<long>(class_result), element.xoffset, element.yoffset,
             element.width, element.height, element.score, bearing_deg);
      continue;
    }
    if (!pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) {
      printf("DETECT type=other raw_type=%d id=%d\n",
             static_cast<int>(object.type), static_cast<int>(object.id));
      continue;
    }

    const auto& tag = object.object.tag;
    double fx = (IMAGE_WIDTH_PX * 0.5) / std::tan(deg_to_rad(HORIZONTAL_FOV_DEG * 0.5));
    double fy = (IMAGE_HEIGHT_PX * 0.5) / std::tan(deg_to_rad(VERTICAL_FOV_DEG * 0.5));
    double center_x = 0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
    double center_y = 0.25 * (tag.y0 + tag.y1 + tag.y2 + tag.y3);
    double top_width_px = distance_px(tag.x0, tag.y0, tag.x1, tag.y1);
    double bottom_width_px = distance_px(tag.x3, tag.y3, tag.x2, tag.y2);
    double left_height_px = distance_px(tag.x0, tag.y0, tag.x3, tag.y3);
    double right_height_px = distance_px(tag.x1, tag.y1, tag.x2, tag.y2);
    double tag_width_px = 0.5 * (top_width_px + bottom_width_px);
    double tag_height_px = 0.5 * (left_height_px + right_height_px);
    double tag_px = 0.5 * (tag_width_px + tag_height_px);
    double bearing_deg = rad_to_deg(std::atan((center_x - IMAGE_WIDTH_PX * 0.5) / fx));
    double elevation_deg = rad_to_deg(std::atan((IMAGE_HEIGHT_PX * 0.5 - center_y) / fy));
    double range_in = TAG_SIZE_IN * fx / tag_px;
    double right_in = range_in * std::tan(deg_to_rad(bearing_deg));
    double up_in = range_in * std::tan(deg_to_rad(elevation_deg));

    printf("[%d] TAG id=%d corners=(%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",
           static_cast<int>(i),
           static_cast<int>(object.id),
           tag.x0, tag.y0,
           tag.x1, tag.y1,
           tag.x2, tag.y2,
           tag.x3, tag.y3);
    printf("DETECT type=tag id=%d x0=%d y0=%d x1=%d y1=%d x2=%d y2=%d x3=%d y3=%d\n",
           static_cast<int>(object.id),
           tag.x0, tag.y0,
           tag.x1, tag.y1,
           tag.x2, tag.y2,
           tag.x3, tag.y3);
    printf("    pose tag_size=%.2fin center=(%.1f,%.1f) size=%.1fpx "
           "bearing=%.1fdeg elevation=%.1fdeg range=%.1fin right=%.1fin up=%.1fin\n",
           TAG_SIZE_IN,
           center_x, center_y,
           tag_px,
           bearing_deg,
           elevation_deg,
           range_in,
           right_in,
           up_in);
  }

  fflush(stdout);
}
}  // namespace

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "AI Vision smoke");
  pros::lcd::set_text(1, "Port 1 tags+objects");
  ai_vision_init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  while (true) {
    apriltag_status_log();
    pros::delay(100);
  }
}
