#include "main.h"
#include "localization_config.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr std::uint8_t kAiVisionPort = localization::kAiVisionPort;
constexpr int kAiVisionDeviceType = 29;
constexpr std::uint32_t kPollPeriodMs = localization::kAiVisionPollPeriodMs;
constexpr double kImageWidthPx = localization::kAiImageWidthPx;
constexpr double kImageHeightPx = localization::kAiImageHeightPx;
constexpr double kHorizontalFovDeg = localization::kAiHorizontalFovDeg;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinQuadAreaPx2 = 40.0;
constexpr int kImageEdgeMarginPx = 2;

AiVisionShadowSnapshot snapshot;
std::uint8_t active_ai_vision_port = kAiVisionPort;
std::array<int, 9> last_geometry{};
bool have_last_geometry = false;
std::uint32_t last_poll_ms = 0;
std::uint32_t last_geometry_change_ms = 0;
std::uint32_t last_init_attempt_ms = 0;

double cross(int ax, int ay, int bx, int by, int cx, int cy) {
  return static_cast<double>(bx - ax) * static_cast<double>(cy - ay) -
         static_cast<double>(by - ay) * static_cast<double>(cx - ax);
}

double quad_area(const pros::aivision_object_tag_s_t& tag) {
  const double twice_area =
      static_cast<double>(tag.x0) * tag.y1 - static_cast<double>(tag.y0) * tag.x1 +
      static_cast<double>(tag.x1) * tag.y2 - static_cast<double>(tag.y1) * tag.x2 +
      static_cast<double>(tag.x2) * tag.y3 - static_cast<double>(tag.y2) * tag.x3 +
      static_cast<double>(tag.x3) * tag.y0 - static_cast<double>(tag.y3) * tag.x0;
  return std::abs(twice_area) * 0.5;
}

double point_distance(int x0, int y0, int x1, int y1) {
  return std::hypot(static_cast<double>(x1 - x0),
                    static_cast<double>(y1 - y0));
}

bool corners_in_bounds(const pros::aivision_object_tag_s_t& tag) {
  const std::array<int, 8> values{
      tag.x0, tag.y0, tag.x1, tag.y1, tag.x2, tag.y2, tag.x3, tag.y3};
  for (std::size_t i = 0; i < values.size(); i += 2) {
    if (values[i] <= kImageEdgeMarginPx ||
        values[i] >= static_cast<int>(kImageWidthPx) - 1 - kImageEdgeMarginPx ||
        values[i + 1] <= kImageEdgeMarginPx ||
        values[i + 1] >= static_cast<int>(kImageHeightPx) - 1 - kImageEdgeMarginPx) {
      return false;
    }
  }
  return true;
}

bool convex_quad(const pros::aivision_object_tag_s_t& tag) {
  const double c0 = cross(tag.x0, tag.y0, tag.x1, tag.y1, tag.x2, tag.y2);
  const double c1 = cross(tag.x1, tag.y1, tag.x2, tag.y2, tag.x3, tag.y3);
  const double c2 = cross(tag.x2, tag.y2, tag.x3, tag.y3, tag.x0, tag.y0);
  const double c3 = cross(tag.x3, tag.y3, tag.x0, tag.y0, tag.x1, tag.y1);
  const bool all_positive = c0 > 0 && c1 > 0 && c2 > 0 && c3 > 0;
  const bool all_negative = c0 < 0 && c1 < 0 && c2 < 0 && c3 < 0;
  return all_positive || all_negative;
}

std::array<int, 9> geometry_key(const pros::aivision_object_s_t& object) {
  const auto& tag = object.object.tag;
  return {object.id, tag.x0, tag.y0, tag.x1, tag.y1,
          tag.x2, tag.y2, tag.x3, tag.y3};
}

void emit_shadow_status() {
  std::printf(
      "VISION_SHADOW t=%lu poll=%lu port=%u installed=%d configured=%d count=%d "
      "tag=%d corners=%d,%d,%d,%d,%d,%d,%d,%d center=%.1f,%.1f "
      "area=%.1f mean_edge=%.2f range=%.2f edge_ratio=%.2f fill=%.2f bearing=%.2f repeat=%d geometry_age=%lu valid=%d reason=%s\n",
      static_cast<unsigned long>(snapshot.brain_ms),
      static_cast<unsigned long>(snapshot.poll_id),
      static_cast<unsigned>(snapshot.port),
      static_cast<int>(snapshot.installed), static_cast<int>(snapshot.configured),
      snapshot.object_count, snapshot.tag_id, snapshot.x0, snapshot.y0,
      snapshot.x1, snapshot.y1, snapshot.x2, snapshot.y2, snapshot.x3,
      snapshot.y3, snapshot.center_x_px, snapshot.center_y_px,
      snapshot.area_px2, snapshot.mean_edge_px, snapshot.range_estimate_in,
      snapshot.edge_ratio, snapshot.fill_ratio,
      snapshot.bearing_deg,
      static_cast<int>(snapshot.repeated_geometry),
      static_cast<unsigned long>(snapshot.geometry_age_ms),
      static_cast<int>(snapshot.tag_valid), snapshot.reason);
  std::fflush(stdout);
}
}  // namespace

void ai_vision_shadow_initialize() {
  last_init_attempt_ms = pros::millis();
  snapshot = {};
  snapshot.brain_ms = pros::millis();
  int detected_count = 0;
  std::uint8_t first_detected_port = 0;
  for (std::uint8_t port = 1; port <= 21; ++port) {
    if (static_cast<int>(pros::c::get_plugged_type(port)) !=
        kAiVisionDeviceType) {
      continue;
    }
    if (first_detected_port == 0) first_detected_port = port;
    ++detected_count;
  }
  active_ai_vision_port =
      static_cast<int>(pros::c::get_plugged_type(kAiVisionPort)) ==
              kAiVisionDeviceType
          ? kAiVisionPort
          : first_detected_port;
  snapshot.port = active_ai_vision_port;
  snapshot.installed = detected_count > 0;
  if (!snapshot.installed) {
    snapshot.reason = "not_installed";
    std::printf("VISION_INIT preferred_port=%u detected=0 reason=not_installed\n",
                static_cast<unsigned>(kAiVisionPort));
    std::fflush(stdout);
    return;
  }

  errno = 0;
  const int reset = pros::c::aivision_reset(active_ai_vision_port);
  const int family = pros::c::aivision_set_tag_family_override(
      active_ai_vision_port, pros::TAG_CIRCLE_21H7);
  const int disable = pros::c::aivision_disable_detection_types(
      active_ai_vision_port,
      pros::E_AIVISION_MODE_COLORS | pros::E_AIVISION_MODE_OBJECTS |
          pros::E_AIVISION_MODE_COLOR_MERGE);
  const int enable = pros::c::aivision_enable_detection_types(
      active_ai_vision_port, pros::E_AIVISION_MODE_TAGS);
  const int enabled =
      pros::c::aivision_get_enabled_detection_types(active_ai_vision_port);
  snapshot.configured = reset == 1 && family == 1 && disable == 1 &&
                        enable == 1 && enabled == pros::E_AIVISION_MODE_TAGS;
  snapshot.reason = snapshot.configured ? "no_tag" : "configure_failed";
  std::printf(
      "VISION_INIT port=%u installed=1 configured=%d reset=%d family=%d "
      "disable=%d enable=%d enabled=%d errno=%d\n",
       static_cast<unsigned>(active_ai_vision_port),
      static_cast<int>(snapshot.configured), reset, family, disable, enable,
      enabled, errno);
  std::fflush(stdout);
}

void ai_vision_shadow_update() {
  const std::uint32_t now = pros::millis();
  if (last_poll_ms != 0 && now - last_poll_ms < kPollPeriodMs) return;
  last_poll_ms = now;
  snapshot.brain_ms = now;
  ++snapshot.poll_id;
  snapshot.tag_valid = false;
  snapshot.repeated_geometry = false;
  snapshot.object_count = 0;
  snapshot.tag_id = -1;
  snapshot.x0 = snapshot.y0 = snapshot.x1 = snapshot.y1 = 0;
  snapshot.x2 = snapshot.y2 = snapshot.x3 = snapshot.y3 = 0;
  snapshot.center_x_px = 0.0;
  snapshot.center_y_px = 0.0;
  snapshot.area_px2 = 0.0;
  snapshot.mean_edge_px = 0.0;
  snapshot.range_estimate_in = 0.0;
  snapshot.edge_ratio = 0.0;
  snapshot.fill_ratio = 0.0;
  snapshot.bearing_deg = 0.0;
  snapshot.geometry_age_ms =
      last_geometry_change_ms == 0 ? 0 : now - last_geometry_change_ms;

  if (!snapshot.installed || !snapshot.configured) {
    // AI Vision can enumerate later than the user program after a Brain boot.
    // Retry at a bounded rate instead of permanently latching not_installed.
    if (now - last_init_attempt_ms >= 1000) {
      ai_vision_shadow_initialize();
      snapshot.brain_ms = now;
    }
    if (!snapshot.installed || !snapshot.configured) {
      snapshot.reason = snapshot.installed ? "configure_failed" : "not_installed";
      emit_shadow_status();
      return;
    }
  }

  errno = 0;
  const int count = pros::c::aivision_get_object_count(active_ai_vision_port);
  snapshot.object_count = count;
  if (count < 0 || count > 24) {
    snapshot.reason = "count_error";
    emit_shadow_status();
    return;
  }

  bool found_tag = false;
  pros::aivision_object_s_t best{};
  double best_area = -1.0;
  for (int i = 0; i < count; ++i) {
    const auto object = pros::c::aivision_get_object(active_ai_vision_port, i);
    if (object.type != pros::E_AIVISION_DETECTED_TAG) continue;
    const double area = quad_area(object.object.tag);
    if (area > best_area) {
      best = object;
      best_area = area;
      found_tag = true;
    }
  }
  if (!found_tag) {
    snapshot.reason = count == 0 ? "no_tag" : "no_tag_type";
    emit_shadow_status();
    return;
  }

  const auto& tag = best.object.tag;
  snapshot.tag_id = best.id;
  snapshot.x0 = tag.x0;
  snapshot.y0 = tag.y0;
  snapshot.x1 = tag.x1;
  snapshot.y1 = tag.y1;
  snapshot.x2 = tag.x2;
  snapshot.y2 = tag.y2;
  snapshot.x3 = tag.x3;
  snapshot.y3 = tag.y3;
  snapshot.center_x_px = 0.25 * (tag.x0 + tag.x1 + tag.x2 + tag.x3);
  snapshot.center_y_px = 0.25 * (tag.y0 + tag.y1 + tag.y2 + tag.y3);
  snapshot.area_px2 = best_area;
  const std::array<double, 4> edges{
      point_distance(tag.x0, tag.y0, tag.x1, tag.y1),
      point_distance(tag.x1, tag.y1, tag.x2, tag.y2),
      point_distance(tag.x2, tag.y2, tag.x3, tag.y3),
      point_distance(tag.x3, tag.y3, tag.x0, tag.y0),
  };
  const auto [minimum_edge, maximum_edge] =
      std::minmax_element(edges.begin(), edges.end());
  snapshot.mean_edge_px =
      0.25 * (edges[0] + edges[1] + edges[2] + edges[3]);
  const double mean_horizontal_edge_px = 0.5 * (edges[0] + edges[2]);
  const double mean_vertical_edge_px = 0.5 * (edges[1] + edges[3]);
  if (mean_horizontal_edge_px > 0.0 && mean_vertical_edge_px > 0.0) {
    const double z_from_width =
        localization::kAiFocalLengthXPx * localization::kAiTagOuterSizeIn /
        mean_horizontal_edge_px;
    const double z_from_height =
        localization::kAiFocalLengthYPx * localization::kAiTagOuterSizeIn /
        mean_vertical_edge_px;
    const double normalized_x =
        (snapshot.center_x_px - localization::kAiImageWidthPx * 0.5) /
        localization::kAiFocalLengthXPx;
    const double normalized_y =
        (snapshot.center_y_px - localization::kAiImageHeightPx * 0.5) /
        localization::kAiFocalLengthYPx;
    const double optical_axis_range = 0.5 * (z_from_width + z_from_height);
    snapshot.range_estimate_in = optical_axis_range *
        std::sqrt(1.0 + normalized_x * normalized_x +
                  normalized_y * normalized_y);
  }
  snapshot.edge_ratio = *minimum_edge > 0.0 ? *maximum_edge / *minimum_edge : INFINITY;
  const int minimum_x = std::min({tag.x0, tag.x1, tag.x2, tag.x3});
  const int maximum_x = std::max({tag.x0, tag.x1, tag.x2, tag.x3});
  const int minimum_y = std::min({tag.y0, tag.y1, tag.y2, tag.y3});
  const int maximum_y = std::max({tag.y0, tag.y1, tag.y2, tag.y3});
  const double bounding_area =
      static_cast<double>(maximum_x - minimum_x) *
      static_cast<double>(maximum_y - minimum_y);
  snapshot.fill_ratio = bounding_area > 0.0 ? best_area / bounding_area : 0.0;
  const double focal_x =
      (kImageWidthPx * 0.5) /
      std::tan((kHorizontalFovDeg * 0.5) * kPi / 180.0);
  snapshot.bearing_deg =
      std::atan((snapshot.center_x_px - kImageWidthPx * 0.5) / focal_x) *
      180.0 / kPi;

  const auto key = geometry_key(best);
  snapshot.repeated_geometry = have_last_geometry && key == last_geometry;
  if (!snapshot.repeated_geometry) {
    last_geometry = key;
    have_last_geometry = true;
    last_geometry_change_ms = now;
  }
  snapshot.geometry_age_ms =
      last_geometry_change_ms == 0 ? 0 : now - last_geometry_change_ms;

  if (best.id > 4) {
    snapshot.reason = "unknown_id";
  } else if (!corners_in_bounds(tag)) {
    snapshot.reason = "clipped";
  } else if (!convex_quad(tag)) {
    snapshot.reason = "nonconvex";
  } else if (*minimum_edge < localization::kAiMinTagEdgePx) {
    snapshot.reason = "short_edge";
  } else if (snapshot.edge_ratio > localization::kAiMaxTagEdgeRatio) {
    snapshot.reason = "edge_ratio";
  } else if (snapshot.fill_ratio < localization::kAiMinTagFillRatio) {
    snapshot.reason = "fill_ratio";
  } else if (best_area < kMinQuadAreaPx2) {
    snapshot.reason = "tiny";
  } else {
    snapshot.tag_valid = true;
    snapshot.reason = snapshot.repeated_geometry ? "repeat" : "shadow_valid";
  }

  emit_shadow_status();
}

const AiVisionShadowSnapshot& ai_vision_shadow_snapshot() { return snapshot; }
