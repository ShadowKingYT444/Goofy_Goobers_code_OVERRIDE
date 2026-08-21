#pragma once

#include <cstdint>

struct AiVisionShadowSnapshot {
  bool installed = false;
  bool configured = false;
  bool tag_valid = false;
  bool repeated_geometry = false;
  std::uint8_t port = 0;
  std::uint32_t poll_id = 0;
  std::uint32_t brain_ms = 0;
  std::uint32_t geometry_age_ms = 0;
  int object_count = 0;
  int tag_id = -1;
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  int x3 = 0;
  int y3 = 0;
  double center_x_px = 0.0;
  double center_y_px = 0.0;
  double area_px2 = 0.0;
  double mean_edge_px = 0.0;
  double range_estimate_in = 0.0;
  double edge_ratio = 0.0;
  double fill_ratio = 0.0;
  double bearing_deg = 0.0;
  const char* reason = "not_initialized";
};

void ai_vision_shadow_initialize();
void ai_vision_shadow_update();
const AiVisionShadowSnapshot& ai_vision_shadow_snapshot();
