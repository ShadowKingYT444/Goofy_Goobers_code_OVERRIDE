#pragma once

#include <array>
#include <cstdint>

namespace cascade_lift {

constexpr std::size_t kStageCount = 4;

struct Snapshot {
  bool sensor_ok = false;
  bool homed = false;
  bool faulted = false;
  bool pid_active = false;
  int target_stage = 0;
  int nearest_stage = 0;
  std::int32_t raw_centidegrees = 0;
  double position_deg = 0.0;
  double velocity_deg_s = 0.0;
  double target_deg = 0.0;
  double error_deg = 0.0;
  double output_mv = 0.0;
};

// Competition-simplified homing: call only while the cascade is physically at
// its resting bottom. Port 16 is reset and becomes the multi-turn zero.
void initialize_at_rest();

// Manual calibration control. Positive power raises the cascade; negative
// power lowers it. A zero command brakes without enabling position control.
void set_manual_power(int power);

// Records a measured encoder position for one of the five cascade heights.
bool record_stage(int stage, double position_deg);
bool record_stage_here(int stage);
bool stage_is_recorded(int stage);
double stage_position_deg(int stage);

// Enables closed-loop control only after the requested stage is calibrated.
bool set_target_stage(int stage);
// Enables closed-loop control for a position measured upward from rest. This
// is used for stage-zero returns and small calibration/test offsets.
bool set_target_position_deg(double position_deg);
void disable_pid();
void clear_fault();

// Run every 10-20 ms from the main operator/autonomous control loop.
void update();
Snapshot snapshot();

// Rate-limited calibration/PID telemetry for terminal capture.
void print_telemetry_if_due(std::uint32_t period_ms = 100);

}  // namespace cascade_lift
