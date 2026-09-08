// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>

#include "agilex_ugv_sdk/core/commands.hpp"
#include "agilex_ugv_sdk/core/state_snapshot.hpp"

namespace agilex::ugv {

enum class RangerMotionMode : std::uint8_t {
  dual_ackermann = 0,
  parallel = 1,
  spinning = 2,
  park = 3,
};

enum class RangerDriveMode : std::uint8_t {
  current = 0,
  voltage = 1,
};

struct RangerDriveModeCommand final : ModelCommand {
  explicit RangerDriveModeCommand(RangerDriveMode value) : mode(value) {}
  RangerDriveMode mode;
};

struct RangerMotionCommand final : ModelCommand {
  RangerMotionCommand() = default;
  RangerMotionCommand(double linear, double steering, double angular = 0.0)
      : linear_velocity_mps(linear),
        steering_angle_rad(steering),
        angular_velocity_radps(angular) {}

  double linear_velocity_mps{0.0};
  double steering_angle_rad{0.0};
  double angular_velocity_radps{0.0};
};

struct RangerMotionModeCommand final : ModelCommand {
  explicit RangerMotionModeCommand(RangerMotionMode value) : mode(value) {}
  RangerMotionMode mode;
};

struct RangerMotionModeState final : ModelFeedback {
  RangerMotionMode mode{RangerMotionMode::dual_ackermann};
  bool changing{false};
  RangerDriveMode drive_mode{RangerDriveMode::current};
};

struct RangerMotorAngles final : ModelFeedback {
  // Steering motors 5, 6, 7, 8, in protocol order.
  std::array<double, 4> angles_rad{};
};

struct RangerMotorSpeeds final : ModelFeedback {
  // Drive motors 1, 2, 3, 4, in protocol order; linear wheel speed.
  std::array<double, 4> speeds_mps{};
};

struct RangerBmsBasicState final : ModelFeedback {
  std::uint8_t state_of_charge_percent{0};
  std::uint8_t state_of_health_percent{0};
  double voltage_v{0.0};
  double current_a{0.0};
  double temperature_c{0.0};
};

struct RangerBmsExtendedState final : ModelFeedback {
  std::uint8_t alarm_status_1{0};
  std::uint8_t alarm_status_2{0};
  std::uint8_t warn_status_1{0};
  std::uint8_t warn_status_2{0};
};

struct RangerRearOdometryState final : ModelFeedback {
  std::int32_t left_distance_mm{0};
  std::int32_t right_distance_mm{0};
};

struct RangerVersionResponse final : ModelFeedback {
  // Raw version-string fragment, not SCOUT's numeric version fields.
  std::array<std::uint8_t, 8> bytes{};
};

struct RangerMiniV3State : StateSnapshot {
  std::optional<RangerMotionModeState> motion_mode;
  std::optional<RangerMotorAngles> motor_angles;
  std::optional<RangerMotorSpeeds> motor_speeds;
  std::optional<RangerBmsBasicState> bms_basic;
  std::optional<RangerBmsExtendedState> bms_extended;
  std::optional<RangerVersionResponse> version_response;
  std::optional<RangerRearOdometryState> rear_odometry;
};

}  // namespace agilex::ugv
