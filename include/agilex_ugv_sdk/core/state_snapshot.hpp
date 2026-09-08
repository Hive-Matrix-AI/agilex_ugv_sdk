// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <vector>

#include "agilex_ugv_sdk/core/feedback.hpp"

namespace agilex::ugv {

struct StateSnapshot {
  using TimePoint = std::chrono::steady_clock::time_point;

  std::optional<SystemState> system;
  std::optional<MotionState> motion;
  std::optional<LightState> lights;
  std::optional<RemoteControlState> remote_control;
  // One-based actuator indices on the wire map to zero-based array slots.
  std::array<std::optional<ActuatorHighSpeedState>, 8> actuators_high_speed{};
  std::array<std::optional<ActuatorLowSpeedState>, 8> actuators_low_speed{};
  std::optional<OdometryState> odometry;
  std::optional<VersionInfo> version;
  std::optional<TimePoint> updated_at;

  // Latest immutable feedback of each model-specific dynamic type.
  std::vector<ModelFeedbackPtr> model_feedback;

  template <typename T>
  [[nodiscard]] std::shared_ptr<const T> model_state() const {
    for (const auto& feedback : model_feedback) {
      if (auto value = std::dynamic_pointer_cast<const T>(feedback))
        return value;
    }
    return {};
  }
};

}  // namespace agilex::ugv
