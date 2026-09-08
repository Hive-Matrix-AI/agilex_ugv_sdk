// SPDX-License-Identifier: Apache-2.0

#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3.hpp"

#include <cmath>
#include <utility>

namespace agilex::ugv {

RangerMiniV3::RangerMiniV3(CanTransportPtr transport)
    : robot_(make_ranger_mini_v3_codec(), std::move(transport)) {}

std::error_code RangerMiniV3::connect(std::string_view interface_name) {
  return robot_.connect(interface_name);
}

void RangerMiniV3::disconnect() noexcept { robot_.disconnect(); }
bool RangerMiniV3::is_connected() const noexcept {
  return robot_.is_connected();
}
ModelCapabilities RangerMiniV3::capabilities() const noexcept {
  return robot_.capabilities();
}

std::error_code RangerMiniV3::set_control_mode(ControlMode mode) {
  return robot_.send(ControlModeCommand{mode});
}

std::error_code RangerMiniV3::set_drive_mode(RangerDriveMode mode) {
  return robot_.send(
      ModelCommandPtr{std::make_shared<RangerDriveModeCommand>(mode)});
}

std::error_code RangerMiniV3::set_motion_mode(RangerMotionMode mode) {
  return robot_.send(
      ModelCommandPtr{std::make_shared<RangerMotionModeCommand>(mode)});
}

std::error_code RangerMiniV3::set_motion(const RangerMotionCommand& command) {
  if (!is_connected()) return Error::not_connected;
  const auto snapshot = state();
  // A requested mode is not proof that the controller has entered that mode.
  // Until parallel-mode feedback arrives, use the Ackermann steering limit.
  if ((!snapshot.motion_mode || snapshot.motion_mode->changing ||
       snapshot.motion_mode->mode != RangerMotionMode::parallel) &&
      std::abs(command.steering_angle_rad) > 0.698) {
    return Error::value_out_of_range;
  }
  return robot_.send(
      ModelCommandPtr{std::make_shared<RangerMotionCommand>(command)});
}

std::error_code RangerMiniV3::stop() {
  return set_motion(RangerMotionCommand{});
}

std::error_code RangerMiniV3::set_lights(const LightCommand& command) {
  return robot_.send(command);
}

std::error_code RangerMiniV3::clear_error(std::uint8_t code) {
  return robot_.send(ClearErrorCommand{code});
}

std::error_code RangerMiniV3::request_version() {
  return robot_.send(VersionRequest{});
}

RangerMiniV3State RangerMiniV3::state() const {
  RangerMiniV3State result;
  static_cast<StateSnapshot&>(result) = robot_.state();
  if (auto value = result.model_state<RangerMotionModeState>())
    result.motion_mode = *value;
  if (auto value = result.model_state<RangerMotorAngles>())
    result.motor_angles = *value;
  if (auto value = result.model_state<RangerMotorSpeeds>())
    result.motor_speeds = *value;
  if (auto value = result.model_state<RangerBmsBasicState>())
    result.bms_basic = *value;
  if (auto value = result.model_state<RangerBmsExtendedState>())
    result.bms_extended = *value;
  if (auto value = result.model_state<RangerVersionResponse>())
    result.version_response = *value;
  if (auto value = result.model_state<RangerRearOdometryState>())
    result.rear_odometry = *value;
  return result;
}

void RangerMiniV3::set_feedback_handler(Robot::FeedbackHandler handler) {
  robot_.set_feedback_handler(std::move(handler));
}

}  // namespace agilex::ugv
