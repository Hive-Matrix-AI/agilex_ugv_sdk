// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "agilex_ugv_sdk/core/robot.hpp"
#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3_codec.hpp"
#include "agilex_ugv_sdk/transport/socketcan_transport.hpp"

namespace agilex::ugv {

class RangerMiniV3 final {
 public:
  explicit RangerMiniV3(CanTransportPtr transport = make_socketcan_transport());

  [[nodiscard]] std::error_code connect(std::string_view interface_name);
  void disconnect() noexcept;
  [[nodiscard]] bool is_connected() const noexcept;
  [[nodiscard]] ModelCapabilities capabilities() const noexcept;

  [[nodiscard]] std::error_code set_control_mode(ControlMode mode);
  [[nodiscard]] std::error_code set_drive_mode(RangerDriveMode mode);
  [[nodiscard]] std::error_code set_motion_mode(RangerMotionMode mode);
  // Steering beyond 0.698 rad requires completed parallel-mode feedback.
  [[nodiscard]] std::error_code set_motion(const RangerMotionCommand& command);
  [[nodiscard]] std::error_code stop();
  [[nodiscard]] std::error_code set_lights(const LightCommand& command);
  // Mini 3.0 fault selectors 0x00-0x10; see the model CAN reference.
  [[nodiscard]] std::error_code clear_error(std::uint8_t code = 0);
  [[nodiscard]] std::error_code request_version();

  [[nodiscard]] RangerMiniV3State state() const;
  void set_feedback_handler(Robot::FeedbackHandler handler);

 private:
  Robot robot_;
};

}  // namespace agilex::ugv
