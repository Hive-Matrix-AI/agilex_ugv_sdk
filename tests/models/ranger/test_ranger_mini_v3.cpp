// SPDX-License-Identifier: Apache-2.0

#include <utility>

#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3.hpp"
#include "support/assertions.hpp"
#include "support/fake_can_transport.hpp"

int main() {
  using namespace agilex::ugv;
  using testing::expect;
  auto transport = std::make_unique<testing::FakeCanTransport>();
  auto* fake = transport.get();
  RangerMiniV3 robot{std::move(transport)};
  expect(robot.stop() == Error::not_connected, "commands require connection");
  expect(!robot.connect("fake-can") && robot.is_connected(), "connect");
  expect(robot.capabilities().actuator_count == 8, "eight actuators");
  expect(!robot.state().system && !robot.state().motion_mode &&
             !robot.state().bms_basic,
         "initial feedback absent");
  expect(!robot.set_control_mode(ControlMode::can) &&
             fake->sent.back().id == 0x421 && fake->sent.back().size == 1,
         "facade uses manual CAN mode frame");
  expect(!robot.set_drive_mode(RangerDriveMode::voltage) &&
             fake->sent.back().id == 0x423,
         "drive mode");
  expect(robot.set_motion({0, 0.699}) == Error::value_out_of_range,
         "unknown mode uses Ackermann limit");
  expect(!robot.set_motion({0.7, 0.698}), "Ackermann steering boundary");
  expect(!robot.set_motion_mode(RangerMotionMode::parallel) &&
             fake->sent.back().id == 0x141,
         "request parallel mode");
  expect(robot.set_motion({0, 1.571}) == Error::value_out_of_range,
         "requested mode is not confirmed mode");
  int callbacks = 0;
  robot.set_feedback_handler([&](const Feedback&) {
    ++callbacks;
    expect(robot.state().updated_at.has_value(),
           "snapshot updated before callback, reentrant state access");
  });
  fake->inject({0x291, 3, {1, 1, 1}});
  expect(robot.set_motion({0, 1.571}) == Error::value_out_of_range,
         "changing mode retains conservative limit");
  fake->inject({0x291, 3, {1, 0, 1}});
  expect(!robot.set_motion({0.7, 1.571}),
         "parallel steering accepted after feedback");
  expect(robot.set_motion({0.701, 1.571}) == Error::value_out_of_range,
         "large steering enforces reduced speed");
  fake->inject({0x211, 8, {2, 1, 1, 0xE0, 0, 1, 2, 0x80}});
  for (std::uint32_t i = 0; i < 8; ++i) {
    fake->inject({0x251U + i, 8, {0, 10, 0, 20, 0xFF, 0xFF, 0xFF, 0xFF}});
    fake->inject({0x261U + i, 8, {1, 0xE0, 0, 30, 0, 0x40}});
  }
  fake->inject({0x271, 8, {0, 1, 0, 2, 0, 3, 0, 4}});
  fake->inject({0x281, 8, {0, 10, 0, 20, 0, 30, 0, 40}});
  fake->inject({0x361, 8, {90, 99, 1, 0xE0, 0, 1, 0, 2}});
  fake->inject({0x362, 4, {1, 2, 3, 4}});
  fake->inject({0x312, 8, {0, 0, 0, 1, 0, 0, 0, 2}});
  const auto snapshot = robot.state();
  expect(snapshot.system && snapshot.system->error_flags == 0x00010280,
         "faults retained");
  for (std::size_t i = 0; i < 8; ++i) {
    expect(snapshot.actuators_high_speed[i] &&
               snapshot.actuators_high_speed[i]->pulse_count == -1 &&
               snapshot.actuators_low_speed[i],
           "every actuator retained in snapshot");
  }
  expect(snapshot.motion_mode && snapshot.motor_angles &&
             snapshot.motor_speeds && snapshot.bms_basic &&
             snapshot.bms_extended && snapshot.rear_odometry,
         "model feedback types coexist");
  expect(snapshot.model_feedback.size() == 6,
         "mode update replaces earlier value of same type");
  fake->inject({0x361, 8, {80, 99, 1, 0xE0}});
  expect(robot.state().bms_basic->state_of_charge_percent == 80 &&
             snapshot.bms_basic->state_of_charge_percent == 90,
         "old snapshots remain immutable");
  expect(robot.state().model_feedback.size() == 6,
         "repeated BMS does not grow snapshot");
  const int before = callbacks;
  fake->inject({0x291, 2, {}});
  fake->inject({0x362, 8, {}});
  fake->inject({0x700, 8, {}});
  expect(callbacks == before &&
             robot.state().motion_mode->mode == RangerMotionMode::parallel,
         "malformed and unknown feedback leaves state and callbacks unchanged");
  expect(
      !robot.set_lights({true, LightMode::on}) && fake->sent.back().id == 0x121,
      "lights");
  expect(!robot.clear_error(8) && fake->sent.back().data[0] == 8,
         "motor 8 error reset");
  expect(!robot.clear_error(0x10) && fake->sent.back().data[0] == 0x10,
         "temperature error reset");
  expect(!robot.request_version() && fake->sent.back().id == 0x4A1,
         "legacy version query");
  fake->inject({0x4A1, 8, {'R', 'M', '3'}});
  expect(robot.state().version_response && !robot.state().version,
         "raw version response distinct from numeric version");
  fake->send_error = std::make_error_code(std::errc::io_error);
  expect(robot.set_motion_mode(RangerMotionMode::dual_ackermann) ==
             fake->send_error,
         "send errors propagate");
  expect(!robot.state().motion_mode->changing,
         "failed sends do not alter feedback state");
  fake->send_error.clear();
  expect(!robot.stop() && fake->sent.back().id == 0x111 &&
             fake->sent.back().data == std::array<std::uint8_t, 8>{},
         "stop sends a single zero motion frame");
  const auto sent = fake->sent.size();
  robot.disconnect();
  expect(!robot.is_connected() && fake->sent.size() == sent,
         "disconnect does not send commands");
  expect(!robot.connect("fake-can"), "reconnect");
  expect(!robot.state().updated_at && !robot.state().bms_basic &&
             !robot.state().motion_mode &&
             !robot.state().actuators_high_speed[7] &&
             robot.state().model_feedback.empty(),
         "reconnect clears all state");
  expect(robot.set_motion({0, 1.571}) == Error::value_out_of_range,
         "old parallel mode cannot survive reconnect");
}
