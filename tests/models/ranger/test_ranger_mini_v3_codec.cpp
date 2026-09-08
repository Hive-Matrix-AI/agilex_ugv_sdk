// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <variant>

#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3_codec.hpp"
#include "agilex_ugv_sdk/models/scout/scout_mini_codec.hpp"
#include "support/assertions.hpp"

namespace {
using namespace agilex::ugv;
using testing::expect;

bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

template <typename T>
T common(const DecodeResult& result) {
  expect(bool(result) && std::holds_alternative<T>(*result.feedback),
         "common feedback type");
  return std::get<T>(*result.feedback);
}

template <typename T>
T model(const DecodeResult& result) {
  auto ptr = common<ModelFeedbackPtr>(result);
  auto value = std::dynamic_pointer_cast<const T>(ptr);
  expect(bool(value), "model feedback type");
  return *value;
}

EncodeResult motion(RangerMiniV3Codec& codec, double linear, double steering,
                    double angular = 0) {
  return codec.encode(ModelCommandPtr{
      std::make_shared<RangerMotionCommand>(linear, steering, angular)});
}

void test_commands() {
  RangerMiniV3Codec codec;
  expect(codec.capabilities().actuator_count == 8 &&
             codec.capabilities().supports_lights &&
             !codec.capabilities().supports_lateral_motion,
         "Ranger capabilities");
  auto result = motion(codec, -0.15, 0.4, -0.2);
  expect(
      bool(result) && result.frame.id == 0x111 && result.frame.size == 8 &&
          result.frame.data == std::array<std::uint8_t, 8>{0xFF, 0x6A, 0xFF,
                                                           0x38, 0, 0, 1, 0x90},
      "signed motion payload: linear, angular, reserved, steering");
  expect(bool(motion(codec, 2.0, 0.0, 3.259)), "positive limits");
  expect(bool(motion(codec, -2.0, 0.0, -3.259)), "negative limits");
  expect(bool(motion(codec, 0.7, 1.571)),
         "parallel steering and turn-speed limits");
  expect(bool(motion(codec, -0.7, -1.571)), "negative parallel limits");
  expect(motion(codec, 2.001, 0).error == Error::value_out_of_range,
         "linear limit");
  expect(motion(codec, 0, 0, -3.260).error == Error::value_out_of_range,
         "spin limit");
  expect(motion(codec, 0, 1.572).error == Error::value_out_of_range,
         "steering limit");
  expect(motion(codec, 0.701, 0.350).error == Error::value_out_of_range,
         "turn speed limit");
  expect(motion(codec, -0.701, -0.350).error == Error::value_out_of_range,
         "negative turn speed limit");
  expect(bool(motion(codec, 2, 0.349)),
         "20-degree wire boundary below threshold");
  expect(motion(codec, 2, 0.3495).error == Error::value_out_of_range,
         "quantized angle cannot bypass turn speed limit");
  for (double value : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::max()}) {
    expect(motion(codec, value, 0).error == Error::value_out_of_range,
           "bad linear rejected");
    expect(motion(codec, 0, value).error == Error::value_out_of_range,
           "bad steering rejected");
    expect(motion(codec, 0, 0, value).error == Error::value_out_of_range,
           "bad angular rejected");
  }
  expect(codec.encode(MotionCommand{0, 0, 0.000001}).error ==
             Error::unsupported_command,
         "even sub-resolution lateral commands are unsupported");
  expect(bool(codec.encode(MotionCommand{})), "generic stop is supported");
  expect(codec.encode(ModelCommandPtr{}).error == Error::unsupported_command,
         "null extension rejected");
  expect(
      codec.encode(ModelCommandPtr{std::make_shared<ModelCommand>()}).error ==
          Error::unsupported_command,
      "foreign extension rejected");
  for (std::uint8_t mode = 0; mode <= 4; ++mode) {
    result =
        codec.encode(ModelCommandPtr{std::make_shared<RangerMotionModeCommand>(
            static_cast<RangerMotionMode>(mode))});
    if (mode == 4)
      expect(result.error == Error::value_out_of_range,
             "Mini 3.0 has no fifth mode");
    else
      expect(bool(result) && result.frame.id == 0x141 &&
                 result.frame.size == 1 &&
                 result.frame.data == std::array<std::uint8_t, 8>{mode},
             "mode command per manual");
  }
  for (std::uint8_t mode = 0; mode <= 2; ++mode) {
    result =
        codec.encode(ModelCommandPtr{std::make_shared<RangerDriveModeCommand>(
            static_cast<RangerDriveMode>(mode))});
    if (mode == 2)
      expect(result.error == Error::value_out_of_range, "invalid drive mode");
    else
      expect(bool(result) && result.frame.id == 0x423 &&
                 result.frame.size == 1 && result.frame.data[0] == mode,
             "drive mode command");
  }
  for (auto mode : {ControlMode::standby, ControlMode::can}) {
    result = codec.encode(ControlModeCommand{mode});
    expect(bool(result) && result.frame.id == 0x421 && result.frame.size == 1 &&
               result.frame.data[0] == static_cast<std::uint8_t>(mode),
           "control-mode DLC 1");
  }
  expect(codec.encode(ControlModeCommand{ControlMode::uart}).error ==
             Error::value_out_of_range,
         "reject UART mode");
  for (std::uint8_t code = 0; code <= 0x11; ++code) {
    result = codec.encode(ClearErrorCommand{code});
    if (code == 0x11)
      expect(result.error == Error::value_out_of_range, "invalid reset code");
    else
      expect(bool(result) && result.frame.id == 0x441 &&
                 result.frame.size == 1 && result.frame.data[0] == code,
             "all documented error-reset codes");
  }
  for (int i = 0; i < 2; ++i) {
    result = codec.encode(LightCommand{true, LightMode::on});
    expect(bool(result) && result.frame.id == 0x121 && result.frame.size == 8 &&
               result.frame.data == std::array<std::uint8_t, 8>{1, 1},
           "light reserved bytes stay zero, no command counter");
  }
  result = codec.encode(LightCommand{false, LightMode::on});
  expect(bool(result) && result.frame.data == std::array<std::uint8_t, 8>{},
         "disable lights");
  expect(codec.encode(LightCommand{true, LightMode::breath}).error ==
             Error::value_out_of_range,
         "no breathing lights");
  expect(
      codec.encode(LightCommand{true, LightMode::on, 0, LightMode::on}).error ==
          Error::unsupported_command,
      "no separate rear lights");
  result = codec.encode(VersionRequest{});
  expect(
      bool(result) && result.frame.id == 0x4A1 && result.frame.size == 1 &&
          result.frame.data[0] == 1,
      "version request follows upstream runtime, absent from Mini 3.0 manual");
  ScoutMiniCodec scout;
  expect(scout.encode(ModelCommandPtr{std::make_shared<RangerMotionCommand>()})
                 .error == Error::unsupported_command,
         "SCOUT does not accept Ranger commands");
}

void test_feedback() {
  RangerMiniV3Codec codec;
  const auto system = common<SystemState>(
      codec.decode({0x211, 8, {2, 1, 1, 0xE0, 0x81, 0x12, 0x34, 0x80}}));
  expect(system.control_mode == ControlMode::can &&
             near(system.battery_voltage_v, 48) &&
             system.error_flags == 0x81123480U && system.count == 0,
         "full 32-bit fault code, byte 7 is not a counter");
  const auto state = common<MotionState>(
      codec.decode({0x221, 8, {0xFF, 0x6A, 0, 0xC8, 0xFF, 0xFF, 0xFE, 0x70}}));
  expect(near(state.linear_velocity_mps, -0.15) &&
             near(state.angular_velocity_radps, 0.2) &&
             state.lateral_velocity_mps == 0 && state.steering_angle_rad &&
             near(*state.steering_angle_rad, -0.4),
         "motion units and reserved lateral bytes");
  for (std::uint8_t i = 1; i <= 8; ++i) {
    const auto high = common<ActuatorHighSpeedState>(
        codec.decode({0x250U + i, 8, {0xFF, 0x9C, 0xFF, 0x85, 0x80, 0, 0, 1}}));
    expect(high.index == i && high.speed_rpm == -100 &&
               near(high.current_a, -12.3) && high.pulse_count == -2147483647,
           "all eight motors preserve signed pulse counts");
    const auto low = common<ActuatorLowSpeedState>(
        codec.decode({0x260U + i, 8, {1, 0xE0, 0xFF, 0xF6, 0xFB, 0x45, 0, 0}}));
    expect(low.index == i && near(low.driver_voltage_v, 48) &&
               low.driver_temperature_c == -10 &&
               low.motor_temperature_c == -5 && low.status_flags == 0x45,
           "all eight low-speed feedback slots");
  }
  const auto angles = model<RangerMotorAngles>(
      codec.decode({0x271, 8, {0, 1, 0xFF, 0xFF, 6, 0x23, 0xF9, 0xDD}}));
  expect(near(angles.angles_rad[0], 0.001) &&
             near(angles.angles_rad[1], -0.001) &&
             near(angles.angles_rad[2], 1.571) &&
             near(angles.angles_rad[3], -1.571),
         "wheel angles have no V1 sign or degree conversion");
  const auto speeds = model<RangerMotorSpeeds>(
      codec.decode({0x281, 8, {0, 1, 0xFF, 0xFF, 7, 0xD0, 0xF8, 0x30}}));
  expect(near(speeds.speeds_mps[0], 0.001) && near(speeds.speeds_mps[3], -2),
         "wheel speeds are m/s");
  const auto mode =
      model<RangerMotionModeState>(codec.decode({0x291, 3, {1, 1, 1}}));
  expect(mode.mode == RangerMotionMode::parallel && mode.changing &&
             mode.drive_mode == RangerDriveMode::voltage,
         "three-byte mode feedback includes drive mode");
  const auto bms = model<RangerBmsBasicState>(
      codec.decode({0x361, 8, {90, 99, 1, 0xE0, 0xFF, 0x85, 0xFF, 0x9C}}));
  expect(bms.state_of_charge_percent == 90 &&
             bms.state_of_health_percent == 99 && near(bms.voltage_v, 48) &&
             near(bms.current_a, -12.3) && near(bms.temperature_c, -10),
         "Mini V3 BMS units");
  expect(
      near(model<RangerBmsBasicState>(codec.decode({0x361, 8, {0, 0, 0x80, 0}}))
               .voltage_v,
           3276.8),
      "BMS voltage is unsigned");
  const auto ext =
      model<RangerBmsExtendedState>(codec.decode({0x362, 4, {2, 3, 4, 5}}));
  expect(ext.alarm_status_1 == 2 && ext.alarm_status_2 == 3 &&
             ext.warn_status_1 == 4 && ext.warn_status_2 == 5,
         "four-byte BMS alarms");
  const auto front = common<OdometryState>(
      codec.decode({0x311, 8, {0xFF, 0xFF, 0xFC, 0x18, 0, 0, 3, 0xE8}}));
  const auto rear = model<RangerRearOdometryState>(
      codec.decode({0x312, 8, {0, 0, 3, 0xE8, 0xFF, 0xFF, 0xFC, 0x18}}));
  expect(front.left_distance_mm == -1000 && front.right_distance_mm == 1000 &&
             rear.left_distance_mm == 1000 && rear.right_distance_mm == -1000,
         "separate front and rear odometry");
  const auto remote = common<RemoteControlState>(
      codec.decode({0x241, 8, {0xE6, 0xFF, 2, 0xFD, 4, 0xFB, 0, 9}}));
  expect(remote.swa == 2 && remote.swb == 1 && remote.swc == 2 &&
             remote.swd == 3 && remote.right_horizontal == -1 &&
             remote.left_vertical == -3 && remote.knob == -5,
         "remote switches and signed axes");
  const auto lights = common<LightState>(
      codec.decode({0x231, 8, {1, 1, 255, 255, 255, 255, 255, 9}}));
  expect(lights.enabled && lights.front_mode == LightMode::on &&
             lights.front_value == 0 && lights.rear_mode == LightMode::off &&
             lights.count == 9,
         "light feedback ignores reserved fields");
  const auto version = model<RangerVersionResponse>(
      codec.decode({0x4A1, 8, {'R', 'M', '3', 0, 1, 2, 3, 4}}));
  expect(version.bytes ==
             std::array<std::uint8_t, 8>{'R', 'M', '3', 0, 1, 2, 3, 4},
         "version bytes preserved without assuming SCOUT format");
}

void test_validation() {
  RangerMiniV3Codec codec;
  for (std::uint32_t id :
       {0x211U, 0x221U, 0x231U, 0x241U, 0x251U, 0x258U, 0x261U, 0x268U, 0x271U,
        0x281U, 0x291U, 0x311U, 0x312U, 0x361U, 0x362U, 0x4A1U}) {
    for (std::uint8_t dlc = 0; dlc <= 8; ++dlc) {
      const auto expected = id == 0x291 ? 3 : id == 0x362 ? 4 : 8;
      const auto result = codec.decode({id, dlc, {}});
      expect(result.status == (dlc == expected ? DecodeStatus::decoded
                                               : DecodeStatus::invalid_dlc),
             "strict manual DLC for each supported feedback");
    }
  }
  for (std::uint32_t id : {0x259U, 0x269U, 0x41AU, 0x700U}) {
    expect(codec.decode({id, 8, {}}).status == DecodeStatus::ignored,
           "unknown IDs ignored");
  }
  expect(codec.decode({0x800, 8, {}}).status == DecodeStatus::invalid_frame,
         "nonstandard frame rejected");
  expect(codec.decode({0x211, 9, {}}).status == DecodeStatus::invalid_frame,
         "DLC above CAN capacity rejected");
}
}  // namespace

int main() {
  test_commands();
  test_feedback();
  test_validation();
}
