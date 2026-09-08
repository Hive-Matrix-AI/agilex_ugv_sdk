// SPDX-License-Identifier: Apache-2.0

// RANGER MINI 3.0 manual V1.0.0 (2024.06) takes precedence over the
// generic V2 layout in ugv_sdk's parser and RangerBase.
// Copyright (c) 2019-2021 Weston Robot Pte. Ltd.
#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3_codec.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

namespace agilex::ugv {
namespace {

constexpr std::uint32_t kMotionCommandId = 0x111;
constexpr std::uint32_t kLightCommandId = 0x121;
constexpr std::uint32_t kSystemStateId = 0x211;
constexpr std::uint32_t kMotionStateId = 0x221;
constexpr std::uint32_t kLightStateId = 0x231;
constexpr std::uint32_t kRemoteControlStateId = 0x241;
constexpr std::uint32_t kActuatorHighSpeedFirstId = 0x251;
constexpr std::uint32_t kActuatorHighSpeedLastId = 0x258;
constexpr std::uint32_t kActuatorLowSpeedFirstId = 0x261;
constexpr std::uint32_t kActuatorLowSpeedLastId = 0x268;
constexpr std::uint32_t kOdometryStateId = 0x311;
// AgilexBase::RequestVersion sends directly to 0x4A1 with DLC 1.
constexpr std::uint32_t kVersionRequestId = 0x4A1;
constexpr std::uint32_t kMotionModeCommandId = 0x141;
constexpr std::uint32_t kMotorAnglesId = 0x271;
constexpr std::uint32_t kMotorSpeedsId = 0x281;
constexpr std::uint32_t kMotionModeStateId = 0x291;
constexpr std::uint32_t kBmsBasicId = 0x361;
constexpr std::uint32_t kBmsExtendedId = 0x362;
constexpr std::uint32_t kVersionFeedbackId = 0x4A1;
constexpr std::uint32_t kDriveModeId = 0x423;
constexpr std::uint32_t kRearOdometryId = 0x312;
constexpr std::uint32_t kControlModeId = 0x421;
constexpr std::uint32_t kClearErrorId = 0x441;

template <class... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

void write_u16_be(std::uint16_t value, std::uint8_t* destination) noexcept {
  destination[0] = static_cast<std::uint8_t>(value >> 8U);
  destination[1] = static_cast<std::uint8_t>(value & 0xFFU);
}

void write_i16_be(std::int16_t value, std::uint8_t* destination) noexcept {
  write_u16_be(static_cast<std::uint16_t>(value), destination);
}

std::uint16_t read_u16_be(const std::uint8_t* source) noexcept {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(source[0]) << 8U) |
      static_cast<std::uint16_t>(source[1]));
}

std::int16_t read_i16_be(const std::uint8_t* source) noexcept {
  const auto raw = read_u16_be(source);
  if (raw <=
      static_cast<std::uint16_t>(std::numeric_limits<std::int16_t>::max())) {
    return static_cast<std::int16_t>(raw);
  }
  return static_cast<std::int16_t>(static_cast<std::int32_t>(raw) - 0x10000);
}

std::int32_t read_i32_be(const std::uint8_t* source) noexcept {
  const auto raw = (static_cast<std::uint32_t>(source[0]) << 24U) |
                   (static_cast<std::uint32_t>(source[1]) << 16U) |
                   (static_cast<std::uint32_t>(source[2]) << 8U) |
                   static_cast<std::uint32_t>(source[3]);
  if (raw <=
      static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    return static_cast<std::int32_t>(raw);
  }
  return static_cast<std::int32_t>(static_cast<std::int64_t>(raw) -
                                   0x100000000LL);
}

std::int8_t read_i8(std::uint8_t raw) noexcept {
  if (raw <=
      static_cast<std::uint8_t>(std::numeric_limits<std::int8_t>::max())) {
    return static_cast<std::int8_t>(raw);
  }
  return static_cast<std::int8_t>(static_cast<std::int16_t>(raw) - 0x100);
}

// Check before scaling/rounding, including huge finite input values.
bool scaled_i16(double value, double limit, std::int16_t& output) noexcept {
  if (!std::isfinite(value) || std::abs(value) > limit) return false;
  output = static_cast<std::int16_t>(std::llround(value * 1000.0));
  return true;
}

EncodeResult encode_motion(const RangerMotionCommand& value) {
  std::int16_t linear{}, angular{}, steering{};
  constexpr double kTwentyDegreesRad = 20.0 * 3.14159265358979323846 / 180.0;
  const double speed_limit =
      std::abs(value.steering_angle_rad) > kTwentyDegreesRad ? 0.7 : 2.0;
  if (!scaled_i16(value.linear_velocity_mps, speed_limit, linear) ||
      !scaled_i16(value.angular_velocity_radps, 3.259, angular) ||
      !scaled_i16(value.steering_angle_rad, 1.571, steering)) {
    return {{}, Error::value_out_of_range};
  }
  CanFrame frame{kMotionCommandId, 8, {}};
  // Enforce the turn-speed restriction on the quantized wire angle too.
  if (std::abs(static_cast<double>(steering) * 0.001) > kTwentyDegreesRad &&
      std::abs(static_cast<int>(linear)) > 700) {
    return {{}, Error::value_out_of_range};
  }
  write_i16_be(linear, &frame.data[0]);
  write_i16_be(angular, &frame.data[2]);
  // RangerBase always sends zero in the lateral-velocity field.
  write_i16_be(steering, &frame.data[6]);
  return {frame, {}};
}

template <typename T>
DecodeResult model_feedback(T value) {
  return {DecodeStatus::decoded,
          Feedback{ModelFeedbackPtr{std::make_shared<T>(std::move(value))}}};
}

DecodeResult invalid_dlc() { return {DecodeStatus::invalid_dlc, std::nullopt}; }

}  // namespace

ModelCapabilities RangerMiniV3Codec::capabilities() const noexcept {
  // Side-slip is selected with a mode and steering, not a lateral command.
  return {"RANGER MINI 3.0", 8, false, true};
}

EncodeResult RangerMiniV3Codec::encode(const Command& command) {
  return std::visit(
      Overloaded{
          [](const MotionCommand& value) -> EncodeResult {
            if (!std::isfinite(value.lateral_velocity_mps)) {
              return {{}, Error::value_out_of_range};
            }
            if (value.lateral_velocity_mps != 0.0) {
              return {{}, Error::unsupported_command};
            }
            return encode_motion(RangerMotionCommand{
                value.linear_velocity_mps, 0.0, value.angular_velocity_radps});
          },
          [](const LightCommand& value) -> EncodeResult {
            if (value.front_mode != LightMode::off &&
                value.front_mode != LightMode::on) {
              return {{}, Error::value_out_of_range};
            }
            if (value.front_value != 0U || value.rear_mode != LightMode::off ||
                value.rear_value != 0U)
              return {{}, Error::unsupported_command};
            CanFrame frame{kLightCommandId, 8, {}};
            frame.data[0] = value.enabled ? 1U : 0U;
            frame.data[1] = value.enabled
                                ? static_cast<std::uint8_t>(value.front_mode)
                                : 0U;
            return {frame, {}};
          },
          [](const ControlModeCommand& value) -> EncodeResult {
            if (value.mode != ControlMode::standby &&
                value.mode != ControlMode::can) {
              return {{}, Error::value_out_of_range};
            }
            CanFrame frame{kControlModeId, 1, {}};
            frame.data[0] = static_cast<std::uint8_t>(value.mode);
            return {frame, {}};
          },
          [](const ClearErrorCommand& value) -> EncodeResult {
            if (value.motor > 0x10U) {
              return {{}, Error::value_out_of_range};
            }
            CanFrame frame{kClearErrorId, 1, {}};
            frame.data[0] = value.motor;
            return {frame, {}};
          },
          [](const VersionRequest&) -> EncodeResult {
            CanFrame frame{kVersionRequestId, 1, {}};
            frame.data[0] = 0x01;
            return {frame, {}};
          },
          [](const ModelCommandPtr& value) -> EncodeResult {
            if (auto motion =
                    std::dynamic_pointer_cast<const RangerMotionCommand>(
                        value)) {
              return encode_motion(*motion);
            }
            if (auto mode =
                    std::dynamic_pointer_cast<const RangerMotionModeCommand>(
                        value)) {
              const auto raw = static_cast<std::uint8_t>(mode->mode);
              if (raw > 3U) return {{}, Error::value_out_of_range};
              return {CanFrame{kMotionModeCommandId, 1, {raw}}, {}};
            }
            if (auto drive =
                    std::dynamic_pointer_cast<const RangerDriveModeCommand>(
                        value)) {
              const auto raw = static_cast<std::uint8_t>(drive->mode);
              if (raw > 1U) return {{}, Error::value_out_of_range};
              return {CanFrame{kDriveModeId, 1, {raw}}, {}};
            }
            return {{}, Error::unsupported_command};
          }},
      command);
}

DecodeResult RangerMiniV3Codec::decode(const CanFrame& frame) const {
  if (!frame.valid()) {
    return {DecodeStatus::invalid_frame, std::nullopt};
  }

  switch (frame.id) {
    case kSystemStateId: {
      if (frame.size != 8U) return invalid_dlc();
      SystemState state;
      state.vehicle_state = static_cast<VehicleState>(frame.data[0]);
      state.control_mode = static_cast<ControlMode>(frame.data[1]);
      state.battery_voltage_v =
          static_cast<double>(read_u16_be(&frame.data[2])) * 0.1;
      state.error_flags =
          (static_cast<std::uint32_t>(read_u16_be(&frame.data[4])) << 16U) |
          read_u16_be(&frame.data[6]);
      return {DecodeStatus::decoded, Feedback{state}};
    }
    case kMotionStateId: {
      if (frame.size != 8U) return invalid_dlc();
      MotionState state;
      state.linear_velocity_mps =
          static_cast<double>(read_i16_be(&frame.data[0])) * 0.001;
      state.angular_velocity_radps =
          static_cast<double>(read_i16_be(&frame.data[2])) * 0.001;
      state.steering_angle_rad =
          static_cast<double>(read_i16_be(&frame.data[6])) * 0.001;
      return {DecodeStatus::decoded, Feedback{state}};
    }
    case kLightStateId: {
      if (frame.size != 8U) return invalid_dlc();
      LightState state;
      state.enabled = frame.data[0] != 0U;
      state.front_mode = static_cast<LightMode>(frame.data[1]);
      state.count = frame.data[7];
      return {DecodeStatus::decoded, Feedback{state}};
    }
    case kRemoteControlStateId: {
      if (frame.size != 8U) return invalid_dlc();
      RemoteControlState state;
      state.swa = frame.data[0] & 0x03U;
      state.swb = (frame.data[0] >> 2U) & 0x03U;
      state.swc = (frame.data[0] >> 4U) & 0x03U;
      state.swd = (frame.data[0] >> 6U) & 0x03U;
      state.right_horizontal = read_i8(frame.data[1]);
      state.right_vertical = read_i8(frame.data[2]);
      state.left_vertical = read_i8(frame.data[3]);
      state.left_horizontal = read_i8(frame.data[4]);
      state.knob = read_i8(frame.data[5]);
      state.count = frame.data[7];
      return {DecodeStatus::decoded, Feedback{state}};
    }
    case kOdometryStateId: {
      if (frame.size != 8U) return invalid_dlc();
      OdometryState state;
      state.left_distance_mm = read_i32_be(&frame.data[0]);
      state.right_distance_mm = read_i32_be(&frame.data[4]);
      return {DecodeStatus::decoded, Feedback{state}};
    }
    case kRearOdometryId: {
      if (frame.size != 8U) return invalid_dlc();
      RangerRearOdometryState state;
      state.left_distance_mm = read_i32_be(&frame.data[0]);
      state.right_distance_mm = read_i32_be(&frame.data[4]);
      return model_feedback(state);
    }
    case kVersionFeedbackId: {
      if (frame.size != 8U) return invalid_dlc();
      RangerVersionResponse version;
      version.bytes = frame.data;
      return model_feedback(version);
    }
    case kMotionModeStateId: {
      if (frame.size != 3U) return invalid_dlc();
      RangerMotionModeState state;
      state.mode = static_cast<RangerMotionMode>(frame.data[0]);
      state.changing = frame.data[1] != 0U;
      state.drive_mode = static_cast<RangerDriveMode>(frame.data[2]);
      return model_feedback(state);
    }
    case kMotorAnglesId: {
      if (frame.size != 8U) return invalid_dlc();
      RangerMotorAngles state;
      for (std::size_t i = 0; i < state.angles_rad.size(); ++i) {
        state.angles_rad[i] =
            static_cast<double>(read_i16_be(&frame.data[2 * i])) * 0.001;
      }
      return model_feedback(state);
    }
    case kMotorSpeedsId: {
      if (frame.size != 8U) return invalid_dlc();
      RangerMotorSpeeds state;
      for (std::size_t i = 0; i < state.speeds_mps.size(); ++i) {
        state.speeds_mps[i] =
            static_cast<double>(read_i16_be(&frame.data[2 * i])) * 0.001;
      }
      return model_feedback(state);
    }
    case kBmsBasicId: {
      if (frame.size != 8U) return invalid_dlc();
      RangerBmsBasicState state;
      state.state_of_charge_percent = frame.data[0];
      state.state_of_health_percent = frame.data[1];
      // Mini V3 uses 0.1 V directly; the V2-specific extra factor is not
      // applied.
      state.voltage_v = static_cast<double>(read_u16_be(&frame.data[2])) * 0.1;
      state.current_a = static_cast<double>(read_i16_be(&frame.data[4])) * 0.1;
      state.temperature_c =
          static_cast<double>(read_i16_be(&frame.data[6])) * 0.1;
      return model_feedback(state);
    }
    case kBmsExtendedId: {
      if (frame.size != 4U) return invalid_dlc();
      RangerBmsExtendedState state;
      state.alarm_status_1 = frame.data[0];
      state.alarm_status_2 = frame.data[1];
      state.warn_status_1 = frame.data[2];
      state.warn_status_2 = frame.data[3];
      return model_feedback(state);
    }
    default:
      break;
  }

  if (frame.id >= kActuatorHighSpeedFirstId &&
      frame.id <= kActuatorHighSpeedLastId) {
    if (frame.size != 8U) return invalid_dlc();
    ActuatorHighSpeedState state;
    state.index =
        static_cast<std::uint8_t>(frame.id - kActuatorHighSpeedFirstId + 1U);
    state.speed_rpm = read_i16_be(&frame.data[0]);
    state.current_a = static_cast<double>(read_i16_be(&frame.data[2])) * 0.1;
    state.pulse_count = read_i32_be(&frame.data[4]);
    return {DecodeStatus::decoded, Feedback{state}};
  }

  if (frame.id >= kActuatorLowSpeedFirstId &&
      frame.id <= kActuatorLowSpeedLastId) {
    if (frame.size != 8U) return invalid_dlc();
    ActuatorLowSpeedState state;
    state.index =
        static_cast<std::uint8_t>(frame.id - kActuatorLowSpeedFirstId + 1U);
    state.driver_voltage_v =
        static_cast<double>(read_u16_be(&frame.data[0])) * 0.1;
    state.driver_temperature_c = read_i16_be(&frame.data[2]);
    state.motor_temperature_c = read_i8(frame.data[4]);
    state.status_flags = frame.data[5];
    return {DecodeStatus::decoded, Feedback{state}};
  }

  return {DecodeStatus::ignored, std::nullopt};
}

ProtocolCodecPtr make_ranger_mini_v3_codec() {
  return std::make_unique<RangerMiniV3Codec>();
}

}  // namespace agilex::ugv
