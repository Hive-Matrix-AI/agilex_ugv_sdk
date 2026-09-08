// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <iostream>
#include <thread>

#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: ranger_mini_v3_state <can-interface>\n";
    return 2;
  }
  agilex::ugv::RangerMiniV3 robot;
  if (const auto error = robot.connect(argv[1])) {
    std::cerr << "connect failed: " << error.message() << '\n';
    return 1;
  }
  for (;;) {
    const auto state = robot.state();
    if (state.system) {
      std::cout << "battery=" << state.system->battery_voltage_v
                << " V, error_flags=0x" << std::hex << state.system->error_flags
                << std::dec << '\n';
    }
    if (state.motion_mode) {
      std::cout << "motion_mode=" << static_cast<int>(state.motion_mode->mode)
                << ", changing=" << state.motion_mode->changing
                << ", drive_mode="
                << static_cast<int>(state.motion_mode->drive_mode) << '\n';
    }
    if (state.bms_basic) {
      std::cout << "SOC="
                << static_cast<int>(state.bms_basic->state_of_charge_percent)
                << "%, current=" << state.bms_basic->current_a << " A\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
