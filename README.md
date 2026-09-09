# AgileX UGV SDK

[English](README.md) | [简体中文](README.zh-CN.md)

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#quick-start)
[![CMake](https://img.shields.io/badge/CMake-3.16%2B-064F8C?logo=cmake&logoColor=white)](#quick-start)
[![Linux](https://img.shields.io/badge/platform-Linux-FCC624?logo=linux&logoColor=black)](#quick-start)
[![SocketCAN](https://img.shields.io/badge/transport-SocketCAN-3C8D6E)](#connect-a-robot)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

**Connect and control new AgileX mobile robots with C++17.**

`agilex_ugv_sdk` brings CAN communication, typed commands, and robot feedback
into one extensible library. Use a model API to connect to your robot, or build
on the shared interfaces to support another AgileX model.

[Quick start](#quick-start) · [Supported models](#supported-models) ·
[API](#api-overview) · [CAN references](#protocol-references) ·
[Changelog](CHANGELOG.md)

## Highlights

- SCOUT MINI, SCOUT MINI OMNI, and RANGER MINI 3.0 communication over SocketCAN.
- Velocity, control-mode, light, and error-clear commands.
- Motion, battery, motor, remote-control, and version feedback.
- RANGER steering modes, wheel telemetry, and BMS feedback.
- Standalone state monitors and tests that run without robot hardware.

## Supported models

| Family | Model | Status |
| --- | --- | --- |
| SCOUT | SCOUT MINI | Supported |
| SCOUT | SCOUT MINI OMNI | Supported |
| RANGER | RANGER MINI 3.0 | Supported (offline tested) |

RANGER support has been validated with offline protocol and API tests, not on a
physical robot. Verify compatibility with your model and firmware before motion.

## Quick start

Requires **Linux**, a **C++17 compiler**, and **CMake 3.16+**.

```bash
git clone https://github.com/Hive-Matrix-AI/agilex_ugv_sdk.git
cd agilex_ugv_sdk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$HOME/.local"
```

Link the installed SDK from your CMake project:

```cmake
find_package(agilex_ugv_sdk 1 REQUIRED)
target_link_libraries(my_target PRIVATE agilex_ugv_sdk::agilex_ugv_sdk)
```

The public namespace is `agilex::ugv`. If CMake cannot locate the package,
configure your project with `-DCMAKE_PREFIX_PATH="$HOME/.local"`.

<details>
<summary>Build options</summary>

| Option | Default | Purpose |
| --- | --- | --- |
| `BUILD_TESTING` | `ON` | Protocol and robot/transport tests without hardware |
| `BUILD_EXAMPLES` | `OFF` | Standalone state-monitor examples |

</details>

## Connect a robot

For the supported models, configure SocketCAN at **500 kbit/s**.
Replace `can0` with your adapter's interface:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
./build/scout_mini_state can0
```

The [state-monitor example](examples/scout/scout_mini_state.cpp) prints battery
voltage and error flags as feedback arrives. It does not enable motion.
Stop it with `Ctrl+C`.

## API overview

| Interface | Responsibility |
| --- | --- |
| `CanFrame` / `CanTransport` | Frame values and CAN I/O; SocketCAN is the provided backend |
| `ProtocolCodec` / `ModelCapabilities` | Model-specific encoding, decoding, and supported features |
| `Robot` | Compose a codec and transport; expose commands, state, and callbacks |
| `ScoutMini` | Ready-to-use API for the supported SCOUT models |
| `RangerMiniV3` | RANGER MINI 3.0 motion modes, steering, drive mode, and BMS |

Common commands and feedback use shared types. `ModelCommand` and
`ModelFeedback` provide extension points for model-specific data.

<details>
<summary>Package layout</summary>

```text
include/agilex_ugv_sdk/
  core/                   Commands, feedback, state, and robot interface
  protocol/               Codec interface and encode/decode results
  transport/              CAN frames, transport interface, and backends
  models/scout/           SCOUT MINI API and codec
  models/ranger/          RANGER MINI 3.0 API, types, and codec
src/
  core/                   Shared robot implementation
  transport/              Transport backends
  models/scout/           SCOUT MINI implementation
  models/ranger/          RANGER MINI 3.0 implementation
tests/
  core/                   Tests with model-independent codecs
  models/scout/           SCOUT API and protocol tests
  models/ranger/          RANGER API and protocol tests
  support/                Shared test transports and assertions
examples/scout/           Standalone SCOUT examples
examples/ranger/          Standalone RANGER examples
docs/reference/models/    Protocol references grouped by model family
```

Each model family groups its API and codec under `models/<family>/`, with
corresponding sources, tests, examples, and protocol references. Family-local
CMake files register their sources with the shared library target.

Headers use the component paths above. The original 1.0 header paths remain
available as forwarding includes for existing applications.

</details>

<details>
<summary>SCOUT MINI methods and command behavior</summary>

Include `agilex_ugv_sdk/models/scout/scout_mini.hpp` and construct
`agilex::ugv::ScoutMini`.
The default variant is `ScoutMiniVariant::skid_steer`; pass
`ScoutMiniVariant::omni` for SCOUT MINI OMNI.

| Method | Purpose |
| --- | --- |
| `connect("can0")` / `disconnect()` | Open or close the transport |
| `state()` | Copy the latest state; fields are optional until feedback arrives |
| `set_feedback_handler(handler)` | Receive decoded feedback on the transport thread |
| `request_version()` | Request controller and driver versions |
| `set_control_mode(ControlMode::can)` | Enable CAN commanded mode |
| `set_motion(command)` / `stop()` | Send one velocity or zero-speed command |
| `set_lights(command)` | Set front and rear light modes |
| `clear_error(motor)` | Clear all errors (`0`) or motor errors (`1`–`4`) |

Check the `std::error_code` returned by connection and command methods.
Commands outside model limits are rejected; feedback with invalid CAN IDs
or DLC is not added to state snapshots.

For SCOUT MINI motion, select CAN control mode and send commands every 20 ms.
The SDK does not repeat commands in the background. `stop()` sends one
zero-speed frame; disconnecting does not send a stop command. Keep the physical
emergency stop within reach during motion tests.

Feedback callbacks must return promptly and must not disconnect or destroy
the robot from the receive thread. See the
[CAN reference](docs/reference/models/scout/scout_mini_can.md) for units, limits, frame
formats, and controller timeout behavior.

</details>

## RANGER MINI 3.0

Include `agilex_ugv_sdk/models/ranger/ranger_mini_v3.hpp` and construct
`agilex::ugv::RangerMiniV3`. Use the same 500 kbit/s SocketCAN setup above, then
run `./build/ranger_mini_v3_state can0` for a feedback-only monitor.

The model supports Ackermann, parallel, spinning, and park modes, eight actuator
channels, wheel angles and speeds, front/rear odometry, and BMS feedback.
`set_motion(RangerMotionCommand{linear, steering, angular})` takes m/s, rad, and
rad/s. Check `state().motion_mode` after `set_motion_mode()` to confirm the
controller has finished switching. Steering above 0.698 rad is accepted only
after parallel-mode feedback confirms completion; large turns are limited to
0.7 m/s. `set_drive_mode()` selects current or voltage drive.

See the [RANGER CAN reference (中文)](docs/reference/models/ranger/ranger_mini_v3_can.md)
for command limits, frame formats, error codes, and firmware compatibility.

## Protocol references

- [SCOUT MINI / SCOUT MINI OMNI (中文)](docs/reference/models/scout/scout_mini_can.md)
- [RANGER MINI 3.0 (中文)](docs/reference/models/ranger/ranger_mini_v3_can.md)

## Integrations

SCOUT ROS 2 drivers and diagnostics are maintained in
[scout_ros2](https://github.com/Hive-Matrix-AI/scout_ros2).

## License

Licensed under [Apache-2.0](LICENSE). Existing copyright and attribution notices
are retained in the source.

## Contributing

Bug reports, tests, and support for additional AgileX models are welcome.
See [CONTRIBUTING.md](CONTRIBUTING.md) for the contribution and test workflow.
