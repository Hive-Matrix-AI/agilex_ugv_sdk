# AgileX UGV SDK

[English](README.md) | [简体中文](README.zh-CN.md)

[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](#快速开始)
[![CMake](https://img.shields.io/badge/CMake-3.16%2B-064F8C?logo=cmake&logoColor=white)](#快速开始)
[![Linux](https://img.shields.io/badge/platform-Linux-FCC624?logo=linux&logoColor=black)](#快速开始)
[![SocketCAN](https://img.shields.io/badge/transport-SocketCAN-3C8D6E)](#连接机器人)
[![许可证](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

**使用 C++17 连接和控制 AgileX 新车型。**

`agilex_ugv_sdk` 提供 CAN 通信、类型明确的控制指令和机器人状态反馈。
可以直接使用车型 API 接入底盘，也可以通过共享接口扩展其他 AgileX 车型。

[快速开始](#快速开始) · [支持车型](#支持车型) · [API 概览](#api-概览) ·
[协议参考](#协议参考) · [更新记录（英文）](CHANGELOG.md)

## 功能亮点

- 通过 SocketCAN 接入 SCOUT MINI、SCOUT MINI OMNI 和 RANGER MINI 3.0。
- 发送速度、控制模式、灯光和故障清除指令。
- 获取运动、电池、电机、遥控器和版本反馈。
- 支持 RANGER 转向模式、车轮数据和 BMS 反馈。
- 提供独立状态监视示例和无需机器人硬件的测试。

## 支持车型

| 系列 | 车型 | 状态 |
| --- | --- | --- |
| SCOUT | SCOUT MINI | 支持 |
| SCOUT | SCOUT MINI OMNI | 支持 |
| RANGER | RANGER MINI 3.0 | 支持，已完成离线测试 |

RANGER 已通过离线协议和 API 测试，尚未经实车验证。
执行运动前，应确认实际车型和固件的兼容性。

## 快速开始

需要 **Linux**、支持 **C++17** 的编译器和 **CMake 3.16+**。

```bash
git clone https://github.com/Hive-Matrix-AI/agilex_ugv_sdk.git
cd agilex_ugv_sdk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix "$HOME/.local"
```

在应用的 CMake 项目中链接已安装的 SDK：

```cmake
find_package(agilex_ugv_sdk 1 REQUIRED)
target_link_libraries(my_target PRIVATE agilex_ugv_sdk::agilex_ugv_sdk)
```

公共命名空间为 `agilex::ugv`。如果 CMake 无法找到 SDK，
配置应用时添加 `-DCMAKE_PREFIX_PATH="$HOME/.local"`。

<details>
<summary>构建选项</summary>

| 选项 | 默认值 | 用途 |
| --- | --- | --- |
| `BUILD_TESTING` | `ON` | 构建无需硬件的协议、机器人和传输测试 |
| `BUILD_EXAMPLES` | `OFF` | 构建独立状态监视示例 |

</details>

## 连接机器人

支持的车型均使用 **500 kbit/s** SocketCAN。将 `can0` 替换为实际适配器接口：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
./build/scout_mini_state can0
```

[状态监视示例](examples/scout/scout_mini_state.cpp)在收到反馈后打印电池电压与故障位，
不会启用运动。按 `Ctrl+C` 退出。

## API 概览

| 接口 | 用途 |
| --- | --- |
| `CanFrame` / `CanTransport` | CAN 帧和收发接口，内置 SocketCAN 后端 |
| `ProtocolCodec` / `ModelCapabilities` | 车型报文编解码与能力描述 |
| `Robot` | 组合编解码器和传输层，提供指令、状态和回调 |
| `ScoutMini` | SCOUT MINI 与 SCOUT MINI OMNI API |
| `RangerMiniV3` | RANGER MINI 3.0 运动模式、转向、驱动模式和 BMS API |

常用指令和反馈使用共享类型。`ModelCommand` 和 `ModelFeedback` 用于扩展车型专属数据。

<details>
<summary>目录结构</summary>

```text
include/agilex_ugv_sdk/
  core/                   指令、反馈、状态与机器人接口
  protocol/               编解码接口及结果类型
  transport/              CAN 帧、传输接口及后端
  models/scout/           SCOUT MINI API 与编解码器
  models/ranger/          RANGER MINI 3.0 API、类型与编解码器
src/
  core/                   共享机器人实现
  transport/              传输后端
  models/scout/           SCOUT MINI 实现
  models/ranger/          RANGER MINI 3.0 实现
tests/
  core/                   不依赖具体车型的测试
  models/scout/           SCOUT API 与协议测试
  models/ranger/          RANGER API 与协议测试
  support/                测试传输层与断言
examples/scout/           独立 SCOUT 示例
examples/ranger/          独立 RANGER 示例
docs/reference/models/    按车型系列组织的协议参考
```

每个车型系列的 API 和编解码器位于 `models/<family>/`，源码、测试、示例和协议文档使用对应目录。
各车型目录中的 CMake 文件将源码加入共享库目标。

新代码使用上述组件路径引用头文件。原 1.0 版本的头文件路径继续通过转发头文件保持兼容。

</details>

<details>
<summary>SCOUT MINI 方法与指令行为</summary>

包含 `agilex_ugv_sdk/models/scout/scout_mini.hpp`，并构造 `agilex::ugv::ScoutMini`。
默认车型为 `ScoutMiniVariant::skid_steer`；SCOUT MINI OMNI 使用 `ScoutMiniVariant::omni`。

| 方法 | 用途 |
| --- | --- |
| `connect("can0")` / `disconnect()` | 打开或关闭传输连接 |
| `state()` | 复制最新状态；各字段在收到对应反馈前为空 |
| `set_feedback_handler(handler)` | 在传输接收线程中处理已解码的反馈 |
| `request_version()` | 请求主控与驱动器版本 |
| `set_control_mode(ControlMode::can)` | 启用 CAN 指令控制模式 |
| `set_motion(command)` / `stop()` | 发送一帧速度或零速指令 |
| `set_lights(command)` | 设置前后灯模式 |
| `clear_error(motor)` | 清除全部故障（`0`）或电机故障（`1`–`4`） |

检查连接与指令方法返回的 `std::error_code`。超出车型限制的指令会被拒绝；
CAN ID 或 DLC 无效的反馈不会加入状态快照。

SCOUT MINI 运动前应切换到 CAN 控制模式，并每 20 ms 发送一次指令。
SDK 不会在后台重复发送。`stop()` 仅发送一帧零速指令，断开连接不会自动发送停止指令。
运动测试时必须确保物理急停触手可及。

反馈回调应快速返回，不能在接收线程中断开连接或销毁机器人对象。
单位、范围、报文格式和底盘控制器超时行为见
[SCOUT CAN 协议参考](docs/reference/models/scout/scout_mini_can.md)。

</details>

## RANGER MINI 3.0

包含 `agilex_ugv_sdk/models/ranger/ranger_mini_v3.hpp`，并构造 `agilex::ugv::RangerMiniV3`。
按上述步骤配置 500 kbit/s SocketCAN 后，运行 `./build/ranger_mini_v3_state can0` 监视反馈。

支持阿克曼、斜移、自旋、驻车四种模式，提供八个电机通道、车轮转角和速度、前后轮里程以及 BMS 反馈。
`set_motion(RangerMotionCommand{linear, steering, angular})` 的单位依次为 m/s、rad、rad/s。
调用 `set_motion_mode()` 后，应通过 `state().motion_mode` 确认控制器已完成切换。
只有斜移模式反馈确认切换完成后，才接受绝对值超过 0.698 rad 的转角；大转角时限速为 0.7 m/s。
`set_drive_mode()` 用于选择电流或电压驱动。

指令限制、报文格式、故障码和固件兼容性见
[RANGER CAN 协议参考](docs/reference/models/ranger/ranger_mini_v3_can.md)。

## 协议参考

- [SCOUT MINI / SCOUT MINI OMNI](docs/reference/models/scout/scout_mini_can.md)
- [RANGER MINI 3.0](docs/reference/models/ranger/ranger_mini_v3_can.md)

## 集成

SCOUT ROS 2 驱动和诊断工具在
[scout_ros2](https://github.com/Hive-Matrix-AI/scout_ros2) 仓库维护。

## 许可证

使用 [Apache-2.0](LICENSE) 许可证。源码保留现有版权与署名声明。

## 参与贡献

欢迎反馈问题、补充测试和增加 AgileX 车型支持。
贡献与测试流程见 [CONTRIBUTING.zh-CN.md](CONTRIBUTING.zh-CN.md)。
