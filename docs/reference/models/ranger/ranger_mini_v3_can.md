# RANGER MINI 3.0 CAN 协议与 API

适用车型：RANGER MINI 3.0。依据厂家《RANGER MINI 3.0 使用手册》
V1.0.0（2024.06）的 3.2 节，访问日期 2026-09-08：
[在线手册](https://agilexsupport.yuque.com/staff-hso6mo/rg519a/fqfo0alugiglumyw?singleDoc)。

同时参考 `ugv_sdk` 提交 `f2704eacdc90357078cd93ec60aae08bb4baab35` 中的
`ranger_base.hpp`、`ranger_interface.hpp`、`agilex_base.hpp`、
`agilex_protocol_v2.h` 和 `agilex_msg_parser_v2.c`。
手册有明确规定的字段以手册为准；版本查询是参考 SDK 的附加功能。
目前仅完成离线验证，尚未进行 Mini 3.0 实车及固件验证。

## 总线与调用方式

500 kbit/s，标准 11-bit CAN ID，多字节数值大端序，有符号字段为二进制补码。
`set_control_mode(ControlMode::can)` 进入 CAN 控制后，以 20 ms 周期发送运动指令；
手册给出的运动指令接收超时为 500 ms。SDK 不自动重发，也不自动清除故障。
遥控器优先级高于 CAN。模式切换期间底盘不响应速度控制指令。

```cpp
#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3.hpp"

agilex::ugv::RangerMiniV3 robot;
if (auto error = robot.connect("can0")) {
  // Handle the connection error before sending commands.
}
const auto snapshot = robot.state();
if (snapshot.motion && snapshot.motion->steering_angle_rad) {
  const double steering = *snapshot.motion->steering_angle_rad;
}
```

| 方法 | 行为 |
| --- | --- |
| `set_control_mode(mode)` | 待机或 CAN 模式 |
| `set_motion_mode(mode)` | 请求运动模式切换；需观察 `state().motion_mode` 确认完成 |
| `set_drive_mode(mode)` | 电流或电压驱动，反馈位于 `motion_mode.drive_mode` |
| `set_motion(RangerMotionCommand{linear, steering, angular})` | 参数依次为 m/s、rad、rad/s |
| `stop()` | 发送一帧全零运动命令；不会切换到驻车模式 |
| `set_lights(LightCommand{true, LightMode::on})` | 开灯；仅支持开/关，不支持独立后灯与亮度 |
| `clear_error(code)` | 发送下表的错误清除码 |
| `request_version()` | 按参考 SDK 查询版本字符串片段，异步接收 |
| `state()` | 原子复制通用状态及每种车型专属反馈的最新值 |
| `set_feedback_handler(handler)` | 接收线程上的反馈回调；可读取状态，不能在回调中断连或销毁对象 |

命令和连接方法返回 `std::error_code`，调用方需检查结果。断开连接不会发停止帧，
重新连接会清空全部状态。`state()` 各字段在首次收到对应报文前为空。

## 命令帧

| ID | DLC | 编码 |
| --- | ---: | --- |
| `0x111` | 8 | bytes 0–1：线速度；2–3：自旋角速度；4–5：零；6–7：内转角。均为 i16，分辨率 0.001 |
| `0x141` | 1 | 0：前后阿克曼；1：斜移/平行转向；2：自旋；3：驻车 |
| `0x421` | 1 | 0：待机；1：CAN 指令控制 |
| `0x423` | 1 | 0：电流驱动；1：电压驱动 |
| `0x441` | 1 | 故障清除码，见下表 |
| `0x121` | 8 | byte 0：使能；byte 1：关=0/开=1；bytes 2–7：零 |

`RangerMotionMode` 枚举依次为 `dual_ackermann`、`parallel`、`spinning`、`park`。
横移通过斜移模式和转角实现，不使用通用 `MotionCommand.lateral_velocity_mps`；
非零横向速度返回 `unsupported_command`。因此 `supports_lateral_motion` 为 false，
表示不接受这个通用命令字段，而非底盘不能横移。

| 参数 | 范围 |
| --- | --- |
| 线速度 | ±2.000 m/s；转角绝对值超过 20° 时为 ±0.700 m/s |
| 自旋角速度 | ±3.259 rad/s，逆时针为正 |
| 阿克曼内转角 | ±0.698 rad，左转为正 |
| 斜移转角 | ±1.571 rad，左转为正 |

编码前拒绝非有限值及超限值，按最接近的 0.001 单位量化；对量化后的转角再次
校验转弯限速，避免舍入绕过限速。codec 校验全局范围（最大转角 1.571 rad），
`RangerMiniV3::set_motion()` 根据反馈执行模式相关转角限制：尚未收到模式反馈、
切换中或非斜移模式均采用 0.698 rad。只有收到已完成切换的斜移反馈才允许更大转角。
直接组合 `Robot` 与 `RangerMiniV3Codec` 的调用方需自行执行同样的模式校验。

| 清除码 | 对象 |
| --- | --- |
| `0x00` | 全部非严重故障，含急停释放后的故障 |
| `0x01`–`0x08` | 电机 1–8 的驱动器通信故障 |
| `0x09` | 电池欠压，并尝试恢复动力供电 |
| `0x0A` | 遥控信号丢失 |
| `0x0B`–`0x0E` | 转向电机 5–8 的校准故障 |
| `0x0F` / `0x10` | 过流 / 过温 |

## 反馈帧

除 `0x291` 为 DLC 3、`0x362` 为 DLC 4 外，下表均为 DLC 8。
不符合长度的已知帧返回 `invalid_dlc`，未知标准帧忽略；超出标准 CAN 范围的
ID 或 DLC 返回 `invalid_frame`。保留字节不作为有效状态解码。

| ID | 状态字段及格式 |
| --- | --- |
| `0x211` | `system`：byte 0 车体状态、byte 1 控制模式、bytes 2–3 u16 ×0.1 V、bytes 4–7 u32 故障码；无计数器 |
| `0x221` | `motion`：i16 ×0.001 的线速度、自旋角速度、保留、内转角；`lateral_velocity_mps` 保持零 |
| `0x231` | `lights`：byte 0 使能、byte 1 开关、bytes 2–6 保留、byte 7 计数 |
| `0x241` | `remote_control`：byte 0 每 2 bit 一个开关；bytes 1–5 有符号摇杆/旋钮；byte 7 计数 |
| `0x251`–`0x258` | `actuators_high_speed`：i16 RPM、i16 ×0.1 A、i32 脉冲位置 |
| `0x261`–`0x268` | `actuators_low_speed`：u16 ×0.1 V、i16 驱动器温度 °C、i8 电机温度原始值、u8 驱动器状态；末 2 bytes 保留 |
| `0x271` | `motor_angles.angles_rad`：4 个 i16 ×0.001 rad，顺序为电机 5–8 |
| `0x281` | `motor_speeds.speeds_mps`：4 个 i16 ×0.001 m/s，顺序为电机 1–4 |
| `0x291` | `motion_mode`：byte 0 运动模式、byte 1 切换中标志、byte 2 驱动模式 |
| `0x311` | `odometry`：前轮左、右里程，各 i32 mm |
| `0x312` | `rear_odometry`：后轮左、右里程，各 i32 mm |
| `0x361` | `bms_basic`：SOC/SOH 各 u8 %、u16 ×0.1 V、i16 ×0.1 A、i16 ×0.1 °C |
| `0x362` | `bms_extended`：四个原始 u8，顺序为 alarm 1/2、warning 1/2 |

电机索引在 API 中为 1–8，数组下标为索引减一。手册注明 RANGM 系列电机温度
无效，`motor_temperature_c` 仅保留原始值，不应作为有效测温使用。

系统故障码完整保存为 `uint32_t`：bit 0 欠压、1 过压、2 遥控失联、3–6 电机
1–4 通信、7 急停、8 驱动器状态、10–13 电机 5–8 通信、14 过温、15 过流、
16–19 四轮转向零位校准、20 转向校准超时；其他位保留。
`SystemState.count` 在此车型中保持零，bytes 6–7 不会被误当成保留位和计数。

## 参考 SDK 的兼容功能与差异

手册未列版本查询。保留参考 `AgilexBase::RequestVersion()` 的实际请求：
`0x4A1`，DLC 1，数据 `01`。`0x4A1` DLC 8 接收为
`RangerVersionResponse.bytes`，快照仅保留最新片段；如需组装完整字符串，
使用反馈回调按设备固件约定收集片段。不按 SCOUT 的数值版本字段解释，
不使用旧解析器中未实现的 `0x411` 编码分支。

旧 SDK 将 Mini V3 直接映射为通用 RangerBase，因此其五种运动模式、DLC 8
配置命令、16-bit 故障码、2-byte 模式反馈、8-byte BMS 扩展反馈以及复杂灯效
不能直接视为本车型规范。本实现遵循上述 Mini 3.0 手册定义，并补上驱动模式
和后轮里程。Mini V1 的符号/角度转换与 Mini V2 的 BMS 电压额外缩放均不适用。

共享 `StateSnapshot` 的电机槽位扩展到 8，并保留每种 `ModelFeedback` 动态类型的
最新不可变对象；可用 `model_state<T>()` 读取。SCOUT 的线协议、4 电机能力及保留字节
处理保持原样。公共状态结构布局有扩展，下游 C++ 程序应重新编译。
