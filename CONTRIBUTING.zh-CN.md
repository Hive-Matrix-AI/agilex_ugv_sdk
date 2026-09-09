# 参与贡献

[English](CONTRIBUTING.md) | [简体中文](CONTRIBUTING.zh-CN.md)

## 反馈问题

请在 [GitHub Issues](https://github.com/Hive-Matrix-AI/agilex_ugv_sdk/issues) 中提供
SDK 提交版本、编译器、操作系统、车型、复现步骤、预期结果，以及相关错误码或 CAN 帧。
发布日志前移除凭据、个人信息和私有网络信息。不要为收集诊断数据重复危险运动。

## 提交修改

Pull Request 请提交到 `main` 分支。保持修改范围明确，保留现有许可证和署名。
行为变更应增加测试，接口变更应同步更新公开 API 文档。
同步维护中英文指南，尤其是支持车型、指令单位与限制、示例和安全要求。

共享核心、协议接口和传输层应独立于具体车型系列。
车型专属 API 和编解码器放在 `models/<family>/` 下，并提供对应测试、示例和协议参考。
ROS 集成应放在各车型系列的 ROS 仓库中，不加入本 SDK。

无需硬件即可构建和测试：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

回归测试使用可注入的 `CanTransport`。公共头文件应能独立包含，
并保持导出的 `agilex_ugv_sdk::agilex_ugv_sdk` CMake 目标兼容。
C++ 修改使用仓库中的 `.clang-format` 配置格式化。

新增车型需提供协议来源、单位和范围、能力描述以及编码和解码测试。
实车验证应说明车型、固件版本，并与离线测试结果明确区分。
