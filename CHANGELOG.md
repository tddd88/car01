# 项目更新日志 (CHANGELOG)

## [2026-04-28] 基于 READMEv4 的架构升级
- **定位方案调整**: 确定以 Google Cartographer 为核心定位方案，移除之前计划的 rf2o + AMCL 组合。
- **底层通讯精简**: `stm32_bridge` 维持 V3 版本的 10 字节精简协议（无 IMU），专供轮式里程计数据给 Cartographer 处理。
- **TF 权限收回**: 明确 `stm32_bridge` 默认不发布 `odom -> base_link` 的 TF，该权限完全移交给 Cartographer 节点。
- **环境安装**: 准备安装 `ros-noetic-cartographer-ros`。
