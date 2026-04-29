# 项目更新日志 (CHANGELOG)

## [2026-04-27] 基于 READMEv4 的架构升级
- **定位方案调整**: 确定以 Google Cartographer 为核心定位方案，移除之前计划的 rf2o + AMCL 组合。
- **底层通讯精简**: `stm32_bridge` 维持 V3 版本的 10 字节精简协议（无 IMU），专供轮式里程计数据给 Cartographer 处理。
- **TF 权限收回**: 明确 `stm32_bridge` 默认不发布 `odom -> base_link` 的 TF，该权限完全移交给 Cartographer 节点。
- **环境安装**: 准备安装 `ros-noetic-cartographer-ros`。

## [2026-04-28] URDF 整合与 TF 梳理
- **模型加载与 TF 结合**: 去掉了原来在 `bringup.launch` 的手动静态 TF（`base_to_laser`）。引入 SolidWorks 导出的 `simulation.urdf` 模型。
- **状态发布器**: `bringup.launch` 中集成拉起了 `robot_state_publisher` 与 `joint_state_publisher`。开机启动就会自动将 URDF 中车体外壳、四个麦轮、相机支架和雷达相对 `base_link` 的尺寸作为静态坐标系广播出去。
- **防冲突检查**: 验证了 URDF 的最底层父节点为 `base_link`，且只发布 `base_link` 之下的子代 TF，绝不会上犯 `odom`；这与 `stm32_bridge`（锁死 odom 发布）及后续 Cartographer（独占拥有发布 odom->base_link 控制权）形成了无缝的完美金字塔结构。

## [2026-04-29] Cartographer V4 配置文件部署
- **增加建图与纯定位脚本**: 在 `robot_bringup/config` 目录下新建了 `mapping.lua`（建图用）和 `localization.lua`（比赛定位用）。为了适配只有轮式里程计且无IMU的STM32，强制启用了 `use_odometry = true` 且 `use_imu_data = false`。
- **配置 TF 霸权**: Lua 文件中指定了 `published_frame = "base_link"` 和 `provide_odom_frame = true`。Cartographer 成为 `map -> odom` 和 `odom -> base_link` 关系链的独裁者，彻底消灭可能导致乱飞的冲突。
- **launch文件集成升级**: 修改了 `bringup.launch` 以便通过 `is_mapping:=true/false` 动态切换建图还是导航，并且包含了将 Cartographer 数据转换为常规 SLAM 图象的 `cartographer_occupancy_grid_node`。
