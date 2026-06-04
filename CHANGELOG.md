# 更新日志 (Changelog)
[2026-04-27] 基于 READMEv4 的架构升级

    定位方案调整: 确定以 Google Cartographer 为核心定位方案，移除之前计划的 rf2o + AMCL 组合。
    底层通讯精简: stm32_bridge 维持 V3 版本的 10 字节精简协议（无 IMU），专供轮式里程计数据给 Cartographer 处理。
    TF 权限收回: 明确 stm32_bridge 默认不发布 odom -> base_link 的 TF，该权限完全移交给 Cartographer 节点。
    环境安装: 准备安装 ros-noetic-cartographer-ros。

[2026-04-28] URDF 整合与 TF 梳理

    模型加载与 TF 结合: 去掉了原来在 bringup.launch 的手动静态 TF（base_to_laser）。引入 SolidWorks 导出的 simulation.urdf 模型。
    状态发布器: bringup.launch 中集成拉起了 robot_state_publisher 与 joint_state_publisher。开机启动就会自动将 URDF 中车体外壳、四个麦轮、相机支架和雷达相对 base_link 的尺寸作为静态坐标系广播出去。
    防冲突检查: 验证了 URDF 的最底层父节点为 base_link，且只发布 base_link 之下的子代 TF，绝不会上犯 odom；这与 stm32_bridge（锁死 odom 发布）及后续 Cartographer（独占拥有发布 odom->base_link 控制权）闭环。

[2026-04-29] Cartographer V4 配置文件部署

    增加建图与纯定位脚本: 在 robot_bringup/config 目录下新建了 mapping.lua（建图用）和 localization.lua（比赛定位用）。为了适配只有轮式里程计且无IMU的STM32，强制启用了 use_odometry = true 且 use_imu_data = false。
    配置 TF: Lua 文件中指定了 published_frame = "base_link" 和 provide_odom_frame = true。Cartographer 成为 map -> odom 和 odom -> base_link 关系链的独裁者，彻底消灭可能导致乱飞的冲突。
    launch文件集成升级: 修改了 bringup.launch 以便通过 is_mapping:=true/false 动态切换建图还是导航，并且包含了将 Cartographer 数据转换为常规 SLAM 图象的 cartographer_occupancy_grid_node。

## [V4.0] - 之前版本
### 完成项
- 实现了 Cartographer 纯定位与建图的稳定运行。
- 完成了逻辑上的底盘串口协议 C++ 重写 (stm32_bridge)。
- 优化了 URDF 模型，消除了冗余的 TF 警告。
- 实现了手柄一键自动保存地图 (.pbstream) 脚本。

## [V4.1] - 2026-06-04
### 新增
- **导航模块 (Navigation)**: 针对麦克纳姆轮配置了 `move_base` 的 YAML 参数。创建了 `costmap_common_params.yaml`、`global_costmap_params.yaml`、`local_costmap_params.yaml` 和 `teb_local_planner_params.yaml`。将 `move_base` 输出的速度重映射至 `/cmd_vel_nav`，以便状态机统一调度。
- **视觉控制 (Vision Control)**: 升级了 `blue_detector_node`，采用动态且宽容的 HSV 阈值进行蓝色目标追踪。添加了形态学开闭运算（Open/Close）逻辑，用于消除噪点并填补蓝色块空洞。实现了目标中心点 (X,Y) 计算与面积占比估算。新增 `/rgb/blue_target_debug_img` 话题，在画面中实时覆盖绿色外接矩形和红色形心点。
- **决策决策机 (Decision State Machine)**: 使用 C++ 开发了 `race_state_machine_node` 逻辑架构。实现了导航速度与视觉伺服速度的多路复用和优先级切换逻辑。
- **全局架构**: 修复了所有 `CMakeLists.txt` 依赖冲突。成功通过 `catkin_make` 编译 `decision`、`vision_control` 和 `navigation` 三个 C++ 功能包。同步更新了项目 README 架构说明。

### 变更
- **避障优化**: 将局部代价图膨胀半径设为 `0.2`（方便钻小缝隙），全局代价图膨胀半径设为 `0.45`（确保全局路径更稳健）。
- **逻辑鲁棒性**: 在 C++ 代码中为 `SEARCHING_BLIND` 模式（盲搜逻辑）增加了变量作用域保护，确保目标丢失与重新锁定时的平滑过渡。

---

