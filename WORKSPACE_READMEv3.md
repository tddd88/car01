# 智能导航避障与视觉识别机器人 - 实车开发文档 (雷达里程计抗滑版)

> **当前阶段核心目标**：针对麦轮打滑且低成本IMU效果差的问题，引入**雷达里程计(Laser Odometry)**与轮式里程计进行EKF融合，彻底解决打滑导致的坐标漂移。建图算法升级为抗打滑能力更强的 **Cartographer**。
> **硬件平台**：树莓派 4B 8GB + 2D激光雷达 + STM32底盘控制板（麦轮+铅酸）
> **软件环境**：Ubuntu 20.04 + ROS Noetic + C++ 

## 1. 优化后的抗滑架构 (无视烂IMU)

| 模块层级 | 功能包/包名 | 节点名称 | 核心功能描述 | 订阅话题 | 发布话题 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **底层通信** | `robot_bringup` | `stm32_bridge` | 串口通讯解析，双向传输控制流。STM32只需发送纯编码器解算的里程计 | `/cmd_vel` | `/odom_raw` |
| **硬件驱动** | `robot_bringup` | `lidar_driver` | 驱动2D激光雷达，提供点云数据 | - | `/scan` |
| **雷达里程** | `rf2o_laser_odometry` | `rf2o` | **新增抗滑核心**。通过前后帧点云匹配(Scan Matching)直接计算平面位移与旋转，完全免疫车轮打滑 | `/scan` | `/odom_rf2o` |
| **多源融合** | `robot_localization`| `ekf_se` | 融合轮式里程计(`/odom_raw`)与雷达里程计(`/odom_rf2o`)。抛弃劣质IMU，输出高精度平滑的里程计 | `/odom_raw`, `/odom_rf2o` | `/odom` |
| **高级建图** | `cartographer_ros` | `cartographer_node` | **取代Gmapping**。利用其强大的后端优化和子图匹配，在极易打滑的瓷砖地面构建高精地图 | `/scan`, `/odom` | `/map` |
| **基础控制** | `teleop_twist_keyboard` | `teleop` | 键盘遥控底盘，验证正逆运动学与通讯延迟 | - | `/cmd_vel` |

---

## 2. 开发路线图 (Timeline: 攻克打滑与建图)

### Phase 1: 树莓派与STM32串口打通 (当前首要任务)
* [ ] **协议精简**：剔除原协议中需要 STM32 发送的 IMU 数据位。STM32 专心通过编码器计算 $v_x, v_y, v_{th}$ 并发送。
* [ ] **下发与接收测试**：编写 `stm32_bridge.cpp`，实现 `/cmd_vel` 的下发和 `/odom_raw` 的解析发布。

### Phase 2: 雷达里程计与 EKF 融合 (解决打滑痛点)
* [ ] **引入 rf2o**：编译安装 `rf2o_laser_odometry` 包。在静止和遥控移动时，观察其输出的 `/odom_rf2o` 是否合理。
* [ ] **配置 EKF 融合**：
  * 将 `/odom_raw` (STM32轮式) 设为主里程计（提供高频连续性，但有累积误差）。
  * 将 `/odom_rf2o` (雷达匹配) 设为绝对参考（无打滑误差，但特征稀疏时可能跳变）。
  * 调整 `robot_localization` 的协方差矩阵，让系统更信任雷达的转向角度 (Yaw) 和平移。
* [ ] **验证效果**：在打滑的瓷砖上猛打方向、急刹车，Rviz 中的 `base_link` 应该始终与真实车体朝向完美一致。

### Phase 3: Cartographer 上车与高精建图
* [ ] **编写 Lua 配置**：为 Cartographer 编写配置文件。关闭 IMU 输入 (`use_imu_data = false`)，开启里程计输入 (`use_odometry = true`)。
* [ ] **实地建图**：在比赛场地或类似走廊环境中遥控建图。Cartographer 的闭环检测(Loop Closure) 会自动拉平因为漂移产生的畸变。
* [ ] **保存地图**：使用 `map_saver` 保存供后续 AMCL 导航使用。

### Phase 4: 导航与视觉预研 (待比赛规则最终确认)
* [ ] 跑通基于完善后 `/odom` 的 `move_base` 和 `amcl`。
* [ ] 准备 OpenCV 节点识别终点绿纸，利用像素坐标准备视觉伺服策略。