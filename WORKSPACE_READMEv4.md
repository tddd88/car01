# 智能导航避障与视觉识别机器人 - Cartographer全栈版

> **架构理念**：彻底抛弃 AMCL 和外部 EKF 融合，将**建图**与**比赛定位**全部交由 Google Cartographer 统一接管。利用其强大的 Ceres 扫描匹配内部消化麦轮打滑问题。
> **硬件平台**：树莓派 4B 8GB + 2D激光雷达 + STM32底盘控制板（麦轮+铅酸，无IMU）
> **软件环境**：Ubuntu 20.04 + ROS Noetic + C++ 

## 1. 极简全家桶架构

| 模块层级 | 功能包/包名 | 节点名称 | 核心功能描述 | 订阅话题 | 发布话题 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **底层与硬件** | `robot_bringup` | `stm32_bridge` | 串口通讯。STM32仅发送轮式里程计(`/odom_raw`)，无需IMU | `/cmd_vel` | `/odom_raw` |
| | | `lidar_driver` | 驱动2D激光雷达 | - | `/scan` |
| **SLAM与定位** | `cartographer_ros` | `cartographer_node` | **核心大脑**。<br>1. **建图模式**：赛前跑全套SLAM，生成 `.pbstream`。<br>2. **纯定位模式**：比赛时加载 `.pbstream`，依靠内部扫描匹配克服打滑，提供精准定位。 | `/scan`, `/odom_raw` | `/map`, `tf(odom->base_link)` |
| **导航规划** | `navigation` | `move_base` | 比赛时接收目标点，进行全局A*与局部TEB避障规划 | `/map`, `/scan` | `/cmd_vel` |
| **视觉与决策** | (待定) | (待定) | 比赛后期加入的绿纸识别与一键启动状态机 | `/rgb/image_raw`| `/cmd_vel_vision` |

---

## 2. 开发路线图 (Timeline)

### Phase 1: 串口打通 (当前首要任务)
* [ ] **底盘通信闭环**：完成 `stm32_bridge`。实车在键盘遥控下能顺滑移动，且能在 Rviz 中看到底盘发来的 `/odom_raw` (有打滑误差无所谓，保证数据连贯即可)。

### Phase 2: 啃下 Cartographer 建图
* [ ] **安装与配置**：`sudo apt install ros-noetic-cartographer-ros`。
* [ ] **编写 mapping.lua**：配置不使用 IMU (`use_imu_data = false`)，使用轮式里程计 (`use_odometry = true`)，发布 `odom -> base_link` TF。
* [ ] **实车建图**：在场地内遥控建图，测试其抵抗打滑的能力。完成建图后，执行指令将状态保存为 `.pbstream` 地图文件。

### Phase 3: 跑通 Cartographer 纯定位与导航
* [ ] **编写 localization.lua**：配置为纯定位模式。
* [ ] **结合 move_base**：启动纯定位模式和 `move_base`，在 Rviz 里用鼠标点发目标点（2D Nav Goal 测试用），看车子能否不撞墙、不迷失地开过去。

### Phase 4: 比赛专属逻辑与视觉介入
* [ ] 添加 USB 摄像头和 OpenCV 色块识别。
* [ ] 根据最终规则，将 Rviz 鼠标点目标替换为代码自动下发或视觉接管。



## 5. 导航定位方案选型备忘 (学长建议版)

**首选方案：Cartographer 2D 纯定位 (推荐)**
*   **机制**：比赛时启动 Cartographer 的 `Pure Localization` 模式，加载赛前生成的 `.pbstream` 地图。
*   **核心优势**：利用 Scan-to-Submap 匹配算法，对麦轮打滑的容忍度极高。相比 AMCL，它在初始定位和纠偏速度上更有优势，且不需要繁琐的粒子滤波调参。
*   **开发重点**：攻克 `.lua` 配置文件中的 `TRAJECTORY_BUILDER_2D.pure_localization = true` 相关参数设置。

**保底方案：AMCL (仅在算力崩溃时考虑)**
*   **机制**：经典的概率定位方案。
*   **局限**：对里程计精度要求较高，在瓷砖打滑环境下容易出现粒子发散，前期调试成本（参数校准）高于 Cartographer。