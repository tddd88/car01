# 智能导航避障与视觉识别机器人 - 项目开发文档

此处是raspi端，本文档也是对REQUSET.md里面问题的回答

> **赛事目标**：自主导航、避障、利用视觉识别终点绿纸并停车（误差<0.8m区域）。
> **硬件平台**：树莓派 4B 8GB + 2D激光雷达 + 单目USB摄像头 + STM32底盘控制板（麦轮+铅酸）+ IMU
> **软件环境**：Ubuntu 20.04 + ROS Noetic + C++ / Python

## 1. 优化后的全栈系统架构

| 模块层级 | 功能包/包名 | 节点名称 | 核心功能描述 | 订阅话题 | 发布话题 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **硬件驱动层** | `robot_bringup` | `lidar_driver` | 驱动2D激光雷达 | - | `/scan` |
| | | `camera_driver` | 驱动USB单目摄像头 | - | `/rgb/image_raw` |
| | | `stm32_bridge` | 串口通讯解析，双向传输底盘控制流与传感器状态 | `/cmd_vel_mux` | `/odom_raw`, `/imu/data` |
| | `robot_localization`| `ekf_se` | **扩展卡尔曼滤波：融合轮式里程计与IMU，应对麦轮打滑** | `/odom_raw`, `/imu/data` | `/odom` |
| **感知与建图层**| `slam_gmapping` | `gmapping` | 赛前构建全局地图（只在建图模式启动） | `/scan`, `/tf` | `/map` |
| | `vision_control` | `color_detector` | OpenCV处理图像：HSV滤波+轮廓分析检测绿纸。计算偏差并输出趋近速度，或者达到要求输出停车标志。 | `/rgb/image_raw` | `/stop_flag`, `/cmd_vel_vision` |
| **规划导航层** | `navigation` | `amcl` | 基于已知地图的蒙特卡洛粒子滤波定位 | `/scan`, `/map`, `/odom` | `/amcl_pose` |
| | | `move_base` | 全局路径规划(A*/Dijkstra)与局部避障(TEB/DWA) | `/map`, `/scan`, `/odom` | `/cmd_vel_nav` |
| **决策控制层** | `twist_mux` | `twist_mux` | **速度多路复用器：决策底层速度权限。优先级：急停 > 视觉伺服速度 > 导航规划速度 > 遥控** | `/cmd_vel_nav`, `/cmd_vel_vision` | `/cmd_vel_mux` |
| | `decision_logic` | `race_state_machine`| 核心状态机：控制机器人在各个阶段的行为逻辑。 | `/stop_flag`, `/start_signal` | `/move_base_simple/goal` (或Action) |

---

## 2. 核心控制逻辑：状态机设计 (race_state_machine)

为了规避“禁止直接发送死板目标点”，并实现自动寻迹，状态机设计如下4个状态：

1. **STATE_WAIT (等待启动)**：
   - 系统上电，Launch全部启动完成。
   - 监听 `/start_signal` (例如键盘回车发送Empty消息，模拟一键启动)。收到信号后跳转至 `STATE_SEARCH`。
   - 记录起始时间用于内部计时。

2. **STATE_SEARCH (全局搜索/巡航)**：
   - 机器人加载地图后，由状态机向 `move_base` 依次下发场内几个**预设的巡航观察点**（这不是终点，而是视野开阔的检查点），或者调用前沿探索算法。
   - 在此过程中，视觉节点持续运行。若 `color_detector` 连续多帧检测到有效绿块，打断 `move_base` (Cancel Goal)，跳转至 `STATE_APPROACH`。

3. **STATE_APPROACH (视觉伺服趋近)**：
   - `move_base` 暂停工作。视觉节点根据绿色色块的中心与画面中心($X$ 轴)的像素偏差，通过PID计算出角速度($V_{th}$)；根据绿色矩形面积计算线速度($V_{x}$)。
   - 视觉节点将速度发送至 `/cmd_vel_vision`。`twist_mux` 自动接管并传给底盘。
   - 当视觉节点发现绿纸面积达到阈值，且位于画面底部时，发布 `/stop_flag` = True。跳转至 `STATE_STOP`。

4. **STATE_STOP (任务完成)**：
   - 向底盘发送持续的零速度指令：`vx=0, vy=0, vth=0`。
   - 输出成功日志，完成3秒的静止停留，任务结束。

---

## 3. 开发阶段规划 (Timeline: 4月中旬 ~ 12月)

### Phase 1: 仿真与基建验证 (4月 - 5月) [在PC上完成]
* [ ] **构建仿真环境**：写一个简单的 URDF 机器人模型（包含雷达和单目相机），在 Gazebo 中搭建一个简易围墙+绿色方块的场景。
* [ ] **跑通导航栈**：在仿真中调通 Gmapping、AMCL 和 Move_base。
* [ ] **跑通视觉与状态机**：写出 OpenCV 识别绿色的节点，以及上述的 4 阶段状态机。实现仿真里的自动寻绿纸停车。

### Phase 2: 底层硬件与通信闭环 (6月 - 7月) [PC联调实车底层]
* [ ] **STM32控制**：机械组装完毕（注意重心居中）。STM32完成单轮闭环PID，麦轮逆运动学解算。
* [ ] **ROS通信桥**：完成 `stm32_bridge.cpp`，STM32能够接收 `/cmd_vel` 走直线、���移、自转；STM32能够向上发送准确的编码器数据和IMU数据。
* [ ] **EKF滤波引入**：配置 `robot_localization` 节点，融合 Odom 和 IMU，在 Rviz 中观察原地旋转时 TF 树是否平滑、无漂移。

### Phase 3: 传感器上车与真实环境适配 (8月 - 9月) [全面转入树莓派]
* [ ] **环境迁移**：将所有代码编译至 Raspberry Pi 4B。
* [ ] **雷达建图实测**：在真实走廊或室内环境使用遥控器建图，保存地图。
* [ ] **视觉鲁棒性测试**：在树莓派上跑 OpenCV，测试不同光照条件下的 HSV 阈值提取。加入形态学处理抗干扰。

### Phase 4: 联调与参数精修 (10月 - 11月) [实车整体测试]
* [ ] **打滑补偿**：针对室内瓷砖，通过限制 `move_base` 中 `teb_local_planner` 的最大加速度 (`acc_lim_x`, `acc_lim_theta`) 来减少起步打滑。
* [ ] **状态机真车测试**：测试一键启动 -> 巡逻 -> 视觉接管 -> 稳停在80cm框内的全流程。
* [ ] **代码优化与容错**：如果机器人被撞偏或者短时间丢失视野，状态机需要有回退机制（从 `APPROACH` 退回到 `SEARCH` 重新寻找）。

### Phase 5: 赛前模拟与答辩准备 (12月)
* [ ] 在组委会开放场地进行至少3次全真模拟。
* [ ] 记录耗时，调整导航速度上限争取时间分。
* [ ] 准备创新性说明文档（重点突出 EKF多传感器融合防打滑、视觉伺服动态接管、无固定航点的状态机策略）。

---

## 4. 关键避坑指南 (Tips)

1. **时间戳同步**：由于你用了多个传感器，确保树莓派的系统时间准确，所有节点的Header时间戳必须使用 `ros::Time::now()`，否则 TF 会疯狂报错。
2. **USB相机延迟**：树莓派处理高分辨率图像会卡顿。建议将相机分辨率调低到 `640x480` 甚至 `320x240`，帧率控制在 15-20 fps 即可满足寻纸需求。
3. **TF树结构**：确保你的 TF 树清晰：`map -> odom -> base_link -> laser/camera`。其中 `map->odom` 由 AMCL 发布，`odom->base_link` 由 EKF(robot_localization) 发布。
4. **TEB vs DWA**：麦轮建议使用 `teb_local_planner`，它支持全向轮（holonomic）模式，可以在避障时直接侧移（利用麦轮优势），这会大幅缩短避障时间。