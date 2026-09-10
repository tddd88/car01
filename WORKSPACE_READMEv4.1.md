# 智能导航避障与视觉识别机器人 - Cartographer 全栈版 (V4.1)

> **架构理念**：摒弃 AMCL，由 Cartographer 统一接管建图与纯定位防滑。同时在 V4.1 版本中，引入了基于“蓝色视觉靶标”伺服与全向底盘（麦轮）的鲁棒决策方案，彻底打通自动导航到视觉接管的比赛闭环。
> **硬件平台**：树莓派 4B 8GB + 2D激光雷达 + STM32底盘控制板（麦轮+铅酸，无IMU）+ USB 摄像头 + 2.4G手柄
> **软件环境**：Ubuntu 20.04 + ROS Noetic + C++ / Python

---

## 1. 极简全家桶架构 (无渲染阅读友好版)

================================================================================
【模块 1：底层硬件与通讯 (robot_bringup, lidar_driver)】
================================================================================
◆ 功能包: robot_bringup
  ├─ 节点: stm32_bridge
  │  ├─ 功能: 串口通讯桥接。将 STM32 发来的轮速解析为里程计，并下发控制速度
  │  ├─ 订阅: /cmd_vel (geometry_msgs/Twist)
  │  └─ 发布: /odom_raw (nav_msgs/Odometry)
  │
  ├─ 节点: joystick & teleop_twist_joy
  │  ├─ 功能: 2.4G 摇杆死人键(R1)操作与差速/全向速度换算
  │  ├─ 订阅: /joy (sensor_msgs/Joy)
  │  └─ 发布: /cmd_vel_joy (geometry_msgs/Twist)
  │
  └─ 节点: usb_cam
     ├─ 功能: 驱动 USB 摄像头并调整像素格式
     ├─ 订阅: 无
     └─ 发布: /rgb/image_raw (sensor_msgs/Image)

◆ 功能包: robot_bringup - 常用 launch 一键入口
  ├─ stm32.launch                → 调硬件 + 手柄遥控（桥+手柄，不开雷达/SLAM）
  ├─ bringup.launch              → 传感器全开但不建图/不定位（雷达+摄像头+TF，排查用）
  ├─ bringup_mapping.launch      → 比赛建图（硬件+手柄+建图+自动存图+手柄B键标点）
  ├─ bringup_localization.launch → 比赛一键启动（硬件+定位+move_base+视觉+决策状态机）
  └─ bringup_auto.launch         → systemd 开机自启精简版（桥+手柄，省电）

◆ 功能包: lidar (外部引入)
  └─ 节点: lidar_scan_node
     ├─ 功能: 驱动 2D 激光雷达
     └─ 发布: /scan (sensor_msgs/LaserScan)


================================================================================
【模块 2：SLAM 与实体状态模型 (robot_slam, simulation)】
================================================================================
◆ 功能包: robot_slam
  └─ 节点: cartographer_node
     ├─ 功能: 【核心大脑】赛前跑扫描图匹配建图生成 pbstream；赛时纯定位抗打滑
     ├─ 订阅: /scan, /odom_raw
     └─ 发布: /map, tf(odom->base_link)

◆ 功能包: simulation
  └─ 节点: robot_state_publisher
     ├─ 功能: 根据 SolidWorks 导出的 URDF 发布机械和传感器的刚性连接关系
     └─ 发布: /tf_static (base_link -> radar_cylinder / camera / wheels)


================================================================================
【模块 3：导航规划调度 (navigation - 待细化)】
================================================================================
◆ 功能包: navigation
  └─ 节点: move_base (集成 TEB Local Planner)
     ├─ 功能: 比赛时接收目标点，进行全局 A* 与局部全向避障规划
     ├─ 订阅: /map, /scan, /move_base_simple/goal
     └─ 发布: /cmd_vel_nav (geometry_msgs/Twist)


================================================================================
【模块 4：决策与视觉伺服 (decision, vision_control - V4.1 新增规划)】
================================================================================
◆ 功能包: vision_control
  ├─ 节点: blue_detector
  │  ├─ 功能: 基于 OpenCV 读取图像，使用宽容HSV+开闭运算提取蓝色矩形靶标，发布占比坐标。供调试查看
  │  ├─ 订阅: /rgb/image_raw
  │  └─ 发布: 
  │     ├─ /blue_target (自定义目标消息或 Pose)
  │     └─ /rgb/blue_target_debug_img (带有绿框与红点标注的视觉画面，用于调试)
  │
  └─ 节点: visual_servo
     ├─ 功能: 接收到蓝纸目标后，利用 PID 算法转化为底盘贴靠速度（全向横移/直行）
     ├─ 订阅: /blue_target
     └─ 发布: /cmd_vel_vision (geometry_msgs/Twist)

◆ 功能包: decision
  ├─ 节点: waypoint_saver (辅助)
  │  ├─ 功能: 建图时，通过手柄 B 键(2号按键)瞬间提取 /map 与 /base_link 关系并保存为 yaml，
  │  │       同时发布 latched 话题 /vision_start_point (PoseStamped) 供状态机发起导航
  │  ├─ 订阅: /joy
  │  └─ 发布: 保存 yaml 配置文件 + /vision_start_point (latched)
  │
  └─ 节点: race_state_machine
     ├─ 功能: 【比赛总控核心】串联整套业务的状态机
     ├─ 状态流转详见下方 4.1 详细说明
     ├─ 订阅: /blue_target, /cmd_vel_nav, /cmd_vel_vision, /scan, /vision_start_point
     └─ 发布: /cmd_vel, ActionClient -> move_base

---

## 4.1 核心状态流与极端情况处理规划 (V4.1 备忘)

**业务逻辑构想）：**
0. [IDLE 等待]: 状态机订阅 /vision_start_point (latched)。等待收到预备点且 move_base 就绪。
1. [起步导航]: 收到预备点后自动向 move_base 发送目标，让底盘自主避障前往预备点。
2. [视觉抢占]: 去预备点途中，如果 `/blue_target` 持续发出有效的蓝色视野框，直接打断导航（取消 Action目标），进入 [视觉伺服] 状态。
3. **[极端情况补救（盲区搜索模式）]**: 如果目标丢失，进入盲区横向搜索。
    * 使用**麦克纳姆轮底盘**横向平移（像螃蟹一样左右来回），当前 V4.1 采用 `V_y=±0.2` 往复。
    * **激光雷达辅助避障**：订阅 /scan，检测横移方向的侧向墙距，当距挡板 < 30cm (search_wall_margin) 时立即反向，避免撞墙。
    * 一旦在平移过程中，摄像头捕捉到了蓝色：立刻刹停平移，转入 [视觉伺服]。
    * 盲搜超时（search_max_time）兜底强制停止，防止无限横移。
4. [视觉伺服]: 对接视觉信息。X轴（直线）控制与蓝纸距离，Y轴（平移）+ 偏航修正保证蓝纸在画面中心，直至蓝纸占比超过预设阈值。下发速度 0 停车。
