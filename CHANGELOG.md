# 更新日志 (Changelog)

[2026-09-08] 麦轮横移漂移补偿（本轮）

    新增 chassis_corrector_node.cpp:
        * 用 STM32 编码器实测 /odom_raw 反馈，对 vx(前后漂) 和 vth(偏航) 做闭环补偿。
        * vy(横移指令) 不补偿，避免正反馈震荡。
        * 一阶低通滤波编码器反馈，消除噪声。
        * 死区设计：误差小于阈值不补偿，避免静止抖动。
        * 仅在横移时激活(strafe_activate)，纯前进/旋转不干预。
        * 下发规则与 joy_repeat_node 一致：非零持续发，零只发一次。
    joystick.launch 链路更新:
        * joy_repeat_node 输出改为 cmd_vel_raw -> chassis_corrector -> cmd_vel -> stm32_bridge。
        * chassis_corrector 参数全部可调(kp_vx/kp_vth/deadband/filter_alpha/strafe_activate)。
    编译验证: robot_bringup 包 catkin_make 通过，chassis_corrector_node 链接成功。

[2026-09-02] 修复手柄回中后仍持续运动

    joy_repeat_node.cpp:
        * 修复松开使能键或摇杆回中后只清零缓存、不发布零速的问题。
        * 恢复原有行为：非零速度按 autorepeat_rate（默认20Hz）持续发布。
        * 从非零变为零时只发送一次停车指令，之后不持续刷零速。
        * 手柄回中/松开使能键后，零速立即经 cmd_vel_raw -> laser_safety_node -> cmd_vel 发送给底盘。
    laser_safety_node.cpp:
        * 保留 0.5 秒输入超时作为手柄/通信断链兜底，不再依赖该延时实现正常停车。
        * 雷达避障仍只抑制朝向障碍的对应线速度分量，前后安全距离0.20m、左右安全距离0.15m。
    验证: catkin_make --pkg robot_bringup 通过；重启 bringup_auto.service 后无手柄操作时 /cmd_vel 持续输出全零。

[2026-09-02] 开机自启加入激光安全避障（容错降级）（本轮）

    bringup_auto.launch 升级为开机自启版"桥+手柄+雷达+安全避障":
        * 新增 include lidar_scan.launch（雷达驱动）。
        * 新增 include joystick.launch（cmd_vel_topic=cmd_vel_raw）。
        * 新增 laser_safety_node（订阅 cmd_vel_raw + /scan，过滤后发 cmd_vel）。
        * 容错设计: 雷达节点 respawn=true（接触不好自动重连）；laser_safety_node 雷达无数据时透传 cmd_vel_raw -> cmd_vel，退化为纯遥控。
        * 主链路（桥+手柄）不依赖雷达，雷达失败不影响开机自启与遥控。
    lidar_scan.launch (Lidar_ROS1_Driver):
        * 雷达节点加 respawn="true" respawn_delay="3"，掉线/接触不好时自动重连。
    关于"雷达失败是否导致开不了机": 不会。roslaunch 主进程不因单节点失败退出；laser_safety_node 内部容错透传；systemd 不会触发重启循环。
    launch 语法校验: roslaunch --files / --dump-params 通过。

[2026-09-02] bringup 升级为全传感器+遥控+激光安全避障入口（本轮）

    bringup.launch 重构为"全传感器驱动 + 手柄遥控 + 激光安全避障"完整入口:
        * 新增 include joystick.launch（手柄输出到 cmd_vel_raw，交由安全节点过滤）。
        * 新增 laser_safety_node 节点（订阅 cmd_vel_raw + /scan，过滤后发 cmd_vel）。
        * 相比 stm32.launch（仅桥+手柄）更全：桥+手柄+雷达+摄像头+TF+安全避障。
    新增 laser_safety_node.cpp (robot_bringup/src):
        * 话题链路: joy_repeat_node -> cmd_vel_raw -> [laser_safety_node] -> cmd_vel -> stm32_bridge。
        * 将 360° 扫描按机器人坐标系分四象限（前/后/左/右），取每象限最小距离。
        * 前后安全距离 0.20m，左右安全距离 0.15m（可参数配置）。
        * 线性减速带: 距离在 [safe, safe*slow_factor(默认2.0)] 之间线性缩放速度，<safe 完全抑制该方向分量。
        * 只抑制朝向障碍的速度分量，远离障碍方向不受影响；旋转不限制（原地转不撞墙）。
        * 无 cmd_vel_raw 输入时不发 cmd_vel，不干扰决策状态机（人工优先，自动让权）。
    joystick.launch 参数化输出话题:
        * 新增 cmd_vel_topic 参数（默认 cmd_vel；bringup.launch 传入 cmd_vel_raw）。
        * stm32.launch 仍用默认 cmd_vel（纯调试，不过滤）。
    bringup_mapping.launch / bringup_localization.launch:
        * 去掉重复的 joystick.launch include（bringup.launch 已包含手柄）。
    编译验证: catkin_make --pkg robot_bringup 全部通过，laser_safety_node / joy_repeat_node / stm32_bridge 三节点均生成。

[2026-09-02] 手柄适配麦轮全向移动与持续发布（前一轮）

    joy_repeat_node.cpp (robot_bringup/src) 新增/重写:
        * 替代 teleop_twist_joy (0.1.3 不支持 autorepeat_rate)，自实现持续发送版手柄控制节点。
        * 麦轮全向移动映射: 左摇杆前后 -> vx, 左摇杆左右 -> vy (横向平移), 右摇杆左右 -> vth (原地转向)。
        * 使能键 R1 (Button 7) 按住时按 autorepeat_rate (默认 20Hz) 持续下发速度；松开立即停车。
        * 仅当有非零速度输入时才持续发送，速度为 0 时不发，避免无意义刷屏。
        * 参数化: enable_button / axis_vx / axis_vy / axis_vth / scale_vx / scale_vy / scale_vth / autorepeat_rate。
    joystick.launch 配套:
        * 启动 joy_node + joy_repeat_node，配置轴映射与速度限制 (scale_vx=0.8, scale_vy=0.5, scale_vth=0.5)。
    CMakeLists.txt:
        * 新增 add_executable(joy_repeat_node ...) 编译目标。

[2026-06-04] launch 一键入口整理（本轮）

    新建 stm32_bridge.launch: 纯 STM32 桥（组件，供 include，不带手柄）。
    stm32.launch 改版: 调硬件 + 手柄遥控入口（桥 + 手柄）。
    bringup.launch 改为 include stm32_bridge.launch: 保持传感器全开但不起图，且不带入手柄。
    bringup_auto.launch 改 include stm32_bridge.launch: systemd 自启精简（桥+手柄）不重复。
    bringup_localization.launch 升级为比赛一键启动: bringup + joystick + 纯定位 + move_base + vision + decision(race)。
    waypoints_saver.py 增强: 启动时自动读取已存 vision_point.yaml 并发布 latched /vision_start_point，
        使比赛一键启动时无需再按手柄 B 键即可加载预备点。
    校验: 全部 13 个 launch XML 通过解析校验；vision_control/decision/navigation 三包编译通过。

[2026-06-04] V4.1 决策/视觉闭环构建（本轮）

    状态机完善 (decision/race_state_machine_node.cpp):
        * 新增 IDLE 态：等待 latched 预备点 /vision_start_point 与 move_base 就绪。
        * 实现导航启动：收到预备点后自动向 move_base 发送目标，前往预备点。
        * 视觉抢占：导航途中发现蓝纸即取消 Action 目标，切换视觉伺服。
        * 盲搜升级：SEARCHING_BLIND 改为订阅 /scan 做激光避障，横移方向距挡板 <30cm 自动反向，并加入盲搜超时兜底。
        * move_base 服务器等待改为非阻塞（3s 超时，防永久卡死）。
        * 依赖补充 sensor_msgs（LaserScan）。
    预备点发布 (robot_bringup/waypoints_saver.py):
        * 保存 yaml 后额外发布 latched /vision_start_point (PoseStamped)，供状态机读取发起导航，避免 C++ 直接读文件。
    视觉伺服增强 (vision_control/visual_servo_node.cpp):
        * 新增偏航角修正 angular.z，避免车头蛇形对准。
        * 加入 x/y/z 最大速度钳制与全部 PID 参数化（kp_x/kp_y/kp_yaw/target_area/max_*）。
    蓝色检测参数化 (vision_control/blue_detector_node.cpp):
        * HSV 上下限、min_area、morph_kernel 全部参数化，便于现场通过 rosparam 调试。
    导航修复 (navigation):
        * move_base.launch 增加 odom->odom_raw、cmd_vel->cmd_vel_nav 重映射及 controller/planner 频率。
        * TEB odom_topic 修正为 /odom_raw。
        * costmap_common 移除重复 inflation_radius，消除被 global(0.45)/local(0.2) 覆盖的歧义。
        * navigation/package.xml 补齐 teb_local_planner、costmap_2d 等运行时依赖。
    新增启动文件:
        * vision_control/launch/vision.launch（蓝纸检测+伺服）
        * decision/launch/race.launch（预备点保存+状态机）
        * navigation/launch/auto_race.launch（全自动比赛总入口）
    编译验证: vision_control / decision / navigation 三包 catkin_make 全部通过，无 error/warning。

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

