# 更新日志 (Changelog)

[2026-11-21 P1] 修复"识别到红/蓝纸后立刻停死不动"（视觉 FAULT 终态锁死）

    现象：终点前检测到纸 -> 状态机进 VISUAL -> 约 50ms 内跳 FAULT -> 车永久静止，
          必须 /race/reset 才能恢复；表现为"前往终点的阶段消失了"。

    根因链（代码级确认）：
        1. visual_servo step(): if (!wall.valid) { if (!allow_area_fallback) return stop("FAULT"); }
           vision.launch 里 allow_area_fallback=false。
           wall.valid 要求同时满足 min_points>=8 / min_width>=0.25 / max_rmse<=0.02 /
           |yaw|<=0.70rad。侧向发现纸时车头与白板法向夹角大 -> 直接被 max_yaw 拒。
        2. 状态机 VISUAL: if (vision_status_=="FAULT") setState(FAULT)
           而 tick() 里 case State::FAULT 只 stop()，除 /race/reset 无出口 -> 死锁。
        3. 隐藏 bug：setState(VISUAL) 只清了 vision_status_recv_time_/stamp_，
           没清 vision_status_ 字符串；而 FAULT 判定不看新鲜度。
           一旦历史上出现过 FAULT，之后每次进 VISUAL 的第一 tick 就立刻再判 FAULT，
           visual_grace_timeout 宽限期也进不去（宽限分支要求 vision_status_!="FAULT"），
           因此 search_max_attempts 形同虚设。

    P1-1 decision/src/race_state_machine_node.cpp
        * 视觉 FAULT 不再进终态：与 LOST 同路径回 SEARCHING 重搜，
          只有 search_attempts_ 真正耗尽才 setState(FAULT)。
    P1-2 同上
        * setState(VISUAL) 增加 vision_status_.clear() + 清 wall_valid_/wall_distance_。
        * 所有视觉状态判定都先经 status_fresh 判定，旧数据不再参与本轮决策。
    P1-3 同上 + race.launch
        * visionStatusCb 解析 wall_valid / wall_distance_m 诊断值。
        * 新增 visual_max_time(25s) / visual_standoff(0.45) / visual_park_window(0.15)：
          VISUAL 超时后，仅当"目标新鲜 + 墙面有效 + 距离已在停车区间"才判 FINISHED，
          否则回 SEARCHING 重试。避免"停在半路也宣告成功"，也避免僵死。
    P1-4 vision_control/launch/vision.launch
        * standoff_distance 0.30 -> 0.45：0.30 时相机(车心前方 0.164m)离墙仅 0.136m，
          纸左右被画面切掉 -> 横向质心算歪 -> 永远对不准。0.45 -> 相机离墙 0.286m，
          车心距 80x80 区域中心 0.05m，全车进入余量 0.175m。
        * distance_tolerance 0.015 -> 0.03：原值比墙面拟合噪声 wall_max_rmse(0.020) 还小，
          数学上不可能稳定达成。
        * center_tolerance 0.025 -> 0.06；yaw_tolerance 0.04 -> 0.10；stable_frames 5 -> 3。
        * wall_max_yaw 0.70 -> 1.00：侧向发现纸时不再被直接拒绝（这是"侧边看到纸"的那一半原因）。
        * wall_front_sector 0.50 -> 0.70：0.45m 处 ±28.6° 只覆盖 ±0.25m，点数可能不够。
        * allow_area_fallback false -> true：墙拟合失败退化为保守接近，而非立刻 FAULT。
    P1-5 decision/launch/race.launch
        * search_max_time 20 -> 35：0.20rad/s*20s=4.0rad≈229°，转不满一圈，
          纸在车后侧永远搜不到；35s -> 7.0rad > 360°。
        * search_max_attempts 2 -> 5：2 次容易被一次抖动耗光直接进终态。

    编译验证：g++ -fsyntax-only -std=c++14 通过（仅剩原有缩进警告）；三个 launch XML 解析通过。
              catkin_make 在本机被 miniconda python3.14 污染（缺少 empy），属既有环境问题，
              树莓派上按原流程编译即可。

    已知局限（P2 再解决）：
        * allow_area_fallback 的降级路径永不宣告 REACHED（纸在 0.45m 处约占画面 80%，
          远大于 target_area 0.30），只保证不立刻 FAULT，不保证停准。
        * wall_max_yaw 放宽到 57° 只让"侧向发现纸"不被拒；真正的
          "先原地转正再横移" 仍需 P2。
        * standoff 0.45 是几何推算值，需现场用 /rgb/blue_target_debug_img 确认
          纸左右两边是否完整在画面内；若仍贴边则改 0.50（不要超过 0.60，
          那里车尾只剩 0.025m 余量）。

    现场排查：
        * rostopic echo /race/state            是否还会永久停在 FAULT
        * rostopic echo /visual_servo/status   wall_valid / wall_distance_m / wall_yaw_rad
        * rostopic echo /safety/reason         若"车体扫掠或制动空间受限" -> B 点贴墙太近，车转不动
        * grep "比赛状态 ->" ~/.ros/log/latest/rosout.log | tail

    回退：
        git checkout -- src/decision/src/race_state_machine_node.cpp \
                        src/decision/launch/race.launch \
                        src/vision_control/launch/vision.launch


[2026-11-21 修复] B 键保存失败（锚点容差过严）+ 建图会话版本识别错误

    现象：新建图后按多次 B 键，重启 localization 车不动，状态机停在 WAIT_READY

    根因 A：B 键保存被锚点容差拒绝（主因）
        * 日志：预备点未保存：最近轨迹节点相差 1.431s，超过 1.000s
                （连续多次 1.4s / 2.4s / 3.1s 均被拒）
        * 树莓派算力低时 Cartographer 轨迹节点稀疏，按 B 瞬间离最近节点
          常超过 anchor_tolerance(1.0s) -> 全部保存失败 -> B 点文件从未更新
        * 修复(waypoints_saver.py)：
          - anchor_tolerance 默认 1.0 -> 3.0
          - 新增 allow_plain_fallback（默认 true）：锚点重投影失败时退化为
            直接保存当前 map->base_link 位姿，保证 B 键始终可用

    根因 B：建图会话中 current_map_identity() 读到了上一张图的哈希
        * 建图进行中 map.pbstream / last_pose.yaml 仍是上一张图的内容，
          此时按 B 写入的 map_sha256 属于旧图，转正后永远不匹配
        * 修复：
          - current_map_identity() 优先读 /mapping_asset 参数（建图会话标识），
            此时 sha256 返回 None
          - auto_save_map.py 新增 _backfill_vision_point()：兜底转正后用
            本会话 map_id + 最终 sha256 回填 vision_point.yaml
          - waypoints_saver.load_legacy_point()：sha256 缺失但 map_id 与当前会话
            一致时允许发布（哈希待回填），其余情况仍拒绝

    重要提醒：
        * 建图后必须"先按 B 标点、再 Ctrl+C 退出"，转正时才来得及回填哈希
        * 已存在的 vision_point.yaml（map_id=map-20261121T055349Z）属于旧图，
          需要在当前图上重新按 B 标点，不要手工修改哈希（会导致导航到错误坐标）


[2026-11-21] B 点绑定地图版本 + 修复停车后不转圈（瞬间跳 FAULT）

    问题现象（2026-11-21 14:05 那次 bringup_localization）：
        1. 发布的 B 点与建图预设点有偏差
        2. 到达停车点后直接不转圈，状态机瞬间跳 FAULT

    根因 1：B 点未绑定地图版本
        * vision_point.yaml 里是 map_id: legacy 的旧坐标 (1.949, 0.150)
        * 当前地图是 map-20261121T055349Z，两者不配套
        * 日志：没有 active bundle -> 已回退发布 legacy 预备点
        * 直接 roslaunch bringup_mapping.launch + Ctrl+C 走兜底转正，
          只产 map.pbstream/last_pose.yaml，不产 bundles/active.yaml，
          于是 waypoints_saver 回退到 legacy 点

    修复 1（robot_bringup/nodes/waypoints_saver.py）：
        * load_legacy_point() 增加版本校验：legacy 点必须带 map_sha256 且与当前地图一致，
          否则拒绝发布并提示现场按 B 键重标
        * save_point()/save_legacy_point() 写入时记录 map_id + map_sha256
        * 新增 current_map_identity()：优先读 last_pose.yaml 的 map_id/map_sha256，
          缺失则直接哈希 map.pbstream

    根因 2：搜索/视觉状态瞬间跳转耗尽重试次数
        * 状态流转（间隔仅 50ms）：NAVIGATING -> SEARCHING -> VISUAL -> FAULT
        * target_first_time_ 在导航途中被反复刷新，导航结束时
          (now - target_first_time_) 早已超过 target_stable_time(0.4s)，
          进入 SEARCHING 第一次 tick 就误判"蓝目标已稳定锁定"，一秒都没转
        * 进 VISUAL 时视觉状态尚未送达（status_fresh=false），立刻被判目标丢失
          并 beginSearch()，第二次 beginSearch() 使 search_attempts_ 达上限 2 -> FAULT

    修复 2（decision/src/race_state_machine_node.cpp）：
        * beginSearch()：搜索开始时若目标新鲜，重置 target_first_time_，
          确保"稳定锁定"从搜索开始计时
        * 新增 visual_grace_timeout 参数（默认 1.0s）：进入 VISUAL 后等待视觉首帧状态，
          宽限期内 status 未就绪不算目标丢失，不消耗重试次数
        * setState() 进入 VISUAL 时记录 visual_entered_ 并清空旧的
          vision_status 新鲜度，避免拿上一轮数据判定 LOST

    编译验证：decision 包编译通过，devel/lib/decision/race_state_machine_node
              时间戳已更新（14:23）；waypoints_saver.py py_compile 通过

    注意事项：
        * 旧 vision_point.yaml（无 map_sha256）现在会被拒绝发布，
          需要在当前地图上重新按 B 键标点
        * 建图仍建议走 mapping_session.sh 正式流程以生成 active.yaml


[2026-10-18] 串口零速不发 + arbiter 失联锁止自动解除

    stm32_bridge.cpp sendTwist()：
        * 连续零速只发一次停车指令，之后不再往串口写（非零仍每条都发）
        * 原因：arbiter 以 ~12.5Hz 持续发布零速，bridge 逐条转串口，导致静止时 TX
          持续闪烁、电机控制器持续收到零速指令产生阻尼
    cmd_vel_arbiter_node.cpp joyCb()：
        * 新增自动解除"失联锁止"：手柄恢复且使能键松开、摇杆全部归中时清 lock_
        * 人工 /control/stop 的锁止(operator_stop_)不自动解除，仍须显式 reset
        * 原因：原逻辑 manual_used_ 后 /joy 静默 >0.5s 即永久 lock_，而 USB 手柄
          接收器偶发掉线（实测 xpad usb_submit_urb failed → disconnect）会正好触发，
          导致"手柄明明在、车却完全没反应"且必须手工 reset 才能恢复

    排查要点（供后续参考）：
        * 手柄控制失效先看 `ls /dev/input/js*` + `lsusb | grep 045e` + `dmesg | grep -i xpad`
        * 若 systemd 自启服务与手动 roslaunch 同时运行，同名节点(joystick/
          teleop_twist_joy/cmd_vel_arbiter_node/stm32_bridge)会互相顶替并把对方杀掉，
          表现为"服务 active 但只剩 stm32_bridge、手柄无响应"
        * 手动建图/导航前务必 `sudo systemctl stop bringup_auto.service`


[2026-10-16] 地图保存兜底转正 + arbiter 零速只发一次

    auto_save_map.py 新增 fallback_promote()：
        * 问题：WinCodex 两阶段提交设计下，直接 roslaunch bringup_mapping.launch + Ctrl+C
          只写 staging 周期快照，不生成 active.yaml，导致定位仍加载旧图
        * 修复：节点退出时若未走正式 finalize，自动校验 staging 快照 SHA256 后转正：
          备份旧 map.pbstream.bak-<时间戳> → 快照替换 map.pbstream → 尽力写 last_pose.yaml
        * 正式 finalize（mapping_session.sh）路径不受影响，且优先级更高
        * 现在任何退出方式都保证"刚建的图"可用；正式流程仍提供更精确的
          最终优化 + 锚点重投影，建议比赛仍走 mapping_session.sh
    cmd_vel_arbiter_node.cpp 下发规则改为（与 cmd_vel_repeat_node 一致）：
        * 非零速度 25Hz 持续发
        * 零速只在"非零→零"或"从静默恢复"时发一次，之后静默让权
        * 失联/锁止恢复时必先发一帧零，确保底盘不残留旧速度
        * 原因：此前 tick() 25Hz 无条件发布，零速也持续刷，与"0 只发一次"约定不符


[2026-10-16] 开机自启关闭雷达避障 + 安全节点加速度限制放宽

    bringup_auto.launch 重构为 include bringup.launch（enable_laser_safety=false）：
        * 开机自启不再启动 laser_safety_node，手柄直通 cmd_vel
        * 原因：前面有障碍物时其他方向也动不了（安全节点全向抑制）
        * 需要避障时用 bringup.launch / nav_test.launch
    bringup.launch 的 laser_safety_node（导航时启动）：
        * max_accel_x/y/yaw 0.4/0.4/0.8 → 10.0（实际不限制加速度）
        * max_vx/vy/wz 0.8/0.5/0.6 → 1.5/1.5/2.5（放宽速度上限，避免二次限速）
        * 原因：安全节点内置加速度斜坡和速度钳制会拖慢响应；TEB 已有 costmap 避障，
              安全节点只做碰撞兜底，不应二次限制速度/加速度
        * 注意：不能设 0（节点参数校验 params_ok_ 要求 >0），设大值等效关闭


[2026-10-16] 建图/导航雷达安全避障开关分离

    bringup.launch 新增 enable_laser_safety 参数（默认 true）：
        * 开启时：arbiter → laser_safety_node → /cmd_vel（正常安全避障链路）
        * 关闭时：arbiter → topic_tools/relay 直连 /cmd_vel（手柄直通，无避障）
    bringup_mapping.launch：默认 enable_laser_safety=false（建图时手柄直通，和以前一样）
        * 可通过 roslaunch robot_bringup bringup_mapping.launch enable_laser_safety:=true 手动开启
    nav_test.launch / bringup_localization.launch：默认 enable_laser_safety=true（导航时开启避障）
    原因：建图时手柄遥控需要直接控制，安全节点会干扰；导航时 TEB 有 costmap 避障，
          但到达导航点后自转找目标+视觉伺服停车阶段没有 TEB，需要激光安全兜底（Codex 建议）。


[2026-10-15] 外部 AI 修复轮次已部署（WinCodex/WorkBuddy 续接，勿误回退）

    本工作树包含 2026-10-13~15 由外部 AI 完成的大规模修复（未提交，叠加在你既有改动之上，基线 aa79e09）：
        * 统一速度安全链：/cmd_vel_auto → cmd_vel_arbiter → laser_safety → /cmd_vel（唯一发布者）；手柄链路同入仲裁
        * 比赛状态机：自动 goal / 有界旋转搜索 / 视觉对中停车（decision/race_state_machine_node）
        * 地图资产事务（map_id/原子写）+ Cartographer 相对初始位姿（robot_bringup/nodes/asset_bundle.py、initial_pose_loader.py）
        * 视觉：/vision/target stamped 接口 + 显式 REACHED 稳态判定（vision_control）
    修改任何 src/ 文件前先读 wincodexchangelog.md（2026-10-15 条目），含完整文件清单与回退方法；
    隔离工作树与完整备份在 /home/ubuntu/car01_wincodex_fix_20261013/。


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

