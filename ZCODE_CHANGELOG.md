# ZCODE_CHANGELOG — ZCode 改动留痕

> 2026-10-25 说明: 本文件记录的是 ZCode 的改动。期间 WinCodex/WorkBuddy(10-13~15)与
> Copilot(10-16~18)的大规模改动见各自的 `wincodexchangelog.md` / `CHANGELOG.md`,
> 以及交接文件 `PROGRESS.md`。ZCode 第一轮(下方 10-07 条目)中的比赛状态机保活改写
> 已被 codex 的全新状态机整体替代(保活职责转移到 arbiter, 功能等价)。

本文件由 **ZCode**(另一个 AI 编码助手, 通过 ssh 在本机工作) 维护, 记录它对本工作空间做过的**全部改动**。

> **给 GitHub Copilot 的说明**: 在修改 `src/navigation`、`src/robot_bringup`、`src/robot_slam`、`src/decision`
> 下的文件之前, 请先读本文件, 了解 ZCode 已经改了什么、为什么改, 避免冲突或误回退。
> 你自己的改动请继续记录在 `CHANGELOG.md`, 不用写进本文件。

每条记录格式: 日期 / 改动文件 / 改了什么 / 为什么 / 如何回退。

---

## [2026-10-25] 导航定位起始点改回"建图起始点(原点)", 清理 bundle 依赖干扰 (ZCode 第三轮)

### 背景与决策

"导航从建图退出点开始"的方案经多方尝试(Copilot 的 last_pose 注入 → codex 的
bundle 事务 + relative 初始位姿)始终不稳定, 用户拍板放弃: **定位起始点回到默认
行为 = 建图起始点(原点)**, 并清理所有会干扰这条路径的东西。

### 问题 1: initial_pose_loader 拒绝启动(是什么/为什么)

codex 版 loader 走"注入建图退出位姿"模式, 启动前有三条严格前置:
必须没有 ACTIVE 轨迹、必须恰好 1 条 FROZEN 轨迹、必须有配套 pose 资产。
实测启动时 Cartographer 里已存在 ACTIVE 轨迹 [1](`-start_trajectory_with_default_topics false`
没能阻止默认轨迹, 或混跑残留), loader 直接 raise 退出 → 不发 `/localization/ready`、
没有 map TF → RViz 显示旧位置、比赛状态机永远 WAIT_READY。
另外严格模式要求 `bundles/active.yaml`, 而该目录只有 staging 快照、active.yaml
从未生成过(直接 Ctrl+C 退出的 fallback 只转正地图+位姿, 不提交 bundle),
比赛入口(`allow_legacy_assets=false`)同样会被拒。

### 改动 1: `src/robot_slam/launch/localization.launch`

- `-start_trajectory_with_default_topics` **false → true**: Cartographer 自己在
  原点(=建图起始点)启动定位轨迹, 不再依赖任何注入。
- initial_pose_loader 节点只传 `inject_pose=false`, 删除 pbstream/active_file/
  allow_legacy_assets/config_* 传参; 顶部同名 `<arg>` 一并删除。
- 回退: 把 flag 改回 false 并恢复 loader 的 5 个参数传参(见 git 历史/下方改动 2 的 inject 模式)。

### 改动 2: `src/robot_bringup/nodes/initial_pose_loader.py`

新增 `~inject_pose` 参数(**默认 false** = 就绪监视模式):
- false: 不读任何 bundle/last_pose、不碰任何轨迹; 只等服务出现后轮询
  `/get_trajectory_states`, 一旦有 ACTIVE 轨迹就发布 `/localization/ready=true`
  (比赛状态机门禁依赖此话题), 完成。
- true: 保留原注入路径(含全部严格校验), 留作以后恢复"建图退出点"模式用。
- 回退: `git checkout -- src/robot_bringup/nodes/initial_pose_loader.py`

### 改动 3: `src/robot_bringup/nodes/waypoints_saver.py` (legacy 回退)

比赛入口的预备点加载也依赖 active.yaml(缺失则不发 `/vision_start_point`,
状态机卡 WAIT_READY), 且 B 键保存在非建图会话下因缺 `/mapping_asset` 参数直接失败。
增加两条回退:
- 启动加载: active.yaml 缺失/校验失败 → 回退读 `save_dir/vision_point.yaml`(legacy)发布;
  再没有则提示等待 B 键现场保存(不再 logerr 卡死)。
- B 键保存: 有 `/mapping_asset`(建图会话)→ 原 bundle 草稿逻辑不变;
  没有 → 直接存 legacy `vision_point.yaml` 并发布(nav_test/比赛前也能补标点)。
- 注意: legacy 点不与地图版本绑定, 换新图后需重新按 B 标点。
- 回退: `git checkout -- src/robot_bringup/nodes/waypoints_saver.py`

### 改动 4: `src/navigation/launch/nav_test.launch` / `auto_race.launch`

- nav_test: 删除对 localization.launch 的 `allow_legacy_assets:=true` 传参(参数已删)。
- auto_race: 删除 `active_file`/`allow_legacy_assets` 对 localization.launch 的传参
  (否则 roslaunch 直接报 unused arg); pbstream 默认值从不存在的
  `bundles/active_map.pbstream` 改回 `map/map.pbstream`; 对 race.launch 的
  active_file 传参保留(saver 已有回退)。
- 回退: `git checkout -- src/navigation/launch/nav_test.launch src/navigation/launch/auto_race.launch`

### 验证

- py_compile / XML 解析全部通过; `roslaunch --files` 包含链检查:
  nav_test / auto_race / bringup_localization / bringup_mapping 四入口全过。
- 未实车跑(等用户验证)。预期: nav_test 启动后车显示在**建图起始点**附近,
  靠扫描匹配收敛; `initial_pose_loader` 日志应出现"定位就绪(默认建图起始点模式)"。

### 追加(同日): B 键预备点双写, 打通"建图按 B → 一键启动自动用"

用户实测建图 + nav_test 2D Nav Goal 均正常后, 准备测 B 点自动导航流程时发现缺口:
建图会话按 B 只写 bundle 草稿(staging 目录), Ctrl+C 兜底不转正预备点,
且 bundles/ 已被清空、`vision_point.yaml` 不存在——比赛入口永远等不到建图时按的 B 点。
另外 `bringup_mapping.launch` 的 waypoints_saver **漏传 save_dir**(默认空串)。

- `src/robot_bringup/nodes/waypoints_saver.py`: 建图会话(/mapping_asset 存在)按 B 时,
  在存 bundle 草稿之外**同步写 legacy `save_dir/vision_point.yaml`**(save_dir 未配置则告警跳过);
  非建图会话按 B 仍直接写 legacy(此前已加)。
- `src/robot_bringup/launch/bringup_mapping.launch`: waypoints_saver 补 `save_dir` 参数
  指向 `robot_slam/map`。
- 回退: `git checkout -- src/robot_bringup/nodes/waypoints_saver.py src/robot_bringup/launch/bringup_mapping.launch`
- 验证: py_compile / XML / `roslaunch --files` 均通过。

### 追加(同日): 摄像头按入口开关 + respawn 自愈

用户实测: 摄像头时好时坏(日志证据: usb_cam 启动 2 秒即 exit code 1, 与手柄/STM32 同源的
USB hub 供电问题); 要求建图不开摄像头、导航入口(nav_test/bringup_localization)开。

- `src/robot_bringup/launch/bringup.launch`: 新增 `start_camera` 参数(**默认 true**,
  导航两入口不传参即开); usb_cam 节点加 `respawn="true" respawn_delay="5"`,
  设备掉线后自动重连自愈。
- `src/robot_bringup/launch/bringup_mapping.launch`: include bringup 时传
  `start_camera=false`, 建图不再起摄像头(省 USB 供电/带宽, 也消除红色报错干扰)。
- 回退: `git checkout -- src/robot_bringup/launch/bringup.launch src/robot_bringup/launch/bringup_mapping.launch`
- 验证: 全部 6 个入口 `roslaunch --files` 通过。

### 追加(同日): "B 点后车没动"排查结论(无代码改动)

日志还原(17:57:53-17:58:03 的比赛入口启动): 18 个节点全部正常启动,
定位 17:57:58 就绪, usb_cam 17:57:58 闪退(不影响导航段), **用户 5 秒后即 Ctrl+C**。
结论: 不是故障——move_base 完全就绪 + 状态机发 goal 在这台 Pi 上需要 10~30 秒,
等待时间不足。重测要点: 看到"比赛状态 -> NAVIGATING"日志再下结论,
`rostopic echo /race/state` 观察状态流转。(后续用户重测确认 <10s 可到 B 点)

### 追加(同日晚): 导航限速放开到 1.8 + 全文件审查结论

用户实测: B 点自动导航成功(<10s 到点), 但到达后旋转搜索时手持蓝色靶标不停。

**限速现状与改动**(链上所有限速点):
| 层 | 旧值 | 新值 |
|---|---|---|
| TEB max_vel_x | 0.4 | **1.8** |
| laser_safety max_vx | 1.5 | **1.8**(节点校验上限 2.0) |
| TEB max_vel_x_backwards / max_vel_theta | 0.2 / 0.5 | 不变 |
| laser_safety max_vy / max_wz | 1.5 / 2.5 | 不变 |
| TEB acc_lim_x | 0.5 | 不变(提速加速段会加大起步打滑风险, 未动) |
| 手柄 scale | 0.8/0.5/0.5 | 不变 |
| arbiter / stm32_bridge | 不限速 | 不变 |
| **STM32 固件** | 历史限幅 MAX_CMD_VX=500mm/s | ROS 侧无法改, 实车若卡在 ~0.5 需 32 侧放行 |

- 改动文件: `src/navigation/config/teb_local_planner_params.yaml`、
  `src/robot_bringup/launch/bringup.launch`
- 物理提醒: laser_safety 制动模型 deceleration=0.6 m/s², 1.8 m/s 需 ~3.2m 刹停距离,
  小场地内安全层会在墙前自动压速, 全速只在长直线中段出现。

**蓝色不停的原因(日志实锤, 非代码 bug)**: 18:05 那轮 usb_cam 启动 1 秒即死,
respawn 每 5s 拉起又立刻死; 当时 /dev/video0 不存在、lsusb 无相机、dmesg 报
`Cannot enable. Maybe the USB cable is bad?`——相机那段 USB 链路枚举失败。
(2026-10-25 用户更正: hub 是独立供电的, 手柄/STM32 的历史问题与 hub 无关,
现在 hub 上只有雷达和相机; 相机掉线优先怀疑相机自身的线材/接口。)
无图像 → blue_detector 不发 /vision/target → 状态机 SEARCHING 无目标可锁,
只能继续旋转。软件链(检测归一化坐标、targetCb 校验、0.4s 稳定判据)审查无误。
恢复后在 PC 看 /rgb/blue_target_debug_img 绿框验证 HSV(手持蓝色物体需落在
H90-130/S>80/V>50 且面积>800px)。

**全文件审查其他发现**(均未改):
- joy_repeat 无新鲜输入时 20Hz 发零到 /cmd_vel_raw, 与"零只发一次"约定不完全一致,
  但下游 arbiter/bridge 都有零速过滤, 无实际影响。
- SEARCHING 固定逆时针搜索(search_speed 恒正), search_max_angle=6.3rad≈361° 基本全覆盖, 可接受。
- 视觉末段 max_vx=0.10/max_vy=0.08 是刻意慢速设计, 与导航 1.8 不冲突。
- chassis_corrector 仍在 joystick.launch 中注释禁用, 无影响。
- 串口协议无 CRC/无 ROS 侧速度钳制, 历史已知, 依赖上游各层限幅。

---

## [2026-10-07] 导航抖动+打滑+斜走优化 (ZCode 第一轮)

### 背景诊断 (实测于 2026-10-07 17:27, 导航栈运行中)

1. **`/cmd_vel_nav` 实际只有 1.1Hz**(参数设的 5Hz): move_base 占 94% CPU + Cartographer 占 125% CPU,
   4 核饱和, 控制周期被拉到 0.8~1.0s。
2. **STM32 有 300ms 指令超时看门狗**: 导航速度间隔 ~900ms >> 300ms → 电机在每两条指令之间都在急停,
   这是"走一下停一下"的直接原因; 且每次都从静止重新起步, 起步扭矩最大 → 轮子打滑加重。
3. **TEB 按"完全全向"规划**(max_vel_y=0.2, weight_kinematics_nh=0, forward_drive=0):
   会规划出车身侧过去蟹行(斜走)的轨迹 —— 麦轮横移靠滚子 45° 传动, 地面摩擦不足时
   对角轮(左前+右后 / 右前+左后)成对打滑, 与用户观察到的"左前和右后打滑"完全吻合。
4. TEB 的 `enable_homotopy_class_planning` 未配置(noetic 默认开启, 多拓扑并行优化), 是最大单项 CPU 消耗。

### 改动 1: `src/navigation/config/teb_local_planner_params.yaml`

| 参数 | 旧值 | 新值 | 原因 |
|---|---|---|---|
| `max_vel_y` | 0.2 | **0.0** | 导航阶段禁横移, 消除蟹行打滑; 盲搜/视觉伺服的横移走状态机/视觉节点, 不受影响 |
| `weight_kinematics_nh` | 0 | **1000** | 恢复非完整约束, TEB 规划"原地转向对准路径→前进", 不再侧身斜走 |
| `weight_kinematics_forward_drive` | 0 | **1.0** | 少倒车, 优先车头对准行驶方向 |
| `max_vel_x` | 0.3 | **0.4** | 提速(32 上限 0.5, 手柄 0.5), 走不打滑的前进轴可以放心加 |
| `enable_homotopy_class_planning` | (默认 true) | **False** | 关闭多拓扑并行优化, 大幅降低 move_base CPU |
| `max_global_plan_lookahead_distance` | (默认 3.0) | **2.0** | 缩短每次优化规模, 降低单周期耗时 |

回退: `cd ~/car01 && git checkout -- src/navigation/config/teb_local_planner_params.yaml`
(注意: 会连 Copilot 之前的未提交修改一起回退, 谨慎; 或手动按上表反向改)

### 改动 2: 新增 `src/robot_bringup/nodes/cmd_vel_repeat_node.py`

速度续发/保活节点: `/cmd_vel_nav` → 20Hz 持续转发最近一次非零速度 → `/cmd_vel`;
上游发 0 只转发一次 0; 上游断流超过 `max_hold`(0.5s) 发一次 0 兜底停车。
解决 move_base 控制周期 > 300ms 时 STM32 看门狗急停造成的"走一下停一下"。

### 改动 3: `src/navigation/launch/nav_test.launch`

用 `cmd_vel_repeat_node` 替换原来的 `topic_tools/relay`(语义兼容, 多了 20Hz 保活)。

回退: 把该 node 块换回 `<node pkg="topic_tools" type="relay" name="cmd_vel_relay" args="/cmd_vel_nav /cmd_vel" />`

### 生效方式

重启导航: `Ctrl+C` 后重新 `roslaunch navigation nav_test.launch`(TEB yaml 是 move_base 启动时加载的)。

### 预期观察

- 不再"侧身斜走", 起步会先原地转向对准路径再直行;
- "走一下停一下"明显减少甚至消失(若 TEB 周期仍偶发 >0.5s, 保活节点日志会打 `upstream cmd_vel stalled`);
- 打滑大幅减轻; 确认稳定后可再逐步上调 `max_vel_x`(0.4 → 0.5)。

### 遗留建议 (未实施)

- 联系 32 侧把看门狗从 300ms 放宽到 500ms, 与 `max_hold` 对齐, 进一步兜底。
- 比赛(带视觉)时 blue_detector/visual_servo 跑在树莓派上, 建议改到 PC 端跑(现有 vision_debug.launch 已支持), 省 CPU。
- 车体机械面: 清洁/检查麦轮滚子, 打滑与滚子磨损、地面粉尘关系很大。

### ✅ 实车验证 (2026-10-07 晚)

用户实测 nav_test: **运动流畅**, "走一下停一下"与斜走消失。本轮改动确认有效。

---

## [2026-10-07 第二轮] 比赛链路(bringup_localization)同步保活修复

nav_test 验证通过后, 把同样的保活逻辑移植进比赛链路。

### 改动 4: `src/decision/src/race_state_machine_node.cpp`

- `navVelCb` 不再直接透传 `/cmd_vel_nav` 到 `/cmd_vel`, 改为缓存, 由 run() 主循环(10Hz)保活续发:
  非零持续发、变零只发一次、断流超 0.5s 发一次零速兜底(与 cmd_vel_repeat_node 语义一致)。
- 新增订阅 `/cmd_vel_raw`(手柄路径): 手柄非零输入 0.3s 内视为**人工接管**, 暂停导航保活,
  让 laser_safety_node 独占 `/cmd_vel`; 松开手柄自动恢复导航。避免保活速度和手柄打架。
- VISUAL_SERVO / SEARCHING_BLIND / FINISHED 三个状态的行为未改动。
- `race.launch` 无需改动(话题没变, 新增的 /cmd_vel_raw 订阅在代码里)。

回退: `cd ~/car01 && git checkout -- src/decision/src/race_state_machine_node.cpp`

### 改动 5: 清除 build/CMakeCache.txt 中缓存的包白名单 (重要环境修复)

发现 `build/CMakeCache.txt` 里缓存了 `CATKIN_WHITELIST_PACKAGES=robot_bringup`,
导致此前的 `catkin_make` **一直在静默跳过 decision / navigation / vision_control 等包**
——devel 里的 race_state_machine_node 还是 9 月 1 日的旧二进制, 而编译命令显示"成功"。

已用 `catkin_make -DCATKIN_WHITELIST_PACKAGES=""` 清除并全量重编(全部包通过)。
**给 Copilot 的提醒**: 以后确认某个包编译成功, 请核对 `devel/lib/<pkg>/` 里二进制的修改时间,
不要只看 catkin_make 退出码(白名单缓存会让编译静默空跑)。

### 生效方式

重启比赛入口: `roslaunch robot_bringup bringup_localization.launch`(需先停掉在跑的 launch)。
建议赛前完整走一遍: 导航段应和 nav_test 一样流畅; 导航中推手柄应能立即接管, 松开 0.3s 后恢复导航。
