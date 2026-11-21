# car01 项目审查与比赛准备结论

审查日期：2026-10-13（Asia/Shanghai）。目标：`ubuntu@192.168.3.151`，工作空间 `/home/ubuntu/car01`。

## 1. 工作空间与调试进度

已核查根目录说明、全部源码包、launch/配置、地图资产、Git 历史、相关构建输出；读取远端和 Windows VS Code 中与 car01 对应的 Copilot 会话，并用项目相关 Codex/Claude 短会话补证。源码共 133 个文件；生成目录和 STL 等二进制以清单、构建目标、时间和引用关系核对，不把二进制逐字读取当作源码审计。

已实测的阶段：串口/手柄驱动、建图、导航测试。2026-10-07 的用户反馈明确表示 `nav_test` 导航流畅稳定。没有找到“按 B 存点→重启读取→自动导航→视觉停车”的完整实车证据。

当前 Git 为 `main`，HEAD `3042df9`；本轮开始前有 13 个 tracked 修改、7 个 untracked 项目文件。Copilot/Zcode 的有效调试改动大量尚未提交，不能整体 checkout 回退。

远端变更统一记入 `/home/ubuntu/car01/wincodexchangelog.md`；阶段交接写入 `/home/ubuntu/car01/PROGRESS.md`。

## 2. Zcode 怎样让车流畅，以及麦轮怎样保留

Zcode 记录的一组改动包括：TEB 禁用导航横移、加强非完整运动约束、关闭多拓扑优化、缩短优化前瞻，并续发上游最新速度。历史有导航控制周期慢、CPU 满载等证据；这些修改共同完成后的运动改善有用户确认，不能把改善隔离归因到其中一项。

**按用户最新纠正，底盘看门狗确定不是问题；本报告不把它作为卡顿根因。**

当前真正的模式是“麦轮硬件＋非全向 TEB 粗导航”，不是更换了差速底盘驱动：

| 参数 | 当前值 |
|---|---:|
| TEB 前进 / 后退上限 | 0.4 / 0.2 m/s |
| TEB 横移上限 `max_vel_y` | 0 |
| TEB 角速度上限 | 0.5 rad/s |
| `weight_kinematics_nh` | 1000 |
| `min_turning_radius` | 0，仍允许原地旋转 |
| 多拓扑规划 | 关闭 |
| 全局路径前瞻 | 2 m |
| move_base 控制 / 全局规划频率 | 5 / 1 Hz |
| 测试入口 / 比赛入口速度续发 | 20 / 10 Hz |

证据：`src/navigation/config/teb_local_planner_params.yaml`、`src/navigation/launch/move_base.launch`、`src/decision/src/race_state_machine_node.cpp`。

**视觉伺服当前就直接输出 `linear.y`，麦轮末段横移能力仍在。** 不需要重刷 MCU，也不需要为精调重新打开全向 TEB。建议：稳定粗导航→到预备点→寻找目标→低速横移对中＋接近→稳定停车。

如果以后确实要恢复长距离全向规划，应同时调整 `max_vel_y` 和 `weight_kinematics_nh`，从低横移速度开始，保留单拓扑和短前瞻，逐项测打滑、周期和定位误差；不能只把 y 上限改大或整表回滚。

设备为 Pi 4B 8GB、四核 A72@1.8GHz。调查时约 7.1GiB 内存可用、45℃、无欠压/降频位；这是基础 bringup 的轻载状态。HSV 检测和低速全向伺服增加的计算不多，但导航、定位、视频和远程显示同开的实时余量尚未测，不能以空闲内存保证整套系统的控制频率。

## 3. 现在能否按所述流程模拟比赛

**目前可以分模块调试，尚不具备可靠的全自动比赛闭环。**

- 建图入口 `roslaunch robot_bringup bringup_mapping.launch` 周期保存 `map.pbstream`；B 保存一个位置＋朝向，文件为 `src/robot_slam/map/vision_point.yaml`，再次按 B 会覆盖，不是多点路线。
- 本次地图目录只有 `map.pbstream` 和 `last_pose.yaml`，没有找到 `vision_point.yaml`。
- `roslaunch robot_slam localization.launch` 只启动定位；不会自动启动 move_base、摄像头或比赛状态机。
- 比赛总入口为 `robot_bringup bringup_localization.launch` 或 `navigation auto_race.launch`，但状态机没有订阅保存点、发送导航 goal、等待导航完成的逻辑。当前只能转发外部发起的导航。
- 末位姿加载脚本读错本机 Cartographer 服务响应字段，保存的 `last_pose` 不能可靠注入。
- 当前识别的是蓝色 `/blue_target`，HSV H=90–130。用户现在所说的红色目标需要实现双段色相阈值，并核实光照和相机曝光。

建议终点策略：到预备点后先有界低速原地扫描，锁定目标再靠近，最后横移对中。若朝向可信、目标只是左右出画面，可保留短程、有边界的横移搜索。终点墙前没有杂物不等于旋转时车角不会碰墙；须检查扫掠空间、相机实际朝向和目标高度。

现有搜索是固定 y=+0.2m/s 的无限单向横移，没有墙距、位移或超时边界；launch 中搜索安全参数实际未被读取。需要修好状态机和安全出口后才能用于比赛。

## 4. 深层不足与优化优先级

以下是实际影响问题，功能代码本轮先不修改。

1. **比赛入口缺导航调度。** 增加保存点有效性、定位/导航就绪门禁、sendGoal、成功/失败/超时处理。外部手动 goal 能走不等于自动比赛完成。
2. **初始位姿恢复 API 错用。** 应读 `response.trajectory_states`，检查服务状态并在读取位姿成功后再变更活动轨迹；否则会留下旧轨迹占用传感器，或在缺文件时退出定位。
3. **视觉零速被误认为完成。** 伺服 0.3 秒丢目标就发零，状态机 1 秒才判丢失；中间可能永久进入 FINISHED。需要显式到达/丢失状态和连续稳定完成判据。
4. **速度控制与人工接管未统一。** race 和激光安全层同时发 `/cmd_vel`，视觉/搜索绕过安全层；人工接管只保护导航态。应统一最终仲裁、安全过滤和各状态的接管规则。
5. **输入掉线仍可能续用旧运动。** 视觉速度缓存无更新时间，手柄重复节点无原始 Joy 新鲜度检查。必须检查真正上游消息时刻，续发不能证明输入仍有效。
6. **视觉和搜索调参实际不生效。** launch 写私参，视觉 C++ 从 public NodeHandle 读根参数；搜索参数完全没读取。应先修参数作用域再调颜色、速度和停车面积。
7. **地图、起点和预备点缺一致版本。** 地图周期保存、末位姿退出保存、点独立写入，可能不是同一快照；应统一结束建图、优化、检查保存状态、原子写入并关联 map ID。
8. **精细停车缺几何闭环。** 当前依据像素中心/面积的 P 控制，不是完整 PID；没有相机标定、标记尺度/姿态、墙距校验，不能保证厘米级停车。先固定目标，再区分偏航、横移和距离控制。

后续性能优化按实测进行：降低不必要的 200Hz TF 展示发布、给定位单独配置采样和匹配、减少雷达逐帧日志与过长队列；同时复验定位精度。历史显示原始轮式里程计转圈和横移仍会漂，不能用 Cartographer 校正后的模型位置代替里程计标定验收。

## 5. Windows Lichtblick

Windows Lichtblick 1.29.1 已实际接通 `ws://192.168.3.151:8765`。树莓派安装官方 arm64 Foxglove bridge 0.8.4，并以独立预览节点提供车模、导出地图、相机和蓝色目标调试画面。没有启动导航、定位、伺服控制或比赛运动流程。

| 内容 | 已验收状态 |
|---|---|
| URDF/STL | URDF 参数可读取，8 种 STL 资源全部成功获取；GUI 显示现有车模。轮角为零姿态预览。 |
| TF / 雷达 / 里程计 | Windows 收到 `/tf`、`/tf_static`、`/scan`、`/odom_raw`；仅现有 URDF 车体内部 TF，没有伪造地图与车体关系。 |
| 已建地图 | 从现有 pbstream 导出，发布隔离 `/wincodex_visualization/map_preview`，不覆盖正式 `/map`。车模和地图分开查看，预览不代表车在地图上的实时定位。 |
| 摄像头调试图 | 已切到 `/wincodex_visualization/blue_target_debug_img/compressed`；使用现有蓝色检测器的绿色外接框、红色质心点，无有效目标时显示原图。未启动伺服。 |
| 全局 / 局部路径 | 正式布局按本机配置准备；当前导航未运行，因此没有路径运行验收。全局 `/move_base/NavfnROS/plan`；局部 `/move_base/TebLocalPlannerROS/local_plan`。 |
| 代价地图 | 正式话题 `/move_base/global_costmap/costmap`、`/move_base/local_costmap/costmap`，增量为 `costmap_updates`。运行时再验收。 |

当前调试预览传输为 **320×240、15fps、JPEG 质量 65**。用户反馈可见延迟约 0.2～1 秒，并明确接受当前可用状态、不再继续调延迟，因此保留此设置。

时钟偏差校准后，10 秒实收 149 帧，源 `header.stamp`→Windows 接收年龄 p50 31.34ms、p95 36.00ms、最大43.63ms。usb_cam 0.3.7 在 `grab_image()` 后才设置 ROS 时间戳，故此数值包含之后的检测、压缩和传输，不包含此前曝光/驱动缓存或最后 GUI 显示，不能宣称完整端到端仅31ms。[对应官方源码](https://github.com/ros-drivers/usb_cam/blob/0.3.7/src/usb_cam.cpp#L1238)

Lichtblick 支持 URDF、TF、OccupancyGrid、Path、LaserScan、图像及 ROS 1 Goal/Pose Estimate，大部分日常 RViz **查看**需求可以覆盖。当前桥按查看用途限制发布/服务能力；RViz 第三方插件和专用交互不会自动等价。

参考：[Lichtblick ROS 1](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/frameworks/ros1/)、[连接能力](https://lichtblick-suite.github.io/docs/docs/connecting-to-data/live-data/)、[ROS 1 Foxglove bridge](https://github.com/foxglove/ros-foxglove-bridge)。

## 验证边界

已完成源码、配置、历史会话、构建目标/二进制一致性、运行节点和轻载资源交叉核对；本地快照的 5 个 Python、25 个 launch XML、5 个 YAML 解析通过。独立可视化通过真实 Windows WebSocket 订阅、URDF 参数、全部 STL 获取与 GUI 图像展示验收。没有重编译运动代码，也没有发送速度、导航目标、初始位姿或进行实车比赛验证。
