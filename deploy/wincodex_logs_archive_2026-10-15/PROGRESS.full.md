# car01 当前交接进度

## 2026-10-13：WinCodex 只读调查开始
- 已做：已通过 SSH 连接并定位 `/home/ubuntu/car01`；已启动项目历史、源码导航链路、Lichtblick 支持调查。
- 改动文件：新增 `wincodexchangelog.md` 与本文件。
- 待办：完成全工作空间清单、Copilot/Zcode 历史、导航与视觉闭环、运行资源、Windows 可视化审核；将结论写入 changelog 供 Copilot 查看。
- 关键决策：先向用户报告实证结论与不足；功能配置暂不变更，不控制车辆；后续所有改动追加 `wincodexchangelog.md`。

## 2026-10-13：用户纠正与阶段结论
- 用户明确纠正：忽略底盘看门狗，它已被确认不是卡顿原因。历史 ZCODE_CHANGELOG 的该解释只保留为旧假设，不作为本次根因或缺陷。
- 已做：核对当前非全向导航配置、比赛状态机及其构建目标；运行资源/话题和 Windows Lichtblick 调查已回收初步结论。
- 初步结论：当前 localization.launch 不能自动完成存点比赛；现有视觉实际检测蓝色；当前只运行基础传感器与底盘节点，缺 WebSocket 可视化桥。详细结论待审计验收。
- 改动文件：本阶段仅追加 wincodexchangelog.md、PROGRESS.md。
- 待办：完成 Copilot 实际会话核查；接通独立可视化桥；形成可追溯报告。
- 关键决策：可视化接通独立部署；运动参数、比赛状态机和视觉停车改动先报告用户。

## 2026-10-13 17:12:38 CST：Windows Lichtblick 可视化桥实施（阶段 1：安装前确认）
- 原态：ROS Noetic/arm64；11311 已运行；8765/9090 未监听；foxglove_bridge 与 rosbridge_server 均未安装、未运行。
- 决策：APT 官方 ROS 源提供 ros-noetic-foxglove-bridge 0.8.4-1focal.20250520.090425（arm64），优先安装该包；它支持 Foxglove WebSocket、参数和 package:// 资源获取。
- 约束：新增独立可视化文件和进程，不改现有 bringup/导航/运动参数，不启动导航、比赛、摄像头，不发布任何控制或导航消息。


## 2026-10-13 17:13:17 CST：Windows Lichtblick 可视化桥实施（阶段 2：包安装）
- 新增系统包：ros-noetic-foxglove-bridge=0.8.4-1focal.20250520.090425；依赖 ros-noetic-ros-babel-fish=0.9.3-1focal.20250520.085837。
- APT 结果：新增 2 包，下载约 921 kB，占用约 4.4 MB；未升级或删除其他包。
- 当前尚未启动 bridge；下一步新增独立 deploy/wincodex_visualization 文件并验证，不修改已有 launch。
- 还原包命令：sudo apt-get remove ros-noetic-foxglove-bridge ros-noetic-ros-babel-fish（仅在确认无其他组件依赖后执行）。


## 2026-10-13 17:20:10 CST：可视化阶段3：独立桥与URDF预览
- 已新增：deploy/wincodex_visualization/{bridge.launch,model_preview.launch,start_bridge.sh,start_model_preview.sh}，日志/PID同目录；未修改现有控制/导航文件。
- 启动独立bridge及零关节姿态模型预览（非实测轮角）。启动前不存在TF话题；模型启动器在已有TF时拒绝启动。未添加map/odom到base_link假TF。
- 桥仅开放参数/资源/图谱与有限话题；未开放clientPublish/services能力。监听192.168.3.151:8765。
- 还原：使用同目录PID停止本次两个roslaunch（kill -INT），检查预览节点退出；正常完整bringup前先停止model_preview避免同frame TF竞争。删除新增目录即可撤销部署文件。
- 待验收：Windows真实WebSocket订阅、参数与全部STL资源获取、布局和窗口连接。

## 2026-10-13 17:21:22 CST：资源allowlist修正与停止脚本
- 修正bridge.launch扩展名正则为[.]，重启仅本轮bridge；新增stop_visualization.sh，只核对本轮launch路径后按PID退出两个可视化进程。

## 2026-10-13 17:23:06 CST：可视化阶段4：隔离相机与离线地图预览
- 核验USB相机/dev/video0为LRCP FHD1080P，V4L2支持MJPG 320x240、无人占用；新增camera_preview.launch/start_camera_preview.sh，低分辨率10fps；实际影像仅隔离命名空间wincodex_visualization/camera，不运行识别/停车控制。
- 通过现成Cartographer工具只读导出原map.pbstream至新增deploy/wincodex_visualization/map/pbstream_preview.{yaml,pgm}，新增map_preview.launch/start_map_preview.sh并发布隔离地图/wincodex_visualization/map_preview。未覆盖原地图，未占用/map，未启动定位/导航。
- 更新stop_visualization.sh同时停止本轮camera/map/model/bridge；正式相机或完整bringup前停止预览，避免相机占用和TF竞争。
- 验证待完成：WS图像/地图订阅与Windows布局；地图与模型分面板，无虚假map-base TF。

## 2026-10-13 17:28:10 CST：可视化验收完成
- Windows真实Foxglove WebSocket订阅：约7秒接收scan204、odom135、tf23、tf_static1、隔离静态地图1、压缩相机68；URDF参数11811字符，全部8种STL资源获取status=0；已在Windows Lichtblick 1.29.1窗口实际加载本轮JSON布局，看到模型/雷达、地图、里程计与实时相机。
- 新增start_preview.sh总预览入口、stop_previews.sh仅停模型/地图/相机保留桥；start_bridge.sh是仅桥入口。所有脚本位于独立deploy目录，无开机自启动变更。
- 未测：正式定位地图与map-base TF、真实全局/局部规划Path、比赛状态和视觉停车；未发布任何控制/goal/initialpose。当前为只读桥，发布/服务能力关闭。
- 正式导航入口切换：先停本轮可视化，再在车静止且操作者确认后停bringup_auto.service/其他硬件launch，启动唯一正式入口，再启动仅桥。ROS master重启后重新连接，不承诺自动跨master恢复。
- 还原：deploy目录stop_visualization.sh停止本轮进程；删除独立目录并按依赖情况卸载新增2包。保留wincodexchangelog.md/PROGRESS.md审计记录。

## 2026-10-13 17:32:17 CST：用户要求调试叠框及延迟验收
- 新增debug_preview.launch/start_debug_preview.sh，只运行现有blue_detector_node，不启动visual_servo/race/move_base；public namespace参数正确绑定实际相机，绝对blue_target/debug_image显式remap至本轮隔离话题。
- 调试算法保留当前蓝色HSV H90-130/S80-255/V50-255、5x5开闭、最大轮廓面积>800；有效目标叠绿框/红质心，无目标则原图。颜色仍是蓝，未擅改原检测/控制代码。
- 相机独立预览改320x240@15fps；调试图仅JPEG压缩输出quality65，queue1检测订阅，限制传输排队；只重启本轮camera进程。
- stop/start总脚本加入本轮debug；待Windows真实延迟和GUI叠框验收。已有正式进程未触碰。

## 2026-10-13 17:38:05 CST：调试图与真实接收延迟验收
- Windows默认布局已切隔离blue_target_debug_img/compressed，实际GUI图像更新。当前没有有效蓝目标，未人为伪造目标框；检测代码仍蓝色，红点仅质心标注。
- 320x240、15fps、JPEG quality65；10秒149帧，Pi-Windows时钟校准+676.56ms，最佳交互RTT3.50ms（校准约±1.75ms）；源ROS Header至Windows接收p50=31.34ms/p95=36.00ms/max=43.63ms。没有修改系统时间。粗校准试测误差较大作废，不用于改善幅度结论。
- source-header至显示/曝光缓存未量化；Foxglove WS player为实时更新，无start/pause/seek能力；GUI截图odom序号/时刻持续更新。双窗口含旧原图会多一流，说明建议用户关不需窗口，不主动关闭用户窗口。
- debug输出blue_target无订阅者，未送到运动/race；仅detector预览进程已新增，PID debug launch7632、camera重启launch7687，文件与日志同独立deploy目录。
- 新增detector约10.1%CPU/RSS65MiB，相机15fps约14.8%/76MiB、bridge manager10%/32MiB（累计ps口径，期间双GUI+测量客户端并发）；不是满栈或瞬时稳定CPU。
- Windows布局隐藏TF大字/轴标签，离线地图按201x205像素、0.05m格、原点(-3.7572,-2.99503)居中；新布局需要导入才能同步所有显示调整。

## 2026-10-13 17:39:46 CST：依据实际采集率缓解预览采集环等待
- 官方usb_cam0.3.7源码核验：mmap请求4缓冲、每轮读取一帧不排空旧帧、grab_image后ros::Time::now打header。当前v4l2读120.101fps而ROS预览循环15fps，存在采集环积压风险，header→接收测试不覆盖该隐藏段；这不是已实测的端到端根因。
- 独立camera_preview.launch循环15→30fps并只重启该预览，相同320x240/JPEG65；减小每轮等待与潜在旧帧滞留，不改系统时钟/原控制代码。
- Lichtblick1.29.1 onmessage用本地getCurrentTime标receiveTime，未使用wire timestamp排队，当前未证实两机676ms时钟差导致GUI等待。
- 待核验30fps真实消息率/资源及用户可见延迟；不保证端到端<200ms。

## 2026-10-13 17:40:17 CST：用户接受当前延迟，停止后续调参
- 用户明确摄像头延迟可用，停止后续调试；刚执行的30fps短试验撤回，camera_preview.launch与独立预览恢复已接受的320x240@15fps，debug JPEG65不变。仅重启本轮camera。
- V4L2实际120.101fps与ROS15循环失配、4缓冲及抓帧后打header，仅记录为后续线索；未量化曝光年龄，不作为已确认根因。
- 后续不再调帧率、系统时钟或采集参数；其余只读桥/模型/地图/debug保持可用，运动节点未触碰。

## 2026-10-13：WinCodex 审计交付与最终验收
- 已做：覆盖工作空间说明/业务源码/配置/构建关系，核查远端与Windows的car01 Copilot会话，归档五项问题的详细结论。已接通Windows Lichtblick与独立URDF、地图、相机、蓝色目标调试预览。
- 新增归档文件：`deploy/WINCODEX_AUDIT_2026-10-13.md`、`deploy/wincodex_visualization/WINDOWS_USAGE.md`。Windows交付包含项目审查、历史/Copilot摘要、详细导航审计、两套Lichtblick布局、连接PS1与使用说明。部署脚本/launch/地图导出文件与包安装/各次进程变化详见前述阶段记录。
- 原地图校验：SHA256 `f174e802a47d57f4af228a96e9a39c106ca2890d439ee8a6f3e722dcd597ce7b`；与导出前记录一致=True。现有运动/比赛源码及原有launch未改动。
- 关键结论：当前为麦轮底盘+TEB非全向导航（max_vel_y=0、weight_kinematics_nh=1000、homotopy=false、前瞻2m）；末端视觉仍能横移，无需重刷底盘。看门狗按用户纠正排除为根因。
- 当前比赛不足：状态机没有消费保存点/sendGoal/到点处理；本次无vision_point.yaml；localization.launch仅定位；初始位姿服务响应字段用错；视觉零速完成判定、速度多发布源/接管/输入新鲜度、参数命名空间和地图资产一致性须先修复。检测目标当前蓝色，用户所述红色尚未实现。功能修复本轮只报告。
- 可视化验收：真实Windows WebSocket订阅、URDF参数和全部8种STL资源通过；相机/调试GUI可见。正式Navfn全局/TEB局部路径已配置但生产者未启动，不宣称运行验收。
- 最新用户决策：延迟约0.2～1秒可用，停止优化；最终320×240@15fps/JPEG65。30fps短试验已撤回并记录。31.34ms中位/36.00ms P95只覆盖grab后ROS时间戳到Windows接收，非曝光到屏幕延迟。
- 验证边界：未发底盘速度、goal、initialpose；未运行导航/伺服/比赛运动验证。只读源码快照5 Python/25 launch XML/5 YAML解析通过，可视化脚本/协议实测详见阶段记录。
- 待办（后续功能工作）：按归档报告修复自动比赛/定位/安全/视觉状态机，再进行有顺序的分模块与实车闭环验收。当前用户本轮审查与可视化请求已交付。
- 关键决策：保留既有流畅导航配置和当前可用调试视频；建议到点有界低速扫描，末段全向横移对中；保留所有历史未提交改动。

## 2026-10-13 17:54:43 CST：用户授权修复自动比赛与软件不足
- 用户要求：补齐自动发导航目标/等待到点、有界低速旋转搜索、全向末段对中接近停车，以及上一轮第4项深层控制/安全/参数/地图问题。明确只检测蓝色，不增加红色或颜色切换。
- 已做：再次核对原工作树与运行状态；既有13 tracked修改仍保留。
- 实施计划：建立包含当前未提交源码的基线和三个隔离git worktree（根 /home/ubuntu/car01_wincodex_fix_20261013），分别修安全仲裁、视觉、决策/定位地图；分批提交、隔离ROS测试与Noetic编译，主代理整合验收后有哈希保护部署。
- 改动记录：隔离分支每阶段给文件与commit/测试摘要，追加本日志和PROGRESS；最终给备份、部署文件清单与回退方法。
- 关键决策：保留稳定非全向TEB和用户接受的视频设置，不处理底盘看门狗、不凭猜测改变MCU坐标符号。测试不向实际11311 ROS master发车速、goal或initialpose，不启动真实比赛/重启底盘服务。
- 待办：本轮功能修复、测试、部署与交付；实际停车距离/标定仍以可配置参数和后续实车验收为准，不以编译通过冒充实车通过。

## 2026-10-13 18:03:53 CST：隔离基线与接口确认
- 已做：从原当前工作树159个非生成文件建立真实dirty基线，BASE `aa79e092c2049ac79971ed0148db19dbd5a316cf`；创建 integration/safety/vision/race 工作树与分支，根 `/home/ubuntu/car01_wincodex_fix_20261013`。
- 备份：根下 `backup_original/`（完整src及其他资产/双日志）、`original_manifest.json`（原status与文件SHA256）。复制前后hash一致，原main/index/源码未动。
- 关键接口：自动TwistStamped `/cmd_vel_auto`→仲裁 `/cmd_vel_requested`→激光安全唯一 `/cmd_vel`；视觉使用源时间戳目标/速度和显式Diagnostic完成状态，旧Point蓝目标与debug画面兼容。
- 地图方案：未来新bundle事务保存，不覆盖现有地图；保存点/最终起点用trajectory anchor重投影；定位正确转换frozen reference相对位姿、检查实际服务返回。最终结束通过受控mapping入口先保存再关闭，避免依赖同时CtrlC回调顺序。
- 已纠正：用户只要蓝色；接口文档legacy目标类型按真实源码改为geometry_msgs/Point。首次准备出现Python3.8不支持zoneinfo/pathlib.readlink，均在功能写入前失败，已分别改固定时区datetime/os.readlink后重试成功。
- 改动文件：隔离工作树与备份/manifest；主仓库本阶段只追加双日志。具体功能改动待各分支编译测试完成再分阶段记录。
- 待办：完成三分支实现/验证，整合复审并以原hash守护部署。


## 2026-10-15 18:15 CST：续接修复完成（WorkBuddy）
- 已做：续接 10-13 中断的执行。safety 50/50、vision（断言修正后）gtest+rostest 全过、race 构建+假环境测试 2/2；三分支提交并合并 integration，重跑三项测试全过；39 文件哈希守护校验后部署至 /home/ubuntu/car01 并重编。
- 改动文件：见 wincodexchangelog.md 2026-10-15 条目（16 修改 + 23 新增）。
- 待办：实车验收（场地建图→B 点保存→localization 一键导航→视觉停车）；如需全量构建恢复空白名单。
- 关键决策：vision rostest 失败判定为 ROS1 rospy↔roscpp header.seq 不保真的测试问题，节点行为正确，未改节点只改断言。
