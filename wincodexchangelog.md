# WinCodex/WorkBuddy 变更记录（精简版）

供 Copilot 和后续维护者查看。时间为 Asia/Shanghai。
完整逐阶段原始记录（含每次安装/进程/脚本/验收细节）归档于 `deploy/wincodex_logs_archive_2026-10-15/`。

## 快速索引（当前状态 @2026-10-15）
- 主机：`ubuntu@192.168.3.151`，工作空间 `/home/ubuntu/car01`（ROS Noetic，Pi 4B 8GB）
- 修复轮根目录：`/home/ubuntu/car01_wincodex_fix_20261013/`（integration/safety/vision/race 四 worktree + `backup_original/` 完整备份 + `original_manifest.json` + `deploy_fix.patch`）
- 提交：基线 `aa79e09`（原 dirty 状态快照）；safety `4cfb925`、vision `8d38357`、race `a6b190f`；integration 合并 `3649e3c`。原 main/index 未动
- **控制链（重要，改代码前必读）**：`/cmd_vel_auto`(TwistStamped，自动唯一入口) → cmd_vel_arbiter → `/cmd_vel_requested` → laser_safety_node → **`/cmd_vel` 唯一发布者**；手柄 `/joy` → joy_repeat → `/cmd_vel_raw` 同入仲裁。新速度源一律走 `/cmd_vel_auto`，禁止直发 `/cmd_vel`
- 可视化：`deploy/wincodex_visualization/`（Foxglove bridge，Windows Lichtblick 连 192.168.3.151:8765；用法见同目录 WINDOWS_USAGE.md）
- 回退：`git -C /home/ubuntu/car01 checkout -- <路径>` 恢复修改文件、删除新增文件；或按 backup_original/ 恢复
- 边界：本轮全部为隔离 master/假输入测试，**未实车验证**

## 2026-10-13：项目审计 + Windows 可视化接通
- 任务：审查 car01 全工作空间与 Copilot/Zcode 历史，回答五项问题（全向底盘、比赛流程、视觉停车、软件不足、可视化）。结论归档 `deploy/WINCODEX_AUDIT_2026-10-13.md`，Windows 侧另交付审查/导航审计/历史摘要/布局/连接脚本。
- 系统级安装：`ros-noetic-foxglove-bridge 0.8.4`（依赖 ros-babel-fish 0.9.3）。回退：`sudo apt-get remove ros-noetic-foxglove-bridge ros-noetic-ros-babel-fish`
- 新增文件：`deploy/wincodex_visualization/`（bridge/model/map/camera/debug 各 preview 的 launch 与 start/stop 脚本，日志 PID 同目录）；地图只读导出 `map/pbstream_preview.{yaml,pgm}`，原 map.pbstream 未动（SHA256 `f174e802...597ce7b` 导出前后一致）
- 验收：Windows Lichtblick 1.29.1 实连 WebSocket（7 秒收 scan204/odom135/tf23/隔离地图/相机压缩流）；URDF 11811 字符 + 全部 8 种 STL status=0；调试图延迟 p50=31.34ms/p95=36.00ms（仅覆盖 ROS header→Windows 接收）
- 用户决策：摄像头延迟 0.2~1s 可接受、停止调参；最终 320×240@15fps/JPEG65（30fps 短试验已撤回）
- 关键结论：麦轮+TEB 非全向粗导航（max_vel_y=0、weight_kinematics_nh=1000、homotopy=false），视觉末段仍可横移，无需重刷底盘；看门狗经用户纠正**排除**为卡顿根因；比赛链路五处不足（状态机不消费存点/goal、视觉检测蓝色、初始位姿 API 字段错、视觉零速完成判定、速度多发布源）——本轮只报告未改
- 边界：未发任何速度/goal/initialpose，未动运动源码

## 2026-10-13 17:54~18:03：用户授权修复 + 隔离基线
- 授权内容：修第 3/4 项（自动比赛链路 + 深层控制/安全/参数/地图问题）；**只检测蓝色**、不加红色；保留非全向 TEB；不碰看门狗；改动必须写本 changelog
- 基线：从原树 159 个非生成文件建 dirty 基线 `aa79e09`；`backup_original/` 完整备份 + `original_manifest.json`（原 status 与文件 SHA256），复制前后 hash 一致
- 接口契约：统一安全出口（如上）；`/vision/target`(Vector3Stamped，源图像 header) + `/visual_servo/status` 显式 WAITING/TRACKING/ALIGNED/REACHED/LOST/FAULT；地图 bundle 事务（map_id/原子写/版本匹配）；Cartographer 相对初始位姿 + status code 检查；测试隔离端口 11401/11402/11403，严禁连 11311

## 2026-10-15 18:15：三分支修复完成 + integration 验收 + 哈希守护部署（WorkBuddy 续接 Codex）
- safety：新增 `cmd_vel_arbiter_node.cpp`，重写 joy_repeat/laser_safety。隔离测试 **50/50 全过**（人工优先、使能粘滞、失联锁止、footprint 旋转扫掠碰撞边界、加速度限制、雷达 TF 变换、陈旧/未来时间戳拒绝、输出断流持续零速）
- vision：blue_detector 发 `/vision/target`（保留源 header）；visual_servo 墙面几何模块（`include/vision_control/servo_geometry.h`）分离偏航/横移/接近，显式 REACHED 稳态判定。gtest 6/6 + rostest 通过。注：rostest 断言修正为只比 frame_id+stamp——ROS1 rospy→roscpp→rospy 往返 header.seq 不保真（实测 C++ 端恒为 0），节点拷贝行为本身正确
- race：比赛状态机（就绪门禁→唯一 goal→有界旋转搜索→锁定蓝目标→视觉 REACHED 稳定→FINISHED；`/race/state` 与 `/race/start|pause|resume|reset`；手动接管即暂停）；地图资产事务 `asset_bundle.py`；`initial_pose_loader.py` 按 Cartographer 相对初始位姿语义 + `/localization/ready`；`auto_save_map.py` 受控保存顺序（先存图再退出定位）。假环境测试 2/2
- 整合部署：三分支无冲突合并 integration（`3649e3c`）全量构建通过、三项测试重跑全过；diff `aa79e09`→integration 共 **39 文件（16 改 23 新增）**，应用前逐文件 hash 守护校验（修改文件与基线 blob 一致、新增文件原树不存在）39/39 零违例，`git apply` 干净应用至 `/home/ubuntu/car01` 并重编，devel 新二进制已生成
- 回退：`git -C /home/ubuntu/car01 checkout -- <路径>`；新增文件直接删；补丁存于 `/home/ubuntu/car01_wincodex_fix_20261013/deploy_fix.patch`
- 边界：未向 11311 发速度/goal/initialpose，未启动比赛，未实车验证——需场地实车闭环验收后方可认为比赛链路可用
