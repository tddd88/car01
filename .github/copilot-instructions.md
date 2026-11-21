# 工作空间协作说明（for GitHub Copilot）

本工作空间同时由多个 AI 编码助手（ssh 远程）维护，各有独立日志、互不覆盖：

- **ZCode**：改动记录在 [ZCODE_CHANGELOG.md](../ZCODE_CHANGELOG.md)。
- **WinCodex / WorkBuddy**：改动记录在 [wincodexchangelog.md](../wincodexchangelog.md)。2026-10-13~15 的修复轮次（统一速度安全链、比赛状态机、地图资产事务、视觉接口）已部署到本工作树，**未提交**，叠加在既有未提交改动之上。

规则：

1. 修改 `src/navigation`、`src/robot_bringup`、`src/robot_slam`、`src/decision`、`src/vision_control` 下的文件前，先读 `ZCODE_CHANGELOG.md` 和 `wincodexchangelog.md`，避免与既有改动冲突或误回退。
2. **`/cmd_vel` 只有 laser_safety_node 一个发布者**。任何新的自动速度源必须发布 `/cmd_vel_auto`（geometry_msgs/TwistStamped，header.stamp 为生成时刻）经仲裁与安全层，禁止直发 `/cmd_vel` 或 `/cmd_vel_raw`。
3. 地图/预备点是按 map_id 绑定的资产组，通过 `robot_bringup/nodes/asset_bundle.py` 的原子写接口操作，不要手工编辑 `src/robot_slam/map/bundles/` 下的文件。
4. 你（Copilot）自己的改动继续记录在 `CHANGELOG.md`。
5. 外部修复的隔离工作树、完整备份与回滚补丁在 `/home/ubuntu/car01_wincodex_fix_20261013/`；如需回退其改动，先看 `wincodexchangelog.md` 的回退方法。
