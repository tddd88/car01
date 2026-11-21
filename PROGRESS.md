# car01 交接进度（@2026-10-15 更新）

## 当前状态
- 本轮（10-13~10-15）外部 AI（WinCodex/WorkBuddy 续接 Codex）的审计、Windows 可视化接通、比赛链路修复已全部完成并部署到本工作树（未提交，叠加在既有未提交改动之上）。
- 详细记录：`wincodexchangelog.md`（精简版）；逐阶段完整原始记录在 `deploy/wincodex_logs_archive_2026-10-15/`。
- **控制链已变更**：`/cmd_vel_auto` → cmd_vel_arbiter → `/cmd_vel_requested` → laser_safety → `/cmd_vel`（唯一发布者）；手柄链路同入仲裁。新速度源必须走 `/cmd_vel_auto`。

## 关键路径
- 修复工作树：`/home/ubuntu/car01_wincodex_fix_20261013/`（integration/safety/vision/race、backup_original/、original_manifest.json、deploy_fix.patch）
- 可视化：`deploy/wincodex_visualization/`（Windows 用法见同目录 WINDOWS_USAGE.md）
- 审计归档：`deploy/WINCODEX_AUDIT_2026-10-13.md`
- 各 agent 日志：Copilot→`CHANGELOG.md`；ZCode→`ZCODE_CHANGELOG.md`；WinCodex/WorkBuddy→`wincodexchangelog.md`

## 待办
1. 实车闭环验收：场地建图（受控 mapping 入口，结束先存图再退）→ B 点保存 → 回起点 → localization 一键导航 → 有界搜索 → 视觉对中停车
2. 停车距离/图像面积阈值按实车标定（面积 fallback 完成判定默认禁用）
3. 实车验收通过后再考虑整理提交工作树改动
