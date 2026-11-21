# 小车 Windows 可视化

已接通：树莓派 Foxglove bridge → Windows Lichtblick 1.29.1。当前窗口已经连接并加载预览布局。重新打开按以下三步：

1. 右键“打开小车可视化.ps1”选择“使用 PowerShell 运行”，或在 Lichtblick 的 Open connection 中选择 Foxglove WebSocket，填 `ws://192.168.3.151:8765`。
2. 打开 Layouts → Import layout，选择 `car01-lichtblick-preview.json`。
3. 左上看车模零关节预览和真实雷达；左下看 pbstream 离线导出的地图；右上看真实蓝色检测调试图（绿框、红色质心）；右下看真实里程计。地图和车模分面板，因为当前没有定位提供的 map→base_link TF。

## 当前实测

约7秒真实订阅收到雷达204条、里程计135条、动态TF23条、静态TF1条、地图1条、压缩相机68条。URDF参数11811字符，全部8种STL资源读取成功。GUI已实际看到模型/雷达、离线地图、实时相机和odom。当前相机320×240、15fps，调试JPEG质量65；CameraInfo尚未标定，Image面板未绑定该参数，不能把图像投影当作已标定结果。

精确来源：
- URDF参数：`/wincodex_visualization/robot_description`；真实内部关节框架 `/tf`、`/tf_static`，轮关节为零值预览，非编码器实测。
- 地图：`/wincodex_visualization/map_preview`，源是已有 `src/robot_slam/map/map.pbstream`，只读导出到新增路径，没有占用 `/map`。
- 相机：`/wincodex_visualization/blue_target_debug_img/compressed`（默认调试画面）；原图对照 `/wincodex_visualization/camera/usb_cam/image_raw/compressed`；CameraInfo为 `/wincodex_visualization/camera/usb_cam/camera_info`。
- 雷达 `/scan`；里程计 `/odom_raw`。

## 树莓派启动与停止

以下命令在树莓派 SSH 中运行，部署目录为 `/home/ubuntu/car01/deploy/wincodex_visualization`。

- 独立预览入口：`bash /home/ubuntu/car01/deploy/wincodex_visualization/start_preview.sh`，已运行者跳过，启动桥/模型/地图/相机。
- 仅桥入口：`bash /home/ubuntu/car01/deploy/wincodex_visualization/start_bridge.sh`，前台运行；已有桥时不要重复启动。
- 停预览、保留桥：`bash /home/ubuntu/car01/deploy/wincodex_visualization/stop_previews.sh`。
- 停本轮全部可视化：`bash /home/ubuntu/car01/deploy/wincodex_visualization/stop_visualization.sh`。只核对本轮launch路径与PID，不停止既有底盘或传感器。

正式完整硬件入口前先停相机和模型预览。若会重启ROS master，先停本轮全部可视化，完成已有硬件入口切换，再启动仅桥入口、重新连接Windows。

## 正式导航布局与尚待验收

`car01-lichtblick-navigation.json` 预置正式 `/map`、URDF `/robot_description`、TF、代价地图与TEB路径，调试图像来源 `/rgb/blue_target_debug_img/compressed`（需正式检测器生产；当前不运行正式入口）。这份布局尚未在正式导航运行态验收，当前没有运行 move_base/比赛。

当前全局规划器使用 move_base 默认 NavfnROS，预置完整全局路径 `/move_base/NavfnROS/plan`；TEB局部路径为 `/move_base/TebLocalPlannerROS/local_plan`，其裁剪/变换后的全局段为 `/move_base/TebLocalPlannerROS/global_plan`。代价地图为 `/move_base/global_costmap/costmap` 和 `/move_base/local_costmap/costmap`。以上已按本机配置与对应版本源码核对，实际启动后仍需运行验收。当前没有路径生产者。正式相机默认 `/usb_cam/image_raw`，旧RViz的 `/rgb/image_raw` 不能当实际来源；只看原图时在Image设置选择实际压缩话题。

Lichtblick可查看地图、模型、TF、路径、雷达和相机，适用于这些RViz查看工作；RViz第三方插件不会自动迁移。本轮桥是只读配置，没有开放发goal、initialpose、底盘速度或服务调用能力。

## 改动、资源与还原

新增官方APT包仅两个：`ros-noetic-foxglove-bridge=0.8.4-1focal.20250520.090425` 和 `ros-noetic-ros-babel-fish=0.9.3-1focal.20250520.085837`，没有全系统升级。项目改动只在新增独立deploy目录和追加 `wincodexchangelog.md`、`PROGRESS.md`，未改运动/比赛源码或现有launch。

验收时新增6个ROS进程RSS合计约190MiB，ps累计CPU约14.3%（单核100%口径，非满栈压力测试）；全机可用约7103MiB，50.15℃，throttled=0x0。

撤销时先运行stop_visualization.sh，再删除该独立deploy目录；若无其他依赖再 `sudo apt-get remove ros-noetic-foxglove-bridge ros-noetic-ros-babel-fish`。审计日志建议保留。全部PID和日志在deploy目录，实际当前roslaunch PID：bridge 6144、model 5753、camera 6607、map 6664（重启会变化，以*.pid为准）。


## 用户要求的调试画面与延迟

当前默认布局已选 `/wincodex_visualization/blue_target_debug_img/compressed`。只运行现有蓝色检测器，HSV H90–130/S80–255/V50–255、5×5开闭、最大轮廓面积>800px，绿框和红色质心来自同一有效蓝色轮廓。当前场景没有满足条件的蓝目标时，调试图仍更新但不画框；红点是标注，目标检测颜色仍为蓝色。未启动视觉伺服、比赛或速度发布。

精校准Pi-Windows时钟差约+676.56ms（未修改任一系统时间），最小SSH交互RTT3.50ms，校准不确定度约±1.75ms。10秒149帧，源ROS Header→Windows WebSocket接收延迟p50=31.34ms、p95=36.00ms、最大43.63ms。该数据不含驱动打时间戳前的曝光缓存与GUI解码/渲染/屏幕刷新，不能用它宣称完整画面延迟已经修复。原图另测p50=39.25ms/p95=42.66ms（不同时间采样，不作为算法改善幅度）。

保留右侧调试窗口，关闭不需要的旧原图窗口可减少重复流；同一布局仅开一个压缩debug Image面板。确认顶部来源为 `ws://192.168.3.151:8765`、图像和odom时间持续变化。此版本的Foxglove WebSocket是实时player，没有录包播放器的暂停/seek能力，不能套用“跳到Live”时间轴说明；若画面停滞，重新Open connection建立该WS连接以清理会话，再只打开debug面板对比。若仍有明显视觉落后，可将独立计时器置于镜头视野，与屏幕同时拍摄比较曝光→显示，当前这部分尚未量化。

独立调试入口 `bash /home/ubuntu/car01/deploy/wincodex_visualization/start_debug_preview.sh`（总preview入口已包含，勿重复运行）；停止脚本同时覆盖本轮debug。默认debug仍保持用户当前代码蓝色逻辑，红色比赛目标尚需单独修改和验收，不会通过可视化悄悄改变控制算法。

用户已确认当前可见延迟约0.2～1秒可用，停止进一步优化。30fps短试验已撤回，最终保留上述15fps设置；具体试验与还原已写远端双日志。


用户已确认当前摄像头延迟可接受，最终保持320×240@15fps、JPEG65，停止继续调参。一次30fps短试验已撤回。

时间戳边界已核实：安装的usb_cam0.3.7是在 `grab_image()` 返回后才用 `ros::Time::now()` 写header，所以上面的31/36ms从取帧结束算起，不包含曝光、USB或V4L2缓存。V4L2只读实测设备120.101fps、ROS循环15fps，官方驱动mmap请求4缓冲且逐帧取队列，这只是后续采集延迟调查线索，尚未直接测曝光年龄。原始源码：[usb_cam0.3.7](https://github.com/ros-drivers/usb_cam/blob/0.3.7/src/usb_cam.cpp#L1238)。
