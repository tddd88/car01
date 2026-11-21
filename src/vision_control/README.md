# 蓝色目标末段精调

正式比赛以 `/vision/target`（Vector3Stamped）与 `/visual_servo/status`（DiagnosticArray）为接口。
检测结果、速度和状态保留图像源时间；旧 `/blue_target`（Point）、`/cmd_vel_vision` 和
`/rgb/blue_target_debug_img` 保留供原调试布局使用。旧无时间戳话题不参与比赛安全决策。

参数优先读取节点私有命名空间，兼容原独立预览 group 的 public 参数。HSV 只检测蓝色，
默认 H90–130/S80–255/V50–255；参数在启动时读取。调试画面仍为原绿框和红质心，
未改相机采集或压缩配置。`vision_debug.launch` 默认只检测；开启伺服仍只发视觉输入，
不再把视觉速度 relay 直通 `/cmd_vel`。

状态按 20 Hz 连续发布：WAITING、TRACKING、ALIGNED、REACHED、LOST、FAULT。
首个目标之前、超过 0.3 秒丢目标、过期/非有限目标、缺雷达/TF、陈旧雷达、无可信墙线均清零。
重复的同一图像时间戳不刷新存活时间、不累计完成帧数。速度为零不代表 REACHED。

雷达点在采集时间经真实 TF 转到 `base_link`，在前方 0.50 rad 半窗口内进行最多128点的
确定性 RANSAC 和正交直线拟合。拟合必须满足点数、宽度、残差、内点比例和朝向约束。
先对齐墙面法向，再以麦轮横移使蓝色目标对中，最后接近/微量后退调整墙距；各阶段只控制一个
自由度。接近和近墙横移默认不超过 0.04 m/s，其他阶段上限 vx=0.10、vy=0.08、wz=0.20。
所有输出必须经过比赛状态机、仲裁器和统一雷达安全层。

`standoff_distance=0.30 m` 指 **base_link 原点到墙面的垂直距离**，不是车头到墙的净空。
完成要求目标对中误差 ≤0.025、墙面朝向误差 ≤0.04 rad、距离误差 ≤0.015 m，
且持续0.8秒、至少5个不同图像时间戳。容差是控制参数，不能承诺厘米级实车精度。
`allow_area_fallback=false` 默认禁用；显式开启只允许保守面积接近，始终不宣告 REACHED。

现场必须核实：目标位于拟合终点墙、墙面可被雷达可靠扫描、雷达 TF 正确、相机固定朝前，
`center_x` 对应真实停车中心、`lateral_sign` 对应 ROS +Y 左移，距离设定大于安全层所需净空。
现有 USB 相机在 grab 后才打 ROS 时间戳，因此图像源时间不能证明曝光无缓存；
墙距/墙朝向以新鲜雷达为准，近墙动作保守限速，摄像头延迟仍需实车标定。

## 隔离测试

仅在私有测试 master `http://127.0.0.1:11402`，ROS_IP=127.0.0.1，禁止实体驱动。
先 source Noetic 和 carto overlay，在隔离 worktree 以 -j1 编译 vision_control。
`servo_geometry_test` 检查带噪墙距/有符号朝向、杂物与多面墙拒绝、分轴/方向、稳态帧数、
丢失/缺雷达立即停车、面积无法单独完成和限速；`vision_interfaces.test` 使用假图像、
带平移/旋转/高度的假雷达 TF，实测私参和旧 public 参数、原 header、首次零、
0.3秒丢失、稳态到达、过期消息、无效值、缺TF与扫描失联。
