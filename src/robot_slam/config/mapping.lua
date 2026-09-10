include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 没有IMU，纯靠雷达和轮式里程计
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 雷达参数，视硬件调节
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 里程计的信任度（麦轮容易打滑，所以不要太信任里程计）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 扫描匹配残差权重（雷达数据的可信度，保持较高）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- 里程计先验权重（麦轮打滑+无IMU，降低里程计对位姿的约束，让扫描匹配主导）
-- 默认为 0，Cartographer 会用 translation_weight/rotation_weight 作为里程计权重；
-- 显式设小值，让里程计只作为初值猜测，不强制约束最终位姿。
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.odometry_translation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.odometry_rotation_weight = 1.0

-- 在线相关扫描匹配的搜索窗口（默认较小，剧烈转弯时里程计先验偏差大，需要扩大搜索范围）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_range = 3.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_range = math.rad(60.0)

-- 子图大小，可根据比赛场地适度调小，加速匹配
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50

return options
