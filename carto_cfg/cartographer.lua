include "map_builder.lua"
include "trajectory_builder.lua"

--?[ Front End Param ]
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
-- 算法跟踪的坐标系: robot base tf
  tracking_frame = "base_link",
-- Odometry
  -- 发布位姿的坐标系(map直连)
  published_frame = "odom",
  -- Carto 提供 odom
  provide_odom_frame = false,
  odom_frame = "odom",
  use_odometry = true,

  publish_frame_projected_to_2d = false, -- 纯2D姿态
  use_pose_extrapolator = true, -- 位姿推算器
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0, -- 点云(3D)
-- 时间
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
-- 采样率
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


--?[ MAP_BUILDER ]
MAP_BUILDER.use_trajectory_builder_2d = true
-- 里程计估计的相对旋转量的权重
MAP_BUILDER.pose_graph.optimization_problem.odometry_translation_weight = 1e5
MAP_BUILDER.pose_graph.optimization_problem.odometry_rotation_weight = 1e5
-- 全局定位采样器的频率
MAP_BUILDER.pose_graph.global_sampling_ratio = 0.003
--* 回环约束
MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.55 -- 筛选约束的最小匹配得分
MAP_BUILDER.pose_graph.constraint_builder.global_localization_min_score = 0.6 --筛选全局约束的最小匹配得分
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.3 -- 回环检测采样频率
MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 6 --! 最大约束限制距离(default = 15.)
-- translation 和 rotation 的权重
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_translation_weight = 1.1e4
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_rotation_weight = 1e5
-- 快速相关匹配: XY方向搜索窗长 / 航向角搜索窗长 / 分支定限深度
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.1
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
-- 匹配权重:
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20 -- 栅格匹配权重
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight = 10    -- 平移量匹配权重
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight = 1.       -- 旋转量匹配权重


--?[ TRAJECTORY_BUILDER_2D ]
-- 单个子图插入激光点云数量: 根据 laser 和 velocity 进行具体的调整
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
-- Laser Range
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 实时相关匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
--! ceres地图的扫描, 平移, 旋转的权重, 影响建图效果, 其他基本上是影响计算量
-- 扫描匹配点云和地图匹配程度, 值越大, 点云和地图匹配置信度越高
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.
-- 残差(平移/旋转)分量, 值越大, 越不相信和地图匹配的效果, 而是越相信先验位姿的结果
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
-- 如果imu不好, 接入后地图旋转厉害, 可以将这里的旋转权重减小
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.


--?[ POSE_GRAPH ]
-- >= num_range_data * 2
POSE_GRAPH.optimize_every_n_nodes = 60.
POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- Huber损失函数参数
-- 值为正: 开启回环 
POSE_GRAPH.optimize_every_n_nodes = 35
--! 回环检测阈值, 如果不稳定有错误匹配, 可以提高这两个值, 场景重复可以降低或者关闭回环
POSE_GRAPH.constraint_builder.min_score = 0.80
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80



return options