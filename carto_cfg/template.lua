include "map_builder.lua"
include "trajectory_builder.lua"
 
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",--MAP坐标系名称
  tracking_frame = "imu_link",--算法跟踪的坐标系名称 imu_link laser
  --  可以看到map与base_link直接缺少tf链接。这里需要在配置文件中把这个  “provide_odom_frame = true,--是否提供里程坐标系 “  参数设置为true；
  -- Could not compute submap fading: Could not find a connection between 'map' and 'base_link' because they are not part of the same tree.Tf has two or more unconnected trees.
  --  rosrun  tf2_ros  static_transform_publisher x y x yaw pitch roll  /odom /base_link  rosrun  tf2_ros  static_transform_publisher 0 0 0 0 0 0  /odom /base_link 10
  --  rosrun  tf  static_transform_publisher x y x yaw pitch roll    /odom /base_link    rosrun  tf  static_transform_publisher 0 0 0 0 0 0    /odom /base_link 10
  published_frame = "base_link",--发布位姿的坐标系名称
  odom_frame = "odom",--里程计坐标系名称（仅在provide_odom_frame为true时使用）
  provide_odom_frame = true,--是否由carto提供里程坐标系
  publish_frame_projected_to_2d = false,--是否布的姿态将被限制为纯2D姿态(无滚动、俯仰或z-偏移)
  use_pose_extrapolator = true,--是否使用位姿推算器
  use_odometry = false,--是否使用编码器提供的里程信息。如果是，需提供主题“odom”上的nav_msgs/Odometry信息
  use_nav_sat = false,--是否使用GNSS信息。如果是，需提供主题“fix”上订阅sensor_msgs/NavSatFix信息
  use_landmarks = false,--是否使用路标点信息。如果是，需提供主题“Landmarks”上订阅cartographer_ros_msgs/LandmarkList信息
  num_laser_scans = 1,--订阅激光扫描话题数量
  num_multi_echo_laser_scans = 0,--订阅多回波激光扫描话题数量
  num_subdivisions_per_laser_scan = 1,--激光扫描数据分割段数量
  num_point_clouds = 0,--订阅点云话题的数量(only 3D)
  lookup_transform_timeout_sec = 0.2,--等待tf变换超时时间（单位：秒）
  submap_publish_period_sec = 0.3,--子图发布时间间隔（单位：秒）
  pose_publish_period_sec = 5e-3,--位姿发布时间间隔（单位：秒）
  trajectory_publish_period_sec = 30e-3,--轨迹发布时间间隔（单位：秒）
  -- imu_publish_period_sec = 2e-2, --it's used when use_pose_smoother is true weite
  odometry_sampling_ratio = 1.,--里程计数据处理采样率
  fixed_frame_pose_sampling_ratio = 1.,--固定帧点数据处理采样率
  imu_sampling_ratio = 1.,--imu数据处理采样率
  rangefinder_sampling_ratio = 1.,--激光扫描数据处理采样率
  landmarks_sampling_ratio = 1.,--地标点数据处理采样率
--weite todo
  -- use_pose_smoother = true, --true --
  -- smoother_variety_distance = 0.05,--
  -- relocalization_variety_distance = 0.1,--
}
TRAJECTORY_BUILDER_2D.use_imu_data = true--是否使用IMU数据
MAP_BUILDER.use_trajectory_builder_2d = true--是否2D定位场景（和3D定位场景二选一）
MAP_BUILDER.collate_by_trajectory = false
TRAJECTORY_BUILDER_2D.max_range = 30.--自适应体素滤波的激光扫描有效测距上限（单位：米）
TRAJECTORY_BUILDER_2D.min_range = 0.3 --自适应体素滤波的激光扫描有效测距下限（单位：米）
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --执行一次前端匹配需要的激光扫描分段数量（建议和参数num_subdivisions_per_laser_scan保持一致即表示整周扫描数据）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60 --单个submap存储的激光点云数量上限 default-90
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2. --超出激光有效距离上限的替换值（单位：米）
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true --是否更新miss栅格
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10. --Ceres匹配的占据栅格权重 default=20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10. --Ceres匹配的平移量权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40. --Ceres匹配的旋转量权重 default=1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 --最大迭代次数 defaul=10
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 --分辨率
MAP_BUILDER.pose_graph.constraint_builder.log_matches = false --是否打开匹配日志输出 default=true
MAP_BUILDER.pose_graph.global_sampling_ratio = 0.003 --全局定位采样器的频率
MAP_BUILDER.pose_graph.log_residual_histograms = false --日志是否输出残差直方图 default = true
MAP_BUILDER.pose_graph.global_constraint_search_after_n_seconds = 10 --计算两次全局约束的最短间隔时间
MAP_BUILDER.pose_graph.optimization_problem.log_solver_summary = false --后端优化器求解时是否启用记录日志
MAP_BUILDER.pose_graph.optimization_problem.huber_scale = 1e1 --Huber损失函数参数
MAP_BUILDER.pose_graph.optimization_problem.ceres_solver_options.num_threads = 2 --开启线程数 default = 7 
MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.5 --筛选约束的最小匹配得分 default = 0.55
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.3 --回环检测采样频率
MAP_BUILDER.pose_graph.constraint_builder.global_localization_min_score = 0.6 --筛选全局约束的最小匹配得分
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_translation_weight = 1.1e4 --回环约束的平移量权重
MAP_BUILDER.pose_graph.constraint_builder.loop_closure_rotation_weight = 1e5 --回环约束的旋转量权重
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7 --快速相关匹配的XY方向搜索窗长（单位：米）default = 0.1
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.) --快速相关匹配的航向角方向搜索窗长（单位：弧度）default=math.rad(20.),
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7 --快速相关匹配的分支定界深度
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20 --Ceres匹配中栅格匹配权重
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight = 10 --Ceres匹配中平移量匹配权重（平移量是指通过其它方法预测的）
MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight = 1. --Ceres匹配中旋转量匹配权重（旋转量是指通过其它方法预测的）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --是否启用实时相关匹配器（2D场景建议使用）defaul=false
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5. --再次往submap插入新激光点云数据的最短时间间隔（单位：秒）
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2 --再次往submap插入新激光点云数据的最小移动距离（单位：米）
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5.) --再次往submap插入新激光点云数据的最小航向转角（单位：弧度）default= math.rad(1.),
MAP_BUILDER.num_background_threads = 3 --MapBuilder后台计算的线程数量 defaul=4
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 60 --启用后端优化每次需要新增的节点数量 default=90
MAP_BUILDER.pose_graph.optimization_problem.odometry_translation_weight = 1e5 --基于里程计估计的相对旋转量的权重
MAP_BUILDER.pose_graph.optimization_problem.odometry_rotation_weight = 1e5 --基于里程计估计的相对旋转量的权重
MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 6 --最大约束限制距离（单位：米）default= 15
--weite
--MAP_BUILDER.pose_graph.optimize_every_meter = 2 --0.5 
--MAP_BUILDER.pose_graph.constraint_builder.max_match_variety_distance = 15.0 --15 
--MAP_BUILDER.pose_graph.local_constraint_search_after_n_seconds = 5.0 --1.0 
return options