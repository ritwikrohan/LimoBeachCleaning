-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "base_link",
--   published_frame = "odom",
--   odom_frame = "odom",
--   provide_odom_frame = false,
--   publish_frame_projected_to_2d = true,
--   use_odometry = true,
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 1,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.min_range = 0.12
-- TRAJECTORY_BUILDER_2D.max_range = 3.5
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)


-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- -- POSE_GRAPH.optimize_every_n_nodes = 0

-- return options

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
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

-- Avoid duplicate key entries
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0  -- Increased max range to capture more data
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- Increased to handle more missing data
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- If you have an IMU, change to true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- Adjusted to balance data collection
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  -- Fine-tuning the filtering

-- Improve robustness of scan matching 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.0

-- Adjust motion filter to reduce frequency of minor map updates
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004

-- Optimize pose graph and set higher constraints 
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimize_every_n_nodes = 90

-- Additional settings
POSE_GRAPH.constraint_builder.sampling_ratio = 0.9
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.0

return options
