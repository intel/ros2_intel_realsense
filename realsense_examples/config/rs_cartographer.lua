-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "camera_pose_optical_frame",
  published_frame = "odom_frame",
  odom_frame = "odom_frame",
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
  submap_publish_period_sec = 0.05,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.07)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 300
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 150
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.035
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 350
POSE_GRAPH.constraint_builder.sampling_ratio = 0.9

POSE_GRAPH.optimization_problem.huber_scale = 1
POSE_GRAPH.constraint_builder.min_score = 0.7

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 300 
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 50
POSE_GRAPH.optimize_every_n_nodes = 150

return options
