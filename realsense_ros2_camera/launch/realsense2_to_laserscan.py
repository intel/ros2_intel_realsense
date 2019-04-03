# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch face detection and rviz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch.actions


def generate_launch_description():
    enable_align_depth = launch.substitutions.LaunchConfiguration('enable_aligned_depth', default="false")
    output_frame = launch.substitutions.LaunchConfiguration('output_frame', default="base_scan")
    range_max = launch.substitutions.LaunchConfiguration('range_max', default="2.0")
    range_min = launch.substitutions.LaunchConfiguration('range_min', default="0.2")
    return LaunchDescription([
        # Realsense
        launch_ros.actions.Node(
            package='realsense_ros2_camera',
            node_executable='realsense_ros2_camera',
            node_name='realsense_ros2_camera',
            parameters=[{'enable_aligned_depth':enable_align_depth}],
            output='screen'), 
        launch_ros.actions.Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[{'output_frame': output_frame},
                        {'range_min': range_min},
                        {'range_max': range_max}],
            arguments=["depth:=/camera/depth/image_rect_raw",
                       "depth_camera_info:=/camera/depth/camera_info",
                       "scan:=/scan"]),
    ])