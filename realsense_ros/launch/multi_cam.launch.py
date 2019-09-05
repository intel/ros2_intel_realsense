# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# /* Author: Gary Liu */

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_ros'), 'config', 'rs_cartographer.rviz')
    return LaunchDescription([

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output = 'screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]),
        Node(
            package='realsense_node',
            node_executable='realsense_camera_node',
            node_namespace="/t265",
            output='screen',
            parameters=[get_package_share_directory('realsense_ros')+'/config/t265.yaml'],
            remappings=[('/t265/camera/odom/sample', '/odom')],
            ),
        Node(
            package='realsense_node',
            node_executable='realsense_camera_node',
            node_namespace="/d435",
            output='screen',
            parameters=[get_package_share_directory('realsense_ros')+'/config/d435.yaml']
            ),
    ])