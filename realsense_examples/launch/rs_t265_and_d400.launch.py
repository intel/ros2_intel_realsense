# Copyright (c) 2019 Intel Corporation
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
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # config the serial number and base frame id of each camera
    t265_base_frame_id = LaunchConfiguration('base_frame_id', default='t265_link')
    t265_serial_no = LaunchConfiguration('serial_no', default='845412110563')

    rgbd_base_frame_id = LaunchConfiguration('base_frame_id', default='d435_link')
    rgbd_serial_no = LaunchConfiguration('serial_no', default='841612070383')

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_examples'), 'config', 'rs_cartographer.rviz')

    rviz_node = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output = 'screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]
            )
    tf_node = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.03', '0', '0', '0', t265_base_frame_id, rgbd_base_frame_id]
            )
    t265_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
        output='screen',
        remappings=[('/t265/camera/odom/sample','/odom')],
        parameters=[{'serial_no':t265_serial_no ,
                'base_frame_id': t265_base_frame_id}]
        )
    rgbd_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/d435",
        output='screen',
        parameters=[{'serial_no':rgbd_serial_no, 
                'base_frame_id': rgbd_base_frame_id,
                'enable_pointcloud':'true',
                'dense_pointcloud' : 'true'}]
        )
    return launch.LaunchDescription([rviz_node, tf_node, t265_node, rgbd_node])
