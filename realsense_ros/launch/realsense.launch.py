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

import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file_path = os.path.join(get_package_share_directory('realsense_ros'), 'config', 'd435.yaml')
    container = ComposableNodeContainer(
            node_name='realsense_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense_ros',
                    node_plugin='realsense::RealSenseNodeFactory',
                    node_namespace='/d435',
                    parameters=[param_file_path])
                # ComposableNode(
                #     package='realsense_node',
                #     node_plugin='realsense::RealSenseNodeFactory',
                #     node_namespace='/d435i',
                #     parameters=[param_file_path,
                #                 {'serial_no' : '843112073259'}],)
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])