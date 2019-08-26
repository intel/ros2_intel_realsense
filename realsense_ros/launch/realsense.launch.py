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
    t265_param = os.path.join(get_package_share_directory('realsense_ros'), 'config', 't265.yaml')
    d435_param = os.path.join(get_package_share_directory('realsense_ros'), 'config', 'd435.yaml')
    d435i_param = os.path.join(get_package_share_directory('realsense_ros'), 'config', 'd435i.yaml')
    container = ComposableNodeContainer(
            node_name='realsense_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='realsense_ros',
                #     node_plugin='realsense::RealSenseNodeFactory',
                #     node_namespace='/t265',
                #     node_name='camera',
                #     parameters=[t265_param,
                #                 {'serial_no':'845412110563'}],),
                # ComposableNode(
                #     package='realsense_ros',
                #     node_plugin='realsense::RealSenseNodeFactory',
                #     node_namespace='/d435i',
                #     node_name='camera',
                #     parameters=[d435i_param,
                #                 {'serial_no':'843112073259'}],),
                ComposableNode(
                    package='realsense_ros',
                    node_plugin='realsense::RealSenseNodeFactory',
                    node_namespace='/d435',
                    node_name='camera',
                    parameters=[d435_param,
                                {'serial_no':'727212071015'}],),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])