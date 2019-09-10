
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
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]),
        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_namespace="/t265",
            output='screen',
            parameters=[get_package_share_directory('realsense_ros')+'/config/t265.yaml'],
            remappings=[('/t265/camera/odom/sample', '/odom')],
            ),

        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_namespace="/d435",
            output='screen',
            parameters=[get_package_share_directory('realsense_ros')+'/config/d435.yaml']
            ),
    ])
