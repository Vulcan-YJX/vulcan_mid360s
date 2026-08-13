import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_vulcan_driver2')
    config = os.path.join(pkg_share, 'config', 'param.yaml')
    livox_config = os.path.join(pkg_share, 'config', 'livox_lidar_config.json')

    tools_launch = os.path.join(
        get_package_share_directory('livox_tools'),
        'launch', 'livox_tools.launch.py')

    return LaunchDescription([
        Node(
            package='livox_vulcan_driver2',
            executable='livox_vulcan_driver2_node',
            name='livox_vulcan_node',
            output='screen',
            parameters=[config, {'config_path': livox_config}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tools_launch)
        ),
    ])
