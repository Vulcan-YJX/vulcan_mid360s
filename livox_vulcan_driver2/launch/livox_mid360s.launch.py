import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('livox_vulcan_driver2')
    config = os.path.join(pkg_share, 'config', 'param.yaml')

    # Default Livox SDK config JSON path (adjust to your setup)
    livox_config = os.path.join(pkg_share, 'config', 'livox_lidar_config.json')

    return LaunchDescription([
        Node(
            package='livox_vulcan_driver2',
            executable='livox_vulcan_driver2_node',
            name='livox_vulcan_node',
            output='screen',
            parameters=[config, {'config_path': livox_config}]
        )
    ])
