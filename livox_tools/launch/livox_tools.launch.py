import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('livox_tools'),
        'config',
        'param.yaml'
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox_tf',
            arguments=[
                '0', '0', '0',
                '0', '0.173648', '0', '0.984807',
                'autocube_link', 'livox_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_vrtk_tf',
            arguments=[
                '-0.01545', '0', '-0.081527',
                '0', '0', '0', '1',
                'autocube_link', 'vrtk_link']
        ),
        Node(
            package='livox_tools',
            executable='livox_tools_node',
            name='livox_tools_node',
            output='screen',
            parameters=[config]
        ),
    ])
