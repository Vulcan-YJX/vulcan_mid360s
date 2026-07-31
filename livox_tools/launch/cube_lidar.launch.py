from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
# import xacro
import os
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.insert(0, os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch'))

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def return_launch_file(package, launch_dir, launch_file):
    package_node_dir = get_package_share_directory(package)
    package_launch_path = os.path.join(package_node_dir, launch_dir, launch_file)
    
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_launch_path)
    )
    return launch_file


def generate_launch_description():
    # Define the nodes to be launched
    lidar_monitor_node = return_launch_file('lidar_monitor','launch','lidar_monitor.launch.py')
    livox_tools_node = return_launch_file('livox_tools','launch','livox_tools.launch.py')

    ld = LaunchDescription()
    ld.add_action(lidar_monitor_node)
    ld.add_action(livox_tools_node)
    
    return ld
