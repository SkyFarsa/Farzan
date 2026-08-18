#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('icm20948_ros2_driver')
    config_file = os.path.join(pkg_dir, 'config', 'icm20948_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('connection_type', default_value='i2c'),
        DeclareLaunchArgument('i2c_address', default_value='0x69'),
        DeclareLaunchArgument('publish_rate', default_value='100.0'),
        
        Node(
            package='icm20948_ros2_driver',
            executable='icm20948_node.py',
            name='icm20948_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'connection_type': LaunchConfiguration('connection_type'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                }
            ]
        )
    ])
