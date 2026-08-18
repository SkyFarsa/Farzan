import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command

def generate_launch_description():
    # Paths
    map_file = os.path.join(os.getenv('HOME'), 'map.yaml')
    urdf_file = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'my_robot', 'urdf', 'my_robot.urdf.xacro'

    )

    return LaunchDescription([

#        # Arduino odometry
#        Node(
#            package='my_robot',
#            executable='serial_odom',  # Make sure this is entry point in setup.py
#            name='arduino_serial',
#            output='screen',
#            parameters=[{'port': '/dev/turtlebot_arduino', 'baud': 115200}]
#        ),

        # LIDAR driver
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='lidar_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}]
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
	    parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
            #parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # SLAM Toolbox
#        Node(
#            package='slam_toolbox',
#            executable='async_slam_toolbox_node',
#            name='slam_toolbox',
#            output='screen',
#            parameters=[{
#                'use_sim_time': False,
#                'do_pose_init': True,
#                'map_save_frequency': 1.0,
#                'save_map_timeout': 30.0
#            }]
#        ),

        # Nav2 bringup
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(
#                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
#            ),
#            launch_arguments={
#                'map': map_file,
#                'use_sim_time': 'False'
#            }.items()
#        ),
    ])
