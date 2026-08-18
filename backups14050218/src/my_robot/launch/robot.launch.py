import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paths
    map_file = os.path.join(os.getenv('HOME'), 'map.yaml')
    urdf_file = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'my_robot', 'urdf', 'my_robot.urdf.xacro'
    )

    # Process the URDF with xacro and wrap in ParameterValue
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # --- LD19 LIDAR driver (ldlidar_ros2) ---
    ld19_lidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ld19_lidar',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'laser_scan_topic_name': 'scan',
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': 'lidar_link',
            'port_name': '/dev/ttyUSB0',
            'serial_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 0.0,
            'angle_crop_max': 0.0,

            # *** THESE TWO ARE CRITICAL ***
            'range_min': 0.12,   # 12 cm
            'range_max': 8.0,    # or 10/12 depending on room size
        }]
    )


    # REMOVED: The old static TF that was incorrect
    # This static transform is no longer needed because:
    # 1. The URDF defines lidar_link properly connected to base_link
    # 2. The robot_state_publisher will publish this transform automatically
    # 3. The lidar sensor uses frame_id='lidar_link' which matches the URDF
    
    # If you still need a static transform for some reason, it should match the URDF:
    # base_link_to_laser_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_lidar',
    #     # x  y  z   roll pitch yaw   parent     child
    #     arguments=['0', '0', '0.235', '0',  '0',  '0', 'base_link', 'lidar_link']
    # )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # If you want Arduino odom later, uncomment this block:
    # arduino_odom_node = Node(
    #     package='my_robot',
    #     executable='serial_odom',
    #     name='arduino_serial',
    #     output='screen',
    #     parameters=[{'port': '/dev/turtlebot_arduino', 'baud': 115200}]
    # )

    # SLAM Toolbox (still commented)
    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': False,
    #         'do_pose_init': True,
    #         'map_save_frequency': 1.0,
    #         'save_map_timeout': 30.0
    #     }]
    # )

    # Nav2 bringup (still commented)
    # nav2_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    #         )
    #     ),
    #     launch_arguments={
    #         'map': map_file,
    #         'use_sim_time': 'False'
    #     }.items()
    # )

    return LaunchDescription([
        ld19_lidar_node,
        # REMOVED: base_link_to_laser_tf (not needed, URDF handles it)
        robot_state_publisher,
        # arduino_odom_node,
        # slam_toolbox_node,
        # nav2_bringup,
    ])
