import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paths
    map_file = os.path.join(os.getenv('HOME'), 'map.yaml')
    urdf_file = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'my_robot', 'urdf', 'my_robot.urdf.xacro'
    )
    serial_odom_script = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'serial_odom_node', 'serial_odom_node.py'
    )
    oakd_params_file = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'my_robot', 'config', 'oakd_params.yaml'
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

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # --- Serial odometry node (Arduino Due bridge) ---
    # This is currently a bare script (not an installed ROS2 package entry point),
    # so it's launched the same way your tmux script ran it: python3 <path>.
    serial_odom_node = ExecuteProcess(
        cmd=['python3', serial_odom_script],
        name='serial_odom_node',
        output='screen'
    )

    # --- ICM20948 IMU driver ---
    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('icm20948_ros2_driver'), 'launch', 'icm20948.launch.py']
            )
        )
    )

    # --- OAK-D Lite depth camera (Phase 2: obstacle avoidance) ---
    # Publishes a depth-derived PointCloud2 (see oakd_params.yaml) consumed by
    # Nav2's local costmap obstacle_layer. Does NOT feed Cartographer — the
    # LD19 LIDAR remains the sole source for mapping/localization.
    #
    # Uses pointcloud.launch.py (not camera.launch.py + pointcloud.enable:=true).
    # That combo loaded PointCloudXyzrgbNode, which needs a synced RGB stream
    # to color each point -- expensive, and the likely cause of /oak/points
    # dropping to ~0.35 Hz once Nav2 + Cartographer were also running.
    # pointcloud.launch.py loads the lighter PointCloudXyziNode instead, which
    # gets its intensity channel from the right camera already used for
    # stereo depth, so no RGB stream is needed at all (see oakd_params.yaml).
    oakd_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('depthai_ros_driver'), 'launch', 'pointcloud.launch.py']
            )
        ),
        launch_arguments={
            'camera_model': 'OAK-D-LITE',
            'params_file': oakd_params_file,
        }.items()
    )

    # SLAM Toolbox (still commented — runs on remote PC per your setup)
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

    # Nav2 bringup (still commented — runs on remote PC per your setup)
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
        robot_state_publisher,
        serial_odom_node,
        imu_driver,
        oakd_camera,
        # slam_toolbox_node,
        # nav2_bringup,
    ])
