from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR driver node
        Node(
            package='rplidar_ros2',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',  # Check your port with `dmesg`
                'frame_id': 'laser_link',       # LiDAR frame name
                'inverted': False,              # Flip if mounted upside-down
                'angle_compensate': True
            }]
        ),
        # Static transform: base_link → laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link']  # 10cm above base_link
        )
    ])
