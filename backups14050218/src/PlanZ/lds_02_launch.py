from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LDS-02 driver node
        Node(
            package='ldsslam_ros2',
            executable='ldsslam_node',
            name='ldsslam_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',  # Check your port with `dmesg`
                'frame_id': 'laser_link',  # LiDAR frame name
                'inverted': False,         # Flip if mounted upside-down
                'angle_compensate': True   # Enable angle correction
            }]
        ),
        # Static transform: base_link → laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link']  # 10cm above base_link
        )
    ])
