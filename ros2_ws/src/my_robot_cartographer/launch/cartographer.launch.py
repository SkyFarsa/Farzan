from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Path to your config directory
    config_dir = PathJoinSubstitution(
        [FindPackageShare('my_robot_cartographer'), 'config']
    )

    # Launch Cartographer (SLAM)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'my_robot_lds_2d.lua',
        ],
        remappings=[
            ('scan', 'scan'),   # Use /scan directly
        ],
    )

    # Convert Cartographer’s submaps → /map topic
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
    ])
