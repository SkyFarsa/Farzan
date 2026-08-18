from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your custom Nav2 params
    params_file = os.path.join(
        get_package_share_directory('my_robot_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # Let user override params if they want
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the Nav2 parameters file'
    )
    # REQUIRE the map path from the user
    map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load'
    )

    # Use standard Nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        params_arg,
        map_arg,
        nav2_launch
    ])

