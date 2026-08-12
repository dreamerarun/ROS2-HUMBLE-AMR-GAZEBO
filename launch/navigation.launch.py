import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=map_file
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([map_arg, nav2])
