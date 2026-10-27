"""Start the indoor simulator and SLAM with a single clock source."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory('mapping_gazebo')
    slam_share = get_package_share_directory('mapping_slam')
    defaults = {
        'world': os.path.join(gazebo_share, 'worlds', 'indoor_v1.sdf'),
        'gui': 'false',
        'headless_rendering': 'true',
        'x': '1.0',
        'y': '1.0',
        'yaw': '0.0',
        'params_file': os.path.join(slam_share, 'config', 'slam.yaml'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'indoor.launch.py')),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name in defaults if name != 'params_file'
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                slam_share, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('params_file'),
            }.items()),
    ])
