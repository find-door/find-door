"""Run SLAM against existing sensor and TF publishers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set true for Gazebo or rosbag replay with /clock'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(
                get_package_share_directory('mapping_slam'), 'config', 'slam.yaml'),
            description='Absolute path to slam_toolbox ROS parameter YAML'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': LaunchConfiguration('params_file'),
            }.items()),
    ])
