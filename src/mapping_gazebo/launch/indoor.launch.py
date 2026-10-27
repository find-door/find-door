"""Spawn the JetAuto simulation (wf_gazebo) in an indoor mapping world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('mapping_gazebo')
    wf_gazebo_share = get_package_share_directory('wf_gazebo')

    # Spawn pose defaults to the corridor start point of indoor_v1 (see README).
    pass_through = {
        'gui': 'true',
        'headless_rendering': 'true',
        'x': '1.0',
        'y': '1.0',
        'yaw': '0.0',
    }

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wf_gazebo_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            **{name: LaunchConfiguration(name) for name in pass_through},
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=os.path.join(
                    package_share, 'worlds', 'indoor_v1.sdf'
                ),
                description='Absolute path to the world SDF',
            ),
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in pass_through.items()
            ],
            spawn_robot,
        ]
    )
