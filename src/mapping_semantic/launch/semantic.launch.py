"""외부 Gazebo/SLAM 위에 합성 fixture mapper만 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'map_id': '', 'output_path': '', 'input_path': '',
        'rgb_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/depth/image_raw',
        'camera_info_topic': '/camera/camera_info',
        'detector_mode': 'color_fixture', 'detections_topic': '/semantic/detections2d',
        'map_frame': 'map', 'odom_frame': 'odom', 'optical_frame': 'camera_color_optical_frame',
    }
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package='mapping_semantic', executable='semantic_mapper', name='semantic_mapper',
             output='screen', parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 **{name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in defaults},
             }]),
    ])
