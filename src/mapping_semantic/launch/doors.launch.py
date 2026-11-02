"""외부 센서·검출·SLAM에 문 지도 노드만 연결한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(map_id='', map_frame='map', odom_frame='odom',
        optical_frame='camera_color_optical_frame', rgb_topic='/camera/color/image_raw',
        depth_topic='/camera/depth/image_raw', camera_info_topic='/camera/camera_info',
        detections_topic='/semantic/detections2d', output_path='', input_path='')
    typed = dict(use_sim_time=('true', bool), reobserve_known_doors=('true', bool), confidence_threshold=('0.1', float),
        state_ttl_sec=('2.0', float), max_source_age_sec=('10.0', float),
        tf_wait_seconds=('1.0', float), queue_size=('20', int),
        association_distance=('0.25', float), minimum_observations=('3', int),
        reset_translation=('0.2', float), reset_rotation=('0.1', float))
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        *[DeclareLaunchArgument(name, default_value=value) for name, (value, _) in typed.items()],
        Node(package='mapping_semantic', executable='door_mapper', name='door_mapper', output='screen',
             parameters=[{**{name: ParameterValue(LaunchConfiguration(name), value_type=str) for name in defaults},
                          **{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                             for name, (_, kind) in typed.items()}}]),
    ])
