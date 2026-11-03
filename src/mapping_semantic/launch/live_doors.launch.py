"""설정으로 선택한 로컬 상주 검출기와 문 mapper를 묶는다. 기본은 YOLOE다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(detector_config='', map_frame='map', odom_frame='odom',
                    rgb_topic='/camera/color/image_raw', depth_topic='/camera/depth/image_raw',
                    camera_info_topic='/camera/camera_info', optical_frame='camera_color_optical_frame',
                    output_path='', input_path='')
    text = lambda name: ParameterValue(LaunchConfiguration(name), value_type=str)
    simulation = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('recon_root', description='기존 recon-report 저장소 절대 경로'),
        DeclareLaunchArgument('map_id', description='현재 지도 자산과 원점 식별자'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('reobserve_known_doors', default_value='true'),
        DeclareLaunchArgument('worker_timeout_sec', default_value='60.0'),
        Node(package='mapping_semantic', executable='live_door_detector', name='live_door_detector', output='screen',
             parameters=[dict(use_sim_time=simulation,
                 worker_timeout_sec=ParameterValue(LaunchConfiguration('worker_timeout_sec'), value_type=float),
                 **{name: text(name) for name in ['recon_root', 'detector_config', 'rgb_topic', 'depth_topic', 'camera_info_topic', 'optical_frame']})]),
        Node(package='mapping_semantic', executable='door_mapper', name='door_mapper', output='screen',
             parameters=[dict(use_sim_time=simulation,
                 reobserve_known_doors=ParameterValue(LaunchConfiguration('reobserve_known_doors'), value_type=bool), rgb_topic='/doors/input/rgb', depth_topic='/doors/input/depth',
                 camera_info_topic='/doors/input/camera_info', detections_topic='/doors/input/detections2d',
                 **{name: text(name) for name in ['map_id', 'map_frame', 'odom_frame', 'optical_frame', 'input_path', 'output_path']})]),
    ])
