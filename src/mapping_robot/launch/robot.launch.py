"""확인된 실물 profile만 허용. 드라이버·Gazebo·구동 명령은 시작하지 않는다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mapping_robot.profile import load_profile, slam_parameters


def launch_robot(context):
    profile, _ = load_profile(LaunchConfiguration('profile').perform(context))
    nodes = [Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                  name='slam_toolbox', output='screen', parameters=[slam_parameters(profile)])]
    semantic = profile['semantic']
    if semantic['enabled']:
        topics, frames = profile['topics'], profile['frames']
        nodes.append(Node(package='mapping_semantic', executable='detector_node',
                          name='nanodet_detector', output='screen', parameters=[{
                              'use_sim_time': False, 'model_path': semantic['model_path'],
                              'expected_sha256': semantic['model_sha256'], 'input_topic': topics['rgb']}],
                          remappings=[('/semantic/detections2d', topics['detections'])]))
        nodes.append(Node(package='mapping_semantic', executable='semantic_mapper',
                          name='semantic_mapper', output='screen', parameters=[{
                              'use_sim_time': False, 'detector_mode': 'external_detections',
                              'map_id': semantic['map_id'], 'map_frame': frames['map'],
                              'odom_frame': frames['odom'], 'optical_frame': frames['optical'],
                              'rgb_topic': topics['rgb'], 'depth_topic': topics['depth'],
                              'camera_info_topic': topics['camera_info'],
                              'detections_topic': topics['detections'],
                              'output_path': semantic['output_path'], 'input_path': semantic['output_path'],
                              'static_classes': semantic['static_classes'],
                              'sync_policy': semantic['sync_policy'],
                              'max_sync_skew_sec': semantic['max_sync_skew_sec']}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('profile', description='검증된 실물 profile YAML 절대 경로'),
                              OpaqueFunction(function=launch_robot)])
