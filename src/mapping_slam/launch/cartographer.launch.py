"""Optional Cartographer comparison using external odom TF without duplication."""

from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    params = Path(LaunchConfiguration('params_file').perform(context)).resolve()
    if not params.is_file():
        raise RuntimeError(f'Cartographer Lua file does not exist: {params}')
    defaults = (Path(get_package_share_directory('cartographer_ros')).parent
                / 'cartographer' / 'configuration_files')
    if not (defaults / 'map_builder.lua').is_file():
        raise RuntimeError(f'Cartographer base configuration is missing: {defaults}')
    directory = Path(tempfile.mkdtemp(prefix='mapping-cartographer-'))
    # Runtime copies also support a user-space package prefix with no /opt installation.
    try:
        for path in defaults.glob('*.lua'):
            shutil.copy2(path, directory / path.name)
        shutil.copy2(params, directory / 'mapping.lua')
    except Exception:
        shutil.rmtree(directory)
        raise

    def cleanup(_context):
        shutil.rmtree(directory, ignore_errors=True)
        return []

    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen',
             arguments=['-configuration_directory', str(directory),
                        '-configuration_basename', 'mapping.lua'],
             parameters=[{'use_sim_time': sim_time}]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             parameters=[{'use_sim_time': sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(
            Path(get_package_share_directory('mapping_slam')) / 'config/cartographer.lua')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=setup),
    ])
