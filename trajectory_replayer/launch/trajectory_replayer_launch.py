from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('trajectory_replayer')
    config_dir = os.path.join(package_dir, 'config')
    params_file = os.path.join(config_dir, 'trajectory_replayer_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to param file'
    )

    trajectory_replayer_node = Node(
        package='trajectory_replayer',
        executable='trajectory_replayer',
        name='trajectory_replayer',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    return LaunchDescription([
        params_arg,
        trajectory_replayer_node
    ])
