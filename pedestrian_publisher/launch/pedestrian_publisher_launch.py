from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('pedestrian_publisher')
    config_dir = os.path.join(package_dir, 'config')
    params_file = os.path.join(config_dir, 'pedestrian_params.yaml')

    # Create launch configuration variables
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to param file'
    )

    # Start the node
    pedestrian_node = Node(
        package='pedestrian_publisher',
        executable='pedestrian_publisher',
        name='pedestrian_publisher',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    return LaunchDescription([
        params_arg,
        pedestrian_node
    ])