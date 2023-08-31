# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from datetime import datetime
import pathlib
import yaml

# This function is used to generate a command to record a ROS 2 rosbag that excludes topics
# topics as provided in the appropriate configuration file. 
def record_ros2_rosbag(context: LaunchContext, vehicle_config_param_file):

    # Convert LaunchConfiguration object to its string representation
    vehicle_config_param_file_string = context.perform_substitution(vehicle_config_param_file)

    # Initialize string that will contain the regex for topics to exclude from the ROS 2 rosbag
    exclude_topics_regex = ""

    # Open vehicle config params file to process various rosbag settings
    with open(vehicle_config_param_file_string, 'r') as f:

        vehicle_config_params = yaml.safe_load(f)

        if "use_ros2_rosbag" in vehicle_config_params:
            if vehicle_config_params["use_ros2_rosbag"] == True:

                if "exclude_default" in vehicle_config_params:
                    if (vehicle_config_params["exclude_default"] == True) and ("excluded_default_topics" in vehicle_config_params):
                        for topic in vehicle_config_params["excluded_default_topics"]:
                            exclude_topics_regex += str(topic) + "|"

                if "exclude_lidar" in vehicle_config_params:
                    if (vehicle_config_params["exclude_lidar"] == True) and ("excluded_lidar_topics" in vehicle_config_params):
                        for topic in vehicle_config_params["excluded_lidar_topics"]:
                            exclude_topics_regex += str(topic) + "|"

                if "exclude_camera" in vehicle_config_params:
                    if (vehicle_config_params["exclude_camera"] == True) and ("excluded_camera_topics" in vehicle_config_params):
                        for topic in vehicle_config_params["excluded_camera_topics"]:
                            exclude_topics_regex += str(topic) + "|"

                if "exclude_can" in vehicle_config_params:
                    if (vehicle_config_params["exclude_can"] == True) and ("excluded_can_topics" in vehicle_config_params):
                        for topic in vehicle_config_params["excluded_can_topics"]:
                            exclude_topics_regex += str(topic) + "|"

                proc = ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', '/opt/carma/logs/rosbag2_' + str(datetime.now().strftime('%Y-%m-%d_%H%M%S')), '-a', '-x', exclude_topics_regex],
                        output='screen',
                        shell='true'
                    )

                return [proc]


def generate_launch_description():
    # Declare the vehicle_config_dir launch argument
    vehicle_config_dir = LaunchConfiguration('vehicle_config_dir')
    declare_vehicle_config_dir_arg = DeclareLaunchArgument(
        name = 'vehicle_config_dir', 
        default_value = "/opt/carma/vehicle/config",
        description = "Path to file containing vehicle config directories"
    )

    # Declare the vehicle_config_param_file launch argument
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')
    declare_vehicle_config_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_config_param_file',
        default_value = [vehicle_config_dir, "/VehicleConfigParams.yaml"],
        description = "Path to file contain vehicle configuration parameters"
    )

    return LaunchDescription([
        declare_vehicle_config_dir_arg,
        declare_vehicle_config_param_file_arg,
        OpaqueFunction(function=record_ros2_rosbag, args=[LaunchConfiguration('vehicle_config_param_file')])
    ])