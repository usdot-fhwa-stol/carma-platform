# Copyright 2023 Leidos
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

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_path = get_package_share_path("carma_cooperative_perception")

    multiple_object_tracker_node = Node(
        package="carma_cooperative_perception",
        executable="multiple_object_tracker_node",
        name="multiple_object_tracker",
        parameters=[package_share_path / "config/params.yaml"],
    )

    host_vehicle_filter_node = Node(
        package="carma_cooperative_perception",
        executable="host_vehicle_filter_node",
        name="host_vehicle_filter",
        parameters=[package_share_path / "config/params.yaml"],
    )

    return LaunchDescription([multiple_object_tracker_node, host_vehicle_filter_node])
