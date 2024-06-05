#! /usr/bin/env python3
#
# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#

import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

# Create the launch description which will create the two managed nodes
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carma_ros2_utils',
            executable='test_carma_lifecycle_node',
            name='test_carma_lifecycle_node_1',
            output='screen',
            ),
        Node(
            package='carma_ros2_utils',
            executable='test_carma_lifecycle_node',
            name='test_carma_lifecycle_node_2',
            output='screen',
            ),
    ])


if __name__ == '__main__':
    launch_description = generate_launch_description()

    test_exec = os.getenv('TEST_EXECUTABLE')

    test_action = ExecuteProcess(
        cmd=[test_exec],
        name='test_carma_lifecycle_node_gtest',
        output='screen'
    )

    test_service = LaunchTestService()
    test_service.add_test_action(launch_description, test_action)

    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(launch_description)
    
    sys.exit(test_service.run(launch_service))
