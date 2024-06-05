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

import launch
import launch_ros
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import pytest
import unittest

import rclpy
from rclpy.node import Node

from carma_msgs.msg import SystemAlert
from lifecycle_msgs.srv import GetState
import time
import os, subprocess

@pytest.mark.launch_test
def generate_test_description():

    ld = launch.LaunchDescription([
        ComposableNodeContainer(
            package='carma_ros2_utils',
            name='wrapper_container',
            executable='lifecycle_component_wrapper_st',
            namespace="/",
            exec_name='wrapper_container',
            composable_node_descriptions=[

                ComposableNode(
                    package='carma_ros2_utils',
                    plugin='carma_ros2_utils_testing::MinimalNode',
                    name='minimal_node'
                ),
            ]
        ),

        # We can start this test right away as there is nothing else to wait on
        launch_testing.actions.ReadyToTest()
    ])

    return ld



# All tests in this class will run  in parallel
class TestRuntime(unittest.TestCase):

    def test_nominal_lifecycle(self, proc_output):

        # Check that that the wrapper has started and did not load any nodes on startup
        proc_output.assertWaitFor("Got request to load node: minimal_node but we are not in the active state, caching for later lifecycle based activation.", process='wrapper_container', strict_proc_matching=True,timeout=5)


        d = dict(os.environ)   # Make a copy of the current environment
        completed_proc = subprocess.run(['ros2', 'lifecycle', 'set', '/wrapper_container', 'configure'], env=d, capture_output=True, timeout=8)

        output_str = completed_proc.stdout.decode()
        self.assertTrue( output_str == 'Transitioning successful\n') # For whatever reason == must be specified manually instead of calling assertEquals which does not play well with the decoded string

        completed_proc = subprocess.run(['ros2', 'lifecycle', 'set', '/wrapper_container', 'activate'], env=d, capture_output=True, timeout=8)

        output_str = completed_proc.stdout.decode()
        self.assertTrue(output_str == 'Transitioning successful\n')

        proc_output.assertWaitFor("libtest_minimal_node.so", process='wrapper_container', strict_proc_matching=True,timeout=5)
        proc_output.assertWaitFor("Found class: rclcpp_components::NodeFactoryTemplate<carma_ros2_utils_testing::MinimalNode>", process='wrapper_container', strict_proc_matching=True,timeout=5)
        proc_output.assertWaitFor("Instantiate class: rclcpp_components::NodeFactoryTemplate<carma_ros2_utils_testing::MinimalNode>", process='wrapper_container', strict_proc_matching=True,timeout=5)

        completed_proc = subprocess.run(['ros2', 'lifecycle', 'set', '/wrapper_container', 'deactivate'], env=d, capture_output=True, timeout=8)

        output_str = completed_proc.stdout.decode()
        self.assertTrue(output_str == 'Transitioning successful\n')

        completed_proc = subprocess.run(['ros2', 'lifecycle', 'set', '/wrapper_container', 'shutdown'], env=d, capture_output=True, timeout=8)

        output_str = completed_proc.stdout.decode()
        self.assertTrue(output_str == 'Transitioning successful\n')

        completed_proc = subprocess.run(['ros2', 'lifecycle', 'get', '/wrapper_container'], env=d, capture_output=True, timeout=8)

        output_str = completed_proc.stdout.decode()
        self.assertTrue(output_str == 'finalized [4]\n')  # Verify nodes are shutdown


# All tests in this class will run on shutdown
@launch_testing.post_shutdown_test()
class TestPostShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
