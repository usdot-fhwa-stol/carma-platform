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
import launch_testing
import pytest
import unittest

import rclpy
from rclpy.node import Node

from carma_msgs.msg import SystemAlert
from lifecycle_msgs.srv import GetState
import time

# Simple test node to publish a system alert from inside a test
class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(SystemAlert, '/system_alert', 10)
        self.service_ = self.create_client(GetState, '/test_carma_lifecycle_node_2/get_state')

    # Helper method waits until subscribers are available
    def wait_for_subscribers(self, timeout_sec=10.0):
        print('Waiting for subscribers to: ' + self.publisher_.topic_name)
        timeout = time.time() + timeout_sec   # Timeout time
        while self.publisher_.get_subscription_count() == 0 and time.time() < timeout:
            time.sleep(0.5)

        print('Subscribers found for: ' + self.publisher_.topic_name)

    # Publish a system alert
    def publish_alert(self, msg):
        print('Publishing alert to: ' + self.publisher_.topic_name)
        self.publisher_.publish(msg)

@pytest.mark.launch_test
def generate_test_description():

    ld = launch.LaunchDescription([
        launch_ros.actions.Node( # Managed node 1
            package='system_controller',
            executable='test_carma_lifecycle_node',
            name='test_carma_lifecycle_node_1',
            exec_name='test_carma_lifecycle_node_1', # Process name used for testing
            ),
        launch_ros.actions.Node( # Managed node 2
            package='system_controller',
            executable='test_carma_lifecycle_node',
            name='test_carma_lifecycle_node_2',
            exec_name='test_carma_lifecycle_node_2', # Process name used for testing
            ),
        launch_ros.actions.Node( # System Controller to test
            package='system_controller',
            executable='system_controller',
            name='test_system_controller', # Node name
            exec_name='test_system_controller', # Process name used for testing
            parameters=[{
                'signal_configure_delay': 5.0, 
                'service_timeout_ms': 500,
                'call_timeout_ms': 500,
                'required_subsystem_nodes': ['/test_carma_lifecycle_node_1', '/test_carma_lifecycle_node_2']}],
            ),

        # We can start this test right away as there is nothing else to wait on
        launch_testing.actions.ReadyToTest()
    ])

    return ld



# All tests in this class will run  in parallel
class TestRuntime(unittest.TestCase):

    def test_fatal_lifecycle(self, proc_output):
        rclpy.init()
        
        # Check that the system controller is starting up
        proc_output.assertWaitFor('Initializing SystemControllerNode', process='test_system_controller', strict_proc_matching=True,timeout=5)
        proc_output.assertWaitFor('Attempting to configure system...', process='test_system_controller', strict_proc_matching=True, timeout=6)
        
        # Check that the system controller has driven the lifecycle of the nodes to active state
        proc_output.assertWaitFor('test_carma_lifecycle_node_1 node is Configured!', process='test_carma_lifecycle_node_1', strict_proc_matching=True, timeout=10)
        proc_output.assertWaitFor('test_carma_lifecycle_node_2 node is Configured!', process='test_carma_lifecycle_node_2', strict_proc_matching=True, timeout=10)
        proc_output.assertWaitFor('test_carma_lifecycle_node_1 node is Activated!', process='test_carma_lifecycle_node_1', strict_proc_matching=True, timeout=5)
        proc_output.assertWaitFor('test_carma_lifecycle_node_2 node is Activated!', process='test_carma_lifecycle_node_2', strict_proc_matching=True, timeout=5)

        # Check that the system controller has driven the lifecycle of the nodes to shutdown state
        alert_node = TestPublisher()

        alert_node.wait_for_subscribers(5.0) # Wait for subscriptions to connect since we just created the node

        shutdown_alert = SystemAlert()
        shutdown_alert.type = SystemAlert.FATAL
        shutdown_alert.source_node = '/test_carma_lifecycle_node_2' # Pretend this is the node that caused the fatal
        alert_node.publish_alert(shutdown_alert)

        print("count_subscribers " + str(alert_node.count_subscribers("/system_alert")))
        print("get_node_names " + str(alert_node.get_node_names()))

        proc_output.assertWaitFor('Received SystemAlert message of type:', process='test_system_controller', strict_proc_matching=True, timeout=10)
        proc_output.assertWaitFor('test_carma_lifecycle_node_1 node is Shutdown!', process='test_carma_lifecycle_node_1', strict_proc_matching=True, timeout=5)
        proc_output.assertWaitFor('test_carma_lifecycle_node_2 node is Shutdown!', process='test_carma_lifecycle_node_2', strict_proc_matching=True, timeout=5)



# All tests in this class will run on shutdown
@launch_testing.post_shutdown_test()
class TestPostShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])




