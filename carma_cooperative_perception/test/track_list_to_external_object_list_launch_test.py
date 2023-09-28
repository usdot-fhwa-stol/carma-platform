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

import unittest

from ament_index_python import get_package_share_path

from carma_launch_testing.transitions import transition_node
from carma_launch_testing.predicates import LenIncreases
from carma_launch_testing.spinning import spin_node_until

import carma_message_utilities

from carma_cooperative_perception_interfaces.msg import TrackList
from carma_perception_msgs.msg import ExternalObjectList

import launch_ros.actions

from launch import LaunchDescription
from launch.actions import TimerAction

from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes

from lifecycle_msgs.msg import Transition

import pytest

import rclpy
from rclpy.context import Context
import rclpy.node


class TestHarnessNode(rclpy.node.Node):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__("test_harness", *args, **kwargs)

        self.track_list_list_pub = self.create_publisher(
            TrackList, "input/track_list", 1
        )

        self.external_object_list_sub = self.create_subscription(
            ExternalObjectList,
            "output/external_object_list",
            lambda msg: self.external_object_list_msgs.append(msg),
            1,
        )

        self.external_object_list_msgs = []


@pytest.mark.launch_test
def generate_test_description():
    node_under_test = launch_ros.actions.Node(
        package="carma_cooperative_perception",
        executable="track_list_to_external_object_list_node",
        name="node_under_test",
    )

    launch_description = LaunchDescription(
        [node_under_test, TimerAction(period=1.0, actions=[ReadyToTest()])]
    )

    return launch_description


class TestMotionComputation(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()

        cls.context = Context()

        rclpy.init(context=cls.context)
        cls.test_harness_node = TestHarnessNode(context=cls.context)

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()

        rclpy.shutdown(context=cls.context)

    def test_track_list_conversion(self):
        transition_id = Transition.TRANSITION_CONFIGURE
        transition_node("node_under_test", transition_id, self.context)

        transition_id = Transition.TRANSITION_ACTIVATE
        transition_node("node_under_test", transition_id, self.context)

        package_share_path = get_package_share_path("carma_cooperative_perception")

        msg_file = package_share_path / "test/data/track_list.yaml"
        msg = carma_message_utilities.msg_from_yaml_file(msg_file)
        self.test_harness_node.track_list_list_pub.publish(msg)

        spin_node_until(
            self.test_harness_node,
            LenIncreases(self.test_harness_node.external_object_list_msgs),
            self.context,
        )

        self.assertGreaterEqual(
            len(self.test_harness_node.external_object_list_msgs), 0
        )
        external_object_list = self.test_harness_node.external_object_list_msgs[-1]

        self.assertEqual(len(external_object_list.objects), 3)

        self.assertEqual(external_object_list.objects[0].id, 1)
        self.assertEqual(external_object_list.objects[1].id, 2)
        self.assertEqual(external_object_list.objects[2].id, 3)


@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
