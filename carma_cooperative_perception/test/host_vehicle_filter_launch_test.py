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

from carma_cooperative_perception_interfaces.msg import DetectionList

from geometry_msgs.msg import PoseStamped

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

        self.detection_list_pub = self.create_publisher(
            DetectionList, "input/detection_list", 1
        )

        self.host_vehicle_pose_pub = self.create_publisher(
            PoseStamped, "input/host_vehicle_pose", 1
        )

        self.detection_list_sub = self.create_subscription(
            DetectionList,
            "output/detection_list",
            lambda msg: self.detection_list_msgs.append(msg),
            1,
        )

        self.detection_list_msgs = []


@pytest.mark.launch_test
def generate_test_description():
    node_under_test = launch_ros.actions.Node(
        package="carma_cooperative_perception",
        executable="host_vehicle_filter_node",
        name="node_under_test",
        parameters=[{"distance_threshold_meters": 2.0}],
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

        host_vehicle_pose = PoseStamped()
        host_vehicle_pose.pose.position.x = 1.0

        self.test_harness_node.host_vehicle_pose_pub.publish(host_vehicle_pose)

        package_share_path = get_package_share_path("carma_cooperative_perception")

        msg_file = package_share_path / "test/data/detection_list.yaml"
        msg = carma_message_utilities.msg_from_yaml_file(msg_file)
        self.test_harness_node.detection_list_pub.publish(msg)

        spin_node_until(
            self.test_harness_node,
            LenIncreases(self.test_harness_node.detection_list_msgs),
            self.context,
        )

        self.assertGreaterEqual(len(self.test_harness_node.detection_list_msgs), 0)
        detection_list = self.test_harness_node.detection_list_msgs[-1]

        self.assertEqual(len(detection_list.detections), 2)

        self.assertEqual(detection_list.detections[0].id, '2')
        self.assertEqual(detection_list.detections[1].id, '3')


@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
