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

from pathlib import Path
import time
import unittest

from ament_index_python import get_package_share_directory

from carma_perception_msgs.msg import ExternalObjectList

from launch import LaunchDescription
from launch.actions import TimerAction

from launch_ros.actions import Node

from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

import pytest

import rclpy
import rclpy.node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor


class TestHarnessNode(rclpy.node.Node):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__("test_harness", *args, **kwargs)

        self.external_object_list_pub = self.create_publisher(
            ExternalObjectList, "external_objects", 1
        )


def try_transition_node(
    node_name: str, transition_id: int, ros_context: Context
) -> bool:
    transitioner = rclpy.create_node("transitioner", context=ros_context)

    srv_client = transitioner.create_client(ChangeState, f"{node_name}/change_state")

    if not srv_client.wait_for_service(timeout_sec=1.0):
        return False

    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(transitioner)

    request = ChangeState.Request()
    request.transition.id = transition_id

    future = srv_client.call_async(request)
    executor.spin_once_until_future_complete(future)

    executor.remove_node(transitioner)
    transitioner.destroy_node()

    return future.result().success


def publish_msg(node, publisher, msg, ros_context) -> None:
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)

    publisher.publish(msg)

    executor.remove_node(node)


@pytest.mark.launch_test
def generate_test_description():
    node_under_test = Node(
        package="roadway_objects",
        executable="roadway_objects_node",
        name="node_under_test",
        parameters=[{"vehicle_participant_type": "vehicle:car"}],
    )

    lanelet2_map_path = (
        Path(get_package_share_directory("roadway_objects"))
        / "test/data/town01_vector_map_1_no_lanelets.osm"
    )

    lanelet2_map_loader = Node(
        package="map_file_ros2",
        executable="lanelet2_map_loader_exec",
        name="lanelet2_map_loader",
        parameters=[{"lanelet2_filename": str(lanelet2_map_path)}],
        remappings=[("lanelet_map_bin", "semantic_map")],
    )

    launch_description = LaunchDescription(
        [
            node_under_test,
            lanelet2_map_loader,
            TimerAction(period=1.0, actions=[ReadyToTest()]),
        ]
    )

    # These will get passed to the unittest test cases as keyword args
    context = {"node_under_test": node_under_test}

    return launch_description, context


class TestEmptyMap(unittest.TestCase):
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

    def test_external_object_list_empty_map(self, proc_output, node_under_test):
        trans_id = Transition.TRANSITION_CONFIGURE
        self.assertTrue(try_transition_node("node_under_test", trans_id, self.context))

        trans_id = Transition.TRANSITION_ACTIVATE
        self.assertTrue(try_transition_node("node_under_test", trans_id, self.context))

        trans_id = Transition.TRANSITION_CONFIGURE
        self.assertTrue(
            try_transition_node("lanelet2_map_loader", trans_id, self.context)
        )

        trans_id = Transition.TRANSITION_ACTIVATE
        self.assertTrue(
            try_transition_node("lanelet2_map_loader", trans_id, self.context)
        )

        # Need to wait for roadway_object's WMListener to build it routing graph
        # Note: This value was arbitrarily chosen. It could probably be reduced.
        time.sleep(5)

        external_object_list = ExternalObjectList()
        publish_msg(
            self.test_harness_node,
            self.test_harness_node.external_object_list_pub,
            external_object_list,
            self.context,
        )

        proc_output.assertWaitFor(
            "roadway_objects could not process external objects as the "
            "semantic map does not contain any lanelets",
            process=node_under_test,
            timeout=1,
            stream="stderr",
        )


@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
