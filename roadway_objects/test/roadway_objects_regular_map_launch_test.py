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

import copy
from pathlib import Path
import time
import unittest
from typing import Sized

from ament_index_python import get_package_share_directory

from carma_perception_msgs.msg import (
    ConnectedVehicleType,
    ExternalObject,
    ExternalObjectList,
    PredictedState,
    RoadwayObstacleList,
)
from geometry_msgs.msg import Pose, Vector3

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

        self.roadway_obstacle_list_sub = self.create_subscription(
            RoadwayObstacleList,
            "roadway_objects",
            self.store_msg,
            1,
        )

        self.roadway_obstacle_lists = []

    def store_msg(self, msg) -> None:
        self.roadway_obstacle_lists.append(msg)


def make_external_object_list() -> ExternalObjectList:
    external_object = ExternalObject()
    external_object.id = 1
    external_object.object_type = ExternalObject.SMALL_VEHICLE
    external_object.header.frame_id = "map"

    pose = Pose()
    pose.position.x = 45.8529
    pose.position.y = -223.004
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.7071081
    pose.orientation.w = 0.7071055

    external_object.pose.pose = pose

    external_object.velocity.twist.linear.x = 1.0

    size = Vector3()
    size.x = 4.0
    size.y = 2.0
    size.z = 1.0

    external_object.size = size

    predicted_state = PredictedState()
    predicted_state.predicted_position = copy.deepcopy(pose)
    predicted_state.predicted_position.position.y += 1.0
    predicted_state.predicted_position_confidence = 1.0

    external_object.predictions.append(predicted_state)

    external_object_list = ExternalObjectList()
    external_object_list.objects.append(external_object)

    return external_object_list


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


def spin_node_until(node, condition, ros_context, timeout_sec=60.0) -> None:
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)

    end_time = time.time() + timeout_sec
    while not condition() and time.time() < end_time:
        executor.spin_once(timeout_sec=0.1)

    executor.remove_node(node)


class LenIncreases:
    def __init__(self, sized_object: Sized) -> None:
        self.sized_object = sized_object
        self.original_size = len(sized_object)

    def __call__(self) -> bool:
        return len(self.sized_object) > self.original_size


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
        / "test/data/town01_vector_map_1.osm"
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


class TestRegularMap(unittest.TestCase):
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

    def test_external_object_list_regular_map(self):
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

        external_object_list = make_external_object_list()
        publish_msg(
            self.test_harness_node,
            self.test_harness_node.external_object_list_pub,
            external_object_list,
            self.context,
        )

        spin_node_until(
            self.test_harness_node,
            LenIncreases(self.test_harness_node.roadway_obstacle_lists),
            self.context,
        )

        self.assertTrue(len(self.test_harness_node.roadway_obstacle_lists) > 0)
        roadway_obstacle_list = self.test_harness_node.roadway_obstacle_lists[-1]

        external_object = external_object_list.objects[0]
        roadway_obstacle = roadway_obstacle_list.roadway_obstacles[0]

        self.assertEqual(external_object, roadway_obstacle.object)

        self.assertEqual(roadway_obstacle.lanelet_id, 144)
        self.assertEqual(
            roadway_obstacle.connected_vehicle_type.type,
            ConnectedVehicleType.NOT_CONNECTED,
        )

        self.assertAlmostEqual(roadway_obstacle.cross_track, 0.0, delta=1e-4)
        self.assertAlmostEqual(roadway_obstacle.down_track, 8.308, delta=1e-3)

        self.assertEqual(len(roadway_obstacle.predicted_lanelet_ids), 1)
        self.assertEqual(roadway_obstacle.predicted_lanelet_ids[0], 144)

        self.assertEqual(len(roadway_obstacle.predicted_lanelet_id_confidences), 1)
        self.assertAlmostEqual(
            roadway_obstacle.predicted_lanelet_id_confidences[0], 0.9
        )

        self.assertEqual(len(roadway_obstacle.predicted_cross_tracks), 1)
        self.assertAlmostEqual(
            roadway_obstacle.predicted_cross_tracks[0], 0.0, delta=1e-4
        )

        self.assertEqual(len(roadway_obstacle.predicted_cross_track_confidences), 1)
        self.assertAlmostEqual(
            roadway_obstacle.predicted_cross_track_confidences[0], 0.9
        )

        self.assertEqual(len(roadway_obstacle.predicted_down_tracks), 1)
        self.assertAlmostEqual(
            roadway_obstacle.predicted_down_tracks[0], 9.308, delta=1e-3
        )

        self.assertEqual(len(roadway_obstacle.predicted_down_track_confidences), 1)
        self.assertAlmostEqual(
            roadway_obstacle.predicted_down_track_confidences[0], 0.9
        )


@post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
