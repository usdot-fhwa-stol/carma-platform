#!/usr/bin/env python3
# trajectory_replayer.py
#
# Replays one or more previously-recorded vehicle trajectories (pose from
# /localization/current_pose, speed from /hardware_interface/vehicle/twist)
# as a single carma_perception_msgs/ExternalObjectList on
# /environment/external_objects, as if those vehicles were being perceived
# right now.
#
# Each trajectory is configured as a (mcap_path, start_offset_sec) pair, with
# `mcap_paths[i]` paired to `start_offset_secs[i]`. The same mcap path may be
# reused across multiple entries (e.g. several objects drawn from one
# recording at different offsets); each unique mcap is only parsed/cached once.
#
# Only the portion of each recording where /guidance/state was ENGAGED is
# used. Every configured object is published immediately at its trajectory's
# starting position (start_offset_sec in) with zero velocity. Objects only
# start actually moving once the *live* /guidance/state topic reports
# ENGAGED, at which point all trajectories begin advancing together in real
# time from their respective start_offset_sec.

import bisect
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Quaternion, Vector3

from carma_perception_msgs.msg import ExternalObject, ExternalObjectList
from carma_planning_msgs.msg import GuidanceState

from trajectory_replayer.mcap_trajectory_loader import load_trajectory, default_cache_path


class TrajectoryReplayer(Node):

    def __init__(self):
        super().__init__('trajectory_replayer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # --- Parameters -----------------------------------------------------
        default_mcap = '/workspaces/carma_ws/src/carma-platform/rosbag2_2026-06-30_083523_0.mcap'
        self.declare_parameter('mcap_paths', [default_mcap])
        self.declare_parameter('start_offset_secs', [0.0])
        self.declare_parameter('storage_id', 'mcap')
        self.declare_parameter('pose_topic', '/localization/current_pose')
        self.declare_parameter('twist_topic', '/hardware_interface/vehicle/twist')
        self.declare_parameter('guidance_state_topic', '/guidance/state')
        self.declare_parameter('engaged_state_value', 4)
        self.declare_parameter('output_topic', '/environment/external_objects')
        self.declare_parameter('frame_id', '')  # empty => use frame_id recorded in each bag
        self.declare_parameter('object_id_base', 1001)
        self.declare_parameter('object_type', ExternalObject.SMALL_VEHICLE)
        self.declare_parameter('vehicle_length', 4.5)
        self.declare_parameter('vehicle_width', 1.9)
        self.declare_parameter('vehicle_height', 1.6)
        self.declare_parameter('confidence', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('loop_playback', False)
        self.declare_parameter('use_cache', True)
        self.declare_parameter('cache_dir', '')  # empty => cache next to each mcap file

        gpv = self.get_parameter
        self.mcap_paths = list(gpv('mcap_paths').get_parameter_value().string_array_value)
        self.start_offset_secs = list(gpv('start_offset_secs').get_parameter_value().double_array_value)
        self.storage_id = gpv('storage_id').get_parameter_value().string_value
        self.pose_topic = gpv('pose_topic').get_parameter_value().string_value
        self.twist_topic = gpv('twist_topic').get_parameter_value().string_value
        self.guidance_state_topic = gpv('guidance_state_topic').get_parameter_value().string_value
        self.engaged_state_value = gpv('engaged_state_value').get_parameter_value().integer_value
        self.output_topic = gpv('output_topic').get_parameter_value().string_value
        self.frame_id_override = gpv('frame_id').get_parameter_value().string_value
        self.object_id_base = gpv('object_id_base').get_parameter_value().integer_value
        self.object_type = gpv('object_type').get_parameter_value().integer_value
        self.vehicle_length = gpv('vehicle_length').get_parameter_value().double_value
        self.vehicle_width = gpv('vehicle_width').get_parameter_value().double_value
        self.vehicle_height = gpv('vehicle_height').get_parameter_value().double_value
        self.confidence = gpv('confidence').get_parameter_value().double_value
        self.publish_rate_hz = gpv('publish_rate_hz').get_parameter_value().double_value
        self.loop_playback = gpv('loop_playback').get_parameter_value().bool_value
        self.use_cache = gpv('use_cache').get_parameter_value().bool_value
        self.cache_dir = gpv('cache_dir').get_parameter_value().string_value

        if len(self.mcap_paths) == 0:
            raise ValueError('mcap_paths must contain at least one entry')
        if len(self.start_offset_secs) != len(self.mcap_paths):
            raise ValueError(
                f'start_offset_secs (len={len(self.start_offset_secs)}) must have the same '
                f'length as mcap_paths (len={len(self.mcap_paths)}); index i of each pairs together.')
        for p in self.mcap_paths:
            if not os.path.isfile(p):
                raise FileNotFoundError(f"mcap path '{p}' does not exist")

        self.publisher = self.create_publisher(ExternalObjectList, self.output_topic, qos)
        self.create_subscription(
            GuidanceState, self.guidance_state_topic, self._guidance_state_callback, qos)

        # --- Load (or restore from cache) each unique mcap's engaged-only trajectory ---
        trajectories_by_path = {}
        for path in dict.fromkeys(self.mcap_paths):  # de-duplicate, preserve order
            cache_path = self._cache_path_for(path)
            self.get_logger().info(
                f"Loading trajectory from '{path}' (use_cache={self.use_cache}, "
                f"cache_path='{cache_path}')...")
            t0 = time.monotonic()
            traj = load_trajectory(
                mcap_path=path,
                storage_id=self.storage_id,
                pose_topic=self.pose_topic,
                twist_topic=self.twist_topic,
                state_topic=self.guidance_state_topic,
                engaged_state_value=self.engaged_state_value,
                use_cache=self.use_cache,
                cache_path=cache_path,
                logger=self.get_logger(),
            )
            load_dt = time.monotonic() - t0
            self.get_logger().info(
                f"Loaded {len(traj['t_rel'])} samples spanning {traj['duration']:.2f}s of "
                f"engaged trajectory in {load_dt:.3f}s (source={traj['source']}).")
            trajectories_by_path[path] = traj

        # --- Build one playback instance per (mcap_path, start_offset_sec) entry ---
        self.instances = []
        for i, (path, offset) in enumerate(zip(self.mcap_paths, self.start_offset_secs)):
            traj = trajectories_by_path[path]
            duration = traj['duration']
            n_samples = len(traj['t_rel'])

            if n_samples == 0:
                self.get_logger().error(
                    f"[{i}] '{path}' has no engaged trajectory samples; this object will "
                    f"not be published.")

            if offset < 0.0:
                self.get_logger().warn(f'[{i}] start_offset_sec < 0; clamping to 0.0')
                offset = 0.0
            if duration > 0.0 and offset >= duration:
                self.get_logger().warn(
                    f'[{i}] start_offset_sec ({offset:.2f}s) >= engaged duration '
                    f'({duration:.2f}s); clamping to just before the last sample.')
                offset = max(0.0, duration - 1e-3)

            self.instances.append({
                'mcap_path': path,
                'trajectory': traj,
                'offset': offset,
                'object_id': self.object_id_base + i,
                'static_idx': self._index_at(traj, offset) if n_samples > 0 else None,
                'finished_logged': False,
            })

        # Replay only starts advancing once the *live* guidance state reports engaged.
        self.live_replay_start_wall = None

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Configured {len(self.instances)} trajectory object(s); holding at start '
            f'position with zero velocity until live {self.guidance_state_topic} reports '
            f'state={self.engaged_state_value}. loop_playback={self.loop_playback}.')

    def _cache_path_for(self, mcap_path):
        if self.cache_dir:
            base = os.path.splitext(os.path.basename(mcap_path))[0]
            return os.path.join(self.cache_dir, base + '.trajectory_cache.npz')
        return default_cache_path(mcap_path)

    def _guidance_state_callback(self, msg):
        if self.live_replay_start_wall is None and msg.state == self.engaged_state_value:
            self.live_replay_start_wall = time.monotonic()
            self.get_logger().info(
                'Live guidance state is ENGAGED; trajectory replay is now advancing in real time.')

    def _index_at(self, traj, elapsed_sec):
        t_rel = traj['t_rel']
        if len(t_rel) == 0:
            return None
        idx = bisect.bisect_left(t_rel, elapsed_sec)
        return min(idx, len(t_rel) - 1)

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        objects = []
        for inst in self.instances:
            obj = self._object_for_instance(inst, stamp)
            if obj is not None:
                objects.append(obj)

        list_msg = ExternalObjectList()
        list_msg.header.stamp = stamp
        list_msg.header.frame_id = self.frame_id_override or 'map'
        list_msg.objects = objects
        self.publisher.publish(list_msg)

    def _object_for_instance(self, inst, stamp):
        traj = inst['trajectory']
        if inst['static_idx'] is None:
            return None

        if self.live_replay_start_wall is None:
            # Real run not engaged yet: hold the object at its starting position.
            idx = inst['static_idx']
            zero_velocity = True
        else:
            duration = traj['duration']
            elapsed = (time.monotonic() - self.live_replay_start_wall) + inst['offset']
            if elapsed > duration:
                if self.loop_playback and duration > 0.0:
                    elapsed = elapsed % duration
                    inst['finished_logged'] = False
                else:
                    if not inst['finished_logged']:
                        self.get_logger().info(
                            f"[{inst['object_id']}] Reached end of recorded engaged "
                            f"trajectory ({inst['mcap_path']}); object will no longer be published.")
                        inst['finished_logged'] = True
                    return None
            idx = self._index_at(traj, elapsed)
            zero_velocity = False

        frame_id = self.frame_id_override or traj['frame_id']
        return self._build_object(inst['object_id'], traj, idx, stamp, frame_id, zero_velocity)

    def _build_object(self, object_id, traj, idx, stamp, frame_id, zero_velocity):
        obj = ExternalObject()
        obj.header.stamp = stamp
        obj.header.frame_id = frame_id

        obj.presence_vector = (
            ExternalObject.ID_PRESENCE_VECTOR |
            ExternalObject.POSE_PRESENCE_VECTOR |
            ExternalObject.VELOCITY_PRESENCE_VECTOR |
            ExternalObject.VELOCITY_INST_PRESENCE_VECTOR |
            ExternalObject.SIZE_PRESENCE_VECTOR |
            ExternalObject.CONFIDENCE_PRESENCE_VECTOR |
            ExternalObject.OBJECT_TYPE_PRESENCE_VECTOR |
            ExternalObject.DYNAMIC_OBJ_PRESENCE
        )
        obj.id = object_id

        obj.pose.pose.position = Point(
            x=float(traj['pos'][idx, 0]), y=float(traj['pos'][idx, 1]), z=float(traj['pos'][idx, 2]))
        obj.pose.pose.orientation = Quaternion(
            x=float(traj['ori'][idx, 0]), y=float(traj['ori'][idx, 1]),
            z=float(traj['ori'][idx, 2]), w=float(traj['ori'][idx, 3]))
        obj.pose.covariance = [0.0] * 36
        obj.pose.covariance[0] = 0.02   # x position uncertainty
        obj.pose.covariance[7] = 0.02   # y position uncertainty
        obj.pose.covariance[14] = 0.02  # z position uncertainty
        obj.pose.covariance[35] = 0.0394384  # ~10% yaw uncertainty: (0.1 * 2*pi)^2

        if zero_velocity:
            obj.velocity.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
            obj.velocity.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        else:
            # vehicle/twist is recorded in the vehicle's own body frame (x = forward
            # speed), which matches what ExternalObject.velocity expects for a
            # tracked object, so it is carried over unchanged.
            obj.velocity.twist.linear = Vector3(
                x=float(traj['lin_vel'][idx, 0]), y=float(traj['lin_vel'][idx, 1]),
                z=float(traj['lin_vel'][idx, 2]))
            obj.velocity.twist.angular = Vector3(
                x=float(traj['ang_vel'][idx, 0]), y=float(traj['ang_vel'][idx, 1]),
                z=float(traj['ang_vel'][idx, 2]))
        obj.velocity.covariance = [0.0] * 36
        obj.velocity.covariance[0] = 0.005
        obj.velocity.covariance[14] = 0.005
        obj.velocity.covariance[35] = 0.005
        obj.velocity_inst = obj.velocity

        obj.size = Vector3(x=self.vehicle_length, y=self.vehicle_width, z=self.vehicle_height)
        obj.confidence = self.confidence
        obj.object_type = self.object_type
        obj.dynamic_obj = True

        return obj


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
