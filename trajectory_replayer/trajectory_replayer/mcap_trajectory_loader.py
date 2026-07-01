#!/usr/bin/env python3
# mcap_trajectory_loader.py
#
# Extracts a single passenger-vehicle trajectory (pose + speed) from a recorded
# rosbag2/mcap file, restricted to the time ranges where /guidance/state was
# ENGAGED, and exposes it as flat numpy arrays for fast real-time replay.
#
# To avoid re-parsing large bags (hundreds of thousands of messages) on every
# node startup, the extracted trajectory can be memoized to a small .npz cache
# file next to the bag. The cache is invalidated automatically if the source
# bag's mtime/size change.

import bisect
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def default_cache_path(mcap_path: str) -> str:
    base, _ = os.path.splitext(mcap_path)
    return base + '.trajectory_cache.npz'


def _read_topic_messages(mcap_path, storage_id, topics):
    storage_options = rosbag2_py.StorageOptions(uri=mcap_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in type_map]
    if missing:
        raise RuntimeError(f"Topic(s) not found in bag '{mcap_path}': {missing}")

    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    samples = {t: [] for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[topic]))
        samples[topic].append((t_ns, msg))

    for t in topics:
        samples[t].sort(key=lambda item: item[0])
    return samples


def _engaged_intervals(state_samples, engaged_state_value):
    """Returns a list of (start_ns, end_ns) ranges during which state == engaged_state_value."""
    intervals = []
    cur_start = None
    last_t = None
    for t_ns, msg in state_samples:
        last_t = t_ns
        if msg.state == engaged_state_value and cur_start is None:
            cur_start = t_ns
        elif msg.state != engaged_state_value and cur_start is not None:
            intervals.append((cur_start, t_ns))
            cur_start = None
    if cur_start is not None:
        intervals.append((cur_start, last_t))
    return intervals


def _nearest_index(sorted_times, t):
    i = bisect.bisect_left(sorted_times, t)
    if i <= 0:
        return 0
    if i >= len(sorted_times):
        return len(sorted_times) - 1
    before, after = sorted_times[i - 1], sorted_times[i]
    return i - 1 if (t - before) <= (after - t) else i


def _relative_time(t_ns, intervals):
    """Maps an absolute stamp inside one of `intervals` to a cumulative,
    gap-free seconds-since-first-engagement value, so that any disengaged
    stretches between separate engagements are skipped over rather than
    replayed as a pause. Returns None if t_ns falls outside every interval."""
    cumulative = 0.0
    for start, end in intervals:
        if start <= t_ns <= end:
            return cumulative + (t_ns - start) / 1e9
        cumulative += (end - start) / 1e9
    return None


def _empty_trajectory(frame_id=''):
    return {
        't_rel': np.zeros((0,), dtype=np.float64),
        'pos': np.zeros((0, 3), dtype=np.float64),
        'ori': np.zeros((0, 4), dtype=np.float64),
        'lin_vel': np.zeros((0, 3), dtype=np.float64),
        'ang_vel': np.zeros((0, 3), dtype=np.float64),
        'frame_id': frame_id,
        'duration': 0.0,
    }


def _parse_mcap(mcap_path, storage_id, pose_topic, twist_topic, state_topic,
                 engaged_state_value, logger=None):
    topics = _read_topic_messages(mcap_path, storage_id, [pose_topic, twist_topic, state_topic])
    pose_samples = topics[pose_topic]
    twist_samples = topics[twist_topic]
    state_samples = topics[state_topic]

    if not pose_samples:
        raise RuntimeError(f"No messages found on pose topic '{pose_topic}'")
    if not state_samples:
        raise RuntimeError(f"No messages found on guidance state topic '{state_topic}'")

    intervals = _engaged_intervals(state_samples, engaged_state_value)
    if not intervals:
        if logger:
            logger.warn(f'Guidance state never reached state={engaged_state_value} in this bag.')
        return _empty_trajectory(pose_samples[0][1].header.frame_id)

    twist_times = [t for t, _ in twist_samples]

    rel_times, pos, ori, lin_vel, ang_vel = [], [], [], [], []
    for t_ns, pose_msg in pose_samples:
        rel_t = _relative_time(t_ns, intervals)
        if rel_t is None:
            continue
        rel_times.append(rel_t)
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        pos.append((p.x, p.y, p.z))
        ori.append((q.x, q.y, q.z, q.w))

        if twist_times:
            tw = twist_samples[_nearest_index(twist_times, t_ns)][1].twist
            lin_vel.append((tw.linear.x, tw.linear.y, tw.linear.z))
            ang_vel.append((tw.angular.x, tw.angular.y, tw.angular.z))
        else:
            lin_vel.append((0.0, 0.0, 0.0))
            ang_vel.append((0.0, 0.0, 0.0))

    if not rel_times:
        if logger:
            logger.warn('Pose topic had no samples within the engaged interval(s).')
        return _empty_trajectory(pose_samples[0][1].header.frame_id)

    return {
        't_rel': np.asarray(rel_times, dtype=np.float64),
        'pos': np.asarray(pos, dtype=np.float64).reshape(-1, 3),
        'ori': np.asarray(ori, dtype=np.float64).reshape(-1, 4),
        'lin_vel': np.asarray(lin_vel, dtype=np.float64).reshape(-1, 3),
        'ang_vel': np.asarray(ang_vel, dtype=np.float64).reshape(-1, 3),
        'frame_id': pose_samples[0][1].header.frame_id,
        'duration': float(rel_times[-1]),
    }


def _cache_is_valid(cache_path, mcap_path):
    if not os.path.isfile(cache_path):
        return False
    try:
        with np.load(cache_path, allow_pickle=False) as data:
            src_mtime = data['source_mtime'].item()
            src_size = data['source_size'].item()
    except Exception:
        return False
    stat = os.stat(mcap_path)
    return src_mtime == stat.st_mtime and src_size == stat.st_size


def _load_cache(cache_path):
    with np.load(cache_path, allow_pickle=False) as data:
        return {
            't_rel': data['t_rel'],
            'pos': data['pos'],
            'ori': data['ori'],
            'lin_vel': data['lin_vel'],
            'ang_vel': data['ang_vel'],
            'frame_id': data['frame_id'].item(),
            'duration': data['duration'].item(),
        }


def _save_cache(cache_path, mcap_path, trajectory):
    stat = os.stat(mcap_path)
    cache_dir = os.path.dirname(os.path.abspath(cache_path))
    os.makedirs(cache_dir, exist_ok=True)
    np.savez_compressed(
        cache_path,
        source_mtime=stat.st_mtime,
        source_size=stat.st_size,
        frame_id=trajectory['frame_id'],
        duration=trajectory['duration'],
        t_rel=trajectory['t_rel'],
        pos=trajectory['pos'],
        ori=trajectory['ori'],
        lin_vel=trajectory['lin_vel'],
        ang_vel=trajectory['ang_vel'],
    )


def load_trajectory(mcap_path, storage_id, pose_topic, twist_topic, state_topic,
                     engaged_state_value, use_cache, cache_path, logger=None):
    """Loads the engaged-only trajectory, preferring a valid cache file when use_cache
    is True. Falls back to (and, if use_cache, rewrites) the cache by parsing the mcap
    directly otherwise. Returns a dict of numpy arrays plus 'frame_id', 'duration',
    and a human-readable 'source' describing where the data came from."""
    if use_cache and _cache_is_valid(cache_path, mcap_path):
        trajectory = _load_cache(cache_path)
        trajectory['source'] = f'cache ({cache_path})'
        return trajectory

    trajectory = _parse_mcap(
        mcap_path, storage_id, pose_topic, twist_topic, state_topic,
        engaged_state_value, logger=logger)

    if use_cache:
        try:
            _save_cache(cache_path, mcap_path, trajectory)
        except OSError as e:
            if logger:
                logger.warn(f"Could not write trajectory cache to '{cache_path}': {e}")

    trajectory['source'] = f'mcap ({mcap_path})'
    return trajectory
