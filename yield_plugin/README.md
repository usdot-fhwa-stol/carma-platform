# Yield Plugin

A CARMA Platform tactical plugin that modifies an existing trajectory to safely yield to obstacles and cooperating vehicles. It operates as a ROS 2 lifecycle node (`YieldPluginNode`) wrapping the worker class (`YieldPlugin`).

---

## Role in the Stack

The yield plugin sits downstream of tactical plugins (e.g., inlanecruising_plugin, cooperative_lanechange). It receives a *fully formed* trajectory from the planner and either passes it through unchanged or replaces the timestamps to slow/stop the vehicle before a predicted collision.

```
Upstream planner  -->  [plan_trajectory service]  -->  YieldPlugin  -->  Controller
                                                          ^
                   external_object_predictions  ----------+
                   incoming_mobility_request    ----------+
```

---

## Two Operating Modes

### 1. Object Avoidance (default)

Runs whenever the ego trajectory needs to be checked against external objects from `/external_object_predictions` (populated from onboard sensors and/or V2X msgs such as SDSM). This is the default path: it is skipped only when `enable_cooperative_behavior` is `true` (see [config/parameters.yaml:76](config/parameters.yaml#L76)) *and* the urgency of the latest `MobilityRequest` (`clc_urgency_`, set in `mobilityrequest_cb`, see [yield_plugin.cpp:185](src/yield_plugin.cpp#L185)) exceeds `acceptable_urgency` (see [config/parameters.yaml:85](config/parameters.yaml#L85)).

Note that the object avoidance is not a handoff to a different tactical plugin. Yield_plugin's `update_traj_for_object` logic re-times the trajectory already produced by the upstream tactical plugin (see "Role in the Stack").

**Pipeline:**

1. **Concurrent collision detection** — `get_collision_times_concurrently()` checks every external object received on `/external_object_predictions` for a predicted collision with the ego trajectory, using either a CPU or GPU approach. The GPU approach is used by default, falling back to CPU if a CUDA error occurs (e.g., GPU driver not accessible). See "Collision Detection: CPU vs. CUDA" for more.

2. **Earliest collision selection** — of the objects with a predicted collision, the one with the closest time to collision is chosen as the adversary.

3. **Goal velocity** — the ego's target speed is set to the obstacle's velocity *projected along the ego trajectory direction* at the collision time (`get_predicted_velocity_at_time`). Stationary obstacles produce a goal velocity of 0.

4. **Safety gap** — the stopping distance ahead of the obstacle is:
   ```
   gap_time   = max(0, object_downtrack_lead - minimum_safety_gap_in_meters) / initial_velocity
   safety_gap = max(goal_velocity * gap_time, minimum_safety_gap_in_meters)
   ```
   where `object_downtrack_lead` is the obstacle's downtrack distance ahead of the ego vehicle. If `enable_adjustable_gap` is true, any `DigitalMinimumGap` regulatory element found along the trajectory overrides this value upward.

5. **JMT generation** — a Jerk Minimizing Trajectory is computed (see below) to reach `goal_pos = object_lead - safety_gap - vehicle_length` at `goal_velocity`.

6. **Commit-to-stop** — if the goal velocity is effectively zero and the collision is within `time_horizon_until_collision_to_commit_to_stop_in_s` seconds, the resulting trajectory is saved as `last_traj_plan_committed_to_stopping_`. On subsequent planning calls, while the vehicle is still moving, this saved trajectory is re-used (trimmed to the nearest point to the current vehicle position) instead of re-planning, preventing oscillation.

### 2. Cooperative Behavior (V2X Cooperative Lane Change)

_**NOTE: This logic has not been tested since around 2021.**_

Enabled via `enable_cooperative_behavior: true`. Activated when a `MobilityRequest` with strategy `carma/cooperative-lane-change` and urgency above `acceptable_urgency` is received and is not stale (within `acceptable_passed_timesteps` planning cycles).

**Mobility request handling (`mobilityrequest_cb`):**

1. Validates that the request targets a lanechange maneuver (`CHANGE_LANE_LEFT` or `CHANGE_LANE_RIGHT`).
2. Converts the requester's ECEF trajectory (encoded as a base location + cumulative offsets in cm) to map-frame 2D points via `convert_eceftrajectory_to_mappoints`.
3. Parses `strategy_params` JSON for the requester's target speed (`s.f` as integer + fractional tenths), start lanelet, and end lanelet.
4. Stores the request and calls `set_incoming_request_info`.
5. Rejects the request if the available planning window (`expiration - now`) is less than `min_obj_avoidance_plan_time_in_s`, or if the previous call to `update_traj_for_cooperative_behavior` determined the request was geometrically unacceptable.
6. Publishes a `MobilityResponse` and a `LaneChangeStatus`. The response's `is_accepted` flag is only ever `true` if both the geometric/feasibility check above passed *and* `always_accept_mobility_request` is `true` (default `false`); otherwise the request is reported as rejected regardless of feasibility.

**Trajectory update (`update_traj_for_cooperative_behavior`):**

1. `detect_trajectories_intersection` finds points in the requester's trajectory within `intervehicle_collision_distance_in_m` of the ego's trajectory (using Boost.Geometry distance).
2. If an intersection exists, the goal position is set to `distance_to_intersection - max(minimum_safety_gap, digital_gap)`, and the planning time is taken from the request expiration window.
3. A feasibility check compares the required planning time against `delta_v / yield_max_deceleration_in_ms2`. If infeasible, the request is rejected and the original trajectory is returned.
4. If feasible, `generate_JMT_trajectory` is called and `cooperative_request_acceptable_` is set true.
5. If trajectories do not intersect, the ego can accept without modifying its own path.

---

## Collision Detection: CPU vs. CUDA

The plugin supports two back-ends for `get_collision_times_concurrently`. At startup, `cuda_is_available()` checks for a reachable CUDA device; the GPU path is used when one is found and falls back to CPU on any runtime error.

```
get_collision_times_concurrently()
        │
        ├─ cuda_is_available()? ──Yes──> get_collision_times_concurrently_cuda()
        │                                        │
        │                       on-route filter (CPU, strided, polygon within)
        │                                        │
        │                       cuda_check_all_collisions()  ← single GPU kernel launch
        │                                        │
        │                       post-process: behind-vehicle check (CPU)
        │
        └─ No / exception ──> get_collision_times_concurrently_cpu()
                                       │
                               one std::thread per object
                               get_collision_time() per object (strided CPU loop)
```

### CPU path

**How it works:**

- One OS thread is spawned per external object via `std::thread` + `std::packaged_task`.
- Each thread runs `get_collision_time()`, which:
  1. Prepends the object's current pose to its prediction list.
  2. Runs a strided on-route check using `wm_->getLaneletsFromPoint()` to confirm at least one predicted state falls on a route lanelet.
  3. If on-route, iterates over `(ego_segment_i, obstacle_prediction_j)` pairs, linearly interpolates the ego position to each obstacle timestamp, and tests whether the distance is within the collision radius.
  4. Early-exits the inner loop when the distance starts increasing (valid only because the obstacle model is constant-velocity).

**Strengths:**

- No hardware dependency — works on any deployment platform.
- Thread-level parallelism amortizes the per-object cost across CPU cores.
- Early-exit heuristic reduces work when objects are clearly not on a collision course.

**Drawbacks:**

- **Between-timestamp miss:** the ego is interpolated *only* at obstacle prediction timestamps. If the true closest approach falls between two consecutive prediction samples it is never evaluated and the collision is silently missed.
- **Stride-skip miss:** when both actors are fast, `iteration_stride` is computed to be > 1 and the inner loop skips prediction indices. A collision that lands on a skipped index is invisible to this path.
- `getLaneletsFromPoint()` is a spatial R-tree query; at ~8 calls per object its cost is measurable (tens of microseconds each) and scales linearly with object count.
- CPU thread count is bounded by available cores; on embedded hardware with many objects the threads queue up.

### CUDA path

**How it works:**

**CPU-side pre-filter (before GPU launch):**

1. Objects with expired predictions or starting beyond `collision_check_radius_in_m` are dropped immediately.
2. The strided on-route check is run using `boost::geometry::within()` against pre-cached per-lanelet polygons (`route_llt_polygons_`). This replaces the expensive `getLaneletsFromPoint` R-tree query with an O(1) polygon containment test per cached lanelet.
3. Only the on-route portion of each object's prediction list is packed into a flat Structure-of-Arrays (SoA) buffer and queued for the GPU.

**GPU kernel (`collision_kernel`):**

- Grid layout: `x` = ego segments, `y` = obstacle segments, `z` = object index (one block layer per object). A single kernel launch covers all objects simultaneously.
- Each thread handles one `(ego_segment_i, obs_segment_j)` pair for its assigned object.
- For the temporal overlap window `[t_lo, t_hi]` both actors move linearly, so relative displacement is a linear function of time. The squared distance is a convex quadratic in time; the kernel solves for its minimum analytically (`s* = -(dx₀·dvx + dy₀·dvy) / (dvx² + dvy²)`) and clamps to `[0, T]`.
- A collision is recorded when `dist²(s*) ≤ collision_radius²`. The earliest collision time per object is stored via a CAS-based atomic float minimum.
- Data layout: separate float arrays for x, y, t (SoA) to maximize memory coalescing across the warp.

**Post-processing (back on CPU):**

- For each object with a detected collision, the ego and obstacle positions are interpolated at the collision time and a `routeTrackPos` downtrack check is run to filter objects that are actually behind the vehicle.

**Strengths:**

- **Analytically exact** for piecewise-linear motion — the between-timestamp miss and stride-skip miss that affect the CPU path are structurally impossible here.
- All objects are dispatched in a single kernel launch; parallelism scales with object count rather than CPU core count.
- The polygon `within()` on-route filter avoids the expensive `getLaneletsFromPoint` queries entirely.
- Benchmark (RTX 4090, 100 ego pts × 100 objects × 50 prediction states = ~485K segment pairs): expected ≥ 10× wall-clock speedup over the concurrent CPU path.

**Drawbacks:**

- Requires a CUDA-capable GPU and a matching driver/runtime installed in the deployment environment. Falls back to the CPU path automatically if unavailable.
- Host-to-device and device-to-host memory transfers (`cudaMemcpy`) add fixed overhead that dominates at small object counts; the GPU break-even point is approximately 10–20 objects depending on hardware.
- Timestamps are normalised and stored as `float32`. Over a ~15-second planning horizon this gives sub-millisecond precision, which is sufficient but is a lossy representation compared to the CPU path's `double`.
- The kernel uses a `cudaDeviceSynchronize()` call, making the GPU dispatch synchronous from the host thread's perspective. Async pipelining with the rest of the planning cycle is not currently implemented.

### Known accuracy gaps in the CPU path (documented in tests)

| Gap | Scenario | CPU result | CUDA result |
|---|---|---|---|
| Between-timestamp miss | True closest approach at t=2.5 s; obstacle predictions at t=0, 1.5, 3.5 s | `nullopt` (miss) | Collision detected at t=2.5 s |
| Stride-skip miss | Both actors at 10–30 m/s; collision at a non-stride-multiple prediction index | `nullopt` (miss) | Collision detected at correct time |

These are covered by [test/test_collision_cuda_benchmark.cpp](test/test_collision_cuda_benchmark.cpp).

---

## Jerk Minimizing Trajectory (JMT)

`generate_JMT_trajectory` re-times the *spatial* path of the original trajectory — it never changes waypoint positions, only timestamps.

1. **Quintic polynomial coefficients** are computed from boundary conditions: `(initial_pos, goal_pos, initial_velocity, goal_velocity, initial_accel, goal_accel=0, planning_time)`. The initial acceleration is estimated from the difference between the current and previous planning cycle speed.
2. The polynomial is sampled at a fixed time step equal to the smallest inter-point interval in the original trajectory. At each sample, the downtrack position and speed derivative are evaluated.
3. Calculated speeds are aligned to the original trajectory's downtrack distances: a speed sample is "accepted" when the JMT downtrack reaches or exceeds the next original waypoint's accumulated distance.
4. If the speed goes negative (vehicle would reverse), the remaining trajectory points are filled with 0 m/s.
5. A **moving average filter** (`speed_moving_average_window_size`) smooths the speed profile before applying it.
6. Timestamps are recomputed from the kinematic equation `dt = 2d / (v_prev + v_curr)`. For zero-speed segments, a sentinel `dt` of 6000 seconds is inserted so the controller steers toward the next waypoint instead of spinning in place.

---

## Node Architecture

| Class / File | Responsibility |
|---|---|
| `YieldPluginNode` — [src/yield_plugin_node.cpp](src/yield_plugin_node.cpp) | ROS 2 lifecycle node, pub/sub wiring |
| `YieldPlugin` — [src/yield_plugin.cpp](src/yield_plugin.cpp) | All planning and collision logic; dispatches to CPU or CUDA path |
| `YieldPluginConfig` — [include/yield_plugin/yield_config.hpp](include/yield_plugin/yield_config.hpp) | Parameter struct |
| `cuda_check_all_collisions` — [src/yield_plugin_cuda.cu](src/yield_plugin_cuda.cu) | CUDA host wrapper + `collision_kernel` for exact segment-pair collision detection |
| `CudaPoint`, `CudaCollisionResult` — [include/yield_plugin/yield_plugin_cuda.cuh](include/yield_plugin/yield_plugin_cuda.cuh) | Flat SoA data types shared between CPU and GPU |

### Subscribed Topics

| Topic | Type | Purpose |
|---|---|---|
| `external_object_predictions` | `ExternalObjectList` | Obstacle positions and predicted states |
| `incoming_mobility_request` | `MobilityRequest` | Cooperative lanechange requests from other vehicles |
| `bsm_outbound` | `BSM` | Host vehicle BSM ID (used in mobility response headers) |
| `georeference` | `std_msgs/String` | Proj string for ECEF→map conversion |

### Published Topics

| Topic | Type | Purpose |
|---|---|---|
| `outgoing_mobility_response` | `MobilityResponse` | Accept/reject cooperative requests |
| `cooperative_lane_change_status` | `LaneChangeStatus` | CLC state machine status |

### Service

| Service | Type | Direction |
|---|---|---|
| `plan_trajectory` | `PlanTrajectory` | Called by the CARMA arbitrator/planner |

---

## Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `intervehicle_collision_distance_in_m` | 6.0 m | Radius for collision detection |
| `collision_check_radius_in_m` | 150.0 m | Early-exit distance: skip object if first point is farther than this |
| `minimum_safety_gap_in_meters` | 10.0 m | Minimum gap to maintain behind an obstacle |
| `yield_max_deceleration_in_ms2` | 3.0 m/s² | Maximum deceleration for yield planning |
| `acceleration_adjustment_factor` | 1.0 | Scales the comfortable deceleration time estimate |
| `min_obj_avoidance_plan_time_in_s` | 2.0 s | Minimum planning horizon for any yield maneuver |
| `time_horizon_until_collision_to_commit_to_stop_in_s` | 3.0 s | Within this window, lock in the stop trajectory |
| `enable_cooperative_behavior` | false | Enable V2X cooperative lanechange support |
| `acceptable_urgency` | 5 | Minimum urgency to act on a cooperative request |
| `acceptable_passed_timesteps` | 5 | Stale request tolerance before falling back to object avoidance |
| `enable_adjustable_gap` | true | Honor `DigitalMinimumGap` regulatory elements from the map |
| `speed_moving_average_window_size` | 3 | Smoothing window applied to JMT speeds |
| `consecutive_clearance_count_for_passed_obstacles_threshold` | 20 | Consecutive detections required to confirm an object is behind the vehicle |
| `obstacle_zero_speed_threshold_in_ms` | 0.25 m/s | Below this, an obstacle's predicted speed is treated as 0 (goal velocity becomes 0) |
| `vehicle_length` | 5.0 m | Ego vehicle length, subtracted when computing `goal_pos` behind the obstacle |
| `safety_collision_time_gap_in_s` | 2.0 s | Time gap subtracted from the predicted intersection time when planning cooperative yields |
| `always_accept_mobility_request` | false | Gates whether a feasible cooperative request is actually reported as accepted in the `MobilityResponse` |

---

## Notable Design Decisions

- **No spatial re-routing.** The plugin only adjusts *when* the vehicle reaches each waypoint, never *where*. Steering behavior is therefore unchanged.
- **Stopping trajectory lock-in.** Once committed to a full stop, the plugin re-uses the saved trajectory until the vehicle reaches zero speed, preventing rapid plan oscillation caused by sensor noise near the collision boundary.
- **Stride-based collision search (CPU path).** The inner collision loop skips obstacle prediction steps proportionally to both actors' speeds, keeping per-object cost bounded. This is a deliberate approximation — it can miss collisions at skipped indices (see CUDA section).
- **CUDA path eliminates CPU accuracy gaps.** The GPU kernel analytically solves for the closest approach within every segment-pair overlap window, making it exact for piecewise-linear motion regardless of prediction step size or speeds.
- **Automatic CPU fallback.** `cuda_is_available()` is called each planning cycle entry. Any CUDA runtime error also triggers fallback, so the plugin degrades gracefully on hardware without a GPU.
- **Cooperative vs. object avoidance are mutually exclusive per planning cycle.** When a high-urgency CLC request is active and fresh, object avoidance is skipped entirely for performance.
- **Zero-speed trajectory encoding.** Stopped trajectory points are given a 6000-second inter-point time rather than collapsing to a single point, so the controller continues to steer toward the intended path of travel.
