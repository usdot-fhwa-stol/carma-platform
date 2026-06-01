# Yield Plugin: CUDA GPU Collision Detection with CPU Fallback

## Summary

Replaces the legacy per-object serial/async CPU collision check with a batched CUDA kernel
that processes all ego–obstacle segment pairs in parallel, while preserving the original
`std::async` + `get_collision_time` path as a CPU-only fallback for non-GPU deployments.

---

## On-Route Filter Optimization (both paths)

- Replaced `getLaneletsFromPoint` (~163 µs spatial tree query per prediction point) with a
  pre-computed per-lanelet AABB check (~nanoseconds per point). Bounding boxes are built once
  when the route is set and reused across all calls.
- The only remaining `getLaneletsFromPoint` call is in `check_traj_for_digital_min_gap`, which
  correctly requires the actual lanelet object to read `DigitalMinimumGap` regulatory elements.
- Measured speedup of the on-route filter: **~163 µs/call → ~4 µs/call**.

---

## CUDA Kernel — `get_collision_times_concurrently` GPU Path

- Analytically minimizes squared distance over every (ego segment, obstacle segment) pair with
  temporal overlap using a closed-form quadratic solve — exact for piecewise-linear motion, no
  stride-based misses.
- All objects are batched into a single `cuda_check_all_collisions` kernel call. Objects are
  pre-filtered by radius and route bbox before GPU upload to minimize data transfer.
- Targets sm_75/sm_80/sm_86 SASS; sm_89 (RTX 40xx / Ada Lovelace) supported via PTX JIT
  fallback since CUDA 11.7 does not natively support sm_89 (CUDA 12.4+ compiles it natively).
- End-to-end measured speedup vs. legacy `std::async` + `get_collision_time`: **~200× on RTX 4090**.

---

## CPU Fallback — `#else` Branch

- Preserved as-is: `std::async` + `get_collision_time` per object, now also using the bbox
  on-route filter.
- Selected at compile time via `YIELD_PLUGIN_WITH_CUDA` preprocessor define, set automatically
  when the CUDA library is linked. Removing the CUDA build targets restores the CPU path with
  no source changes required.

---

## Startup Log

Node logs which collision path is active at configure time:

```
[INFO] YieldPlugin collision detection: GPU path (bbox on-route filter + CUDA kernel)
[INFO] YieldPlugin collision detection: CPU path (bbox on-route filter + std::async get_collision_time)
```

---

## Debug Logging

Added `[GPU]` / `[CPU]` prefixed `RCLCPP_DEBUG_STREAM` logs at every key decision point:

| Event | Tag |
|---|---|
| Object count entering the pipeline | `[GPU]` / `[CPU]` |
| Per-object: expired prediction skip | `[GPU]` |
| Per-object: radius filter skip + distance | `[GPU]` |
| Per-object: on-route result, speed, stride | `[GPU]` / `[CPU]` |
| Objects sent to CUDA kernel vs total | `[GPU]` |
| Per-object: collision time, ego pos, obs pos, downtrack gap | `[GPU]` |
| Per-object: no collision | `[GPU]` / `[CPU]` |
| Exit: total collisions confirmed | `[GPU]` / `[CPU]` |

Toggle at runtime without restarting:

```bash
ros2 service call /yield_plugin/set_logger_level rcl_interfaces/srv/SetLoggerLevel \
  "{logger_name: 'yield_plugin', level: 10}"
```

---

## Tests — `test_collision_cuda_benchmark`

Three GTest cases:

| Test | What it verifies |
|---|---|
| `CollisionDetectionAccuracy.BetweenTimestampMiss` | CUDA finds a collision that falls between two consecutive prediction timestamps (old algorithm misses it) |
| `CollisionDetectionAccuracy.StrideSkipMiss` | CUDA finds a collision at an index skipped by the old stride logic |
| `CollisionDetectionBenchmark.WorstCasePerformance` | 100 objects × 50 states: times legacy CPU path vs new GPU path; cross-checks a planted collision (t ≈ 0.7 s) is detected by both |
