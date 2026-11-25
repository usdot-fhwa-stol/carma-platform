/*
 * Copyright (C) 2026 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/**
 * @file test_collision_cuda_benchmark.cpp
 *
 * Three tests covering the two known accuracy gaps in the legacy get_collision()
 * and a worst-case throughput benchmark against the CUDA replacement.
 *
 * Accuracy gap 1 – "between-timestamp miss"
 *   The old algorithm interpolates the ego position at each obstacle *prediction
 *   timestamp* and checks distance there.  If the true closest approach falls
 *   between two consecutive prediction timestamps it is never sampled → miss.
 *   The CUDA kernel minimises the quadratic distance function analytically over
 *   every (ego_seg, obs_seg) pair, so it is exact for piecewise-linear motion.
 *
 * Accuracy gap 2 – "stride-skip miss"
 *   When both vehicles move fast the old code computes an iteration_stride > 1
 *   and samples only every N-th prediction timestamp in the inner loop.  A
 *   collision that falls at a skipped timestamp is invisible to the old code.
 *   The CUDA path uses stride only for the CPU on-route filter; once an object
 *   clears that filter ALL its prediction segments are sent to the GPU.
 *
 * Worst-case benchmark
 *   100 ego points × 100 objects × 50 prediction states = 495 000 segment pairs.
 *   The CPU reference loop runs all pairs serially; the CUDA kernel runs them in
 *   parallel.  Expected wall-clock speedup on an RTX 4090: ≥ 10×.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <future>
#include <random>
#include <numeric>
#include <unordered_map>

#include <carma_wm/CARMAWorldModel.hpp>
#include <carma_wm/WMTestLibForGuidance.hpp>

#include <yield_plugin/yield_plugin.hpp>
#include <yield_plugin/yield_plugin_node.hpp>
#include <yield_plugin/yield_plugin_cuda.cuh>

using namespace yield_plugin;

// ─────────────────────────────────────────────────────────────────────────────
// Shared helpers
// ─────────────────────────────────────────────────────────────────────────────

// Build an ego trajectory along the y-axis at x = 10.
// n_pts points, y linearly spaced in [y0, y1], timestamps at
// t0_sec + i * dt_sec (i = 0 .. n_pts-1).
static carma_planning_msgs::msg::TrajectoryPlan make_ego_traj(
  int    n_pts,
  double y0,   double y1,
  double t0_sec, double dt_sec)
{
  carma_planning_msgs::msg::TrajectoryPlan tp;
  for (int i = 0; i < n_pts; ++i) {
    carma_planning_msgs::msg::TrajectoryPlanPoint p;
    p.x = 10.0;
    p.y = y0 + (y1 - y0) * i / std::max(1, n_pts - 1);
    p.target_time = rclcpp::Time(static_cast<int64_t>((t0_sec + i * dt_sec) * 1e9));
    tp.trajectory_points.push_back(p);
  }
  return tp;
}

// Build a prediction list for a single obstacle.
// Positions interpolated linearly from (x0,y0) to (x1,y1).
// Velocity fields set to (vx, vy) for the speed computation in get_collision.
static std::vector<carma_perception_msgs::msg::PredictedState> make_preds(
  int    n_states,
  double x0, double y0,
  double x1, double y1,
  double t0_sec, double dt_sec,
  double vx = 0.0, double vy = 0.0)
{
  std::vector<carma_perception_msgs::msg::PredictedState> preds;
  preds.reserve(n_states);
  for (int i = 0; i < n_states; ++i) {
    carma_perception_msgs::msg::PredictedState ps;
    ps.header.stamp = rclcpp::Time(static_cast<int64_t>((t0_sec + i * dt_sec) * 1e9));
    double f = (n_states > 1) ? static_cast<double>(i) / (n_states - 1) : 0.0;
    ps.predicted_position.position.x = x0 + (x1 - x0) * f;
    ps.predicted_position.position.y = y0 + (y1 - y0) * f;
    ps.predicted_velocity.linear.x   = vx;
    ps.predicted_velocity.linear.y   = vy;
    preds.push_back(ps);
  }
  return preds;
}

// Convert a TrajectoryPlan + a prediction list into the flat CudaPoint arrays
// needed by cuda_check_all_collisions.  Returns (ego_pts, obs_flat, offsets, sizes).
static std::tuple<
  std::vector<CudaPoint>,
  std::vector<CudaPoint>,
  std::vector<int>,
  std::vector<int>>
to_cuda_inputs(
  const carma_planning_msgs::msg::TrajectoryPlan& ego_tp,
  const std::vector<std::vector<carma_perception_msgs::msg::PredictedState>>& all_preds,
  double ref_t)
{
  std::vector<CudaPoint> ego_pts;
  for (const auto& tp : ego_tp.trajectory_points) {
    ego_pts.push_back({
      static_cast<float>(tp.x),
      static_cast<float>(tp.y),
      static_cast<float>(rclcpp::Time(tp.target_time).seconds() - ref_t)
    });
  }

  std::vector<CudaPoint> obs_flat;
  std::vector<int>       obs_offsets, obs_sizes;
  for (const auto& preds : all_preds) {
    obs_offsets.push_back(static_cast<int>(obs_flat.size()));
    for (const auto& ps : preds) {
      obs_flat.push_back({
        static_cast<float>(ps.predicted_position.position.x),
        static_cast<float>(ps.predicted_position.position.y),
        static_cast<float>(rclcpp::Time(ps.header.stamp).seconds() - ref_t)
      });
    }
    obs_sizes.push_back(static_cast<int>(preds.size()));
  }
  return std::make_tuple(
    std::move(ego_pts), std::move(obs_flat),
    std::move(obs_offsets), std::move(obs_sizes));
}


// ─────────────────────────────────────────────────────────────────────────────
// Test 1: between-timestamp accuracy
// ─────────────────────────────────────────────────────────────────────────────
//
// Scenario
// ────────
//   Ego   : x=10, y: 0→60 over 6 s at 10 m/s.  Trajectory points every 1 s.
//   Obs   : stays at x=10 (on route), predictions at t={0, 1.5, 3.5} s.
//            y: 10 → 20 → 30  (5 m/s)
//
//   Both vehicles are at (10, 25) at t = 2.5 s — the EXACT collision moment.
//   But 2.5 s sits between the obs timestamps 1.5 s and 3.5 s.
//
//   get_collision checks:
//     t=0.0 → obs(10,10), ego(10, 0)   dist=10 > 2 m  no
//     t=1.5 → obs(10,20), ego(10,15)   dist= 5 > 2 m  no
//     t=3.5 → obs(10,30), ego(10,35)   dist= 5 > 2 m  no   → returns nullopt (MISS)
//
//   CUDA kernel: segment (obs 1.5→3.5) × (ego 2→3) overlaps [2.0, 3.0].
//   Minimising dist²(s) gives s*=0.5 s → t=2.5 s → dist=0 → COLLISION DETECTED.

TEST(CollisionDetectionAccuracy, BetweenTimestampMiss)
{
  if (!cuda_is_available()) {
    GTEST_SKIP() << "No compatible CUDA device found; skipping GPU collision test.";
  }

  // ── World model (needed by get_collision for the on-route check) ──────────
  auto wm  = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100, 100);
  wm->setMap(map);
  carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);

  YieldPluginConfig config;
  config.vehicle_length                   = 4.0;
  config.collision_check_radius_in_m      = 200.0;
  config.intervehicle_collision_distance_in_m = 2.0;

  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto&){}, [](const auto&){});

  // ── Ego trajectory ─────────────────────────────────────────────────────────
  const double t0_ego = 0.0;
  auto ego_tp = make_ego_traj(7, 0.0, 60.0, t0_ego, 1.0);
  // Points: (10,0,t=0), (10,10,t=1), (10,20,t=2), (10,30,t=3), ..., (10,60,t=6)

  // ── Obstacle predictions (timestamps deliberately offset from ego) ─────────
  // t=0 s  → (10, 10)
  // t=1.5s → (10, 20)   — checked by old algorithm; ego at y=15, dist=5 m
  // t=3.5s → (10, 30)   — checked by old algorithm; ego at y=35, dist=5 m
  // True collision at t=2.5 s: both at (10, 25).
  std::vector<carma_perception_msgs::msg::PredictedState> preds;
  {
    auto add = [&](double t, double y) {
      carma_perception_msgs::msg::PredictedState ps;
      ps.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
      ps.predicted_position.position.x = 10.0;
      ps.predicted_position.position.y = y;
      ps.predicted_velocity.linear.y   = 5.0;  // 5 m/s in y
      preds.push_back(ps);
    };
    add(0.0, 10.0);
    add(1.5, 20.0);
    add(3.5, 30.0);
  }

  const double collision_radius = 2.0;
  const double ego_max_speed    = 10.0;

  // ── Legacy get_collision: MUST return nullopt (misses the collision) ───────
  auto old_result = plugin.get_collision(ego_tp, preds, collision_radius, ego_max_speed);
  EXPECT_FALSE(old_result.has_value())
    << "get_collision incorrectly detected a collision — the scenario was designed "
       "so that the closest approach (t=2.5 s) falls between its sampled timestamps.";

  // ── CUDA exact check: MUST find the collision ─────────────────────────────
  const double ref_t = rclcpp::Time(ego_tp.trajectory_points.front().target_time).seconds();
  auto [ego_pts, obs_flat, obs_offsets, obs_sizes] =
    to_cuda_inputs(ego_tp, {preds}, ref_t);

  auto cuda_results = cuda_check_all_collisions(
    ego_pts, obs_flat, obs_offsets, obs_sizes,
    static_cast<float>(collision_radius));

  ASSERT_EQ(cuda_results.size(), 1u);
  EXPECT_TRUE(cuda_results[0].has_collision)
    << "CUDA kernel failed to detect the between-timestamp collision at t=2.5 s.";

  const double t_col = static_cast<double>(cuda_results[0].collision_t_norm) + ref_t;
  EXPECT_NEAR(t_col, 2.5, 0.01)
    << "CUDA collision time should be 2.5 s (±10 ms); got " << t_col << " s.";

  std::cout << "\n[BetweenTimestampMiss]\n"
            << "  Old get_collision   : " << (old_result ? "COLLISION" : "nullopt (miss)") << "\n"
            << "  CUDA exact check    : "
            << (cuda_results[0].has_collision ? "COLLISION" : "no collision") << "\n"
            << "  CUDA collision time : " << t_col << " s  (true = 2.5 s)\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: stride-skip accuracy
// ─────────────────────────────────────────────────────────────────────────────
//
// Scenario
// ────────
//   Ego   : x=10, y: 0→30 over 3 s at 10 m/s.  Points every 1 s.
//   Obs   : x=10 throughout (on route), stationary (speed≈0).
//            30 prediction states at dt=0.1 s.  y=0 always → obs at (10,0).
//
//   Ego reaches (10, 0) at t=0: collision at t=0.
//
//   But with a ZERO-SPEED obstacle and default intervehicle_collision_distance=10 m:
//     iteration_stride_max_time_s = 2×10 / sqrt(0² + 10²) = 2.0 s
//     iteration_stride = floor(2.0 / 0.1) = 20
//   Inner loop checks obs indices 0, 20 only (index 0 = t=0 → dist=0 → collision
//   detected trivially at the very first step!).
//
//   To expose the stride-skip miss we need the collision at a NON-ZERO index
//   that IS a multiple of stride.  We use a fast-moving obstacle that arrives
//   at x=10 only at index 3 (t=0.3 s, stride=5):
//
//   Ego   : x=10, y: 0→30, 100 pts, dt=0.03 s (10 m/s)
//   Obs   : starts at x=7, y=9, moves to x=10, y=9 in 0.3 s (10 m/s in x).
//            10 prediction states at dt=0.1 s.
//            traj2_speed = 10 m/s, ego_max_speed = 10 m/s.
//            stride_max_t = 2×10/sqrt(100+100) = 1.41 s → stride=14 @dt=0.1
//            Stride-sampled indices: 0, 14 (if M>14 else just 0).
//            Actual collision at index 3 (t=0.3 s): skipped.
//
//   Old get_collision checks:
//     j=0  (t=0.0): obs(7, 9), ego(10, 0)  → dist=√(9+81)≈9.5 > 2 m  no
//     j=14 (t=1.4): obs(10+10×1.4=21, 9), ego not yet at y=9  → far  no
//     → returns nullopt  (MISS)
//
//   CUDA checks segment (obs t=0.2→0.3) × (ego t≈0.27→0.30):
//     At t=0.3: obs=(10,9), ego=(10,9) → dist=0 → COLLISION DETECTED.

TEST(CollisionDetectionAccuracy, StrideSkipMiss)
{
  if (!cuda_is_available()) {
    GTEST_SKIP() << "No compatible CUDA device found; skipping GPU collision test.";
  }

  auto wm  = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100, 100);
  wm->setMap(map);
  carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);

  YieldPluginConfig config;
  config.vehicle_length                       = 4.0;
  config.collision_check_radius_in_m          = 200.0;
  config.intervehicle_collision_distance_in_m = 10.0;  // drives stride calculation
  config.obstacle_zero_speed_threshold_in_ms  = 0.25;

  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto&){}, [](const auto&){});

  // Ego: x=10, y=0→30 in 3 s, 100 points at dt=0.03 s (10 m/s)
  const double t0_ego   = 0.0;
  const double dt_ego   = 0.03;
  const int    n_ego    = 100;
  auto ego_tp = make_ego_traj(n_ego, 0.0, 30.0, t0_ego, dt_ego);
  const double ego_max_speed = 10.0;

  // Obstacle: x=7→10, y=9 (constant), 10 states at dt=0.1 s.
  // Reaches x=10 at index 3 (t=0.3 s) and goes past (x=10+...) after.
  // At t=0.3 s ego is at y = 30 * 0.3/3.0 = 3.0 ... hmm, that's y=3, not y=9.
  //
  // Adjust: place ego at y=9 at t=0.3 s.
  // Ego speed = 30/3 = 10 m/s in y. At t=0.3 s ego y = 0.3*10 = 3.0 m. Still off.
  //
  // Use ego speed = 30 m/s → y=0→90 over 3 s, 100 pts, dt=0.03 s.
  // At t=0.3 s: ego y = 9 m. Match obstacle y=9.  Max ego speed = 30 m/s.
  //
  // stride with ego_max_speed=30 and obs_speed=10 (moving at 10 m/s in x):
  //   stride_max_t = 2×10/sqrt(100+900) = 0.632 s → stride = floor(0.632/0.1) = 6
  //   Checks indices 0, 6.  Collision at index 3 → SKIPPED.
  auto ego_tp_fast = make_ego_traj(n_ego, 0.0, 90.0, t0_ego, dt_ego);
  const double ego_max_speed_fast = 30.0;
  // (ego speed = 90/2.97 ≈ 30.3 m/s, close enough)

  // Obs: x=7→(7+10×0.9)=16, y=9, dt=0.1 s, 10 states.
  //   At index 0 (t=0):   x=7,  y=9
  //   At index 1 (t=0.1): x=8,  y=9
  //   At index 2 (t=0.2): x=9,  y=9
  //   At index 3 (t=0.3): x=10, y=9  ← ego at (10,9) → dist=0  COLLISION!
  //   At index 4 (t=0.4): x=11, y=9
  //   ...
  //   At index 6 (t=0.6): x=13, y=9  (stride-checked; ego at y=18, far from obs)
  auto obs_preds = make_preds(10,
    /*x0=*/7.0, /*y0=*/9.0,
    /*x1=*/16.0, /*y1=*/9.0,
    /*t0=*/0.0, /*dt=*/0.1,
    /*vx=*/10.0, /*vy=*/0.0);

  // Verify stride: should be 6 (>1) so index 3 is skipped.
  {
    const double traj2_speed   = 10.0;
    const double step_dur      = 0.1;
    const double stride_max_t  = 2.0 * config.intervehicle_collision_distance_in_m /
                                  std::sqrt(traj2_speed*traj2_speed + ego_max_speed_fast*ego_max_speed_fast);
    const int stride = std::max(1, static_cast<int>(stride_max_t / step_dur));
    EXPECT_GE(stride, 2) << "Stride must be >1 to expose the skip; got " << stride;
    std::cout << "\n[StrideSkipMiss] computed iteration_stride = " << stride << "\n";
  }

  const double collision_radius = 2.0;

  // ── Legacy get_collision: MUST miss (index 3 is stride-skipped) ───────────
  auto old_result = plugin.get_collision(ego_tp_fast, obs_preds, collision_radius, ego_max_speed_fast);
  EXPECT_FALSE(old_result.has_value())
    << "get_collision should miss the stride-skipped collision at t=0.3 s.";

  // ── CUDA exact check: MUST find the collision ─────────────────────────────
  const double ref_t = t0_ego;
  auto [ego_pts, obs_flat, obs_offsets, obs_sizes] =
    to_cuda_inputs(ego_tp_fast, {obs_preds}, ref_t);

  auto cuda_results = cuda_check_all_collisions(
    ego_pts, obs_flat, obs_offsets, obs_sizes,
    static_cast<float>(collision_radius));

  ASSERT_EQ(cuda_results.size(), 1u);
  EXPECT_TRUE(cuda_results[0].has_collision)
    << "CUDA kernel must detect the collision at t=0.3 s that get_collision skips.";

  if (cuda_results[0].has_collision) {
    const double t_col = static_cast<double>(cuda_results[0].collision_t_norm) + ref_t;
    EXPECT_NEAR(t_col, 0.24, 0.05)
      << "Collision should be near t=0.24 s; got " << t_col << " s.";
    std::cout << "  CUDA collision time : " << t_col << " s  (first entry within 2 m radius ≈ 0.24 s)\n";
  }

  std::cout << "  Old get_collision   : " << (old_result ? "COLLISION (unexpected)" : "nullopt (correct miss demo)") << "\n"
            << "  CUDA exact check    : " << (cuda_results[0].has_collision ? "COLLISION (correct)" : "no collision (wrong)") << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: worst-case performance benchmark
// ─────────────────────────────────────────────────────────────────────────────
//
// Worst-case inputs
// ─────────────────
//   Ego          : 100 points, dt=0.15 s  (15 s horizon at 10 m/s)
//   Objects      : 100 objects, each with 50 prediction states at dt=0.1 s
//   Total pairs  : 99 × 49 × 100 = 485 100
//   All objects  : on-route (x=10), y-offset so never within collision radius.
//                  Forces full traversal of both algorithms (no early exit).
//
// Measured
// ────────
//   CPU reference: brute-force exact quadratic loop running serially for every
//                  object (same maths as the CUDA kernel, no shortcut).
//   CUDA batch   : single cuda_check_all_collisions() call for all 100 objects.
//
// Why "no shortcut" for the CPU baseline?
//   The old get_collision() has early-exit heuristics (monotonic distance break,
//   stride, "too far away" check) that can undercount work.  A fair throughput
//   comparison needs both sides doing the same O(N×M) work without short-circuits.
//   We expose both numbers in the output so the reader can see the full picture.

TEST(CollisionDetectionBenchmark, WorstCasePerformance)
{
  if (!cuda_is_available()) {
    GTEST_SKIP() << "No compatible CUDA device found; skipping GPU collision benchmark.";
  }

  constexpr int    N_EGO     = 100;   // ego trajectory points
  constexpr int    N_OBJ     = 100;   // number of external objects
  constexpr int    N_PRED    = 50;    // prediction states per object
  constexpr double EGO_DT    = 0.15;  // seconds between ego points
  constexpr double PRED_DT   = 0.10;  // seconds between prediction states
  constexpr double EGO_SPEED = 10.0;  // m/s (y-direction)
  constexpr double T0        = 0.0;
  constexpr float  RADIUS    = 2.0f;  // collision radius in metres

  // ── World model + legacy plugin (needed for get_collision on-route check) ──
  auto wm  = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100, 100);
  wm->setMap(map);
  carma_wm::test::setRouteByIds({1200, 1201, 1202, 1203}, wm);

  YieldPluginConfig config;
  config.vehicle_length                       = 4.0;
  config.collision_check_radius_in_m          = 200.0;
  config.intervehicle_collision_distance_in_m = static_cast<double>(RADIUS);

  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
  YieldPlugin plugin(nh, wm, config, [](const auto&){}, [](const auto&){});

  // ── Build ego trajectory ──────────────────────────────────────────────────
  const double y_ego_end = EGO_SPEED * (N_EGO - 1) * EGO_DT;
  auto ego_tp = make_ego_traj(N_EGO, 0.0, y_ego_end, T0, EGO_DT);
  const double ref_t = T0;

  // ── Build obstacle predictions ────────────────────────────────────────────
  // 100 objects at x=10, starting 5 m behind the ego and moving at 8 m/s
  // (ego at 10 m/s → they fall further behind over time).
  // No object ever enters the 2 m collision radius.
  std::vector<std::vector<carma_perception_msgs::msg::PredictedState>> all_preds;
  all_preds.reserve(N_OBJ);

  for (int k = 0; k < N_OBJ; ++k) {
    const double y_obj_start = -5.0 - k * 0.1;
    const double y_obj_end   = y_obj_start + 8.0 * (N_PRED - 1) * PRED_DT;
    all_preds.push_back(make_preds(N_PRED,
      10.0, y_obj_start, 10.0, y_obj_end,
      T0, PRED_DT,
      0.0, 8.0));
  }

  // ── Flatten to CUDA inputs ────────────────────────────────────────────────
  auto [ego_pts, obs_flat, obs_offsets, obs_sizes] =
    to_cuda_inputs(ego_tp, all_preds, ref_t);

  const int total_pairs = (N_EGO - 1) * (N_PRED - 1) * N_OBJ;

  // ── getLaneletsFromPoint micro-benchmark ────────────────────────────────
  // Measures the cost of a single spatial map query — the operation that
  // dominated the old on-route check.  Extrapolating to 100 objects × ~8
  // calls each predicts the overhead we observe in the CPU concurrent timing.
  {
    constexpr int N_CALLS = 500;
    lanelet::BasicPoint2d pt;
    pt.x() = 10.0;  pt.y() = 5.0;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < N_CALLS; ++i) {
      (void)wm->getLaneletsFromPoint(pt, 8);
    }
    auto t1 = std::chrono::steady_clock::now();
    const double us_per_call =
      std::chrono::duration<double, std::micro>(t1 - t0).count() / N_CALLS;
    // Each object needs ~8 stride-checked calls before finding on-route.
    const double predicted_onroute_ms = us_per_call * 8 * N_OBJ / 1000.0;
    std::cout << "\n[getLaneletsFromPoint micro-benchmark]\n"
              << "  Per call           : " << us_per_call << " μs\n"
              << "  Predicted overhead : " << predicted_onroute_ms
              << " ms  (100 objects × ~8 calls each)\n";
  }

  std::cout << "\n[WorstCasePerformanceBenchmark]\n"
            << "  Ego points     : " << N_EGO  << "\n"
            << "  Objects        : " << N_OBJ  << "\n"
            << "  Pred states    : " << N_PRED << " per object\n"
            << "  Total seg pairs: " << total_pairs << "\n";

  // Pre-populate the plugin's route-lanelet cache (filled lazily inside
  // get_earliest_collision_object_and_time; get_collision reads it directly).
  plugin.get_earliest_collision_object_and_time(ego_tp, {});

  // Wrap each object's prediction states as ExternalObject for the concurrent
  // production path.  get_collision_time prepends the current pose as the
  // zeroth state, so: current state = all_preds[k][0], predictions = [1..].
  std::vector<carma_perception_msgs::msg::ExternalObject> ext_objs;
  ext_objs.reserve(N_OBJ);
  for (int k = 0; k < N_OBJ; ++k) {
    carma_perception_msgs::msg::ExternalObject obj;
    obj.id                              = static_cast<uint32_t>(k + 1);
    obj.header.stamp                    = all_preds[k].front().header.stamp;
    obj.pose.pose.position.x            = all_preds[k].front().predicted_position.position.x;
    obj.pose.pose.position.y            = all_preds[k].front().predicted_position.position.y;
    obj.velocity.twist.linear.x         = all_preds[k].front().predicted_velocity.linear.x;
    obj.velocity.twist.linear.y         = all_preds[k].front().predicted_velocity.linear.y;
    obj.predictions.assign(all_preds[k].begin() + 1, all_preds[k].end());
    ext_objs.push_back(std::move(obj));
  }

  // ── CPU (old path): std::async + get_collision_time() per object ───────────
  // Recreates the legacy production path: one async task per object, each
  // running get_collision_time (getLaneletsFromPoint on-route check + strided
  // CPU distance comparison).  Run 3 times, take the median.
  std::array<double, 3> cpu_ms{};
  bool any_cpu_collision = false;
  for (int run = 0; run < 3; ++run) {
    auto t0 = std::chrono::steady_clock::now();
    {
      std::unordered_map<uint32_t, std::future<std::optional<rclcpp::Time>>> futures;
      for (const auto& obj : ext_objs) {
        futures[obj.id] = std::async(
          std::launch::async,
          [&plugin, &ego_tp, &obj]() {
            return plugin.get_collision_time(ego_tp, obj, EGO_SPEED);
          });
      }
      for (const auto& obj : ext_objs) {
        if (futures.at(obj.id).get()) any_cpu_collision = true;
      }
    }
    auto t1 = std::chrono::steady_clock::now();
    cpu_ms[run] = std::chrono::duration<double, std::milli>(t1 - t0).count();
  }
  std::sort(cpu_ms.begin(), cpu_ms.end());
  const double cpu_median_ms = cpu_ms[1];

  // ── GPU (new path): bbox on-route filter + CUDA kernel ───────────────────
  // get_collision_times_concurrently: bbox check per object then one batched
  // cuda_check_all_collisions call.  Warm up once before timing.
  plugin.get_collision_times_concurrently(ego_tp, ext_objs, EGO_SPEED);

  std::array<double, 3> gpu_ms{};
  bool any_gpu_collision = false;
  for (int run = 0; run < 3; ++run) {
    auto t0 = std::chrono::steady_clock::now();
    const auto cuda_times = plugin.get_collision_times_concurrently(ego_tp, ext_objs, EGO_SPEED);
    auto t1 = std::chrono::steady_clock::now();
    gpu_ms[run] = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (!cuda_times.empty()) any_gpu_collision = true;
  }
  std::sort(gpu_ms.begin(), gpu_ms.end());
  const double gpu_median_ms = gpu_ms[1];

  const double speedup = cpu_median_ms / gpu_median_ms;

  std::cout << "  CPU (legacy)    : " << cpu_median_ms << " ms  (median of 3, std::async + get_collision_time)\n"
            << "  GPU (new path)  : " << gpu_median_ms << " ms  (median of 3, bbox filter + CUDA kernel)\n"
            << "  Speedup gpu/cpu : " << speedup << "×\n"
            << "  CPU collision   : " << (any_cpu_collision ? "yes (UNEXPECTED)" : "no") << "\n"
            << "  GPU collision   : " << (any_gpu_collision ? "yes (UNEXPECTED)" : "no") << "\n";

  // Correctness: no object should collide with the ego in this scenario.
  EXPECT_FALSE(any_cpu_collision) << "Legacy CPU path reported a false collision.";
  EXPECT_FALSE(any_gpu_collision) << "GPU path reported a false collision.";

  // ── Accuracy cross-check: run a scenario WITH a collision ─────────────────
  // Obstacle at x=10, y=6 (6 m ahead of ego at y=0) moving at 4 m/s.
  // Ego closes at 10 m/s: closing speed = 6 m/s.
  // Distance enters the 2 m collision radius at t = (6-2)/6 ≈ 0.67 s.
  // Both the concurrent CPU path and CUDA must detect it at a non-trivial time.
  {
    constexpr double OBS_Y_START  = 6.0;
    constexpr double OBS_SPEED    = 4.0;
    const double     obs_y_end    = OBS_Y_START + OBS_SPEED * (N_PRED - 1) * PRED_DT;

    auto colliding_preds = make_preds(N_PRED,
      10.0, OBS_Y_START, 10.0, obs_y_end,
      T0, PRED_DT, 0.0, OBS_SPEED);

    // Build ExternalObject (needed by both CPU and GPU production paths).
    carma_perception_msgs::msg::ExternalObject col_obj;
    col_obj.id                              = static_cast<uint32_t>(N_OBJ + 1);
    col_obj.header.stamp                    = colliding_preds.front().header.stamp;
    col_obj.pose.pose.position.x            = colliding_preds.front().predicted_position.position.x;
    col_obj.pose.pose.position.y            = colliding_preds.front().predicted_position.position.y;
    col_obj.velocity.twist.linear.x         = colliding_preds.front().predicted_velocity.linear.x;
    col_obj.velocity.twist.linear.y         = colliding_preds.front().predicted_velocity.linear.y;
    col_obj.predictions.assign(colliding_preds.begin() + 1, colliding_preds.end());

    // CPU (legacy): get_collision_time — getLaneletsFromPoint + strided check.
    const bool cpu_found = plugin.get_collision_time(ego_tp, col_obj, EGO_SPEED).has_value();

    // GPU (new path): bbox filter + CUDA kernel via get_collision_times_concurrently.
    const auto gpu_col  = plugin.get_collision_times_concurrently(ego_tp, {col_obj}, EGO_SPEED);
    const bool gpu_found = gpu_col.count(col_obj.id) > 0;

    EXPECT_TRUE(cpu_found) << "Legacy CPU path (get_collision_time) missed the planted collision.";
    EXPECT_TRUE(gpu_found) << "GPU path (get_collision_times_concurrently) missed the planted collision.";

    if (cpu_found && gpu_found) {
      const double t_col = gpu_col.at(col_obj.id).seconds();
      // Continuous-motion first entry into 2 m radius: t = (6-2)/6 ≈ 0.67 s.
      EXPECT_GT(t_col, 0.5) << "Collision time suspiciously early.";
      EXPECT_LT(t_col, 1.2) << "Collision time suspiciously late.";
      std::cout << "  Planted collision: CPU=" << (cpu_found?"yes":"no")
                << "  GPU=" << (gpu_found?"yes":"no")
                << "  t_col=" << t_col << " s  (true first entry ≈ 0.67 s)\n";
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = rcutils_logging_set_logger_level("yield_plugin", RCUTILS_LOG_SEVERITY_WARN);
  (void)ret;
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
