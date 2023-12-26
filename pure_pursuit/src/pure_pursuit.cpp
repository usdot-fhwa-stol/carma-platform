// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>
#include <algorithm>
#include <limits>
#include <utility>
#include "pure_pursuit/pure_pursuit.hpp"


#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace motion
{
namespace control
{
namespace pure_pursuit
{

constexpr uint32_t CAPACITY = autoware_auto_msgs::msg::Trajectory::CAPACITY;
////////////////////////////////////////////////////////////////////////////////
PurePursuit::PurePursuit(const Config & cfg)
: ControllerBase{::motion::control::controller_common::BehaviorConfig{
            5.0F,
            std::chrono::milliseconds{100LL},
            ::motion::control::controller_common::ControlReference::SPATIAL}},
  m_lookahead_distance(0.0F),
  m_target_point{},
  m_command{},
  m_config(cfg),
  m_iterations(0U)
{
  m_integrator_config = IntegratorConfig();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "Loaded PP Config:" << m_integrator_config);
}

////////////////////////////////////////////////////////////////////////////////
PurePursuit::PurePursuit(const Config & cfg, const IntegratorConfig& i_cfg)
: ControllerBase{::motion::control::controller_common::BehaviorConfig{
            5.0F,
            std::chrono::milliseconds{100LL},
            ::motion::control::controller_common::ControlReference::SPATIAL}},
  m_lookahead_distance(0.0F),
  m_target_point{},
  m_command{},
  m_config(cfg),
  m_integrator_config(i_cfg),
  m_iterations(0U)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "Loaded PP Config:" << m_integrator_config);
}

////////////////////////////////////////////////////////////////////////////////
VehicleControlCommand PurePursuit::compute_command_impl(const TrajectoryPointStamped & current_pose)
{
  const auto start = std::chrono::system_clock::now();

  TrajectoryPoint current_point = current_pose.state;  // copy 32bytes
  compute_errors(current_point);

  if (m_config.get_is_delay_compensation()) {
    current_point =
      predict(current_point, start - time_utils::from_message(current_pose.header.stamp));
  }

  compute_lookahead_distance(current_point.longitudinal_velocity_mps);
  const auto is_success = compute_target_point(current_point);

  if (is_success) {
    m_command.long_accel_mps2 = compute_command_accel_mps(current_point, false);
    m_command.front_wheel_angle_rad = compute_steering_rad(current_point);
    m_command.rear_wheel_angle_rad = 0.0F;
    // Use velocity of the next immediate trajectory point as the target velocity
    m_command.velocity_mps = get_reference_trajectory().points[get_current_state_spatial_index()
      ].longitudinal_velocity_mps;
  } else {
    m_command = compute_stop_command(current_pose);
  }
  ++m_iterations;



  return m_command;
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuit::setIntegratorConfig(const IntegratorConfig& i_cfg)
{
  m_integrator_config = i_cfg;
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuit::compute_errors(const TrajectoryPoint & current_point)
{
  TrajectoryPoint target;
  const auto & traj = get_reference_trajectory();
  if (traj.points.empty()) {
    // Do nothing
    target.x = current_point.x;
    target.y = current_point.y;
    target.heading = current_point.heading;
    target.longitudinal_velocity_mps = 0.0F;  // should be stopped
    target.heading_rate_rps = 0.0F;
    target.acceleration_mps2 = 0.0F;
  } else if (traj.points.size() == 1U) {
    target = traj.points[0U];
  } else {
    // This is the first point on trajectory just after the current_point; closest is either this
    // index, or the one just before
    const auto first_idx_after_current = get_current_state_spatial_index();
    auto nearest_idx = decltype(first_idx_after_current) {};
    auto second_nearest_idx = decltype(first_idx_after_current) {1U};
    auto dist_nearest = std::numeric_limits<float32_t>::max();
    auto dist_second_nearest = std::numeric_limits<float32_t>::max();
    if (0U != first_idx_after_current) {
      const auto & next_point = traj.points[first_idx_after_current];
      const auto dist_next = compute_points_distance_squared(current_point, next_point);
      const auto last_idx = first_idx_after_current - 1U;
      const auto & last_point = traj.points[last_idx];
      const auto dist_last = compute_points_distance_squared(current_point, last_point);
      if (dist_last < dist_next) {
        nearest_idx = last_idx;
        second_nearest_idx = first_idx_after_current;
        dist_nearest = dist_last;
        dist_second_nearest = dist_next;
      } else {
        nearest_idx = first_idx_after_current;
        second_nearest_idx = last_idx;
        dist_nearest = dist_next;
        dist_second_nearest = dist_last;
      }
    }
    const auto & nearest_point = traj.points[nearest_idx];
    const auto & second_point = traj.points[second_nearest_idx];
    // Decide which side of the point is the second nearest point
    const auto dist_nearest_second =
      compute_points_distance_squared(nearest_point, second_point);
    if (dist_nearest_second < 0.1F) {
      target = nearest_point;
    } else {
      // linear interpolation: Compute the interpolated point `P` between `A` and `B`.
      // A is the nearest point, B is the second nearest point, and C is the current point.
      //     AP (r)       P   BP (1 - r)
      // A----------------*-------B
      //                  |
      //                  * C
      const float32_t rate_a =
        0.5F * (((dist_nearest_second + dist_nearest) - dist_second_nearest) / dist_nearest_second);
      target = ::motion::motion_common::interpolate(nearest_point, second_point, rate_a);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuit::compute_lookahead_distance(const float32_t current_velocity)
{
  const float32_t lookahead_distance =
    fabsf(current_velocity * m_config.get_speed_to_lookahead_ratio());
  m_lookahead_distance =
    std::max(
    m_config.get_minimum_lookahead_distance(),
    std::min(lookahead_distance, m_config.get_maximum_lookahead_distance()));
}
////////////////////////////////////////////////////////////////////////////////
bool8_t PurePursuit::in_traveling_direction(
  const TrajectoryPoint & current_point,
  const TrajectoryPoint & point) const
{
  bool8_t in_traveling_direction;
  const float32_t x_offset = compute_relative_xy_offset(current_point, point).first;
  const bool8_t is_frontward = (x_offset > 0.0F) ? true : false;
  const float32_t speed_thres = m_config.get_speed_thres_traveling_direction();
  if (((current_point.longitudinal_velocity_mps > speed_thres) && is_frontward) ||
    ((current_point.longitudinal_velocity_mps < speed_thres) && (!is_frontward)))
  {  // uncrustify
    // if the current velocity is over 0, the current target should be in the frontward
    // if the current velocity is under 0 (reversed),
    // the current target should be in the backward
    in_traveling_direction = true;
  } else if (fabsf(current_point.longitudinal_velocity_mps) < speed_thres) {
    // if the current velocity is under speed_thres,
    // can not decide which direction is the traveling direction at the moment.
    in_traveling_direction = true;
  } else {
    in_traveling_direction = false;
  }
  return in_traveling_direction;
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuit::compute_interpolate_target_point(
  const TrajectoryPoint & current_point,
  const TrajectoryPoint & target_point,
  const uint32_t idx)
{
  const bool8_t has_idx = (idx != 0U);
  const uint32_t prev_idx = has_idx ? (idx - 1U) : 0U;
  const TrajectoryPoint & prev_target_point =
    has_idx ? get_reference_trajectory().points[prev_idx] : current_point;
  // Compute the intersection between the line and the circle
  // The center of circle is the current position and moved into (0, 0)
  // compute the linear equation (ax + by + c = 0) from the prev and current points
  // link: https://cp-algorithms.com/geometry/circle-line-intersection.html
  const float32_t target_rel_x = target_point.x - current_point.x;
  const float32_t target_rel_y = target_point.y - current_point.y;
  const float32_t prev_target_rel_x = prev_target_point.x - current_point.x;
  const float32_t prev_target_rel_y = prev_target_point.y - current_point.y;
  const float32_t diff_x = target_rel_x - prev_target_rel_x;
  const float32_t diff_y = target_rel_y - prev_target_rel_y;
  const float32_t a = diff_y;
  const float32_t b = -diff_x;
  const float32_t c = -(diff_y * prev_target_rel_x) + (diff_x * prev_target_rel_y);
  // Using the distance between the point and line, and the circule radius, compute the intersection
  const float32_t aa_bb = (a * a) + (b * b);
  const float32_t aa_bb_inv = 1.0F / aa_bb;
  const float32_t cc = c * c;
  const float32_t rr = m_lookahead_distance * m_lookahead_distance;
  constexpr float32_t epsilon = 0.001F;
  if ((aa_bb < epsilon) || ((cc - (rr * aa_bb)) > epsilon)) {
    // No intersection. Do not interpolate, and use the original target point
    m_target_point = target_point;
  } else {
    const float32_t origin_x = -(a * c) * aa_bb_inv;
    const float32_t origin_y = -(b * c) * aa_bb_inv;
    if (fabsf(cc - (rr * aa_bb)) < epsilon) {
      // 1 intersection
      m_target_point.x = origin_x + current_point.x;
      m_target_point.y = origin_y + current_point.y;
    } else {
      // 2 intersections
      const float32_t d = rr - (cc * aa_bb_inv);
      const float32_t offset = sqrtf(d * aa_bb_inv);
      const float32_t point1_x = origin_x + (b * offset);
      const float32_t point1_y = origin_y - (a * offset);
      // the predicted point should be between current and its previous target points
      const float32_t dist_prev_current_target = sqrtf(aa_bb);
      const float32_t diff_pred_curr_x = target_rel_x - point1_x;
      const float32_t diff_pred_curr_y = target_rel_y - point1_y;
      const float32_t dist_pred_current_target =
        sqrtf((diff_pred_curr_x * diff_pred_curr_x) + (diff_pred_curr_y * diff_pred_curr_y));
      if (dist_pred_current_target <= dist_prev_current_target) {
        m_target_point.x = point1_x + current_point.x;
        m_target_point.y = point1_y + current_point.y;
      } else {
        m_target_point.x = (origin_x - (b * offset)) + current_point.x;
        m_target_point.y = (origin_y + (a * offset)) + current_point.y;
      }
    }
    // a^2 - r^2 = b^2 - (c - r)^2 -> rate_a = r / c
    const float32_t rate_a =
      sqrtf(compute_points_distance_squared(m_target_point, prev_target_point) / aa_bb);
    m_target_point = ::motion::motion_common::interpolate(prev_target_point, target_point, rate_a);
  }
}
////////////////////////////////////////////////////////////////////////////////
bool8_t PurePursuit::compute_target_point(const TrajectoryPoint & current_point)
{
  auto idx = static_cast<uint32_t>(get_current_state_spatial_index());
  bool8_t is_travel_direct = false;
  uint32_t last_idx_for_noupdate = 0U;
  const auto & traj = get_reference_trajectory();
  for (; idx < traj.points.size(); ++idx) {
    const TrajectoryPoint & target_point = traj.points[idx];
    // judge wheter the target point is in the forward of the traveling direction
    if (in_traveling_direction(current_point, target_point)) {
      is_travel_direct = true;
      last_idx_for_noupdate = idx;

      // Search the closest point over the lookahead distance
      if ((sqrtf(compute_points_distance_squared(current_point, target_point)) >=
        m_lookahead_distance))
      {
        if (m_config.get_is_interpolate_lookahead_point()) {
          // interpolate points between idx-1 and idx
          compute_interpolate_target_point(current_point, target_point, idx);
        } else {
          m_target_point = target_point;
        }
        break;
      }
    }
  }

  bool8_t is_success = true;
  // If all points are within the distance threshold,
  if (idx == traj.points.size()) {
    if (is_travel_direct) {
      // use the farthest target index in the traveling direction
      m_target_point = traj.points[last_idx_for_noupdate];
    } else {
      // If the trajectory was not updated and there is no point in the traveling direction
      is_success = false;
    }
  }
  return is_success;
}
////////////////////////////////////////////////////////////////////////////////
float32_t PurePursuit::compute_points_distance_squared(
  const TrajectoryPoint & point1,
  const TrajectoryPoint & point2)
{
  const float32_t diff_x = point1.x - point2.x;
  const float32_t diff_y = point1.y - point2.y;
  return (diff_x * diff_x) + (diff_y * diff_y);
}
////////////////////////////////////////////////////////////////////////////////
std::pair<float32_t, float32_t> PurePursuit::compute_relative_xy_offset(
  const TrajectoryPoint & current,
  const TrajectoryPoint & target) const
{
  const float32_t diff_x = target.x - current.x;
  const float32_t diff_y = target.y - current.y;
  const auto cos_pose =
    (current.heading.real + current.heading.imag) * (current.heading.real - current.heading.imag);
  const auto sin_pose = 2.0F * current.heading.real * current.heading.imag;
  const float32_t relative_x = (cos_pose * diff_x) + (sin_pose * diff_y);
  const float32_t relative_y = (-sin_pose * diff_x) + (cos_pose * diff_y);
  const std::pair<float32_t, float32_t> relative_xy(relative_x, relative_y);
  return relative_xy;
}
////////////////////////////////////////////////////////////////////////////////
float32_t PurePursuit::compute_steering_rad(const TrajectoryPoint & current_point)
{
  // Compute the steering angle by arctan(carvature * wheel_distance)
  // link: https://www.ri.cmu.edu/pub_files/2009/2/
  //       Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
  const float32_t denominator = compute_points_distance_squared(current_point, m_target_point);
  const float32_t numerator = compute_relative_xy_offset(current_point, m_target_point).second;
  constexpr float32_t epsilon = 0.0001F;
  // equivalent to (2 * y) / (distance * distance) = (2 * sin(th)) / distance
  const float32_t curvature = (denominator > epsilon) ? ((2.0F * numerator) / denominator) : 0.0F;
  float32_t steering_angle_rad = atanf(curvature * m_config.get_distance_front_rear_wheel());

  if (m_integrator_config.is_integrator_enabled)
  {
    // error=kappa*lookahead*lookahead/2;
    double error = curvature * denominator / 2;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "error integrator: " << error);

    // Integral term
    m_integrator_config.integral += error * m_integrator_config.dt;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), "Integral integrator: " << m_integrator_config.integral);

    if (m_integrator_config.integral > m_integrator_config.integrator_max_pp){
        m_integrator_config.integral = m_integrator_config.integrator_max_pp;
    }
    else if (m_integrator_config.integral < m_integrator_config.integrator_min_pp){
      m_integrator_config.integral = m_integrator_config.integrator_min_pp;
    }

    double I_out = m_integrator_config.Ki_pp * m_integrator_config.integral;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("pure_pursuit_wrapper"), " I_out pp: " <<  I_out);

    steering_angle_rad = steering_angle_rad + static_cast<float>(I_out);
  }

  return steering_angle_rad;
}
////////////////////////////////////////////////////////////////////////////////
float32_t PurePursuit::compute_command_accel_mps(
  const TrajectoryPoint & current_point,
  const bool8_t is_emergency) const
{
  float32_t distance;
  float32_t target_velocity;
  if (is_emergency) {
    distance = m_config.get_emergency_stop_distance();
    target_velocity = 0.0F;
  } else {
    const float32_t diff = sqrtf(compute_points_distance_squared(current_point, m_target_point));
    constexpr float32_t diff_thres = 0.01F;  // meter: To avoid dividing 0 and strong vibration
    distance = std::max(diff, diff_thres);
    target_velocity = m_target_point.longitudinal_velocity_mps;
  }
  // v^2 - v0^2 = 2ax
  const float32_t abs_accel =
    fabsf(
    (target_velocity * target_velocity) -
    (current_point.longitudinal_velocity_mps * current_point.longitudinal_velocity_mps)) /
    (2.0F * distance);
  const float32_t accel_meter =
    ((target_velocity - current_point.longitudinal_velocity_mps) > 0.0F) ? abs_accel : -abs_accel;
  return accel_meter;
}
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware
