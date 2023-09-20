// Copyright 2020-2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTION_COMPUTATION__IMPL__PSM_TO_EXTERNAL_OBJECT_HELPERS_HPP_
#define MOTION_COMPUTATION__IMPL__PSM_TO_EXTERNAL_OBJECT_HELPERS_HPP_

#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tf2/LinearMath/Quaternion.h>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace motion_computation
{

namespace conversion
{

// Namespace for functionality not meant to be part of public API but valuable
// to unit test in isolation
namespace impl
{
std::vector<geometry_msgs::msg::Pose> sample_2d_path_from_radius(
  const geometry_msgs::msg::Pose & pose, double velocity, double radius_of_curvature, double period,
  double step_size);

std::vector<geometry_msgs::msg::Pose> sample_2d_linear_motion(
  const geometry_msgs::msg::Pose & pose, double velocity, double period, double step_size);

geometry_msgs::msg::PoseWithCovariance pose_from_gnss(
  const lanelet::projection::LocalFrameProjector & projector,
  const tf2::Quaternion & ned_in_map_rotation, const lanelet::GPSPoint & gps_point,
  const double & heading, const double lat_variance, const double lon_variance,
  const double heading_variance);

std::vector<carma_perception_msgs::msg::PredictedState> predicted_poses_to_predicted_state(

  const std::vector<geometry_msgs::msg::Pose> & poses, double constant_velocity,
  const rclcpp::Time & start_time, const rclcpp::Duration & step_size, const std::string & frame,
  double initial_pose_confidence, double initial_vel_confidence);

rclcpp::Time get_psm_timestamp(
  const carma_v2x_msgs::msg::PSM & in_msg, rclcpp::Clock::SharedPtr clock);

}  // namespace impl
}  // namespace conversion
}  // namespace motion_computation

#endif  // MOTION_COMPUTATION__IMPL__PSM_TO_EXTERNAL_OBJECT_HELPERS_HPP_
