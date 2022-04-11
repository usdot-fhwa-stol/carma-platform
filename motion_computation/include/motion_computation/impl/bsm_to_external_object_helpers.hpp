#pragma once

/*
 * Copyright (C) 2022 LEIDOS.
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

#include <carma_perception_msgs/external_object.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <tf2/LinearMath/Quaternion.h>



namespace object {

namespace conversion {

namespace impl  // Namespace for functionality not meant to be part of public API but valueable to unit test in
                // isolation
{
std::vector<geometry_msgs::msg::Pose> sample_2d_path_from_radius(const geometry_msgs::msg::Pose &pose, double velocity,
                                                                 double radius_of_curvature, double period,
                                                                 double step_size);

std::vector<geometry_msgs::msg::Pose> sample_2d_linear_motion(const geometry_msgs::msg::Pose &pose, double velocity,
                                                              double period, double step_size);

geometry_msgs::PoseWithCovariance pose_from_gnss(const lanelet::projection::LocalFrameProjector &projector,
                                                 const tf2::Quaternion &ned_in_map_rotation, const GPSPoint &gps_point,
                                                 const double &heading);

std::vector<carma_perception_msgs::PredictedState> predicted_poses_to_predicted_state(
    const std::vector<geometry_msgs::msg::Pose> &poses, double constant_velocity, const rclcpp::Time &start_time,
    const rclcpp::Duration &step_size, const std::string &frame, double initial_pose_confidence,
    double initial_vel_confidence);

rclcpp::Time get_bsm_timestamp(const carma_v2x_msgs::msg::BSM &in_msg);

}  // namespace impl
}  // namespace conversion
}  // namespace object