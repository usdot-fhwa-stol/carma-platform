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

#ifndef MOTION_COMPUTATION__IMPL__MOBILITY_PATH_TO_EXTERNAL_OBJECT_HELPERS_HPP_
#define MOTION_COMPUTATION__IMPL__MOBILITY_PATH_TO_EXTERNAL_OBJECT_HELPERS_HPP_

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/predicted_state.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/time.hpp>
#include <tuple>
#include <utility>

namespace motion_computation
{

namespace conversion
{
namespace impl
{
/**
 * \brief Composes a PredictedState message form a provided current point and previous point. It
 * calculates the speed from these points using mobility_path_time_step
 * \param curr_pt current point
 * \param prev_pt prev_pt. this point is recorded in the state
 * \param prev_time_stamp prev_pt's time stamp. This time is recorded in the state
 * \param curr_time_stamp The timestamp of the second point when used to compute velocity at prev point
 * \param prev_yaw A previous yaw value in radians to use if the two provided points are equivalent
 * \return std::pair<carma_perception_msgs::msg::PredictedState,double> where the first element is the prediction
 * including linear velocity, last_time, orientation filled in and the second element is the yaw in radians used to
 * compute the orientation
 */
std::pair<carma_perception_msgs::msg::PredictedState, double> composePredictedState(
  const tf2::Vector3 & curr_pt, const tf2::Vector3 & prev_pt, const rclcpp::Time & prev_time_stamp,
  const rclcpp::Time & curr_time_stamp, double prev_yaw);
/**
 * \brief Helper function to fill in the angular velocity of the external object
 * \param ExternalObject to fill in its angular velocities
 */
void calculateAngVelocityOfPredictedStates(carma_perception_msgs::msg::ExternalObject & object);

/**
 * \brief Gets the yaw angle in degrees described by the provided quaternion
 *
 * \param quaternion The quaternion to extract yaw from
 * \return The yaw in degrees
 */
double getYawFromQuaternionMsg(const geometry_msgs::msg::Quaternion & quaternion);

/**
 * \brief Transforms ecef point to map frame using internally saved map transform
 * \param ecef_point ecef_point to transform
 * \param map_projector Projector used for frame conversion
 * \return point in map
 */
tf2::Vector3 transform_to_map_frame(
  const tf2::Vector3 & ecef_point, const lanelet::projection::LocalFrameProjector & map_projector);

}  // namespace impl
}  // namespace conversion
}  // namespace motion_computation

#endif  // MOTION_COMPUTATION__IMPL__MOBILITY_PATH_TO_EXTERNAL_OBJECT_HELPERS_HPP_
