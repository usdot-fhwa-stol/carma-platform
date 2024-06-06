/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "motion_predict/predict_ctrv.hpp"
#include "motion_predict/motion_predict.hpp"
#include <math.h>
#include "Eigen/Dense"


namespace motion_predict
{
namespace ctrv
{
struct CTRV_State
{
  double x = 0;
  double y = 0;
  double yaw = 0;
  double v = 0;  // magnitude of the speed
  double yaw_rate = 0;
};

std::tuple<double, double> localVelOrientationAndMagnitude(const double v_x, const double v_y)
{
  Eigen::Vector2f x_y_vel(v_x, v_y);
  double v_mag = x_y_vel.norm();

  double local_v_orientation;  // The orientation of the velocity vector in the local object frame
  if (fabs(v_mag) < 0.0000001)
  {
    local_v_orientation = 0;
  }
  else
  {
    local_v_orientation = atan2(v_y, v_x);
  }
  return std::make_tuple(local_v_orientation, v_mag);
}

CTRV_State buildCTRVState(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Twist& twist)
{
  geometry_msgs::msg::Quaternion quat = pose.orientation;
  Eigen::Quaternionf e_quat(quat.w, quat.x, quat.y, quat.z);
  Eigen::Vector3f rpy = e_quat.toRotationMatrix().eulerAngles(0, 1, 2);

  auto vel_angle_and_mag = localVelOrientationAndMagnitude(twist.linear.x, twist.linear.y);

  CTRV_State state;
  state.x = pose.position.x;
  state.y = pose.position.y;
  state.yaw = std::get<0>(vel_angle_and_mag);
  //    rpy[2] + std::get<0>(vel_angle_and_mag);  // The yaw is relative to the velocity vector so take the heading and
  //                                              // add it to the angle of the velocity vector in the local frame
  state.v = std::get<1>(vel_angle_and_mag);
  state.yaw_rate = twist.angular.z;

  return state;
}

carma_perception_msgs::msg::PredictedState buildPredictionFromCTRVState(const CTRV_State& state, const geometry_msgs::msg::Pose& original_pose,
                                                      const geometry_msgs::msg::Twist& original_twist)
{
  carma_perception_msgs::msg::PredictedState pobj;

  // Map position
  pobj.predicted_position.position.x = state.x;
  pobj.predicted_position.position.y = state.y;
  pobj.predicted_position.position.z = original_pose.position.z;

  // Map orientation
  Eigen::Quaternionf original_quat(original_pose.orientation.w, original_pose.orientation.x,
                                   original_pose.orientation.y, original_pose.orientation.z);
  Eigen::Vector3f original_rpy = original_quat.toRotationMatrix().eulerAngles(0, 1, 2);

  auto vel_angle_and_mag = localVelOrientationAndMagnitude(original_twist.linear.x, original_twist.linear.y);

  Eigen::Quaternionf final_quat;
  final_quat = Eigen::AngleAxisf(original_rpy[0], Eigen::Vector3f::UnitX()) *
               Eigen::AngleAxisf(original_rpy[1], Eigen::Vector3f::UnitY()) *
               Eigen::AngleAxisf(state.yaw - std::get<0>(vel_angle_and_mag), Eigen::Vector3f::UnitZ());

  pobj.predicted_position.orientation.x = final_quat.x();
  pobj.predicted_position.orientation.y = final_quat.y();
  pobj.predicted_position.orientation.z = final_quat.z();
  pobj.predicted_position.orientation.w = final_quat.w();

  // Map twist
  // Constant velocity model means twist remains unchanged
  pobj.predicted_velocity = original_twist;

  return pobj;
}

CTRV_State CTRVPredict(const CTRV_State& state, const double delta_t)
{
  CTRV_State next_state;

  // Handle divide by 0 case
  if (fabs(state.yaw_rate) < 0.0000001)
  {
    next_state.x = state.x + state.v * cos(state.yaw) * delta_t;
    next_state.y = state.y + state.v * sin(state.yaw) * delta_t;
    next_state.yaw = state.yaw;
    next_state.v = state.v;
    next_state.yaw_rate = state.yaw_rate;

    return next_state;
  }

  double v_w = state.v / state.yaw_rate;
  double sin_yaw = sin(state.yaw);
  double cos_yaw = cos(state.yaw);
  double wT = state.yaw_rate * delta_t;

  next_state.x = state.x + v_w * (sin(state.yaw + wT) - sin_yaw);
  next_state.y = state.y + v_w * (cos_yaw - cos(state.yaw + wT));
  next_state.yaw = state.yaw + wT;
  next_state.v = state.v;
  next_state.yaw_rate = state.yaw_rate;

  return next_state;
}

// Forward predict an external object
carma_perception_msgs::msg::PredictedState predictStep(const carma_perception_msgs::msg::ExternalObject& obj, const double delta_t,
                                     const float process_noise_max, const double confidence_drop_rate)
{
  // Get initial state
  CTRV_State state = buildCTRVState(obj.pose.pose, obj.velocity.twist);

  // Predict Motion
  CTRV_State next_state = CTRVPredict(state, delta_t);

  // Convert CTRV to predicted state object
  carma_perception_msgs::msg::PredictedState pobj = buildPredictionFromCTRVState(next_state, obj.pose.pose, obj.velocity.twist);

  // Compute confidence values

  double x_x = obj.pose.covariance[0];                   // X
  double y_y = obj.pose.covariance[7];                   // Y
  double yaw_yaw = obj.pose.covariance[35];              // Yaw
  double vx_vx = obj.velocity.covariance[0];             // Vx
  double vy_vy = obj.velocity.covariance[7];             // Vy
  double yawrate_yawrate = obj.velocity.covariance[35];  // Yaw rate

  // Average diagonal of process noise
  double position_process_noise_avg = (x_x + y_y + yaw_yaw) / 3;
  double velocity_process_noise_avg = (vx_vx + vy_vy + yawrate_yawrate) / 3;

  // Map process noise average to confidence
  pobj.predicted_position_confidence = motion_predict::cv::Mapping(position_process_noise_avg, process_noise_max) * confidence_drop_rate;
  pobj.predicted_velocity_confidence = motion_predict::cv::Mapping(velocity_process_noise_avg, process_noise_max) * confidence_drop_rate;

  // Update header
  pobj.header = obj.header;
  rclcpp::Time updated_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(delta_t * 1e9)));
  pobj.header.stamp = builtin_interfaces::msg::Time(updated_time);

  return pobj;
}

// Forward predict a prediction
carma_perception_msgs::msg::PredictedState predictStep(const carma_perception_msgs::msg::PredictedState& obj, const double delta_t,
                                     const double confidence_drop_rate)
{
  // Get initial state
  CTRV_State state = buildCTRVState(obj.predicted_position, obj.predicted_velocity);

  // Predict Motion
  CTRV_State next_state = CTRVPredict(state, delta_t);

  // Convert CTRV to predicted state object
  carma_perception_msgs::msg::PredictedState pobj =
      buildPredictionFromCTRVState(next_state, obj.predicted_position, obj.predicted_velocity);

  // Map process noise average to confidence
  pobj.predicted_position_confidence = obj.predicted_position_confidence * confidence_drop_rate;
  pobj.predicted_velocity_confidence = obj.predicted_velocity_confidence * confidence_drop_rate;

  // Update header
  pobj.header = obj.header;
  rclcpp::Time updated_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(delta_t * 1e9)));
  pobj.header.stamp = builtin_interfaces::msg::Time(updated_time);

  return pobj;
}

std::vector<carma_perception_msgs::msg::PredictedState> predictPeriod(const carma_perception_msgs::msg::ExternalObject& obj, const double delta_t,
                                                    const double period, const float process_noise_max,
                                                    const double confidence_drop_rate)
{
  std::vector<carma_perception_msgs::msg::PredictedState> predicted_states = { predictStep(obj, delta_t, process_noise_max,
                                                                         confidence_drop_rate) };

  double t = delta_t;
  while (t < period)
  {
    predicted_states.emplace_back(predictStep(predicted_states.back(), delta_t, confidence_drop_rate));
    t += delta_t;
  }

  return predicted_states;
}

}  // namespace ctrv

}  // namespace motion_predict
