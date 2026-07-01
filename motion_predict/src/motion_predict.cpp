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

#include "motion_predict/motion_predict.hpp"

namespace motion_predict{

namespace cv{

double Mapping(const double input,const double process_noise_max)
{
  // Note as the value of the covariance increases confidence value increase.

  double input_start = 1; // The lowest number of the range input.
  double input_end = process_noise_max; // The largest number of the range input.
  double output_start = 1; // The lowest number of the range output.
  double output_end = 0; // The largest number of the range ouput.
  double output = (input - input_start) / (input_end - input_start) * (output_end - output_start) + output_start;

  return output;
}

carma_perception_msgs::msg::PredictedState predictState(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Twist& twist,const double delta_t)
{
  Eigen::VectorXd x(4); // State Vector

  x(0)=pose.position.x; // Position X
  x(1)=pose.position.y; // Position Y

  // TODO: Need a logic here to possible detect whether if twist.linear.x,y
  // is in map frame. https://github.com/usdot-fhwa-stol/carma-platform/issues/2407
  x(2)=twist.linear.x; // Linear Velocity X
  x(3)=twist.linear.y; // Linear Velocity Y

  Eigen::MatrixXd F=Eigen::MatrixXd::Identity(x.size(), x.size()); // Generate identity matrix for state transition matrix

  F(0,2)=delta_t;
  F(1,3)=delta_t;

  x = F * x; // Predict

  carma_perception_msgs::msg::PredictedState pobj;

  pobj.predicted_position.position.x=x(0); // Predicted Position X
  pobj.predicted_position.position.y=x(1); // Predicted Position Y
  pobj.predicted_position.position.z=pose.position.z; // Predicted Position Z

  pobj.predicted_position.orientation.x=pose.orientation.x;
  pobj.predicted_position.orientation.y=pose.orientation.y;
  pobj.predicted_position.orientation.z=pose.orientation.z;
  pobj.predicted_position.orientation.w=pose.orientation.w;

  pobj.predicted_velocity.linear.x=x(2); // Predicted Linear Velocity X
  pobj.predicted_velocity.linear.y=x(3); // Predicted Linear Velocity Y
  pobj.predicted_velocity.linear.z=twist.linear.z; // Predicted Linear Velocity Z

  pobj.predicted_velocity.angular.x=twist.angular.x;
  pobj.predicted_velocity.angular.y=twist.angular.y;
  pobj.predicted_velocity.angular.z=twist.angular.z;

  return pobj;
}


// Forward predict an external object
carma_perception_msgs::msg::PredictedState externalPredict(const carma_perception_msgs::msg::ExternalObject &obj,const double delta_t,const double ax,const double ay,const double process_noise_max)
{

  carma_perception_msgs::msg::PredictedState pobj = predictState(obj.pose.pose, obj.velocity.twist,delta_t);

  Eigen::MatrixXd F=Eigen::MatrixXd::Identity(4,4); // Generate identity matrix for state transition matrix

  F(0,2)=delta_t;
  F(1,3)=delta_t;

  Eigen::MatrixXd P(4,4); // State Covariance Matrix
  P.fill(0.0);  // Filling whole matrix with zero

  // Covariance extraction
  P(0,0)=obj.pose.covariance[0]; // X
  P(1,1)=obj.pose.covariance[7]; // Y
  P(2,2)=obj.velocity.covariance[0]; // Vx
  P(3,3)=obj.velocity.covariance[7]; // Vy

  Eigen::MatrixXd Q(4,4); // Process Noise Matrix

  double delta_t2 = delta_t * delta_t; //t^2
  double delta_t3 = delta_t2 * delta_t;//t^3
  double delta_t4 = delta_t3 * delta_t;//t^4

  Q << (delta_t4 / 4 * ax), 0,( delta_t3 / 2 * ax), 0, 0, (delta_t4 / 4 * ay), 0,( delta_t3 / 2 * ay), (delta_t3 / 2 * ax), 0,(delta_t2*ax), 0 , 0 , (delta_t3 / 2 * ay), 0 , (delta_t2*ay); // Process Noise Matrix

  Eigen::MatrixXd Ft = F.transpose(); // Transpose of State Transition Function

  P = F * P * Ft + Q; //State Covariance Matrix

  double position_process_noise_avg=(P(0,0)+P(1,1))/2; // Position process noise average

  pobj.predicted_position_confidence=Mapping(position_process_noise_avg,process_noise_max); // Position process noise confidence

  double velocity_process_noise_avg=(P(2,2)+P(3,3))/2; // Position velocity process noise average

  pobj.predicted_velocity_confidence=Mapping(velocity_process_noise_avg,process_noise_max); // Velocity process noise confidence

  // Update header
  pobj.header = obj.header;
  rclcpp::Time updated_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int64_t(delta_t * SEC_TO_NANOSEC)));
  pobj.header.stamp = builtin_interfaces::msg::Time(updated_time);

  return pobj;

}

// Forward predict a prediction
carma_perception_msgs::msg::PredictedState predictStep(const carma_perception_msgs::msg::PredictedState& obj, const double delta_t, const double confidence_drop_rate)
{
  // Predict Motion
  carma_perception_msgs::msg::PredictedState pobj = predictState(obj.predicted_position, obj.predicted_velocity,delta_t);

  // Map process noise average to confidence
  pobj.predicted_position_confidence = obj.predicted_position_confidence * confidence_drop_rate;
  pobj.predicted_velocity_confidence = obj.predicted_velocity_confidence * confidence_drop_rate;

  // Update header
  pobj.header = obj.header;
  rclcpp::Time updated_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int64_t(delta_t * SEC_TO_NANOSEC)));
  pobj.header.stamp = builtin_interfaces::msg::Time(updated_time);

  return pobj;
}

std::vector<carma_perception_msgs::msg::PredictedState> predictPeriod(const carma_perception_msgs::msg::ExternalObject& obj, const double delta_t, const double period,const double ax,const double ay ,const double process_noise_max,const double confidence_drop_rate)
{
  std::vector<carma_perception_msgs::msg::PredictedState> predicted_states = { externalPredict(obj,delta_t,ax,ay,process_noise_max) };

  double t = delta_t;
  while (t < period)
  {
    predicted_states.emplace_back(predictStep(predicted_states.back(), delta_t, confidence_drop_rate));
    t += delta_t;
  }

  return predicted_states;
}

}//cv

}//motion_predict
