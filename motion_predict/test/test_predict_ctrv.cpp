/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "../src/predict_ctrv.cpp"
#include <gtest/gtest.h>

namespace motion_predict
{
namespace ctrv
{
TEST(predict_ctrv, buildCTRVState)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.3;
  pose.position.y = 1.4;
  pose.position.z = 2.5;

  // 90 Deg rotation about z and 0.1 deg rotation around x,y for noisy input
  pose.orientation.x = 0.0012341;
  pose.orientation.y = 0;
  pose.orientation.z = 0.7071068;
  pose.orientation.w = 0.7071057;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 4.5;
  twist.linear.y = 2;
  twist.linear.z = 5;

  // 5 Deg per sec rotation about z and 0.1 deg/s rotation around x,y for noisy input
  twist.angular.x = 0.00174533;
  twist.angular.y = 0.00174533;
  twist.angular.z = 0.0872665;

  CTRV_State result = buildCTRVState(pose, twist);
  ASSERT_NEAR(result.x, 1.3, 0.000001);
  ASSERT_NEAR(result.y, 1.4, 0.000001);
  ASSERT_NEAR(result.yaw, 1.98902433, 0.00001);
  ASSERT_NEAR(result.v, 4.9244289009, 0.000001);
  ASSERT_NEAR(result.yaw_rate, 0.0872665, 0.0000001);
}

TEST(predict_ctrv, buildPredictionFromCTRVState)
{
  CTRV_State state;
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.98902433;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0872665;

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.3;
  pose.position.y = 1.4;
  pose.position.z = 2.5;

  // 90 Deg rotation about z and 0.1 deg rotation around x,y for noisy input
  pose.orientation.x = 0.0012341;
  pose.orientation.y = 0;
  pose.orientation.z = 0.7071068;
  pose.orientation.w = 0.7071057;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 4.5;
  twist.linear.y = 2;
  twist.linear.z = 5;

  // 5 Deg per sec rotation about z and 0.1 deg/s rotation around x,y for noisy input
  twist.angular.x = 0.00174533;
  twist.angular.y = 0.00174533;
  twist.angular.z = 0.0872665;

  carma_perception_msgs::msg::PredictedState result = buildPredictionFromCTRVState(state, pose, twist);
  ASSERT_NEAR(result.predicted_position.position.x, pose.position.x, 0.00001);
  ASSERT_NEAR(result.predicted_position.position.y, pose.position.y, 0.00001);
  ASSERT_NEAR(result.predicted_position.position.z, pose.position.z, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.x, pose.orientation.x, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.y, pose.orientation.y, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.z, pose.orientation.z, 0.00001);
  ASSERT_NEAR(result.predicted_position.orientation.w, pose.orientation.w, 0.00001);

  ASSERT_NEAR(result.predicted_velocity.linear.x, twist.linear.x, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.linear.y, twist.linear.y, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.linear.z, twist.linear.z, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.x, twist.angular.x, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.y, twist.angular.y, 0.00001);
  ASSERT_NEAR(result.predicted_velocity.angular.z, twist.angular.z, 0.00001);
}

TEST(predict_ctrv, CTRVPredict) {
  // Regular prediction
  CTRV_State state;
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.5708;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0872665;

  CTRV_State result = CTRVPredict(state, 0.1);

  ASSERT_NEAR(result.x, 1.29785, 0.00001);
  ASSERT_NEAR(result.y, 1.89244, 0.0001);
  ASSERT_NEAR(result.yaw, 1.57953, 0.00001);
  ASSERT_NEAR(result.v, state.v, 0.00001);
  ASSERT_NEAR(result.yaw_rate, state.yaw_rate, 0.00001);

  // Divide by 0 case
  state.x = 1.3;
  state.y = 1.4;
  state.yaw = 1.5708;
  state.v = 4.9244289009;
  state.yaw_rate = 0.0;

  result = CTRVPredict(state, 0.1);

  ASSERT_NEAR(result.x, 1.3, 0.0001);
  ASSERT_NEAR(result.y, 1.89244, 0.0001);
  ASSERT_NEAR(result.yaw, 1.5708, 0.00001);
  ASSERT_NEAR(result.v, state.v, 0.00001);
  ASSERT_NEAR(result.yaw_rate, state.yaw_rate, 0.00001);
}

TEST(predict_ctrv, predictStepExternal)
{
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.header.frame_id = "my_frame";
  obj.pose.pose.position.x = 5.0;
  obj.pose.pose.position.y = 1.4; // Added y position
  // Set the orientation quaternion for 0.79 radians yaw
  double yaw_angle = 0.79;
  obj.pose.pose.orientation.w = cos(yaw_angle / 2);
  obj.pose.pose.orientation.x = 0.0;
  obj.pose.pose.orientation.y = 0.0;
  obj.pose.pose.orientation.z = sin(yaw_angle / 2);
  obj.pose.covariance[0] = 1;
  obj.pose.covariance[7] = 1;
  obj.pose.covariance[35] = 1;
  obj.velocity.twist.linear.x = 4.9244289009; // Initial velocity speed
  obj.velocity.covariance[0] = 999;
  obj.velocity.covariance[7] = 999;
  obj.velocity.covariance[35] = 999;

  carma_perception_msgs::msg::PredictedState result = motion_predict::ctrv::predictStep(obj, 0.1, 1000, 0.99);

  EXPECT_NEAR(5.3466, result.predicted_position.position.x, 0.00001);  // Verify x position update
  EXPECT_NEAR(1.7498, result.predicted_position.position.y, 0.00001);  // Verify y position update
  EXPECT_NEAR(4.9244289009, result.predicted_velocity.linear.x, 0.00001); // Verify velocity speed
  EXPECT_NEAR(0.99, result.predicted_position_confidence, 0.01);
  EXPECT_NEAR(0.001, result.predicted_velocity_confidence, 0.001);

  rclcpp::Time new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.1 * 1e9))); // Increase by 0.1 sec
  int32_t new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  uint32_t new_time_nanosec = new_time.nanoseconds() - (new_time_sec * 1e9);
  EXPECT_EQ(result.header.stamp.sec, new_time_sec);
  EXPECT_EQ(result.header.stamp.nanosec, new_time_nanosec);
  EXPECT_EQ(result.header.frame_id, obj.header.frame_id);
}

TEST(predict_ctrv, predictStep)
{
  carma_perception_msgs::msg::PredictedState obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.header.frame_id = "my_frame";
  obj.predicted_position.position.x = 5.0;
  obj.predicted_position.orientation.w = 1.0;
  obj.predicted_position_confidence = 1;
  obj.predicted_velocity_confidence = 0.5;

  carma_perception_msgs::msg::PredictedState result = motion_predict::ctrv::predictStep(obj, 0.1, 0.99);

  ASSERT_NEAR(5.0, result.predicted_position.position.x, 0.00001);  // Verify update functions were called
  ASSERT_NEAR(0.99, result.predicted_position_confidence, 0.01);
  ASSERT_NEAR(0.495, result.predicted_velocity_confidence, 0.0001);

  rclcpp::Time new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.1 * 1e9))); // Increase by 0.1 sec
  int32_t new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  uint32_t new_time_nanosec = new_time.nanoseconds() - (new_time_sec*1e9);
  ASSERT_EQ(result.header.stamp.sec, new_time_sec);
  ASSERT_EQ(result.header.stamp.nanosec, new_time_nanosec);
  ASSERT_EQ(result.header.frame_id, obj.header.frame_id);
}

TEST(predict_ctrv, predictPeriod)
{
  carma_perception_msgs::msg::ExternalObject obj;
  obj.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(5, 0));
  obj.header.frame_id = "my_frame";
  obj.pose.pose.position.x = 5.0;
  obj.pose.pose.orientation.w = 1.0;
  obj.velocity.twist.linear.x = 1.0;
  obj.pose.covariance[0] = 1;
  obj.pose.covariance[7] = 1;
  obj.pose.covariance[35] = 1;
  obj.velocity.covariance[0] = 999;
  obj.velocity.covariance[7] = 999;
  obj.velocity.covariance[35] = 999;

  std::vector<carma_perception_msgs::msg::PredictedState> results = motion_predict::ctrv::predictPeriod(obj, 0.1, 0.21, 1000, 0.99);

  ASSERT_NEAR(5.1, results[0].predicted_position.position.x, 0.00001);  // Verify update functions were called
  ASSERT_NEAR(0.99, results[0].predicted_position_confidence, 0.01);
  ASSERT_NEAR(0.001, results[0].predicted_velocity_confidence, 0.001);

  rclcpp::Time new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.1 * 1e9))); // Increase by 0.1 sec
  int32_t new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  uint32_t new_time_nanosec = new_time.nanoseconds() - (new_time_sec*1e9);
  ASSERT_EQ(results[0].header.stamp.sec, new_time_sec);
  ASSERT_EQ(results[0].header.stamp.nanosec, new_time_nanosec);
  ASSERT_EQ(results[0].header.frame_id, obj.header.frame_id);

  ASSERT_NEAR(5.2, results[1].predicted_position.position.x, 0.00001);  // Verify update functions were called
  ASSERT_NEAR(0.9801, results[1].predicted_position_confidence, 0.01);
  ASSERT_NEAR(0.00099, results[1].predicted_velocity_confidence, 0.00001);

  new_time = rclcpp::Time(obj.header.stamp) + rclcpp::Duration(std::chrono::nanoseconds(int32_t(0.2 * 1e9))); // Increase by 0.2 sec
  new_time_sec = int32_t(new_time.nanoseconds() / 1e9);
  new_time_nanosec = new_time.nanoseconds() - (new_time_sec*1e9);
  ASSERT_EQ(results[1].header.stamp.sec, new_time_sec);
  ASSERT_EQ(results[1].header.stamp.nanosec, new_time_nanosec);
  ASSERT_EQ(results[1].header.frame_id, obj.header.frame_id);
}

}  // namespace ctrv
}  // namespace motion_predict
