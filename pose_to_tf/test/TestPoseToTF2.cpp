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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <pose_to_tf/PoseToTF2.hpp>
#include <pose_to_tf/PoseToTF2Config.hpp>
#include <pose_to_tf/PoseToTF2Node.hpp>

using namespace pose_to_tf;

class PoseToTfNodeTest : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:

    rclcpp::Time current_time_;
  
  public:
  
  explicit PoseToTfNodeTest(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {

  }

  void setNow(rclcpp::Time c_time_)
  {
     current_time_=c_time_;
  }

  rclcpp::Time now()
  {
    return current_time_;
  }

  };

TEST(PoseToTF2, test_methods)
{
  PoseToTF2Config config;
  config.child_frame = "base_link";
  config.default_parent_frame = "map";
  bool msg_set = false;
  geometry_msgs::msg::TransformStamped msg;
  auto node = std::make_shared<PoseToTfNodeTest>(rclcpp::NodeOptions());
  
  PoseToTF2 manager(config, [&msg, &msg_set](auto in) { msg_set = true; msg = in; },node);
  
  rclcpp::Time now(1.5*1e9);
  node->setNow(now);

  geometry_msgs::msg::Pose poseA;
  poseA.position.x = 1.2;
  poseA.position.y = 2.3;
  poseA.position.z = 5.0;
  poseA.orientation.x = 3.0;
  poseA.orientation.y = 3.4;
  poseA.orientation.z = 9.9;
  poseA.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped poseB;

  poseB.pose = poseA;
  poseB.header.stamp = rclcpp::Time(0.5*1e9);
  poseB.header.frame_id = "odom";

  geometry_msgs::msg::PoseWithCovariance poseC;

  poseC.pose = poseA;

  geometry_msgs::msg::PoseWithCovarianceStamped poseD;

  poseD.pose.pose = poseA;
  poseD.header.stamp = rclcpp::Time(0.5*1e9);
  poseD.header.frame_id = "odom";


  ASSERT_FALSE(msg_set);
  auto poseA_ptr = std::make_unique<geometry_msgs::msg::Pose>(poseA);
  manager.poseCallback(std::move(poseA_ptr));

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseA.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseA.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseA.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseA.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseA.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseA.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseA.orientation.w, msg.transform.rotation.w);

  ASSERT_EQ(0, msg.header.frame_id.compare(config.default_parent_frame));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(1.5, rclcpp::Time(msg.header.stamp).seconds(), 0.000000001);

  msg_set = false;
  auto poseB_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>(poseB);
  manager.poseStampedCallback(std::move(poseB_ptr));

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseB.pose.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseB.pose.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseB.pose.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseB.pose.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseB.pose.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseB.pose.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseB.pose.orientation.w, msg.transform.rotation.w);
  
  ASSERT_EQ(0, msg.header.frame_id.compare(poseB.header.frame_id));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(rclcpp::Time(poseB.header.stamp).seconds(), rclcpp::Time(msg.header.stamp).seconds(), 0.000000001);

  msg_set = false;
  auto poseC_ptr = std::make_unique<geometry_msgs::msg::PoseWithCovariance>(poseC);
  manager.poseWithCovarianceCallback(std::move(poseC_ptr));

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseC.pose.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseC.pose.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseC.pose.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseC.pose.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseC.pose.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseC.pose.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseC.pose.orientation.w, msg.transform.rotation.w);
  
  ASSERT_EQ(0, msg.header.frame_id.compare(config.default_parent_frame));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(1.5, rclcpp::Time(msg.header.stamp).seconds(), 0.000000001);

  msg_set = false;
  auto poseD_ptr = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>(poseD);
  manager.poseWithCovarianceStampedCallback(std::move(poseD_ptr));

  ASSERT_TRUE(msg_set);
  ASSERT_EQ(poseD.pose.pose.position.x, msg.transform.translation.x);
  ASSERT_EQ(poseD.pose.pose.position.y, msg.transform.translation.y);
  ASSERT_EQ(poseD.pose.pose.position.z, msg.transform.translation.z);
  ASSERT_EQ(poseD.pose.pose.orientation.x, msg.transform.rotation.x);
  ASSERT_EQ(poseD.pose.pose.orientation.y, msg.transform.rotation.y);
  ASSERT_EQ(poseD.pose.pose.orientation.z, msg.transform.rotation.z);
  ASSERT_EQ(poseD.pose.pose.orientation.w, msg.transform.rotation.w);
  
  ASSERT_EQ(0, msg.header.frame_id.compare(poseD.header.frame_id));
  ASSERT_EQ(0, msg.child_frame_id.compare(config.child_frame));
  ASSERT_NEAR(rclcpp::Time(poseD.header.stamp).seconds(), rclcpp::Time(msg.header.stamp).seconds(), 0.000000001);
}
