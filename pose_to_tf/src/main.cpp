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

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pose_to_tf/PoseToTF2.hpp>
#include <pose_to_tf/PoseToTF2Config.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pose_to_tf");
 
  pose_to_tf::PoseToTF2Config config;
  // Create default config
  //config_ = Config();
  //Declare parameters and defaults
  config.child_frame = declare_parameter<std::string>("message_type", config.child_frame);
  config.default_parent_frame = declare_parameter<std::string>("target_frame", config.default_parent_frame);

  //pnh.param<std::string>("child_frame", config.child_frame, config.child_frame);
  //pnh.param<std::string>("default_parent_frame", config.default_parent_frame, config.default_parent_frame);

  tf2_ros::TransformBroadcaster tf_broadcaster;
  pose_to_tf::PoseToTF2 worker(config, [&tf_broadcaster](auto msg){tf_broadcaster.sendTransform(msg);}, shared_from_this());

  carma_ros2_utils::SubPtr<geometry_msgs::msg::Pose> pose_sub;
  carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> pose_stamped_sub;
  carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovariance> pose_with_cov_sub;
  carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_with_cov_stamped_sub;

  pose_sub = create_subscription<geometry_msgs::msg::Pose>("pose_to_tf", 1,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseCallback, &worker, std_ph::_1));
  pose_stamped_sub = create_subscription<geometry_msgs::msg::PoseStamped>("pose_stamped_to_tf", 1,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseStampedCallback, &worker, std_ph::_1));
  pose_with_cov_sub = create_subscription<geometry_msgs::msg::PoseWithCovariance>("pose_with_cov_to_tf", 1,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseWithCovarianceCallback, &worker, std_ph::_1));
  pose_with_cov_stamped_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_with_cov_stamped_to_tf", 1,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseWithCovarianceStampedCallback, &worker, std_ph::_1));
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
};
