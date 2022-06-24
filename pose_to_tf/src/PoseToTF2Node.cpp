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

#include "pose_to_tf/PoseToTF2Node.hpp"
#include <tf2_ros/transform_broadcaster.h>

namespace pose_to_tf
{
  namespace std_ph = std::placeholders;

  PoseToTfNode::PoseToTfNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = PoseToTF2Config();
 
    // Declare parameters
    config_.child_frame = declare_parameter<std::string>("message_type", config_.child_frame);
    config_.default_parent_frame = declare_parameter<std::string>("target_frame", config_.default_parent_frame);
  }

  carma_ros2_utils::CallbackReturn PoseToTfNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    tf2_ros::TransformBroadcaster tf_broadcaster(shared_from_this());
    pose_to_tf::PoseToTF2 worker(config_, [&tf_broadcaster](auto msg){tf_broadcaster.sendTransform(msg);}, shared_from_this());
    pose_to_tf_worker_= std::make_shared<PoseToTF2>(worker);

    // Setup subscribers
    pose_sub = create_subscription<geometry_msgs::msg::Pose>("pose_to_tf", 50,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseCallback, pose_to_tf_worker_.get(), std_ph::_1));
    pose_stamped_sub = create_subscription<geometry_msgs::msg::PoseStamped>("pose_stamped_to_tf", 50,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseStampedCallback, pose_to_tf_worker_.get(), std_ph::_1));
    pose_with_cov_sub = create_subscription<geometry_msgs::msg::PoseWithCovariance>("pose_with_cov_to_tf", 50,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseWithCovarianceCallback, pose_to_tf_worker_.get(), std_ph::_1));
    pose_with_cov_stamped_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_with_cov_stamped_to_tf", 50,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseWithCovarianceStampedCallback, pose_to_tf_worker_.get(), std_ph::_1));
  
    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

} // pose_to_tf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(pose_to_tf::PoseToTfNode)
