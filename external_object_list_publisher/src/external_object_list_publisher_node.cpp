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
#include "external_object_list_publisher_node.h"
#include "external_object_list_publisher_config.h"
#include <carma_ros2_utils/timers/TimerFactory.hpp>
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>

namespace external_object_list_publisher
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      :  carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = ExternalObjectListPublisherConfig();

    declare_parameter<double>("emergency_vehicle_distance", config_.emergency_vehicle_distance);

  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {

    auto error = update_params<double>({
      {"emergency_vehicle_distance", config_.emergency_vehicle_distance}
    }, parameters);
    
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::on_configure_plugin()
  {
    // Reset config
    config_ = ExternalObjectListPublisherConfig();

    // Load parameters
    get_parameter<double>("emergency_vehicle_distance", config_.emergency_vehicle_distance);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    // publishers
    external_object_list_pub = create_publisher<carma_perception_msgs::msg::ExternalObjectList>("external_object_list", 10);

    // subscribe to current pose
    current_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 10,
                                           std::bind(&ExternalObjectListPublisher::pose_cb, worker_.get(), std_ph::_1));
    // subscribe to current twist command
    current_twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 10,
                                            std::bind(&ExternalObjectListPublisher::twist_cb, worker_.get(), std_ph::_1));
    // subscribe to current geo 
    georeference_sub = create_subscription<std_msgs::msg::String>("georeference", 10,
                                           std::bind(&ExternalObjectListPublisher::georeference_cb, worker_.get(), std_ph::_1));
    // manual emergency detection result subscriber 
    emergency_detection_sub = create_subscription<std_msgs::msg::Bool>("emergency_detection_manual_result", 10,
                                                  std::bind(&ExternalObjectListPublisher::emergncy_detection_cb, worker_.get(), std_ph::_1));
    
    // loop timer
    loop_timer_ = create_timer(get_clock(),
                               std::chrono::milliseconds(100), // 10 Hz frequency
                               std::bind(&ExternalObjectListPublisher::onSpin, worker_.get()));

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn Node::on_cleanup_plugin()
  {
    // Ensure subscribers are disconnected incase cleanup is called, we don't want to keep driving the worker
    current_pose_sub.reset();
    current_twist_sub.reset();
    georeference_sub.reset();
    emergency_detection_sub.reset();
    worker_.reset();

    return CallbackReturn::SUCCESS;
  }


} // external_object_list_publisher

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(external_object_list_publisher::Node)
