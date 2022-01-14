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
#include "object_visualizer/object_visualizer_node.hpp"

namespace object_visualizer
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.enable_external_objects_viz = declare_parameter<bool>("enable_external_objects_viz", config_.enable_external_objects_viz);
    config_.enable_roadway_objects_viz = declare_parameter<bool>("enable_roadway_objects_viz", config_.enable_roadway_objects_viz);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    // Update parameters
    auto error = update_params<bool>(
      {
        {"enable_external_objects_viz", config_.enable_external_objects_viz},
        {"enable_roadway_objects_viz", config_.enable_roadway_objects_viz}
      }, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<bool>("enable_external_objects_viz", config_.enable_external_objects_viz);
    get_parameter<bool>("enable_roadway_objects_viz", config_.enable_roadway_objects_viz);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    external_objects_sub_ = create_subscription<carma_perception_msgs::msg::ExternalObjectList>("external_objects", 10,
                                                              std::bind(&Node::external_objects_callback, this, std_ph::_1));

    roadway_obstacles_sub_ = create_subscription<carma_perception_msgs::msg::RoadwayObstacleList>("roadway_obstacles", 10,
                                                              std::bind(&Node::roadway_obstacles_callback, this, std_ph::_1));

    // Setup publishers
    external_objects_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("external_objects_viz", 10);
    roadway_obstacles_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("roadway_obstacles_viz", 10);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void Node::external_objects_callback(std_msgs::msg::String::UniquePtr msg)
  {
    RCLCPP_DEBUG_STREAM(  get_logger(), "external_objects_callback called");

    if (!config_.enable_external_objects_viz) {
      RCLCPP_DEBUG_STREAM(  get_logger(), "external_objects_callback called, but visualization is not enabled.");
      return;
    }

    // TODO add logic for visualization

  }

  void Node::roadway_obstacles_callback(std_msgs::msg::String::UniquePtr msg)
  {
    RCLCPP_DEBUG_STREAM(  get_logger(), "roadway_obstacles_callback called");

    if (!config_.enable_roadway_objects_viz) {
      RCLCPP_DEBUG_STREAM(  get_logger(), "roadway_obstacles_callback called, but visualization is not enabled.");
      return;
    }

    // TODO add logic for visualization

  }

} // object_visualizer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(object_visualizer::Node)
