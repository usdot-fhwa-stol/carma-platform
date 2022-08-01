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
    config_.external_objects_viz_ns = declare_parameter<std::string>("external_objects_viz_ns", config_.external_objects_viz_ns);
    config_.roadway_obstacles_viz_ns = declare_parameter<std::string>("roadway_obstacles_viz_ns", config_.roadway_obstacles_viz_ns);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    // Update parameters
    auto error = update_params<bool>(
      {
        {"enable_external_objects_viz", config_.enable_external_objects_viz},
        {"enable_roadway_objects_viz", config_.enable_roadway_objects_viz}
      }, parameters);

    auto error2 = update_params<std::string>(
      {
        {"external_objects_viz_ns", config_.external_objects_viz_ns},
        {"roadway_obstacles_viz_ns", config_.roadway_obstacles_viz_ns}
      }, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error2;

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<bool>("enable_external_objects_viz", config_.enable_external_objects_viz);
    get_parameter<bool>("enable_roadway_objects_viz", config_.enable_roadway_objects_viz);
    get_parameter<std::string>("external_objects_viz_ns", config_.external_objects_viz_ns);
    get_parameter<std::string>("roadway_obstacles_viz_ns", config_.roadway_obstacles_viz_ns);

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

  void Node::external_objects_callback(carma_perception_msgs::msg::ExternalObjectList::UniquePtr msg)
  {
    RCLCPP_DEBUG_STREAM(  get_logger(), "external_objects_callback called");

    if (!config_.enable_external_objects_viz) {
      RCLCPP_DEBUG_STREAM(  get_logger(), "external_objects_callback called, but visualization is not enabled.");
      return;
    }

    visualization_msgs::msg::MarkerArray viz_msg;
    //delete all markers before adding new ones
    visualization_msgs::msg::Marker marker;
    viz_msg.markers.reserve(msg->objects.size() + 1); //+1 to account for delete all marker
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    viz_msg.markers.push_back(marker);

    
    size_t id = 0; // We always count the id from zero so we can delete markers later in a consistent manner

    for (auto obj : msg->objects) {
      visualization_msgs::msg::Marker marker;
      
      marker.header = msg->header;

      marker.ns = config_.external_objects_viz_ns;

      // Pedestrian's will be represented as red box
      if (obj.object_type == carma_perception_msgs::msg::ExternalObject::PEDESTRIAN) {

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

      } // All other detected objects will be blue 
      else {

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
      }

      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = obj.pose.pose;
      marker.scale.x = obj.size.x * 2.0; // Size in carma is half the length/width/height 
      marker.scale.y = obj.size.y * 2.0;
      marker.scale.z = obj.size.z * 2.0;

      viz_msg.markers.push_back(marker);

      id++;
    }

    clear_and_update_old_objects(viz_msg, prev_external_objects_size_);

    external_objects_viz_pub_->publish(viz_msg);

  }

  void Node::clear_and_update_old_objects(visualization_msgs::msg::MarkerArray &viz_msg, size_t &old_size)
  {
    size_t size_swap = viz_msg.markers.size();

    // Delete any markers from the previous message that are no longer in the current message
    if (viz_msg.markers.size() < old_size) {

      for (size_t i = viz_msg.markers.size(); i < old_size; ++i) {
        visualization_msgs::msg::Marker marker;
        marker.id = i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        viz_msg.markers.push_back(marker);
      }

    }

    old_size = size_swap; // Assign the new size of added objects to the old size
  }

  void Node::roadway_obstacles_callback(carma_perception_msgs::msg::RoadwayObstacleList::UniquePtr msg)
  {
    RCLCPP_DEBUG_STREAM(  get_logger(), "roadway_obstacles_callback called");

    if (!config_.enable_roadway_objects_viz) {
      RCLCPP_DEBUG_STREAM(  get_logger(), "roadway_obstacles_callback called, but visualization is not enabled.");
      return;
    }

    visualization_msgs::msg::MarkerArray viz_msg;
    //delete all markers before adding new ones
    visualization_msgs::msg::Marker marker;
    viz_msg.markers.reserve(msg->roadway_obstacles.size() + 1); //+1 to account for delete all marker
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    viz_msg.markers.push_back(marker);

    size_t id = 0; // We always count the id from zero so we can delete markers later in a consistent manner
    for (auto obj : msg->roadway_obstacles) {
      visualization_msgs::msg::Marker marker;
      
      marker.header = obj.object.header;

      marker.ns = config_.roadway_obstacles_viz_ns;

      // Pedestrian's will be represented as a red box
      if (obj.object.object_type == carma_perception_msgs::msg::ExternalObject::PEDESTRIAN) {

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

      } // Connected vehicles will be green
      else if (obj.connected_vehicle_type.type != carma_perception_msgs::msg::ConnectedVehicleType::NOT_CONNECTED){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      } // All other detected objects will be blue
      else {

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
      }

      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = obj.object.pose.pose;
      marker.scale.x = obj.object.size.x * 2.0; // Size in carma is half the length/width/height 
      marker.scale.y = obj.object.size.y * 2.0;
      marker.scale.z = obj.object.size.z * 2.0;

      viz_msg.markers.push_back(marker);

      id++;
    }

    clear_and_update_old_objects(viz_msg, prev_roadway_obstacles_size_);

    roadway_obstacles_viz_pub_->publish(viz_msg);

  }

} // object_visualizer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(object_visualizer::Node)
