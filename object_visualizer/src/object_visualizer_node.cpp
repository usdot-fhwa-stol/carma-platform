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
    config_.marker_shape = declare_parameter<uint8_t>("marker_shape", config_.marker_shape);

    // Declare new parameters for pedestrian visualization
    config_.use_pedestrian_icon = declare_parameter<bool>("use_pedestrian_icon", config_.use_pedestrian_icon);
    config_.pedestrian_icon_path = declare_parameter<std::string>("pedestrian_icon_path", config_.pedestrian_icon_path);
    config_.pedestrian_icon_scale = declare_parameter<double>("pedestrian_icon_scale", config_.pedestrian_icon_scale);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    // Update parameters
    auto error = update_params<bool>(
      {
        {"enable_external_objects_viz", config_.enable_external_objects_viz},
        {"enable_roadway_objects_viz", config_.enable_roadway_objects_viz},
        {"use_pedestrian_icon", config_.use_pedestrian_icon}
      }, parameters);

    auto error2 = update_params<std::string>(
      {
        {"external_objects_viz_ns", config_.external_objects_viz_ns},
        {"roadway_obstacles_viz_ns", config_.roadway_obstacles_viz_ns},
        {"pedestrian_icon_path", config_.pedestrian_icon_path}
      }, parameters);

    auto error3 = update_params<uint8_t>(
      {
        {"marker_shape", config_.marker_shape}
      }, parameters);

    auto error4 = update_params<double>(
      {
        {"pedestrian_icon_scale", config_.pedestrian_icon_scale}
      }, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error2 && !error3 && !error4;

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
    get_parameter<uint8_t>("marker_shape", config_.marker_shape);

    // Load new parameters for pedestrian visualization
    get_parameter<bool>("use_pedestrian_icon", config_.use_pedestrian_icon);
    get_parameter<std::string>("pedestrian_icon_path", config_.pedestrian_icon_path);
    get_parameter<double>("pedestrian_icon_scale", config_.pedestrian_icon_scale);

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

  void Node::createPedestrianMarker(visualization_msgs::msg::Marker& marker,
                                  const std_msgs::msg::Header& header,
                                  const geometry_msgs::msg::Pose& pose,
                                  size_t id,
                                  const std::string& ns,
                                  const geometry_msgs::msg::Vector3& size)
  {
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.pose = pose;

    if (config_.use_pedestrian_icon && !config_.pedestrian_icon_path.empty()) {
      // Use a mesh resource marker for pedestrian representation
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.mesh_resource = config_.pedestrian_icon_path;
      marker.mesh_use_embedded_materials = true;

      // Scale the pedestrian mesh appropriately
      marker.scale.x = config_.pedestrian_icon_scale;
      marker.scale.y = - config_.pedestrian_icon_scale; // Invert Y to fix the STL orientation
      marker.scale.z = config_.pedestrian_icon_scale;

      // White color for the pedestrian icon
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
    } else {
      // Fallback to a person-shaped marker using cylinder (if mesh isn't available)
      marker.type = visualization_msgs::msg::Marker::CYLINDER;

      // Use the provided size or default to a human-like proportion
      marker.scale.x = std::max(0.5, size.x * 2.0);  // Width
      marker.scale.y = std::max(0.5, size.y * 2.0);  // Depth
      marker.scale.z = std::max(1.7, size.z * 2.0);  // Height - typical human height

      // Set the color to red for the pedestrian marker
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
    }

    marker.action = visualization_msgs::msg::Marker::ADD;
  }

  void Node::external_objects_callback(carma_perception_msgs::msg::ExternalObjectList::UniquePtr msg)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "external_objects_callback called");

    if (!config_.enable_external_objects_viz) {
      RCLCPP_DEBUG_STREAM(get_logger(), "external_objects_callback called, but visualization is not enabled.");
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

      // Pedestrian's will be represented as specialized icon/marker
      if (obj.object_type == carma_perception_msgs::msg::ExternalObject::PEDESTRIAN) {
        createPedestrianMarker(marker, msg->header, obj.pose.pose, id, config_.external_objects_viz_ns, obj.size);

        // Adjust Z position to prevent clipping with ground
        marker.pose.position.z = std::max(1.0, obj.size.z);
      }
      // All other detected objects will use the configured marker shape
      else {
        marker.header = msg->header;
        marker.ns = config_.external_objects_viz_ns;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose = obj.pose.pose;
        marker.pose.position.z = std::max(1.0, obj.size.z); // lifts the marker above ground so that it doesn't clip

        marker.id = id;
        if (static_cast<int>(config_.marker_shape) < 1 || static_cast<int>(config_.marker_shape) > 3)
        {
          throw std::invalid_argument("Marker shape is not valid: " + std::to_string(static_cast<int>(config_.marker_shape)) + ". Please choose from 1: CUBE, 2: SPHERE, 3: CYLINDER");
        }
        marker.type = config_.marker_shape;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // overwrite size in case any previous stack doesn't provide size
        // such as carma_cooperative_perception at the moment
        marker.scale.x = std::max(1.0, obj.size.x) * 2;  // Size in carma is half the length/width/height
        marker.scale.y = std::max(1.0, obj.size.y) * 2;
        marker.scale.z = std::max(1.0, obj.size.z) * 2;
      }

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
    RCLCPP_DEBUG_STREAM(get_logger(), "roadway_obstacles_callback called");

    if (!config_.enable_roadway_objects_viz) {
      RCLCPP_DEBUG_STREAM(get_logger(), "roadway_obstacles_callback called, but visualization is not enabled.");
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

      // Pedestrian's will be represented with a specialized icon
      if (obj.object.object_type == carma_perception_msgs::msg::ExternalObject::PEDESTRIAN) {
        createPedestrianMarker(marker, obj.object.header, obj.object.pose.pose, id,
                              config_.roadway_obstacles_viz_ns, obj.object.size);
      }
      // Connected vehicles will be green
      else if (obj.connected_vehicle_type.type != carma_perception_msgs::msg::ConnectedVehicleType::NOT_CONNECTED) {
        marker.header = obj.object.header;
        marker.ns = config_.roadway_obstacles_viz_ns;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = obj.object.pose.pose;
        marker.scale.x = obj.object.size.x * 2.0; // Size in carma is half the length/width/height
        marker.scale.y = obj.object.size.y * 2.0;
        marker.scale.z = obj.object.size.z * 2.0;
      }
      // All other detected objects will be blue
      else {
        marker.header = obj.object.header;
        marker.ns = config_.roadway_obstacles_viz_ns;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = obj.object.pose.pose;
        marker.scale.x = obj.object.size.x * 2.0;
        marker.scale.y = obj.object.size.y * 2.0;
        marker.scale.z = obj.object.size.z * 2.0;
      }

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
