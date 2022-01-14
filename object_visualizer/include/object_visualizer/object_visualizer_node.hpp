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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "object_visualizer/object_visualizer_config.hpp"

namespace object_visualizer
{

  /**
   * \brief TODO for USER: Add class description
   * 
   */
  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_perception_msgs::msg::ExternalObjectList> external_objects_sub_;
    carma_ros2_utils::SubPtr<carma_perception_msgs::msg::RoadwayObstacleList> roadway_obstacles_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> external_objects_viz_pub_;
    carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> roadway_obstacles_viz_pub_;

    // Node configuration
    Config config_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * \brief Example callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief External objects callback. Converts the message to a visualization message to republish
     * 
     * \param msg The received message
     */
    void external_objects_callback(std_msgs::msg::String::UniquePtr msg);

    /**
     * \brief Roadway obstacles callback. Converts the message to a visualization message to republish
     * 
     * \param msg The received message
     */
    void roadway_obstacles_callback(std_msgs::msg::String::UniquePtr msg);


    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

  };

} // object_visualizer
