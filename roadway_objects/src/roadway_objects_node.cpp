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
#include "roadway_objects/roadway_objects_node.hpp"

namespace roadway_objects
{
  namespace std_ph = std::placeholders;

  RoadwayObjectsNode::RoadwayObjectsNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options),
        wm_listener_(this->get_node_base_interface(), this->get_node_logging_interface(), 
                     this->get_node_topics_interface(), this->get_node_parameters_interface()),
        object_worker_(wm_listener_.getWorldModel(), std::bind(&RoadwayObjectsNode::publishObstacles, this, std_ph::_1), get_node_logging_interface())
  {
  }

  carma_ros2_utils::CallbackReturn RoadwayObjectsNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "RoadwayObjectsNode trying to configure");
    
    // Setup subscribers
    external_objects_sub_ = create_subscription<carma_perception_msgs::msg::ExternalObjectList>("external_objects", 10,
                                                              std::bind(&RoadwayObjectsWorker::externalObjectsCallback, &object_worker_, std_ph::_1));

    // Setup publishers
    roadway_obs_pub_ = create_publisher<carma_perception_msgs::msg::RoadwayObstacleList>("roadway_objects", 10);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void RoadwayObjectsNode::publishObstacles(const carma_perception_msgs::msg::RoadwayObstacleList& msg)
  {
    roadway_obs_pub_->publish(msg);
  }

} // roadway_objects

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(roadway_objects::RoadwayObjectsNode)
