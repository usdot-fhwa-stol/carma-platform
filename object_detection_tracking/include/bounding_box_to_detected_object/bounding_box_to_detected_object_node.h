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
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>

#include "carma_ros2_utils/carma_lifecycle_node.hpp"

namespace bounding_box_to_detected_object {

class Node : public carma_ros2_utils::CarmaLifecycleNode
{

 private:
  
  //subscriber
  carma_ros2_utils::SubPtr<autoware_auto_msgs::msg::BoundingBoxArray> bbox_sub_;

  //publisher
  carma_ros2_utils::PubPtr<autoware_auto_msgs::msg::DetectedObjects> object_pub_;

 public:
  
   /*! \fn Node()
    \brief Node constructor 
   */
  explicit Node(const rclcpp::NodeOptions& );

  /**
   * \brief Callback for bounding box data to be converted. Publishes a DetectedObjects message from the bounding box array
   * 
   * \param msg The message to convert and republish
   */ 
  void bbox_callback(autoware_auto_msgs::msg::BoundingBoxArray::UniquePtr msg);

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  
};

}//bounding_box_to_detected_object
