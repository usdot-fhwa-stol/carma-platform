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
#include "bounding_box_to_detected_object/bounding_box_to_detected_object_node.h"
#include <geometry/bounding_box/bounding_box_common.hpp>

namespace bounding_box_to_detected_object {

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  Node::Node(const rclcpp::NodeOptions& options)
  : carma_ros2_utils::CarmaLifecycleNode(options)
    {
    }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &) {    

    // Setup pub/sub
    bbox_sub_= create_subscription<autoware_auto_msgs::msg::BoundingBoxArray>("bounding_boxes",10,
      std::bind(&Node::bbox_callback, this, _1)
    );

    object_pub_= create_publisher<autoware_auto_msgs::msg::DetectedObjects>("lidar_detected_objects", 10);

    return CallbackReturn::SUCCESS;
  }

  autoware_auto_msgs::msg::DetectedObject detected_obj_from_bounding_box(const autoware_auto_msgs::msg::BoundingBox& box) {

    autoware_auto_msgs::msg::DetectedObject obj;

    // The following method currently does not properly set the orientation
    // so that will be set manually following this call
    obj = autoware::common::geometry::bounding_box::details::make_detected_object(box);
    
    obj.kinematics.orientation.x = box.orientation.x;
    obj.kinematics.orientation.y = box.orientation.y;
    obj.kinematics.orientation.z = box.orientation.z;
    obj.kinematics.orientation.w = box.orientation.w;
    obj.kinematics.orientation_availability = autoware_auto_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;

    return obj;

  }

  void Node::bbox_callback(autoware_auto_msgs::msg::BoundingBoxArray::UniquePtr msg) {
    autoware_auto_msgs::msg::DetectedObjects objects;

    objects.header = msg->header;
    objects.objects.reserve(msg->boxes.size());

    for (const auto& bb : msg->boxes) {
      objects.objects.emplace_back(detected_obj_from_bounding_box(bb));
    }

    object_pub_->publish(objects);

  }
    

}//bounding_box_to_detected_object

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(bounding_box_to_detected_object::Node)
