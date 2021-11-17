/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include "object_detection_tracking_node.h"

namespace object{

  using std::placeholders::_1;

  ObjectDetectionTrackingNode::ObjectDetectionTrackingNode(const rclcpp::NodeOptions&): pnh_("~"), object_worker_(std::bind(&ObjectDetectionTrackingNode::publishObject, this, _1)){}

  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) {

    // Load parameters
    std::string map_frame_ = "map";

    map_frame_ = this->declare_parameter<std::string>("map_frame", map_frame_);

    this->add_on_set_parameters_callback(
        [this](auto param_vec) {
          
          auto error = update_params<std::string>({ {"map_frame", map_frame_} }, param_vec);

          rcl_interfaces::msg::SetParametersResult result;

          result.successful = !!error;

          if (error) {
            result.reason = error.get();
          } else {
            object_worker_.setMapFrame(map_frame_);
          }

          return result;
        }
      );
    

    // Setup pub/sub
    autoware_obj_sub_= create_subscriber("detected_objects",10,&ObjectDetectionTrackingWorker::detectedObjectCallback,&object_worker_);
    carma_obj_pub_= create_publisher<carma_perception_msgs::msg::ExternalObjectList>("external_objects", 10);

    return CallbackReturn::SUCCESS;
  }

  void ObjectDetectionTrackingNode::publishObject(const carma_perception_msgs::msg::ExternalObjectList& obj_msg)
  {
    carma_obj_pub_.publish(obj_msg);
  }

}//object
