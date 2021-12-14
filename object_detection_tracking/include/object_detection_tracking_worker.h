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

#ifndef EXTERNAL_OBJECT_WORKER_H
#define EXTERNAL_OBJECT_WORKER_H

#include <rclcpp/rclcpp.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <functional>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/optional.hpp>

namespace object{

class ObjectDetectionTrackingWorker
{

 public:

  using PublishObjectCallback = std::function<void(const carma_perception_msgs::msg::ExternalObjectList&)>;

  /**
   * Function which will return the most recent transform between the provided frames
   * First frame is target second is source
   * If the transform does not exist or cannot be computed the optional returns false
   */
  using TransformLookupCallback =
      std::function<boost::optional<geometry_msgs::msg::TransformStamped>(const std::string&, const std::string&, const rclcpp::Time& stamp)>;

  /*!
   * \brief Constructor
   */

  ObjectDetectionTrackingWorker(PublishObjectCallback obj_pub, TransformLookupCallback tf_lookup, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger);
    
    /*! \fn detectedObjectCallback(const autoware_auto_msgs::autoware_msgs &msg)
    \brief detectedObjectCallback populates detected object along with its velocity,yaw, yaw_rate and static/dynamic class to DetectedObject message.
    \param  msg array of detected objects.
    */

  void detectedObjectCallback(autoware_auto_msgs::msg::TrackedObjects::UniquePtr msg);

  // Setters for the parameters
  void setMapFrame(std::string map_frame);

 
 private:

  // local copy of external object publihsers

  PublishObjectCallback obj_pub_;


  // local copy of transform lookup callback
  TransformLookupCallback tf_lookup_;

  // Parameters
  std::string map_frame_ = "map";

  // Logger interface
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  

  /**
   * \brief Helper method to check if the provided tracked object has the provided class type
   * 
   * \param obj The object to check
   * \param class_id The class type to check
   * 
   * \return true if the object has the class type
   */ 
  bool isClass(const autoware_auto_msgs::msg::TrackedObject& obj, uint8_t class_id);
};

}//object

#endif /* EXTERNAL_OBJECT_WORKER_H */
