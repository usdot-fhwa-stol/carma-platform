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

#ifndef EXTERNAL_OBJECT_H
#define EXTERNAL_OBJECT_H

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <boost/optional.hpp>

#include "object_detection_tracking_worker.h"
#include "carma_ros2_utils/carma_lifecycle_node.hpp"

namespace object{

class ObjectDetectionTrackingNode : public carma_ros2_utils::CarmaLifecycleNode
{

 private:
  
  //subscriber
  carma_ros2_utils::SubPtr<autoware_auto_msgs::msg::TrackedObjects> autoware_obj_sub_;

  //publisher
  carma_ros2_utils::PubPtr<carma_perception_msgs::msg::ExternalObjectList> carma_obj_pub_;
  
  //ObjectDetectionTrackingWorker class object
  ObjectDetectionTrackingWorker object_worker_;
  
  // Buffer which holds the tree of transforms
  tf2_ros::Buffer tfBuffer_;
  
  // tf2 listeners. Subscribes to the /tf and /tf_static topics
  tf2_ros::TransformListener tfListener_ {tfBuffer_};

  // Output frame id
  std::string map_frame_;

 public:
  
   /*! \fn ObjectDetectionTrackingNode()
    \brief ObjectDetectionTrackingNode constructor 
   */
  explicit ObjectDetectionTrackingNode(const rclcpp::NodeOptions& );

     /*! \fn publishObject()
    \brief Callback to publish ObjectList
   */
  void publishObject(const carma_perception_msgs::msg::ExternalObjectList& obj_msg);

  /*!
  * \brief Callback to lookup a transform between two frames
  * \param parent The parent frame
  * \param child The child frame
  * \param stamp The time stamp of the transform
  * 
  * \return boost::optional<geometry_msgs::TransformStamped> The transform between the two frames or boost::none if the transform does not exist or cannot be computed
  */
  boost::optional<geometry_msgs::msg::TransformStamped> lookupTransform(const std::string& parent, const std::string& child, const rclcpp::Time& stamp);

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  
};

}//object

#endif /* EXTERNAL_OBJECT_H */
