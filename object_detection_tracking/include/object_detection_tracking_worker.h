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

#include <ros/ros.h>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <functional>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
namespace object{

class ObjectDetectionTrackingWorker
{

 public:

  using PublishObjectCallback = std::function<void(const carma_perception_msgs::msg::ExternalObjectList&)>;

  /*!
   * \brief Constructor
   */

  ObjectDetectionTrackingWorker(PublishObjectCallback obj_pub);
    
    /*! \fn detectedObjectCallback(const autoware_msgs::DetectedObjectArray &msg)
    \brief detectedObjectCallback populates detected object along with its velocity,yaw, yaw_rate and static/dynamic class to DetectedObject message.
    \param  msg array of detected objects.
    */

  void detectedObjectCallback(const autoware_msgs::DetectedObjectArray &msg);

  // Setters for the parameters
  void setMapFrame(std::string map_frame);

 
 private:

  // local copy of external object publihsers

  PublishObjectCallback obj_pub_;

  // Parameters
  std::string map_frame_;

  // Buffer which holds the tree of transforms
  tf2_ros::Buffer tfBuffer_;
  
  // tf2 listeners. Subscribes to the /tf and /tf_static topics
  tf2_ros::TransformListener tfListener_ {tfBuffer_};
  
};

}//object

#endif /* EXTERNAL_OBJECT_WORKER_H */
