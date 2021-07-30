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
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
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

  using PublishObjectCallback = std::function<void(const cav_msgs::ExternalObjectList&)>;

  /*!
   * \brief Constructor
   */

  ObjectDetectionTrackingWorker(PublishObjectCallback obj_pub);
    
    /*! \fn detectedObjectCallback(const autoware_msgs::DetectedObjectArray &msg)
    \brief detectedObjectCallback populates detected object along with its velocity,yaw, yaw_rate and static/dynamic class to DetectedObject message.
    \param  msg array of detected objects.
    */

  void detectedObjectCallback(const autoware_msgs::DetectedObjectArray &msg);

  // Setters for the prediction parameters
  void setPredictionTimeStep(double time_step);
  void setPredictionPeriod(double period);
  void setXAccelerationNoise(double noise);
  void setYAccelerationNoise(double noise);
  void setProcessNoiseMax(double noise_max);
  void setConfidenceDropRate(double drop_rate);

  void setVelodyneFrame(std::string velodyne_frame);
  void setMapFrame(std::string map_frame);

 
 private:

  // local copy of external object publihsers

  PublishObjectCallback obj_pub_;

  // Prediction parameters
  double prediction_time_step_ = 0.1;
  double prediction_period_ = 2.0;
  double cv_x_accel_noise_ = 9.0;
  double cv_y_accel_noise_ = 9.0;
  double prediction_process_noise_max_ = 1000.0;
  double prediction_confidence_drop_rate_ = 0.9;
  std::string velodyne_frame_;
  std::string map_frame_;

  // Buffer which holds the tree of transforms
  tf2_ros::Buffer tfBuffer_;
  // tf2 listeners. Subscribes to the /tf and /tf_static topics
  tf2_ros::TransformListener tfListener_ {tfBuffer_};
  
};

}//object

#endif /* EXTERNAL_OBJECT_WORKER_H */
