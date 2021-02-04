/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#ifndef MOTION_COMPUTATION_WORKER_H
#define MOTION_COMPUTATION_WORKER_H

#include <ros/ros.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/MobilityPath.h>
#include <std_msgs/String.h>
#include <functional>
#include <motion_predict/motion_predict.h>
#include <motion_predict/predict_ctrv.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <carma_utils/CARMAUtils.h>
#include <tf2/LinearMath/Transform.h>

namespace object{

//! @brief Enum describing the possible operational modes of the MotionComputation
enum MotionComputationMode
{
  MOBILITY_PATH_ONLY = 0,   // MobilityPath used as only source of external object data
  SENSORS_ONLY = 1,         // Sensors used as only source of external object data (mobility paths dropped)
  PATH_AND_SENSORS = 2,     // Both MobilityPath and sensors used without fusion but synchronized so the output message contains both
};

class MotionComputationWorker
{

 public:

  using PublishObjectCallback = std::function<void(const cav_msgs::ExternalObjectList&)>;

  /*!
   * \brief Constructor
   */

  MotionComputationWorker(const PublishObjectCallback& obj_pub);
    
    /*! \fn predictionLogic(cav_msgs::ExternalObjectListPtr obj_list)
    \brief predictionLogic populates duplicated detected object along with its velocity,yaw, yaw_rate and static/dynamic class to ExternalObject message.
    \param  msg array of detected objects.
    */
  void predictionLogic(cav_msgs::ExternalObjectListPtr obj_list);

  // Setters for the prediction parameters
  void setPredictionTimeStep(double time_step);
  void setMobilityPathPredictionTimeStep(double time_step);
  void setPredictionPeriod(double period);
  void setXAccelerationNoise(double noise);
  void setYAccelerationNoise(double noise);
  void setProcessNoiseMax(double noise_max);
  void setConfidenceDropRate(double drop_rate);
  void setExternalObjectPredictionMode(int external_object_prediction_mode);
  
  
  //callbacks
  void mobilityPathCallback(const cav_msgs::MobilityPath& msg);
  void geoReferenceCallback(const std_msgs::String& georef);
  // TODO
  cav_msgs::PredictedState composePredictedState(const lanelet::BasicPoint3d& curr_pt, const lanelet::BasicPoint3d& prev_pt, const ros::Time& prev_time_stamp);

  // msg converter
  cav_msgs::ExternalObject mobilityPathToExternalObject(const cav_msgs::MobilityPath& msg);
 private:

  // local copy of external object publishers

  PublishObjectCallback obj_pub_;

  // Prediction parameters
  double prediction_time_step_ = 0.1;
  double mobility_path_prediction_time_step_ = 0.1;
  double prediction_period_ = 2.0;
  double cv_x_accel_noise_ = 9.0;
  double cv_y_accel_noise_ = 9.0;
  double prediction_process_noise_max_ = 1000.0;
  double prediction_confidence_drop_rate_ = 0.9;
  
  // External object conversion mode
  MotionComputationMode external_object_prediction_mode_ = MOBILITY_PATH_ONLY;

  // Queue for mobility path msgs to synchronize them with sensor msgs 
  cav_msgs::ExternalObjectList mobility_path_list_;

  // Projector variables to convert from ECEF to Map frame
  std::string georeference_ = "";
  std::shared_ptr<lanelet::projection::LocalFrameProjector> local_projector_;

};

}//object

#endif /* MOTION_COMPUTATION_WORKER_H */
