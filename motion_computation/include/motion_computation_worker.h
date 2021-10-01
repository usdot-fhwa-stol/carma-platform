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
#include <carma_utils/CARMAUtils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tuple>

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
  using LookUpTransform = std::function<void()>;

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

  /**
   * \brief Callback for map projection string to define lat/lon -> map conversion
   * \brief msg The proj string defining the projection.
   */ 
  void georeferenceCallback(const std_msgs::StringConstPtr& msg);
  
  /*!
  \brief Compose cav_msgs::PredictedState msg current point and previous point. It calculates the speed from these points using mobility_path_time_step
  \param curr_pt current point
  \param prev_pt prev_pt. this point is recorded in the state
  \param prev_time_stamp prev_pt's time stamp. This time is recorded in the state
  \param prev_yaw A previous yaw value in radians to use if the two provided points are equivalent
  \return std::pair<cav_msgs::PredictedState,double> where the first element is the prediction including linear velocity, last_time, orientation filled in and the second element is the yaw in radians used to compute the orientation
  */
  std::pair<cav_msgs::PredictedState, double> composePredictedState(const tf2::Vector3& curr_pt, const tf2::Vector3& prev_pt, const ros::Time& prev_time_stamp, double prev_yaw) const;

  /*!
  \brief Convert from mobilitypath's predicted points in ECEF to local map and other fields in external object
  \param msg Mobility path msg to convert
  \return ExternalObject object
  */
  cav_msgs::ExternalObject mobilityPathToExternalObject(const cav_msgs::MobilityPath& msg) const;

  /*!
  \brief Appends external objects list behind sensor_list. This does not do sensor fusion.
         When doing so, it drops the predictions points that start before the first prediction is sensor list.
         And interpolates the remaining predictions points to match the mobility_path_time_step using its average sped between points
  \param curr_pt sensor_list sensor list from object detection
  \param prev_pt mobility_path_list. from incoming mobility path msg from other cars
  \return append and synchronized list of external objects
  */
  cav_msgs::ExternalObjectList synchronizeAndAppend(const cav_msgs::ExternalObjectList& sensor_list, cav_msgs::ExternalObjectList mobility_path_list) const;

  /*!
  \brief It cuts ExternalObject's prediction points before the time_to_match. And uses the average 
         velocity in its predictions to match the starting point to the point it would have crossed at time_to_match
         It uses mobility_path_time_step between prediction points to interpolate.
  \param path External object with predictions to modify
  \param time_to_match time stamp to have the object start at
  \return cav_msgs::ExternalObject
  \note  It assumes time_to_match falls in prediction time's whole interval.
  */
  cav_msgs::ExternalObject matchAndInterpolateTimeStamp(cav_msgs::ExternalObject path, const ros::Time& time_to_match) const;

  /*!
  \brief Transforms ecef point to map frame using internally saved map transform
  \param ecef_point ecef_point to transform
  \return point in map
  */
  tf2::Vector3 transform_to_map_frame(const tf2::Vector3& ecef_point) const;

 private:

  /*! 
  \brief Helper function to fill in the angular velocity of the external object
  \param ExternalObject to fill in its angular velocities
  */
  void calculateAngVelocityOfPredictedStates(cav_msgs::ExternalObject& object) const;
  
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

  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

};

}//object

#endif /* MOTION_COMPUTATION_WORKER_H */
