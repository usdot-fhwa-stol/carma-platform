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
#include "motion_computation_worker.h"



namespace object
{
MotionComputationWorker::MotionComputationWorker(const PublishObjectCallback& obj_pub) : obj_pub_(obj_pub){};


void MotionComputationWorker::predictionLogic(cav_msgs::ExternalObjectListPtr obj_list)
{
  cav_msgs::ExternalObjectList output_list;
  cav_msgs::ExternalObjectList sensor_list;

  for (auto obj : obj_list->objects)
  {
    // Header contains the frame rest of the fields will use
   // obj.header = obj_list.objects[i].header;

    // Object id. Matching ids on a topic should refer to the same object within some time period, expanded
   // obj.id = obj_list.objects[i].id;

    // Update the object type and generate predictions using CV or CTRV vehicle models.
		// If the object is a bicycle or motor vehicle use CTRV otherwise use CV.

    bool use_ctrv_model;

    if (  obj.object_type == obj.UNKNOWN)
    {
      use_ctrv_model = true;
    }
    else if (obj.object_type == obj.MOTORCYCLE)
    {
      use_ctrv_model = true;
    }
    else if (obj.object_type == obj.SMALL_VEHICLE)
    {
      use_ctrv_model = true;
    }
    else if (obj.object_type == obj.LARGE_VEHICLE)
    {
      use_ctrv_model = true;
    }
    else if ( obj.object_type == obj.PEDESTRIAN)
    {
      use_ctrv_model = false;
    }
    else
    {
      obj.object_type = obj.UNKNOWN;
      use_ctrv_model = false;
    }//end if-else */

    
    if (use_ctrv_model == true)
    {
      obj.predictions =
          motion_predict::ctrv::predictPeriod(obj, prediction_time_step_, prediction_period_,
                                              prediction_process_noise_max_, prediction_confidence_drop_rate_);
    }
    else
    {
      obj.predictions = motion_predict::cv::predictPeriod(
          obj, prediction_time_step_, prediction_period_, cv_x_accel_noise_, cv_y_accel_noise_,
          prediction_process_noise_max_, prediction_confidence_drop_rate_);
    }

    sensor_list.objects.emplace_back(obj);
  }//end for-loop

  // Determine mode
  switch(external_object_prediction_mode_)
  {
    case SENSORS_ONLY:
      output_list = sensor_list;
    break;
    case PATH_AND_SENSORS:
      output_list = synchronizeAndAppend(sensor_list, mobility_path_list_);
    break;
    default:
      ROS_WARN_STREAM("Received invalid motion computation operational mode:" << external_object_prediction_mode_ << " publishing empty list.");
    break;
  }
  
  obj_pub_(output_list);

  // Clear mobility msg path queue since it is published
  mobility_path_list_.objects = {};
}

void MotionComputationWorker::setPredictionTimeStep(double time_step)
{
  prediction_time_step_ = time_step;
}

void MotionComputationWorker::setMobilityPathPredictionTimeStep(double time_step)
{
  mobility_path_prediction_time_step_ = time_step;
}

void MotionComputationWorker::setPredictionPeriod(double period)
{
  prediction_period_ = period;
}

void MotionComputationWorker::setXAccelerationNoise(double noise)
{
  cv_x_accel_noise_ = noise;
}

void MotionComputationWorker::setYAccelerationNoise(double noise)
{
  cv_y_accel_noise_ = noise;
}

void MotionComputationWorker::setProcessNoiseMax(double noise_max)
{
  prediction_process_noise_max_ = noise_max;
}

void MotionComputationWorker::setConfidenceDropRate(double drop_rate)
{
  prediction_confidence_drop_rate_ = drop_rate;
}

void MotionComputationWorker::setExternalObjectPredictionMode(int external_object_prediction_mode)
{
  external_object_prediction_mode_ = static_cast<MotionComputationMode>(external_object_prediction_mode);
}

void MotionComputationWorker::geoReferenceCallback(const std_msgs::String& georef)
{
  if (georeference_ == georef.data)
    return;
  else
  {
    georeference_ = georef.data;
  }
  local_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(lanelet::projection::LocalFrameProjector(georeference_.c_str()));
}

void MotionComputationWorker::mobilityPathCallback(const cav_msgs::MobilityPath& msg)
{
  mobility_path_list_.objects.push_back(mobilityPathToExternalObject(msg));
  if (external_object_prediction_mode_ == MOBILITY_PATH_ONLY)
  {
    obj_pub_(mobility_path_list_);
    // Clear mobility msg path queue since it is published
    mobility_path_list_.objects = {};
    return;
  }
}

cav_msgs::ExternalObject MotionComputationWorker::mobilityPathToExternalObject(const cav_msgs::MobilityPath& msg)
{
  cav_msgs::ExternalObject output;

  if (!local_projector_)
  {
    ROS_WARN_STREAM("Motion computation has not received georeference string yet!");
    return output;
  }

  // get reference origin in ECEF (convert from cm to m)
  double ecef_x = (double)msg.trajectory.location.ecef_x/100.0;
  double ecef_y = (double)msg.trajectory.location.ecef_y/100.0;
  double ecef_z = (double)msg.trajectory.location.ecef_z/100.0;

  // Convert general information
  for (size_t i = 0; i < msg.header.sender_bsm_id.size(); i+=2) // convert hex std::string to uint8_t array
  {
    int num = 0;
    sscanf(msg.header.sender_bsm_id.substr(i, i + 2).c_str(), "%x", &num);
    output.bsm_id.push_back((uint8_t)num);
  }
  // first point's timestamp
  output.header.stamp = ros::Time((double)msg.header.timestamp/ 1000.0);

  // If it is a static object, we finished processing
  if (msg.trajectory.offsets.size() < 2)
  {
    output.dynamic_obj = false;
    return output;
  }
  output.dynamic_obj = true;

  // get planned trajectory points
  auto prev_pt_msg = msg.trajectory.offsets[0]; // setup first point to be processed later
  cav_msgs::PredictedState prev_state;
  lanelet::BasicPoint3d prev_pt_ecef {ecef_x + (double)prev_pt_msg.offset_x /100.0, ecef_y + (double)prev_pt_msg.offset_y /100.0, ecef_z + (double)prev_pt_msg.offset_z /100.0};

  auto prev_pt_map = local_projector_->projectECEF(prev_pt_ecef, -1);

  for (size_t i = 1; i < msg.trajectory.offsets.size(); i ++)
  {
    auto curr_pt_msg = msg.trajectory.offsets[i];
    lanelet::BasicPoint3d curr_pt_ecef {ecef_x + (double)curr_pt_msg.offset_x /100.0, ecef_y + (double)curr_pt_msg.offset_y /100.0, ecef_z + (double)curr_pt_msg.offset_z /100.0};
    auto curr_pt_map = local_projector_->projectECEF(curr_pt_ecef, -1);

    cav_msgs::PredictedState curr_state;

    if (i == 1) // First point's state should be stored outside "predictions"
    {
      curr_state = composePredictedState(curr_pt_map, prev_pt_map, output.header.stamp); 
      output.pose.pose = curr_state.predicted_position;
      output.velocity.twist = curr_state.predicted_velocity;
    }
    else    
    {
      curr_state = composePredictedState(curr_pt_map, prev_pt_map, prev_state.header.stamp + ros::Duration(mobility_path_prediction_time_step_));
      output.predictions.push_back(curr_state);
    }

    if (i == msg.trajectory.offsets.size() - 1) // if last point, copy the prev_state velocity & orientation to the last point too
    {
      curr_state = composePredictedState(curr_pt_map, prev_pt_map, curr_state.header.stamp + ros::Duration(mobility_path_prediction_time_step_));
      curr_state.predicted_position.position.x = curr_pt_map.x();
      curr_state.predicted_position.position.y = curr_pt_map.y();
      curr_state.predicted_position.position.z = curr_pt_map.z();
      output.predictions.push_back(curr_state);
    }
    
    prev_state = curr_state;
    prev_pt_map = curr_pt_map;
  }

  return output;
}

cav_msgs::PredictedState MotionComputationWorker::composePredictedState(const lanelet::BasicPoint3d& curr_pt, const lanelet::BasicPoint3d& prev_pt, const ros::Time& prev_time_stamp)
{
  cav_msgs::PredictedState output_state;
  // Set Position
  output_state.predicted_position.position.x = prev_pt.x();
  output_state.predicted_position.position.y = prev_pt.y();
  output_state.predicted_position.position.z = prev_pt.z();

  // Set Orientation
  Eigen::Vector2d vehicle_vector = {curr_pt.x() - prev_pt.x() ,curr_pt.y() - prev_pt.y()};
  Eigen::Vector2d x_axis = {1, 0};
  double yaw = std::acos(vehicle_vector.dot(x_axis)/(vehicle_vector.norm() * x_axis.norm()));

  tf2::Quaternion vehicle_orientation;
  vehicle_orientation.setRPY(0, 0, yaw);
  output_state.predicted_position.orientation.x = vehicle_orientation.getX();
  output_state.predicted_position.orientation.y = vehicle_orientation.getY();
  output_state.predicted_position.orientation.z = vehicle_orientation.getZ();
  output_state.predicted_position.orientation.w = vehicle_orientation.getW();

  // Set velocity
  output_state.predicted_velocity.linear.x = vehicle_vector.norm() / mobility_path_prediction_time_step_;

  // Set timestamp
  output_state.header.stamp = prev_time_stamp;

  return output_state;
}

cav_msgs::ExternalObjectList MotionComputationWorker::synchronizeAndAppend(const cav_msgs::ExternalObjectList& sensor_list, cav_msgs::ExternalObjectList mobility_path_list)
{
  cav_msgs::ExternalObjectList output_list;
  // Compare time_stamps of first elements of each list as they are guaranteed to be the earliest of the respective lists
  
  for (auto &path: mobility_path_list.objects)
  {
    // interpolate and match timesteps
    path = matchAndInterpolateTimeStamp(path, sensor_list.objects[0].header.stamp);
  }
  
  output_list.objects.insert(output_list.objects.begin(),sensor_list.objects.begin(),sensor_list.objects.end());
  output_list.objects.insert(output_list.objects.end(),mobility_path_list.objects.begin(),mobility_path_list.objects.end());
  return output_list;
}

cav_msgs::ExternalObject MotionComputationWorker::matchAndInterpolateTimeStamp(cav_msgs::ExternalObject path, const ros::Time& time_to_match) const
{
  cav_msgs::ExternalObject output = path;
  // empty predictions
  output.predictions = {};

  // add the first point to start of the predictions to easily loop over
  cav_msgs::PredictedState prev_state;
  prev_state.header.stamp = output.header.stamp;
  prev_state.predicted_position.orientation = output.pose.pose.orientation ;         
  prev_state.predicted_velocity = output.velocity.twist ;    
  prev_state.predicted_position.position.x = output.pose.pose.position.x ;        
  prev_state.predicted_position.position.y = output.pose.pose.position.y ;        
  prev_state.predicted_position.position.z = output.pose.pose.position.z ;  
  path.predictions.insert(path.predictions.begin(), prev_state);

  ros::Time curr_time_to_match = time_to_match;
  // because of this logic, we would not encounter mobility path
  // that starts later than the time we are trying to match (which is starting time of sensed objects)
  bool is_first_point = true;
  cav_msgs::PredictedState new_state;
  for (auto curr_state : path.predictions)
  { 
    if (curr_time_to_match > curr_state.header.stamp )
    {
      prev_state = curr_state;
      continue;
    }
    // reaching here means curr_state starts later than the time we are trying to match
    ros::Duration delta_t = curr_state.header.stamp - curr_time_to_match;
    double ratio = delta_t.toSec() / mobility_path_prediction_time_step_;
    double delta_x = curr_state.predicted_position.position.x - prev_state.predicted_position.position.x;
    double delta_y = curr_state.predicted_position.position.y - prev_state.predicted_position.position.y;
    double delta_z = curr_state.predicted_position.position.z - prev_state.predicted_position.position.z;
    // copy old unchanged parts
    new_state.header.stamp = curr_time_to_match;
    new_state.predicted_velocity = prev_state.predicted_velocity;
    new_state.predicted_position.orientation = prev_state.predicted_position.orientation;
    // interpolate position
    // we are "stepping back in time" to match the position
    new_state.predicted_position.position.x = curr_state.predicted_position.position.x - delta_x * ratio; 
    new_state.predicted_position.position.y = curr_state.predicted_position.position.y - delta_y * ratio; 
    new_state.predicted_position.position.z = curr_state.predicted_position.position.z - delta_z * ratio; 

    if (is_first_point) // we store in the body if it is the first point, not predictions
    {
      output.header.stamp = curr_time_to_match;
      output.pose.pose.orientation = new_state.predicted_position.orientation;
      output.velocity.twist = new_state.predicted_velocity;
      output.pose.pose.position.x = new_state.predicted_position.position.x;
      output.pose.pose.position.y = new_state.predicted_position.position.y;
      output.pose.pose.position.z = new_state.predicted_position.position.z;
      is_first_point = false;
    }
    else
    {
      output.predictions.push_back(new_state);
    }
    prev_state = curr_state;
    curr_time_to_match += ros::Duration(mobility_path_prediction_time_step_);
  }

  return output;
}

}  // namespace object
