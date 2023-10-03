// Copyright 2019-2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTION_COMPUTATION__MOTION_COMPUTATION_WORKER_HPP_
#define MOTION_COMPUTATION__MOTION_COMPUTATION_WORKER_HPP_

#include <gtest/gtest_prod.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_v2x_msgs/msg/mobility_path.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <motion_predict/motion_predict.hpp>
#include <motion_predict/predict_ctrv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>

namespace motion_computation
{

/**
 * \class MotionComputationWorker
 * \brief The class containing the primary business logic for the Motion Computation Package
 */
class MotionComputationWorker
{
public:
  using PublishObjectCallback =
    std::function<void(const carma_perception_msgs::msg::ExternalObjectList &)>;
  using LookUpTransform = std::function<void()>;

  /*!
   * \brief Constructor for MotionComputationWorker
   */
  MotionComputationWorker(
    const PublishObjectCallback & obj_pub,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock);
  /**
   * \brief Function to populate duplicated detected objects along with their velocity, yaw,
   * yaw_rate and static/dynamic class to the provided ExternalObjectList message.
   * \param  obj_list ExternalObjectList message
   */
  void predictionLogic(carma_perception_msgs::msg::ExternalObjectList::UniquePtr obj_list);

  // Setters for the prediction parameters
  void setPredictionTimeStep(double time_step);
  void setPredictionPeriod(double period);
  void setXAccelerationNoise(double noise);
  void setYAccelerationNoise(double noise);
  void setProcessNoiseMax(double noise_max);
  void setConfidenceDropRate(double drop_rate);
  void setDetectionInputFlags(
    bool enable_sensor_processing, bool enable_bsm_processing, bool enable_psm_processing,
    bool enable_mobility_path_processing);

  // callbacks
  void mobilityPathCallback(const carma_v2x_msgs::msg::MobilityPath::UniquePtr msg);

  void bsmCallback(const carma_v2x_msgs::msg::BSM::UniquePtr msg);

  void psmCallback(const carma_v2x_msgs::msg::PSM::UniquePtr msg);

  /**
   * \brief Callback for map projection string to define lat/lon -> map conversion
   * \brief msg The proj string defining the projection.
   */
  void georeferenceCallback(const std_msgs::msg::String::UniquePtr msg);

  /**
   * \brief Converts from MobilityPath's predicted points in ECEF to local map and other fields in an ExternalObject
   * object
   * \param msg MobilityPath message to convert
   * \return ExternalObject object
   */
  carma_perception_msgs::msg::ExternalObject mobilityPathToExternalObject(
    const carma_v2x_msgs::msg::MobilityPath::UniquePtr & msg) const;

  /**
   * \brief Appends external objects list behind base_objects. This does not do sensor fusion.
   * When doing so, it drops the predictions points that start before the first prediction is sensor list.
   * And interpolates the remaining predictions points to match the timestep using its average speed
   * between points
   * \param base_objects object detections to append to and synchronize with
   * \param new_objects new objects to add and be synchronized
   * \return append and synchronized list of external objects
   */
  carma_perception_msgs::msg::ExternalObjectList synchronizeAndAppend(
    const carma_perception_msgs::msg::ExternalObjectList & base_objects,
    carma_perception_msgs::msg::ExternalObjectList new_objects) const;

  /*!
   * \brief It cuts ExternalObject's prediction points before the time_to_match. And uses the average
   *         velocity in its predictions to match the starting point to the point it would have crossed at time_to_match
   *         It uses mobility_path_time_step between prediction points to interpolate.
   * \param path External object with predictions to modify
   * \param time_to_match time stamp to have the object start at
   * \return carma_perception_msgs::msg::ExternalObject
   * \note  It assumes time_to_match falls in prediction time's whole interval.
   */
  carma_perception_msgs::msg::ExternalObject matchAndInterpolateTimeStamp(
    carma_perception_msgs::msg::ExternalObject path, const rclcpp::Time & time_to_match) const;

private:
  // Local copy of external object publisher
  PublishObjectCallback obj_pub_;

  // Prediction parameters

  double prediction_time_step_ = 0.1;  // Seconds
  double prediction_period_ = 2.0;     // Seconds
  double cv_x_accel_noise_ = 9.0;
  double cv_y_accel_noise_ = 9.0;
  double prediction_process_noise_max_ = 1000.0;
  double prediction_confidence_drop_rate_ = 0.9;

  // Flags for the different possible detection inputs
  bool enable_sensor_processing_ = true;
  bool enable_bsm_processing_ = false;
  bool enable_psm_processing_ = false;
  bool enable_mobility_path_processing_ = false;

  // Map frame
  std::string map_frame_id_ = "map";

  // Logger interface
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  // Clock interface - gets the ros simulated clock from Node
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;

  // Queue for v2x msgs to synchronize them with sensor msgs
  carma_perception_msgs::msg::ExternalObjectList mobility_path_list_;
  carma_perception_msgs::msg::ExternalObjectList bsm_list_;
  carma_perception_msgs::msg::ExternalObjectList psm_list_;

  // Maps of external object id to index in synchronization queues
  std::unordered_map<uint32_t, size_t> mobility_path_obj_id_map_;
  std::unordered_map<uint32_t, size_t> bsm_obj_id_map_;
  std::unordered_map<uint32_t, size_t> psm_obj_id_map_;

  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

  // Rotation of a North East Down frame located on the map origin described in the map frame
  tf2::Quaternion ned_in_map_rotation_;

  // Unit Test Accessors
  FRIEND_TEST(MotionComputationWorker, MobilityPathToExternalObject);
  FRIEND_TEST(MotionComputationWorker, PsmToExternalObject);
  FRIEND_TEST(MotionComputationWorker, BSMtoExternalObject);
};

}  // namespace motion_computation

#endif  // MOTION_COMPUTATION__MOTION_COMPUTATION_WORKER_HPP_
