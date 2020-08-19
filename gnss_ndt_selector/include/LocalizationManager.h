#pragma once
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

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/LocalizationStatusReport.h>
#include <functional>
#include <unordered_set>
#include <carma_utils/timers/Timer.h>
#include <carma_utils/timers/TimerFactory.h>
#include "LocalizationTypes.h"
#include "LocalizationManagerConfig.h"
#include "LocalizationTransitionTable.h"

namespace localizer
{
class LocalizationManager
{
public:
  using PosePublisher = std::function<void(const geometry_msgs::PoseStamped&)>;
  using TransformPublisher = std::function<void(const geometry_msgs::TransformStamped&)>;
  using StatePublisher = std::function<void(const cav_msgs::LocalizationStatusReport&)>;
  using TimerUniquePtr = std::unique_ptr<carma_utils::timers::Timer>;

  /**
   * \brief Constructor
   *
   * \param pose_pub A callback to trigger publication of the selected pose
   * \param transform_pub A callback to trigger transform broadcast
   * \param state_pub A callback to trigger publication of the localization state
   * \param timer_factory A pointer to a timer factory to support dependency injection of timing functionality for use
   * in unit testing
   */
  LocalizationManager(PosePublisher pose_pub, TransformPublisher transform_pub, StatePublisher state_pub,
                      LocalizationManagerConfig config,
                      std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory);

  /**
   * \brief Callback for new GNSS messages
   *
   * \param msg The pose of vehicle in map frame provided by GNSS
   */
  void gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  /**
   * \brief Synced callback for the NDT reported pose and status messages
   *
   * \param pose The pose reported by NDT matching of the vehicle in the map frame
   * \param stats The stats reported by NDT matching for the accuracy of the provided pose
   */
  void poseAndStatsCallback(const geometry_msgs::PoseStampedConstPtr& pose,
                            const autoware_msgs::NDTStatConstPtr& stats);

  /**
   * \brief Callback for SystemAlert data. Used to check for lidar failure
   *
   * \param alert The alert message to evaluate
   */
  void systemAlertCallback(const cav_msgs::SystemAlertConstPtr& alert);

  /**
   * \brief Callback for the initial pose provided by Rviz or some external localization initializer
   *
   * \param msg The initial pose message
   */
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  /**
   * \brief Spin callback
   *
   * \return True if the node should continue operation
   */
  bool onSpin();

  /**
   * \brief Callback for when a new state was triggered. Allows creation of entry/exit behavior for states
   *
   * \param prev_state The previous state
   * \param new_state The new current state. This should never equal prev_state.
   * \param signal The signal that triggered the state transition
   */
  void stateTransitionCallback(LocalizationState prev_state, LocalizationState new_state, LocalizationSignal signal);

  /**
   * \brief Callback for timeouts. Used to trigger timeout signals for the state machine.
   *
   * \param event Details of the timing trigger
   * \param origin_state The state that was the current state when the timer that triggered this callback was setup
   */
  void timerCallback(const ros::TimerEvent& event, const LocalizationState origin_state);

  /**
   * \brief Returns the current localization state
   *
   * \return The current localization state
   */
  LocalizationState getState();

private:
  static const std::unordered_set<std::string> LIDAR_FAILURE_STRINGS;  // Static const container defined in cpp file

  PosePublisher pose_pub_;
  TransformPublisher transform_pub_;
  StatePublisher state_pub_;

  LocalizationManagerConfig config_;

  std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory_;

  LocalizationTransitionTable transition_table_;

  ros::Time prev_ndt_stamp_ = ros::Time(0);

  TimerUniquePtr current_timer_;
  std::vector<TimerUniquePtr> expired_timers_;

  void publishPoseStamped(const geometry_msgs::PoseStamped& pose);

  double computeNDTFreq(const ros::Time& new_stamp);
};

}  // namespace localizer

// TODO how to handle GPS failure