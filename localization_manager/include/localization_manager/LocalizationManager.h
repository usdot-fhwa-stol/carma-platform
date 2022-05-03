#pragma once
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

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <boost/optional.hpp>
#include <autoware_msgs/msg/ndt_stat.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_localization_msgs/msg/localization_status_report.hpp>
#include <functional>
#include <unordered_set>
#include <carma_utils/timers/Timer.h>
#include <carma_utils/timers/TimerFactory.h>
#include "LocalizationTypes.h"
#include "LocalizationManagerConfig.h"
#include "LocalizationTransitionTable.h"

namespace localization_manager
{
/**
 * \brief Primary logic class for the localization manager node.
 */
class LocalizationManager
{
public:
  using PosePublisher = std::function<void(const geometry_msgs::msg::PoseStamped&)>;
  using ManagedInitialPosePublisher = std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped&)>;
  using StatePublisher = std::function<void(const carma_localization_msgs::msg::LocalizationStatusReport&)>;
  using TimerUniquePtr = std::unique_ptr<carma_utils::timers::Timer>;


  /**
   * \brief Constructor
   *
   * \param pose_pub A callback to trigger publication of the selected pose
   * \param state_pub A callback to trigger publication of the localization state
   * \param initialpose_pub A callback to trigger publication of the intial pose
   * \param config The configuration settings to use for this manager
   * \param timer_factory A pointer to a timer factory to support dependency injection of timing functionality for use
   * in unit testing
   */
  LocalizationManager(PosePublisher pose_pub, StatePublisher state_pub,
                      ManagedInitialPosePublisher initialpose_pub,
                      const LocalizationManagerConfig& config,
                      std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory);

  /**
   * \brief Callback for new GNSS messages
   *
   * \param msg The pose of vehicle in map frame provided by GNSS
   */
  void gnssPoseCallback(const geometry_msgs::msg::PoseStampedConstPtr& msg);

  /**
   * \brief Synced callback for the NDT reported pose and status messages
   *
   * \param pose The pose reported by NDT matching of the vehicle in the map frame
   * \param stats The stats reported by NDT matching for the accuracy of the provided pose
   */
  void poseAndStatsCallback(const geometry_msgs::msg::PoseStampedConstPtr& pose,
                            const autoware_msgs::msg::NDTStatConstPtr& stats);

  /**
   * \brief Callback for SystemAlert data. Used to check for lidar failure
   *
   * \param alert The alert message to evaluate
   */
  void systemAlertCallback(const carma_msgs::msg::SystemAlertConstPtr& alert);

  /**
   * \brief Callback for the initial pose provided by Rviz or some external localization initializer
   *
   * \param msg The initial pose message
   */
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStampedConstPtr& msg);

  /**
   * \brief Timer callback that controls the publication of the selected pose and status report
   *
   */
  void posePubTick(const ros::TimerEvent& te);

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
  LocalizationState getState() const;

private:
  //! The set of strings which mark a lidar failure in a system alert message
  static const std::unordered_set<std::string> LIDAR_FAILURE_STRINGS;  // Static const container defined in cpp file

  PosePublisher pose_pub_;
  StatePublisher state_pub_;
  ManagedInitialPosePublisher initialpose_pub_;

  LocalizationManagerConfig config_;

  std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory_;

  LocalizationTransitionTable transition_table_;

  boost::optional<ros::Time> prev_ndt_stamp_;

  TimerUniquePtr current_timer_;
  int lidar_init_sequential_timesteps_counter_ = 0;
  bool is_sequential_ = false;
  std::vector<TimerUniquePtr> expired_timers_;
  boost::optional<geometry_msgs::msg::PoseStamped> last_raw_gnss_value_;
  boost::optional<tf2::Vector3> gnss_offset_;
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;

  /**
   * \brief Helper function to both compute the NDT Frequency and update the previous pose timestamp
   *
   * \param new_stamp The new pose timestamp
   *
   * \return The computed instantaneous frequency in Hz
   */
  double computeNDTFreq(const ros::Time& new_stamp);

  /**
   * \brief Helper function to compute the instantaneous frequency between two times
   *
   * \param old_stamp The old timestamp
   * \param new_stamp The new timestamp
   *
   * \return The computed instantaneous frequency in Hz
   */
  double computeFreq(const ros::Time& old_stamp, const ros::Time& new_stamp) const;
};

}  // namespace localization_manager
