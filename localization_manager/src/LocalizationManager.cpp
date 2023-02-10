/*
 * Copyright (C) 2022 LEIDOS.
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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include "localization_manager/LocalizationManager.hpp"

namespace localization_manager
{
    // Initialize static values
    const std::unordered_set<std::string> LocalizationManager::LIDAR_FAILURE_STRINGS({"One LIDAR Failed", "Both LIDARS Failed"});

    LocalizationManager::LocalizationManager(PosePublisher pose_pub,
                                             StatePublisher state_pub, ManagedInitialPosePublisher initialpose_pub,
                                             const LocalizationManagerConfig &config,
                                             rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
                                             std::unique_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory)
        : pose_pub_(pose_pub), state_pub_(state_pub), initialpose_pub_(initialpose_pub), config_(config), logger_(logger), timer_factory_(std::move(timer_factory)), transition_table_(static_cast<LocalizerMode>(config_.localization_mode))

    {

        transition_table_.setTransitionCallback(std::bind(&LocalizationManager::stateTransitionCallback, this,
                                                          std::placeholders::_1, std::placeholders::_2,
                                                          std::placeholders::_3));
        timer_clock_type_ = timer_factory_->now().get_clock_type();
    }

    double LocalizationManager::computeFreq(const rclcpp::Time &old_stamp, const rclcpp::Time &new_stamp) const
    {
        return 1.0 / (new_stamp - old_stamp).seconds(); // Convert delta to frequency (Hz = 1/s)
    }

    void LocalizationManager::setConfig(const LocalizationManagerConfig& config)
    {
        config_ = config;
    }

    double LocalizationManager::computeNDTFreq(const rclcpp::Time &new_stamp)
    {
        if (!prev_ndt_stamp_)
        { // Check if this is the first data point
            prev_ndt_stamp_ = new_stamp;
            // When no historic data is available force the frequency into the operational range
            return config_.ndt_frequency_degraded_threshold * 2;
        }

        if (new_stamp <= rclcpp::Time(prev_ndt_stamp_.get(), timer_clock_type_))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("localization_manager"), "LocalizationManager received NDT data out of order. Prev stamp was "
                                                                                << prev_ndt_stamp_.get().seconds() << " new stamp is " << new_stamp.seconds());
            // When invalid data is received from NDT force the frequency into the fault range
            return config_.ndt_frequency_fault_threshold / 2;
        }
        return computeFreq(rclcpp::Time(prev_ndt_stamp_.get(), timer_clock_type_), new_stamp); // Convert delta to frequency (Hz = 1/s)
    }

    void LocalizationManager::poseAndStatsCallback(const geometry_msgs::msg::PoseStamped::ConstPtr pose,
                                                   const autoware_msgs::msg::NDTStat::ConstPtr stats)
    {
        double ndt_freq = computeNDTFreq(rclcpp::Time(pose->header.stamp, timer_clock_type_));
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "Received pose resulting in frequency value of " << ndt_freq << " with score of " << stats->score);

        if (stats->score >= config_.fitness_score_fault_threshold || ndt_freq <= config_.ndt_frequency_fault_threshold)
        {
            transition_table_.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
        }
        else if (stats->score >= config_.fitness_score_degraded_threshold || ndt_freq <= config_.ndt_frequency_degraded_threshold)
        {
            transition_table_.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
        }
        else
        {
            // In GNSS_WITH_NDT_INIT mode, we shouldn't switch back to NDT and only rely on GPS
            if (!(static_cast<LocalizerMode>(config_.localization_mode) == LocalizerMode::GNSS_WITH_NDT_INIT &&
                  transition_table_.getState() == LocalizationState::DEGRADED_NO_LIDAR_FIX &&
                  lidar_init_sequential_timesteps_counter_ >= config_.sequential_timesteps_until_gps_operation))
            {
                transition_table_.signal(LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE);
            }
        }

        const LocalizationState state = transition_table_.getState();

        if (static_cast<LocalizerMode>(config_.localization_mode) == LocalizerMode::GNSS_WITH_NDT_INIT && state == LocalizationState::OPERATIONAL && last_raw_gnss_value_)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "lidar_init_sequential_timesteps_counter_: " << lidar_init_sequential_timesteps_counter_);
            if (lidar_init_sequential_timesteps_counter_ < config_.sequential_timesteps_until_gps_operation)
            {
                if (is_sequential_)
                    lidar_init_sequential_timesteps_counter_++;

                is_sequential_ = true;
            }
            else
            {
                transition_table_.signal(LocalizationSignal::LIDAR_INITIALIZED_SWITCH_TO_GPS);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "Switched to LIDAR_INITIALIZED_SWITCH_TO_GPS at lidar_init_sequential_timesteps_counter_: " << lidar_init_sequential_timesteps_counter_
                                                                                                                                                                            << ", with new state: " << transition_table_.getState());
            }
        }
        else if (!(static_cast<LocalizerMode>(config_.localization_mode) == LocalizerMode::GNSS_WITH_NDT_INIT &&
                   state == LocalizationState::DEGRADED_NO_LIDAR_FIX &&
                   last_raw_gnss_value_ &&
                   lidar_init_sequential_timesteps_counter_ >= config_.sequential_timesteps_until_gps_operation)) // don't reset counter if Lidar initialized and switched to GPS
                                                                                                                  // as it will reset state to OPERATIONAL
        {
            is_sequential_ = false;
            lidar_init_sequential_timesteps_counter_ = 0;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "Resetting lidar_init_sequential_timesteps_counter_: " << lidar_init_sequential_timesteps_counter_ << ", with new state: " << transition_table_.getState());
        }

        if (state == LocalizationState::OPERATIONAL && last_raw_gnss_value_)
        {
            tf2::Vector3 ndt_translation(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);

            tf2::Vector3 gnss_translation(last_raw_gnss_value_->pose.position.x, last_raw_gnss_value_->pose.position.y, last_raw_gnss_value_->pose.position.z);

            gnss_offset_ = ndt_translation - gnss_translation;
        }

        if (state != LocalizationState::DEGRADED_NO_LIDAR_FIX)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "Publishing mixed pose with lidar_init_sequential_timesteps_counter_: " << lidar_init_sequential_timesteps_counter_ << ", and state" << state);
            current_pose_ = *pose;
        }

        prev_ndt_stamp_ = pose->header.stamp;
    }

    void LocalizationManager::gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_raw_gnss_value_ = *msg;
        // Just like ndt_matching the gnss pose is treated as an initialize signal if the system is not yet intialized
        if (transition_table_.getState() == LocalizationState::UNINITIALIZED)
        {
            lidar_init_sequential_timesteps_counter_ = 0;
            transition_table_.signal(LocalizationSignal::INITIAL_POSE);

            geometry_msgs::msg::PoseWithCovarianceStamped new_initial_pose;
            new_initial_pose.header = msg->header;
            new_initial_pose.pose.pose = msg->pose;

            initialpose_pub_(new_initial_pose);
        }

        if (transition_table_.getState() == LocalizationState::DEGRADED_NO_LIDAR_FIX)
        {
            geometry_msgs::msg::PoseStamped corrected_pose = *msg;
            if (gnss_offset_)
            {
                corrected_pose.pose.position.x = corrected_pose.pose.position.x + gnss_offset_->x();
                corrected_pose.pose.position.y = corrected_pose.pose.position.y + gnss_offset_->y();
                corrected_pose.pose.position.z = corrected_pose.pose.position.z + gnss_offset_->z();
            }
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "Publishing GNSS pose with lidar_init_sequential_timesteps_counter_: " << lidar_init_sequential_timesteps_counter_);
            current_pose_ = corrected_pose;
        }
    }

    void LocalizationManager::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        lidar_init_sequential_timesteps_counter_ = 0;
        transition_table_.signal(LocalizationSignal::INITIAL_POSE);
        current_pose_ = boost::none; // Reset the current pose until a new pose is recieved
        initialpose_pub_(*msg);      // Forward the initial pose to the rest of the system
    }

    void LocalizationManager::systemAlertCallback(const carma_msgs::msg::SystemAlert::SharedPtr alert)
    {
        if (LIDAR_FAILURE_STRINGS.find(alert->description) != LIDAR_FAILURE_STRINGS.end())
        {
            transition_table_.signal(LocalizationSignal::LIDAR_SENSOR_FAILURE);
        }
    }

    void LocalizationManager::timerCallback(const LocalizationState origin_state)
    {
        // If there is already a timer callback in progress or the expected state has changed then return
        if (origin_state != transition_table_.getState())
        {
            return;
        }

        transition_table_.signal(LocalizationSignal::TIMEOUT);
    }

    uint32_t LocalizationManager::nextId()
    {
        next_id_++;
        return next_id_;
    }

    void LocalizationManager::clearTimers()
    {
        std::lock_guard<std::mutex> guard(mutex_);

        // Clear expired timers
        auto it = timers_.begin();
        while (it != timers_.end())
        {
            // Check if timer is marked for deletion
            if (it->second.second)
            {
                // erase() function returns the iterator of the next to last deleted element
                it = timers_.erase(it);
            }
            else
            {
                it++;
            }
        }
    }

    void LocalizationManager::stateTransitionCallback(LocalizationState prev_state, LocalizationState new_state,
                                                      LocalizationSignal signal)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("localization_manager"), "State transition from " << prev_state << " to " << new_state << " with signal " << signal);
        // Mark the expired timers as expired if any
        if (current_timer_)
        {
            // NOTE unit testing of current algorithm depends on all timers being one-shot timers.
            timers_[current_timer_id_].second = true;
        }

        switch (new_state)
        {
        case LocalizationState::INITIALIZING:
            gnss_offset_ = boost::none;
            prev_ndt_stamp_ = boost::none;

            current_timer_id_ = nextId();
            current_timer_ = timer_factory_->buildTimer(current_timer_id_, rclcpp::Duration(config_.auto_initialization_timeout * 1e6),
                                                        std::bind(&LocalizationManager::timerCallback, this, new_state), true, true);

            timers_[current_timer_id_] = std::make_pair(std::move(current_timer_), false); // Add start timer to map by Id
            break;
        case LocalizationState::DEGRADED_NO_LIDAR_FIX:
            current_timer_id_ = nextId();
            current_timer_ = timer_factory_->buildTimer(current_timer_id_, rclcpp::Duration(config_.gnss_only_operation_timeout * 1e6),
                                                        std::bind(&LocalizationManager::timerCallback, this, new_state), true, true);

            timers_[current_timer_id_] = std::make_pair(std::move(current_timer_), false); // Add start timer to map by Id

            break;
        default:
            break;
        }
    }

    void LocalizationManager::posePubTick()
    {
        // Clear any expired timers
        clearTimers();

        // Evaluate NDT Frequency if we have started receiving messages
        // This check provides protection against excessively long NDT computation times that do not trigger the callback
        if (prev_ndt_stamp_)
        {
            double freq = computeFreq(rclcpp::Time(prev_ndt_stamp_.get(), timer_clock_type_), timer_factory_->now());
            if (freq <= config_.ndt_frequency_fault_threshold)
            {
                transition_table_.signal(LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE);
            }
            else if (freq <= config_.ndt_frequency_degraded_threshold)
            {
                transition_table_.signal(LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE);
            }
        }

        // check if last gnss time stamp is older than threshold and send the corresponding signal
        if (last_raw_gnss_value_ && timer_factory_->now() - rclcpp::Time(last_raw_gnss_value_->header.stamp, timer_clock_type_) > rclcpp::Duration(config_.gnss_data_timeout * 1e6))
        {
            transition_table_.signal(LocalizationSignal::GNSS_DATA_TIMEOUT);
        }

        // Used in LocalizerMode::GNSS_WITH_NDT_INIT
        // If the state is not Operational with good NDT, or already using GPS only, we reset the counter
        if (transition_table_.getState() != LocalizationState::OPERATIONAL &&
            transition_table_.getState() != LocalizationState::DEGRADED_NO_LIDAR_FIX)
        {
            lidar_init_sequential_timesteps_counter_ = 0;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("localization_manager"), "Resetting lidar_init_sequential_timesteps_counter_: " << lidar_init_sequential_timesteps_counter_ << ", with new state: " << transition_table_.getState());
        }

        // Publish current pose message if available
        if (current_pose_)
        {
            auto pose_to_publish = *current_pose_;
            if (static_cast<LocalizerMode>(config_.localization_mode) == LocalizerMode::GNSS_WITH_FIXED_OFFSET)
            {
                pose_to_publish.pose.position.x += config_.x_offset;
                pose_to_publish.pose.position.y += config_.y_offset;
                pose_to_publish.pose.position.z += config_.z_offset;
            }
            pose_pub_(pose_to_publish);
        }

        // Create and publish status report message
        carma_localization_msgs::msg::LocalizationStatusReport msg = stateToMsg(transition_table_.getState(), timer_factory_->now());
        state_pub_(msg);
    }

    LocalizationState LocalizationManager::getState() const
    {
        return transition_table_.getState();
    }

}