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

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <boost/optional.hpp>
#include <autoware_msgs/msg/ndt_stat.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_localization_msgs/msg/localization_status_report.hpp>
#include <functional>
#include <unordered_set>
#include <carma_ros2_utils/timers/Timer.hpp>
#include <carma_ros2_utils/timers/TimerFactory.hpp>
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>
#include "localization_manager/LocalizationTypes.hpp"
#include "localization_manager/LocalizationManagerConfig.hpp"
#include "localization_manager/LocalizationTransitionTable.hpp"

namespace localization_manager
{
    /**
     * \brief Primary logic class for the localization manager node.
     */
    class LocalizationManager
    {

    public:
        using PosePublisher = std::function<void(const geometry_msgs::msg::PoseStamped &)>;
        using ManagedInitialPosePublisher = std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped &)>;
        using StatePublisher = std::function<void(const carma_localization_msgs::msg::LocalizationStatusReport &)>;
        using TimerUniquePtr = std::unique_ptr<carma_ros2_utils::timers::Timer>;

        /**
         * \brief Constructor
         *
         * \param pose_pub A callback to trigger publication of the selected pose
         * \param state_pub A callback to trigger publication of the localization state
         * \param initialpose_pub A callback to trigger publication of the intial pose
         * \param config The configuration settings to use for this manager
         * \param logger  Logger interface of node calling manager object
         * \param node_timers Timer interface of node calling manager object
         */
        LocalizationManager(PosePublisher pose_pub, StatePublisher state_pub,
                            ManagedInitialPosePublisher initialpose_pub,
                            const LocalizationManagerConfig &config,
                            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
                            std::unique_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory);

        /**
         * \brief Callback for new GNSS messages
         *
         * \param msg The pose of vehicle in map frame provided by GNSS
         */
        void gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * \brief Synced callback for the NDT reported pose and status messages
         *
         * \param pose The pose reported by NDT matching of the vehicle in the map frame
         * \param stats The stats reported by NDT matching for the accuracy of the provided pose
         */
        void poseAndStatsCallback(const geometry_msgs::msg::PoseStamped::ConstPtr pose,
                                  const autoware_msgs::msg::NDTStat::ConstPtr stats);

        /**
         * \brief Callback for SystemAlert data. Used to check for lidar failure
         *
         * \param alert The alert message to evaluate
         */
        void systemAlertCallback(const carma_msgs::msg::SystemAlert::SharedPtr alert);

        /**
         * \brief Callback for the initial pose provided by Rviz or some external localization initializer
         *
         * \param msg The initial pose message
         */
        void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        /**
         * \brief Timer callback that controls the publication of the selected pose and status report
         *
         */
        void posePubTick();

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
         * \param origin_state The state that was the current state when the timer that triggered this callback was setup
         */
        void timerCallback(const LocalizationState origin_state);

        /**
         * \brief Returns the current localization state
         *
         * \return The current localization state
         */
        LocalizationState getState() const;

        /**
         * @brief Clears the expired timers from the memory of this scheduler
         */
        void clearTimers();

        /**
         * \brief Set config.
         * 
         * \param config localization manager config
         */
        void setConfig(const LocalizationManagerConfig& config);

    private:
        //! The set of strings which mark a lidar failure in a system alert message
        static const std::unordered_set<std::string> LIDAR_FAILURE_STRINGS; // Static const container defined in cpp file

        PosePublisher pose_pub_;
        StatePublisher state_pub_;
        ManagedInitialPosePublisher initialpose_pub_;

        LocalizationManagerConfig config_;

        LocalizationTransitionTable transition_table_;

        boost::optional<rclcpp::Time> prev_ndt_stamp_;

        int lidar_init_sequential_timesteps_counter_ = 0;
        bool is_sequential_ = false;
        boost::optional<geometry_msgs::msg::PoseStamped> last_raw_gnss_value_;
        boost::optional<tf2::Vector3> gnss_offset_;
        boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;

        // Logger interface
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

        // Using timer factory
        std::mutex mutex_;
        std::unique_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory_;
        std::unordered_map<uint32_t, std::pair<TimerUniquePtr, bool>> timers_;
        TimerUniquePtr current_timer_;
        uint32_t next_id_ = 0; // Timer id counter
        uint32_t current_timer_id_;
        rcl_clock_type_t timer_clock_type_ = RCL_SYSTEM_TIME;

        /**
         * \brief Helper function to both compute the NDT Frequency and update the previous pose timestamp
         *
         * \param new_stamp The new pose timestamp
         *
         * \return The computed instantaneous frequency in Hz
         */
        double computeNDTFreq(const rclcpp::Time &new_stamp);

        /**
         * \brief Helper function to compute the instantaneous frequency between two times
         *
         * \param old_stamp The old timestamp
         * \param new_stamp The new timestamp
         *
         * \return The computed instantaneous frequency in Hz
         */
        double computeFreq(const rclcpp::Time &old_stamp, const rclcpp::Time &new_stamp) const;

        /**
         * @brief Generates the next id to be used for a timer
         *
         * @return The next available timer id
         */
        uint32_t nextId();
    };
}