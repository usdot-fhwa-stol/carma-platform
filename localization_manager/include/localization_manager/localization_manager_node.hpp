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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "localization_manager/LocalizationManagerConfig.hpp"
#include "localization_manager/LocalizationManager.hpp"
#include "localization_manager/LocalizationTypes.hpp"
#include <message_filters_humble/subscriber.h>
#include <message_filters_humble/time_synchronizer.h>
#include <message_filters_humble/sync_policies/exact_time.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <boost/optional.hpp>
#include <autoware_msgs/msg/ndt_stat.hpp>

namespace localization_manager
{
    /**
     * \brief Core execution node for this package
     */

    class Node : public carma_ros2_utils::CarmaLifecycleNode
    {
    private:
        // Subscribers
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped, Node> ndt_pose_sub_;
        message_filters::Subscriber<autoware_msgs::msg::NDTStat, Node> ndt_score_sub_;
        carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> gnss_pose_sub_;
        carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> initialpose_sub_;
        carma_ros2_utils::SubPtr<carma_msgs::msg::SystemAlert> system_alert_sub_;

        // Publishers
        carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseStamped> pose_pub_;
        carma_ros2_utils::PubPtr<carma_localization_msgs::msg::LocalizationStatusReport> state_pub_;
        carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> managed_initial_pose_pub_;

        // Timers
        rclcpp::TimerBase::SharedPtr pose_timer_;

        // Node configuration
        LocalizationManagerConfig config_;

        // Worker object
        std::unique_ptr<LocalizationManager> manager_;

        // Message filters policies (TimeSynchronizer by default uses ExactTime Policy)
        typedef message_filters::TimeSynchronizer<geometry_msgs::msg::PoseStamped, autoware_msgs::msg::NDTStat> TimeSynchronizer;
        std::shared_ptr<TimeSynchronizer> pose_stats_synchronizer_;

    public:
        /**
         * \brief Node constructor
         */
        explicit Node(const rclcpp::NodeOptions &);

        /**
         * \brief Callback to publish the selected pose
         * \param msg The pose to publish
         */
        void publishPoseStamped(const geometry_msgs::msg::PoseStamped &msg) const;

        /**
         * \brief Callback to publish the provided localization status report
         * \param msg The report to publish
         */
        void publishStatus(const carma_localization_msgs::msg::LocalizationStatusReport &msg) const;

        /**
         * \brief Callback to publish the initial pose deemed suitable to intialize NDT
         * \param msg The msg to publish
         */
        void publishManagedInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg) const;

        /**
         * \brief Synchronized callback for pose and stats data for usage with message_filters.
         *        Provides exception handling.
         * \param pose The received pose message
         * \param stats The received stats message
         */
        void poseAndStatsCallback(const geometry_msgs::msg::PoseStamped::ConstPtr pose,
                                  const autoware_msgs::msg::NDTStat::ConstPtr stats);

         /**
         * \brief Callback for dynamic parameter updates
         */
        rcl_interfaces::msg::SetParametersResult 
        parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);
        
        ////
        // Overrides
        ////
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    };
}