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
#include "localization_manager/localization_manager_node.hpp"
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>

namespace localization_manager
{
    namespace std_ph = std::placeholders;

    Node::Node(const rclcpp::NodeOptions &options)
        : carma_ros2_utils::CarmaLifecycleNode(options)

    {
        // Create initial config
        config_ = LocalizationManagerConfig();

        // Declare parameters
        config_.fitness_score_degraded_threshold = declare_parameter<double>("fitness_score_degraded_threshold", config_.fitness_score_degraded_threshold);
        config_.fitness_score_fault_threshold = declare_parameter<double>("fitness_score_fault_threshold", config_.fitness_score_fault_threshold);
        config_.ndt_frequency_degraded_threshold = declare_parameter<double>("ndt_frequency_degraded_threshold", config_.ndt_frequency_degraded_threshold);
        config_.ndt_frequency_fault_threshold = declare_parameter<double>("ndt_frequency_fault_threshold", config_.ndt_frequency_fault_threshold);
        config_.auto_initialization_timeout = declare_parameter<int>("auto_initialization_timeout", config_.auto_initialization_timeout);
        config_.gnss_only_operation_timeout = declare_parameter<int>("gnss_only_operation_timeout", config_.gnss_only_operation_timeout);
        config_.sequential_timesteps_until_gps_operation = declare_parameter<int>("sequential_timesteps_until_gps_operation", config_.sequential_timesteps_until_gps_operation);
        config_.gnss_data_timeout = declare_parameter<int>("gnss_data_timeout", config_.gnss_data_timeout);
        config_.localization_mode = declare_parameter<int>("localization_mode", config_.localization_mode);
        config_.pose_pub_rate = declare_parameter<double>("pose_pub_rate", config_.pose_pub_rate);
        config_.x_offset = declare_parameter<double>("x_offset", config_.x_offset);
        config_.y_offset = declare_parameter<double>("y_offset", config_.y_offset);
        config_.z_offset = declare_parameter<double>("z_offset", config_.z_offset);
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        // Reset config
        config_ = LocalizationManagerConfig();

        // Load parameters
        get_parameter<double>("fitness_score_degraded_threshold", config_.fitness_score_degraded_threshold);
        get_parameter<double>("fitness_score_fault_threshold", config_.fitness_score_fault_threshold);
        get_parameter<double>("ndt_frequency_degraded_threshold", config_.ndt_frequency_degraded_threshold);
        get_parameter<double>("ndt_frequency_fault_threshold", config_.ndt_frequency_fault_threshold);
        get_parameter<int>("auto_initialization_timeout", config_.auto_initialization_timeout);
        get_parameter<int>("gnss_only_operation_timeout", config_.gnss_only_operation_timeout);
        get_parameter<int>("sequential_timesteps_until_gps_operation", config_.sequential_timesteps_until_gps_operation);
        get_parameter<int>("gnss_data_timeout", config_.gnss_data_timeout);
        get_parameter<int>("localization_mode", config_.localization_mode);
        get_parameter<double>("pose_pub_rate", config_.pose_pub_rate);
        get_parameter<double>("x_offset", config_.x_offset);
        get_parameter<double>("y_offset", config_.y_offset);
        get_parameter<double>("z_offset", config_.z_offset);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("localization_manager"), "Loaded params: ");

        // Initialize worker object
        manager_.reset(new LocalizationManager(std::bind(&Node::publishPoseStamped, this, std_ph::_1),
                                               std::bind(&Node::publishStatus, this, std_ph::_1),
                                               std::bind(&Node::publishManagedInitialPose, this, std_ph::_1),
                                               config_,
                                               get_node_logging_interface(),
                                               std::make_unique<carma_ros2_utils::timers::ROSTimerFactory>(shared_from_this())));

        // Register runtime parameter update callback
        add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

        // Setup subscribers
        gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("gnss_pose", 5,
                                                                              std::bind(&LocalizationManager::gnssPoseCallback, manager_.get(), std_ph::_1));
        initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1,
                                                                                              std::bind(&LocalizationManager::initialPoseCallback, manager_.get(), std_ph::_1));

        // Setup synchronized message_filters subscribers
        rclcpp::QoS qos(5);
        ndt_pose_sub_.subscribe(this, "ndt_pose", qos.get_rmw_qos_profile());
        ndt_score_sub_.subscribe(this, "ndt_stat", qos.get_rmw_qos_profile());

        pose_stats_synchronizer_ = std::make_shared<TimeSynchronizer>(ndt_pose_sub_, ndt_score_sub_, 5);
        pose_stats_synchronizer_->registerCallback(std::bind(&Node::poseAndStatsCallback, this, std_ph::_1, std_ph::_2));

        // system_alert_topic_ protected member of CarmaLifecycleNode
        system_alert_sub_ = create_subscription<carma_msgs::msg::SystemAlert>(system_alert_topic_, 1,
                                                                              std::bind(&LocalizationManager::systemAlertCallback, manager_.get(), std_ph::_1));
        // Setup publishers
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("selected_pose", 5);
        state_pub_ = create_publisher<carma_localization_msgs::msg::LocalizationStatusReport>("localization_status", 5);

        // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
        rclcpp::PublisherOptions intra_proc_disabled;
        intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object

        auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepLast(1)); // A publisher with this QoS will store all messages that it has sent on the topic
        pub_qos_transient_local.transient_local();                       // A publisher with this QoS will re-send all (when KeepAll is used) messages to all late-joining subscribers
                                                                         // NOTE: The subscriber's QoS must be set to transient_local() as well for earlier messages to be resent to the later-joiner.
        managed_initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("managed_initialpose", pub_qos_transient_local, intra_proc_disabled);

        // Setup timer
        pose_timer_ = create_timer(get_clock(), std::chrono::milliseconds(int(1 / config_.pose_pub_rate * 1000)),
                                   std::bind(&LocalizationManager::posePubTick, manager_.get()));

        // Return success if everything initialized successfully
        return CallbackReturn::SUCCESS;
    }

    void Node::publishPoseStamped(const geometry_msgs::msg::PoseStamped &msg) const
    {
        pose_pub_->publish(msg);
    }

    void Node::publishStatus(const carma_localization_msgs::msg::LocalizationStatusReport &msg) const
    {
        state_pub_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto error = update_params<double>(
        {{"pose_pub_rate", config_.pose_pub_rate},
        {"fitness_score_degraded_threshold", config_.fitness_score_degraded_threshold},
        {"fitness_score_fault_threshold", config_.fitness_score_fault_threshold},
        {"ndt_frequency_degraded_threshold", config_.ndt_frequency_degraded_threshold},
        {"ndt_frequency_fault_threshold", config_.ndt_frequency_fault_threshold},
        {"x_offset", config_.x_offset},
        {"y_offset", config_.y_offset},
        {"z_offset", config_.z_offset}}, parameters);
        
        auto error_2 = update_params<int>(
        {{"auto_initialization_timeout", config_.auto_initialization_timeout},
        {"gnss_only_operation_timeout", config_.gnss_only_operation_timeout},
        {"gnss_data_timeout", config_.gnss_data_timeout},
        {"sequential_timesteps_until_gps_operation", config_.sequential_timesteps_until_gps_operation}
        }, parameters);

        rcl_interfaces::msg::SetParametersResult result;

        result.successful = !error && !error_2;

        if (result.successful)
        {
            manager_->setConfig(config_);
        }

        return result;
    }

    void Node::publishManagedInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg) const
    {
        managed_initial_pose_pub_->publish(msg);
    }

    void Node::poseAndStatsCallback(const geometry_msgs::msg::PoseStamped::ConstPtr pose,
                                    const autoware_msgs::msg::NDTStat::ConstPtr stats)
    {
        try
        {
            manager_->poseAndStatsCallback(pose, stats);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("localization_manager"), "Uncaught Exception in localization_manager. Exception: " << e.what());
            handle_primary_state_exception(e);
        }
    }
} // namespace localization_manager

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(localization_manager::Node)
