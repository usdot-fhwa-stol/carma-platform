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
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "localization_manager/LocalizationManagerConfig.h"
#include "LocalizationManager.h"

namespace localization_manager
{

  /**
   * \brief Core execution node for this package
   * 
   */
  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> ndt_pose_sub_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::NDTStat> ndt_score_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> gnss_pose_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> initialpose_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseStamped> pose_pub_;
    carma_ros2_utils::PubPtr<carma_localization_msgs::msg::LocalizationStatusReport> state_pub_;
    carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> managed_initial_pose_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr pose_timer_;

    // Node configuration
    Config config_;

    std::unique_ptr<LocalizationManager> manager_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * \brief Example callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Example timer callback
     */
    void example_timer_callback();

    /**
     * \brief Example subscription callback
     */
    void example_callback(std_msgs::msg::String::UniquePtr msg);

    /**
      * \brief Example service callback
      */
    void example_service_callback(const std::shared_ptr<rmw_request_id_t> header,
                                  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                  std::shared_ptr<std_srvs::srv::Empty::Response> response);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    /**
     * TODO for USER: The following lifecycle overrides are also available if needed
     * handle_on_activate
     * handle_on_deactivate
     * handle_on_cleanup
     * handle_on_shutdown
     * handle_on_error
     */
  };

} // localization_manager
