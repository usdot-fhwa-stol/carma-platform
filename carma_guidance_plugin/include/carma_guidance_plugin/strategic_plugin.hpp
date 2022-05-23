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
#include <carma_planning_msgs/msg/plugin.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_guidance_plugin
{

  /**
   * \brief StrategicPlugin provides default functionality for all carma guidance plugins.
   *        This includes basic state machine management (largely delegated to lifecycle behavior), required interfaces, and plugin discovery
   * 
   */
  class StrategicPlugin : public PluginBaseNode
  {

  private:
    // Services
    carma_ros2_utils::ServicePtr<carma_planning_msgs::msg::PlanManeuvers> plan_maneuvers_service_;

  public:
    /**
     * \brief StrategicPlugin constructor 
     */
    explicit StrategicPlugin(const rclcpp::NodeOptions &);

    // TODO comments
    virtual void plan_maneuvers_callback(
      std::shared_ptr<rmw_request_id_t> srv_header, 
      carma_planning_msgs::srv::PlanManeuvers::Request &req, 
      carma_planning_msgs::srv::PlanManeuvers::Response &resp) = 0;
    

    ////
    // Overrides
    ////

    // Non-Final
    std::string get_capability() override;

    // Final
    uint8_t get_type() override final;

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &) override final; 
    carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &) override final;
  };

} // carma_guidance_plugin
