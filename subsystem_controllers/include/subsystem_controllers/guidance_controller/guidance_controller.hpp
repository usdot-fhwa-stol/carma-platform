#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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



#include <memory>

#include <carma_msgs/msg/system_alert.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/get_plugin_api.hpp>
#include <carma_planning_msgs/srv/plugin_list.hpp>
#include <carma_planning_msgs/srv/plugin_activation.hpp>
#include <ros2_lifecycle_manager/ros2_lifecycle_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller.hpp"
#include "plugin_manager.h"
#include "guidance_controller_config.hpp"

namespace subsystem_controllers
{
  namespace cr2 = carma_ros2_utils;

  class GuidanceControllerNode : public BaseSubsystemController
  {
  public:
    
    GuidanceControllerNode() = delete;

    ~GuidanceControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit GuidanceControllerNode(const rclcpp::NodeOptions &options);

    cr2::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    cr2::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

    cr2::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &);

    cr2::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &);

    cr2::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &);

  private:
    //! Plugin manager to handle all the plugin specific discovery and reporting
    std::shared_ptr<PluginManager> plugin_manager_; 

    //! Config for user provided parameters
    GuidanceControllerConfig config_;

    //! ROS handles

    cr2::SubPtr<carma_planning_msgs::msg::Plugin> plugin_discovery_sub_;

    cr2::ServicePtr<carma_planning_msgs::srv::PluginList> get_registered_plugins_server_;

    cr2::ServicePtr<carma_planning_msgs::srv::PluginList> get_active_plugins_server_;

    cr2::ServicePtr<carma_planning_msgs::srv::PluginActivation> activate_plugin_server_;

    cr2::ServicePtr<carma_planning_msgs::srv::GetPluginApi> get_strategic_plugins_by_capability_server_;

    cr2::ServicePtr<carma_planning_msgs::srv::GetPluginApi> get_tactical_plugins_by_capability_server_;

    cr2::ServicePtr<carma_planning_msgs::srv::GetPluginApi> get_control_plugins_by_capability_server_;

  };

} // namespace v2x_controller

