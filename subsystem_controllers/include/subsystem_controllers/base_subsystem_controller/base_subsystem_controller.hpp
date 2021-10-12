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

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller_config.hpp"

namespace subsystem_controllers
{

  class BaseSubsystemController : public carma_ros2_utils::CarmaLifecycleNode
  {
  public:
    
    BaseSubsystemController() = delete;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit BaseSubsystemController(const rclcpp::NodeOptions &options);

    ~BaseSubsystemController() = default;

    void set_config(BaseSubSystemControllerConfig config);

    void on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg);

    ////
    // Overrides
    ////
    virtual carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    virtual carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &prev_state);
    virtual carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &prev_state);
    virtual carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &prev_state);
    virtual carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &prev_state, const std::string &exception_string);
    virtual carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &prev_state);

  protected:

    std::vector<std::string> get_nodes_in_namespace(const std::string& node_namespace) const;
    std::vector<std::string> get_non_intersecting_set(const std::vector<std::string>& superset, const std::vector<std::string>& subset) const;

    //! Lifecycle Manager which will track the managed nodes and call their lifecycle services on request
    ros2_lifecycle_manager::Ros2LifecycleManager lifecycle_mgr_;

    //! The subscriber for the system alert topic
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;

    //! The configuration struct
    BaseSubSystemControllerConfig base_config_;
  };

} // namespace subsystem_controllers

