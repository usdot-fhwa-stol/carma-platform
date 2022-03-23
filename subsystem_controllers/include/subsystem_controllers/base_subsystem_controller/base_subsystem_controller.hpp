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
  /**
   * \brief A base class for all subsystem_controllers which provides default lifecycle behavior for subsystems
   * 
   * The default behavior which is provided is as follows
   *  - Takes in a list of required nodes and a namespace
   *  - Manages the lifecycle of all nodes which are the union of the required nodes and the namespace
   *  - Monitors the system_alert topic and if a node within its required node set crashes it notifies the larger system that the subsystem has failed  
   */ 
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

    virtual void on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg);

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

    /**
     * \brief Returns the list of fully qualified node names for all ROS2 nodes in the provided namespace
     * 
     * \param node_namespace The ros namespace to get all nodes within. For example /guidance
     * 
     * \return The list of node names
     */ 
    std::vector<std::string> get_nodes_in_namespace(const std::string& node_namespace) const;

    /**
     * \brief Returns all elements of the provided set_a which are NOT contained in the provided set_b
     * 
     * \brief set_a The set of strings which will have its intersection checked against
     * \brief set_b The set of strings which will NOT be in the returned set
     * 
     * \return A set of not intersecting strings which are in set_a but not set_b
     */ 
    std::vector<std::string> get_non_intersecting_set(const std::vector<std::string>& set_a, const std::vector<std::string>& set_b) const;

    //! Lifecycle Manager which will track the managed nodes and call their lifecycle services on request
    ros2_lifecycle_manager::Ros2LifecycleManager lifecycle_mgr_;

    //! The subscriber for the system alert topic
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;

    //! The configuration struct
    BaseSubSystemControllerConfig base_config_;
  };

} // namespace subsystem_controllers

