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

#ifndef SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_NODE_HPP_
#define SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_NODE_HPP_

#include <memory>

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "system_controller_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace system_controller
{

  class SystemControllerNode : public rclcpp::Node
  {
  public:
    SystemControllerNode() = delete;

    ~SystemControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     * \param auto_init If true this node will automatically call its initialize method. If false the call will wait for the user. 
     *                  This is meant to support unit testing
     */
    explicit SystemControllerNode(const rclcpp::NodeOptions &options, bool auto_init = true);

    /**
     * \brief Initialize this node by loading parameters from the ROS Network. 
     */
    void initialize();

    /**
     * \brief Reset the configurations of this node. This is mean to support testing or other non-standard launch mechanisms
     * 
     * \param config The config to set
     */
    void set_config(SystemControllerConfig config);

  protected:
    /**
     * \brief Callback for system alert messages used to evaluate system fault handling.
     * 
     * \param msg The message which was received 
     */
    void on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg);

    /**
     * \brief Callback to be triggered when the startup delay has passed
     */ 
    void startup_delay_callback();

    /**
     * \brief Exception handling method for processing all internal exceptions;
     */ 
    void on_error(const std::exception &e);

    /**
     * \brief Publishes a SystemAlert message to the rest of the carma-platform system.
     *        NOTE: This callback will automatically populate the msg.source_node field based on this node name.
     * \param msg The message to publish
     */
    void publish_system_alert(carma_msgs::msg::SystemAlert msg);

    //! The default topic name for the system alert topic
    const std::string system_alert_topic_{"/system_alert"};

    //! The subscriber for the system alert topic
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;

    //! System alert publisher
    std::shared_ptr<rclcpp::Publisher<carma_msgs::msg::SystemAlert>> system_alert_pub_;

    //! Lifecycle Manager which will track the managed nodes and call their lifecycle services on request
    ros2_lifecycle_manager::Ros2LifecycleManager lifecycle_mgr_;

    //! Timer which triggers when the startup delay has passed
    rclcpp::TimerBase::SharedPtr startup_timer_;

    //! The configuration of this node
    SystemControllerConfig config_;
  };

} // namespace system_controller

#endif // SYSTEM_CONTROLLER__SYSTEM_CONTROLLER_NODE_HPP_
