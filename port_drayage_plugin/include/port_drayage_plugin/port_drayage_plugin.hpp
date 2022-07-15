/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_planning_msgs/msg/route_event.hpp>
#include <carma_planning_msgs/srv/set_active_route.hpp>
#include <carma_msgs/msg/ui_instructions.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "port_drayage_plugin/port_drayage_plugin_config.hpp"
#include "port_drayage_plugin/port_drayage_worker.hpp"

namespace port_drayage_plugin
{

  /**
   * Primary Port Drayage Plugin implementation class. Split into this class
   * primarily concerned with the handling of ROS message processing and the
   * PortDrayageWorker class responsible for handling the core business logic
   * of the Port Drayage functionality.
   */ 
  class PortDrayagePlugin : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> pose_subscriber_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityOperation> inbound_mobility_operation_subscriber_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_subscriber_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::RouteEvent> route_event_subscriber_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_subscriber_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> outbound_mobility_operations_publisher_;
    carma_ros2_utils::PubPtr<carma_msgs::msg::UIInstructions> ui_instructions_publisher_;

    // Service Clients
    carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::SetActiveRoute> set_active_route_client_;

    // Node configuration
    Config config_;

    // PortDrayageWorker object
    PortDrayageWorker pdw_;

  public:
    /**
     * \brief PortDrayagePlugin constructor 
     */
    explicit PortDrayagePlugin(const rclcpp::NodeOptions &);

    /**
     * \brief Calls the /guidance/set_active_route service client to set an active route 
     * \param req The service request being used to call the service client
     * \return If the service client call was successful and no errors occurred while setting the new active route
     */
    bool callSetActiveRouteClient(std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> req);

    /**
     * \brief Publishes a Mobility Operation message to the outgoing mobility operation topic
     * \param msg The Mobility Operation message to be published
     */
    void publishMobilityOperation(const carma_v2x_msgs::msg::MobilityOperation& msg);

    /**
     * \brief Publishes a UI Instructions message to the outgoing UI Instructions topic
     * \param msg The UI Instructions message to be published
     */
    void publishUIInstructions(const carma_msgs::msg::UIInstructions& msg);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
  };

} // port_drayage_plugin
