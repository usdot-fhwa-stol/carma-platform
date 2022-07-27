/*
 * Copyright (C) 2018-2022 LEIDOS.
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
#include <carma_planning_msgs/srv/set_guidance_active.hpp>
#include <carma_driver_msgs/srv/set_enable_robotic.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_driver_msgs/msg/robot_enabled.hpp>
#include <carma_planning_msgs/msg/route_event.hpp>
#include <autoware_msgs/msg/vehicle_status.hpp>
#include "guidance/guidance_state_machine.hpp"

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "guidance/guidance_config.hpp"

namespace guidance
{

  /**
   * \brief Worker class for the Guidance node
   * 
   */
  class GuidanceWorker : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_driver_msgs::msg::RobotEnabled> robot_status_subscriber_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::RouteEvent> route_event_subscriber_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::VehicleStatus> vehicle_status_subscriber_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::GuidanceState> state_publisher_;

    // Service Clients
    carma_ros2_utils::ClientPtr<carma_driver_msgs::srv::SetEnableRobotic> enable_client_;

    // Service Servers
    carma_ros2_utils::ServicePtr<carma_planning_msgs::srv::SetGuidanceActive> guidance_activate_service_server_;

    // Timers
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Node configuration
    Config config_;

    // Guidance state machine
    GuidanceStateMachine gsm_;

  public:
    /**
     * \brief GuidanceWorker constructor 
     */
    explicit GuidanceWorker(const rclcpp::NodeOptions &);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Timer callback
     */
    bool spin_cb();

    /**
      * \brief Callback for route event messages
      * \param msg Latest route event message 
      */
    void route_event_cb(carma_planning_msgs::msg::RouteEvent::UniquePtr msg);

    /**
      * \brief Callback for robot enabled messages
      * \param msg Latest robot enabled message
      */
    void robot_status_cb(carma_driver_msgs::msg::RobotEnabled::UniquePtr msg);

    /**
      * \brief Callback for vehicle status messages
      * \param msg Latest vehicle status message
      */
    void vehicle_status_cb(const autoware_msgs::msg::VehicleStatus::UniquePtr msg);

    /**
     * \brief Set_guidance_active service callback. User can attempt to activate the guidance system by calling this service
     * \param req A carma_planning_msgs::srv::SetGuidanceActive::Request msg 
     * \param resp A carma_planning_msgs::srv::SetGuidanceActive::Response msg 
     */
    bool guidance_activation_cb(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_planning_msgs::srv::SetGuidanceActive::Request> req,
                                std::shared_ptr<carma_planning_msgs::srv::SetGuidanceActive::Response> resp);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &prev_state);
  };

} // guidance
