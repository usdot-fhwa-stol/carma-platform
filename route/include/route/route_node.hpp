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
#include <carma_planning_msgs/msg/route.hpp>
#include <carma_planning_msgs/msg/route_state.hpp>
#include <carma_planning_msgs/msg/route_event.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <carma_planning_msgs/srv/get_available_routes.hpp>
#include <carma_planning_msgs/srv/set_active_route.hpp>
#include <carma_planning_msgs/srv/abort_active_route.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <tf2_ros/transform_listener.h>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "route/route_config.hpp"
#include "route/route_generator_worker.hpp"

namespace route
{

  /**
   * \class Route
   * \brief The route package provides the following functionality:
   *      - Route generation which provides the list of available routes and provides vehicle travel route 
            description and management.
   *      - Route state management which provides the current state of the route following,
   *        including tracking vehicle cross track and down track distances along the active route
   */
  class Route : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> geo_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::Route> route_pub_;
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteState> route_state_pub_;
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::RouteEvent> route_event_pub_;
    carma_ros2_utils::PubPtr<visualization_msgs::msg::Marker> route_marker_pub_;

    // Service Servers
    carma_ros2_utils::ServicePtr<carma_planning_msgs::srv::GetAvailableRoutes> get_available_route_srv_;
    carma_ros2_utils::ServicePtr<carma_planning_msgs::srv::SetActiveRoute> set_active_route_srv_;
    carma_ros2_utils::ServicePtr<carma_planning_msgs::srv::AbortActiveRoute> abort_active_route_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // Node configuration
    Config config_;

    // tf2 buffer holds the tree of transforms
    tf2_ros::Buffer tf2_buffer_;

    // wm listener and pointer to the actual wm object
    carma_wm::WMListener wml_;
    carma_wm::WorldModelConstPtr wm_;

    // route generator worker
    RouteGeneratorWorker rg_worker_;

  public:
    /**
     * \brief Route constructor 
     */
    explicit Route(const rclcpp::NodeOptions &);

    /**
     * \brief Function callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  };

} // route
