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
#include <carma_planning_msgs/srv/plan_trajectory.hpp>

#include "carma_guidance_plugins/plugin_base_node.hpp"

namespace carma_guidance_plugins
{

  /**
   * \brief TacticalPlugin base class which can be extended by user provided plugins which wish to implement the Tactical Plugin ROS API. 
   * 
   * A tactical plugin is responsible for planning the detailed trajectory which a vehicle should execute to complete a maneuver.
   * 
   */
  class TacticalPlugin : public PluginBaseNode
  {

  private:
    // Services
    //! The ros service which can be called by the arbitrator or other plugins to have this plugin generate a trajectory plan 
    carma_ros2_utils::ServicePtr<carma_planning_msgs::srv::PlanTrajectory> plan_trajectory_service_;

  public:
    /**
     * \brief TacticalPlugin constructor 
     */
    explicit TacticalPlugin(const rclcpp::NodeOptions &);

    //! Virtual destructor for safe deletion
    virtual ~TacticalPlugin() = default;

    /**
     * \brief Extending class provided callback which should return a planned trajectory based on the provided trajectory planning request.
     * 
     * \param srv_header RCL header for services calls. Can usually be ignored by implementers. 
     * \param req The service request containing the maneuvers to plan trajectories for and current vehicle state
     * \param resp The response containing the planned trajectory
     */ 
    virtual void plan_trajectory_callback(
      std::shared_ptr<rmw_request_id_t> srv_header, 
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) = 0;
    

    ////
    // Overrides
    ////

    // Non-Final to allow extending plugins to provide more detailed capabilities
    std::string get_capability() override;

    // Final
    uint8_t get_type() override final;

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &) override final; 
    carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &, const std::string &exception_string) override final;
  };

} // carma_guidance_plugins
