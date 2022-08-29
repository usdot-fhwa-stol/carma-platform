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

#include <carma_guidance_plugins/tactical_plugin.hpp>
#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_config.hpp"
#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin.hpp"

namespace light_controlled_intersection_tactical_plugin
{
  /**
   * \brief ROS node for the LightControlledIntersectionTransitPluginNode
   * 
   */
  class LightControlledIntersectionTransitPluginNode : public carma_guidance_plugins::TacticalPlugin
  {
  private:    
    // Config for this object
    Config config_;


    // Worker object
    std::shared_ptr<LightControlledIntersectionTacticalPlugin> worker_;

  public:
  
    /**
     * \brief LightControlledIntersectionTransitPluginNode constructor 
     */
    explicit LightControlledIntersectionTransitPluginNode(const rclcpp::NodeOptions &);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////
    void plan_trajectory_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) override;

    bool get_availability() override;

    std::string get_version_id() override;
    
    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */ 
    carma_ros2_utils::CallbackReturn on_configure_plugin() override;
  };

} // light_controlled_intersection_tactical_plugin
