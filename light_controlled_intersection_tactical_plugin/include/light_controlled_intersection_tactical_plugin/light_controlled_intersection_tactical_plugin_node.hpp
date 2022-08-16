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

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver types
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                        ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                                throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))

namespace light_controlled_intersection_tactical_plugin
{
  /**
   * \brief ROS node for the LightControlledIntersectionTransitPluginNode
   * 
   */
  class LightControlledIntersectionTransitPluginNode : public carma_guidance_plugins::TacticalPlugin
  {
  private:    
    // LightControlledIntersectionTransitPluginNode configuration
    Config config_;

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
