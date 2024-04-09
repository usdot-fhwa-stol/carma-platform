#pragma once

/*
 * Copyright (C) 2023 LEIDOS.
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

#include <rclcpp/rclcpp.hpp>
#include <carma_guidance_plugins/tactical_plugin.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <functional>
#include "intersection_transit_maneuvering/itm_service.hpp"
#include "intersection_transit_maneuvering/call_interface.hpp"

namespace intersection_transit_maneuvering
{
    /**
 * \brief ROS node for the inlanecruising_plugin
 */
class IntersectionTransitManeuveringNode : public carma_guidance_plugins::TacticalPlugin
{
    private:    

    // Worker object
    
    std::vector<carma_planning_msgs::msg::Maneuver> converted_maneuvers_;
    carma_planning_msgs::msg::VehicleState vehicle_state_;
    
    public:
    std::shared_ptr<CallInterface> object_;
    carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> client_; 
    /**
     * \brief IntersectionTransitManeuveringNode constructor 
     */
    explicit IntersectionTransitManeuveringNode(const rclcpp::NodeOptions &);
  
    /**
     *  \brief Converts a sequence of INTERSECTION_TRANSIT maneuvers to LANE_FOLLOWING maneuvers
     * 
     * \param maneuvers The list of maneuvers to convert
     * 
     * \return The new list of converted maneuvers
     */
    std::vector<carma_planning_msgs::msg::Maneuver> convert_maneuver_plan(const std::vector<carma_planning_msgs::msg::Maneuver>& maneuvers);

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


} //intersection_transit_maneuvering