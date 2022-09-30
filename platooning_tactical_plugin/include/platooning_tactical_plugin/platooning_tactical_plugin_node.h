#pragma once

/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <carma_guidance_plugins/tactical_plugin.hpp>
#include "platooning_tactical_plugin.h"
#include "platooning_tactical_plugin_config.h"


namespace platooning_tactical_plugin
{
  /**
   * \brief ROS node for the PlatooningTacticalPlugin
   */ 
  class Node : public carma_guidance_plugins::TacticalPlugin
  {
    private:
      PlatooningTacticalPluginConfig config_;

      std::shared_ptr<PlatooningTacticalPlugin> worker_;
    
    public:
      /**
       * \brief Node constructor 
       */
      explicit Node(const rclcpp::NodeOptions &);

      /**
       * \brief Example callback for dynamic parameter updates
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
      carma_ros2_utils::CallbackReturn on_configure_plugin();
  };

}  // namespace platooning_tactical_plugin