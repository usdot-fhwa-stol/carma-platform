/*
 * Copyright (C) 2021-2022 LEIDOS.
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

/*
 * Originally Developed for ROS1 by the UCLA Mobility Lab, 10/20/2021. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
 * 
 * 8/15/2022: Ported to ROS2
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_v2x_msgs/msg/mobility_response.hpp>
#include <carma_v2x_msgs/msg/mobility_request.hpp>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_planning_msgs/msg/platooning_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <functional>

#include <carma_guidance_plugins/strategic_plugin.hpp>

#include "platoon_strategic_ihp.h"
#include "platoon_config_ihp.h"

namespace platoon_strategic_ihp
{

  /**
   * \brief ROS Node to for Platooning Strategic Plugin IHP2 variant
   */ 
  class Node : public carma_guidance_plugins::StrategicPlugin
  {

  private:

    // Publishers
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::PlatooningInfo> platoon_info_pub;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityResponse> mob_response_pub;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityRequest> mob_request_pub;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> mob_operation_pub;

    // Subscribers
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityRequest> mob_request_sub;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityResponse> mob_response_sub;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityOperation> mob_operation_sub;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> current_pose_sub;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> current_twist_sub;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> cmd_sub;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub;

    // Timers
    rclcpp::TimerBase::SharedPtr loop_timer_;

    // Node configuration
    PlatoonPluginConfig config_;

    // Worker
    std::shared_ptr<PlatoonStrategicIHPPlugin> worker_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);


    ////
    // Overrides
    ////
    void plan_maneuvers_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp) override;

    bool get_availability() override;

    std::string get_version_id() override;
    
    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */ 
    carma_ros2_utils::CallbackReturn on_configure_plugin();
    carma_ros2_utils::CallbackReturn on_cleanup_plugin();

  };

} // platoon_strategic_ihp
