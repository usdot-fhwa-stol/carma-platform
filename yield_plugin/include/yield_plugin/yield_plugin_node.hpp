#pragma once

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

#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_guidance_plugins/tactical_plugin.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_planning_msgs/msg/lane_change_status.hpp>
#include <carma_v2x_msgs/msg/mobility_response.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <functional>

#include "yield_plugin.hpp"
#include "yield_config.hpp"

namespace yield_plugin
{
/**
 * \brief ROS node for the YieldPlugin
 */ 
class YieldPluginNode : public carma_guidance_plugins::TacticalPlugin
{
public:

     /**
    * \brief Node constructor 
    */
    explicit YieldPluginNode(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn on_configure_plugin();
   
    bool get_availability() override;

    std::string get_version_id() override final;

    void plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) override;

private:

    // Config
    YieldPluginConfig config_;
    
    // Worker
    std::shared_ptr<YieldPlugin> worker_;
    
    std::string version_id_;
    
    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityResponse> mob_resp_pub_;
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::LaneChangeStatus> lc_status_pub_;

    // Subscribers
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityRequest> mob_request_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> bsm_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;
   
};

}  // namespace yield_plugin

 
      
