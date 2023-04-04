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
#include <carma_wm/WMListener.hpp>
#include <functional>
#include "inlanecruising_plugin.hpp"
#include "inlanecruising_config.hpp"
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_debug_ros2_msgs/msg/trajectory_curvature_speeds.hpp>

namespace inlanecruising_plugin
{
/**
 * \brief ROS node for the InLaneCruisingPlugin
 */ 
class InLaneCruisingPluginNode : public carma_guidance_plugins::TacticalPlugin
{
public:
    
  /**
   * \brief Node constructor 
   */
  explicit InLaneCruisingPluginNode(const rclcpp::NodeOptions &);

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn on_configure_plugin();
   
  bool get_availability() override;

  std::string get_version_id() override final;

  rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

  void plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) override;

private:

  // Node configuration
  InLaneCruisingPluginConfig config_;

  std::string plugin_name_;
  std::string version_id_;

  carma_ros2_utils::PubPtr<carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds> trajectory_debug_pub_;

  // Service Clients
  carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> yield_client_;

  // Worker
  std::shared_ptr<InLaneCruisingPlugin> worker_;

  // Unit Test Accessors
  FRIEND_TEST(InLaneCruisingPluginTest, rostest1);

};

}  // namespace inlanecruising_plugin