#pragma once

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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <carma_guidance_plugins/tactical_plugin.hpp>
#include <sstream>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include "stop_and_wait_plugin.hpp"
#include "stop_and_wait_config.hpp"
#include <vector>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <math.h>
#include <std_msgs/msg/float64.hpp>
#include <carma_planning_msgs/msg/stop_and_wait_maneuver.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>

namespace stop_and_wait_plugin
{
class StopandWaitNode : public carma_guidance_plugins::TacticalPlugin
{

private:

    StopandWaitConfig config_;

    // Worker
    std::shared_ptr<StopandWait> plugin_;

    std::string version_id_;
    std::string plugin_name_;

public:
  
  /**
   * \brief Node constructor 
   */
    explicit StopandWaitNode(const rclcpp::NodeOptions &);

    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */ 
    carma_ros2_utils::CallbackReturn on_configure_plugin();

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////

    void plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) override;

    bool get_availability() override;

    std::string get_version_id() override final;

};
}  // namespace stop_and_wait_plugin