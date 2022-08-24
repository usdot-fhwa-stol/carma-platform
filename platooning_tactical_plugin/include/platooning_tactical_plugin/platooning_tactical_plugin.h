#pragma once

/*------------------------------------------------------------------------------
* Copyright (C) 2020-2022 LEIDOS.
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

------------------------------------------------------------------------------*/

#include <vector>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <functional>
#include <unordered_set>
#include <carma_debug_ros2_msgs/msg/trajectory_curvature_speeds.hpp>
#include <basic_autonomy_ros2/basic_autonomy.hpp>
#include <carma_ros2_utils/timers/TimerFactory.hpp>

#include "platooning_tactical_plugin_config.h"

namespace platooning_tactical_plugin
{
using DebugPublisher = std::function<void(const carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds&)>;
/**
 * \brief Convenience class for pairing 2d points with speeds
 */ 
struct PointSpeedPair
{
  lanelet::BasicPoint2d point;
  double speed = 0;
};

/**
 * \brief Class containing primary business logic for the Platooning Tactical Plugin
 * 
 */ 
class PlatooningTacticalPlugin
{
public:
  /**
   * \brief Constructor
   * 
   * \param wm Pointer to initalized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   */ 
  PlatooningTacticalPlugin(carma_wm::WorldModelConstPtr wm, PlatooningTacticalPluginConfig config,
                       std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory);

  /**
   * \brief Service callback for trajectory planning
   * 
   * \param req The service request
   * \param resp The service response
   * 
   * \return True if success. False otherwise
   */ 
  bool plan_trajectory_cb(carma_planning_msgs::srv::PlanTrajectory::Request& req, carma_planning_msgs::srv::PlanTrajectory::Response& resp);

  /**
   * \brief Set the current config
   */ 
  void set_config(PlatooningTacticalPluginConfig config);


  carma_planning_msgs::msg::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

private:
  carma_wm::WorldModelConstPtr wm_;
  PlatooningTacticalPluginConfig config_;

  carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds debug_msg_;
  DebugPublisher debug_publisher_;

  carma_planning_msgs::msg::VehicleState ending_state_before_buffer; //state before applying extra points for curvature calculation that are removed later

  std::shared_ptr<carma_ros2_utils::timers::TimerFactory> timer_factory_;
};
}  // namespace platooning_tactical_plugin