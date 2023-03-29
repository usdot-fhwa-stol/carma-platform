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

#include <vector>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <boost/shared_ptr.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.hpp>
#include <basic_autonomy/basic_autonomy.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_wm/WMListener.hpp>
#include <functional>
#include "inlanecruising_config.hpp"
#include <unordered_set>
#include <autoware_msgs/msg/lane.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_debug_ros2_msgs/msg/trajectory_curvature_speeds.hpp>
#include <gtest/gtest.h>

namespace inlanecruising_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const carma_planning_msgs::msg::Plugin&)>;
using DebugPublisher = std::function<void(const carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds&)>;
using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;

static const std::string ILC_LOGGER = "inlanecruising_plugin";

/**
 * \brief Class containing primary business logic for the In-Lane Cruising Plugin
 * 
 */ 
class InLaneCruisingPlugin
{
public:
  /**
   * \brief Constructor
   * \param nh Pointer to the lifecyle node
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   * \param debug_publisher Callback which will publish a debug message. The callback defaults to no-op.
   * \param plugin_name Retrieved from the plugin node
   * \param version_id Retrieved from the plugin node
   */ 
  InLaneCruisingPlugin(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh, 
                      carma_wm::WorldModelConstPtr wm, 
                      const InLaneCruisingPluginConfig& config, 
                      const DebugPublisher& debug_publisher=[](const auto& msg){},
                      const std::string& plugin_name = "inlanecruising_plugin",
                      const std::string& version_id = "v1.0");

  /**
   * \brief Service callback for trajectory planning
   * \param srv_header header
   * \param req The service request
   * \param resp The service response
   * 
   */ 
  void plan_trajectory_callback(
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp);

  /**
   * \brief set the yield service
   * 
   * \param yield_srv input yield service
   */
  void set_yield_client(carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> client);

   /**
   * \brief verify if the input yield trajectory plan is valid
   * 
   * \param yield_plan input yield trajectory plan
   *
   * \return true or false
   */
  bool validate_yield_plan(const carma_planning_msgs::msg::TrajectoryPlan& yield_plan) const;

  carma_planning_msgs::msg::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

private:

  std::string plugin_name_;
  std::string version_id_;
  carma_wm::WorldModelConstPtr wm_;
  InLaneCruisingPluginConfig config_;
  carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> yield_client_;
  DebugPublisher debug_publisher_;
  carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds debug_msg_;
  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;
  
  // Access members for unit test
  FRIEND_TEST(InLaneCruisingPluginTest, rostest1);
};



};  // namespace inlanecruising_plugin