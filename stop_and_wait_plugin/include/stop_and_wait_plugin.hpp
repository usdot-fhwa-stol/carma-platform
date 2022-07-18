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

#include <vector>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <boost/optional.hpp>
#include "stop_and_wait_config.hpp"
#include "basic_autonomy_ros2/basic_autonomy.hpp"
#include "basic_autonomy_ros2/helper_functions.hpp"
#include <boost/shared_ptr.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <functional>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <boost/geometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>

namespace stop_and_wait_plugin
{
/**
 * \brief Convenience class for pairing 2d points with speeds
 */
struct PointSpeedPair
{
  lanelet::BasicPoint2d point;
  double speed = 0;
};

class StopandWait
{
public:
  /**
   * \brief Constructor
   */
  StopandWait(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh, 
                                          carma_wm::WorldModelConstPtr wm, 
                                          const StopandWaitConfig& config, 
                                          const std::string& plugin_name,
                                          const std::string& version_id);

  /**
   * \brief Service callback for trajectory planning
   *
   * \param req The service request
   * \param resp The service response
   *
   * \return True if success. False otherwise
   */
  bool plan_trajectory_cb(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp);

  /**
   * \brief Converts a set of requested STOP_AND_WAIT maneuvers to point speed limit pairs.
   *
   * \param maneuvers The list of maneuvers to convert
   * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to
   * the vehicle position or earlier. If the first maneuver exceeds this then it's downtrack will be shifted to this
   * value.
   *
   * ASSUMPTION: Since the vehicle is trying to stop the assumption made is that the speed limit is irrelevant. 
   * ASSUMPTION: The provided maneuver lies on the route shortest path
   * 
   * \return List of centerline points paired with speed limits. All output points will have speed matching state.logitudinal_velocity
   */
  std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<carma_planning_msgs::msg::Maneuver>& maneuvers,
                                                  const carma_wm::WorldModelConstPtr& wm,
                                                  const carma_planning_msgs::msg::VehicleState& state);
  /**
   * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of
   * trajectory points for trajectory planning
   *
   * \param points The set of points that define the current lane the vehicle is in and are defined based on the request
   * planning maneuvers. These points must be in the same lane as the vehicle and must extend in front of it though it
   * is fine if they also extend behind it. \param state The current state of the vehicle
   * \param initial_speed Returns the initial_speed used to generate the trajectory
   * \return A list of trajectory points to send to the carma planning stack
   */
  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> compose_trajectory_from_centerline(
      const std::vector<PointSpeedPair>& points, double starting_downtrack, double starting_speed, double stop_location,
      double stop_location_buffer, rclcpp::Time start_time, double stopping_acceleration, double& initial_speed);

  /**
   * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists
   */
  void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                            std::vector<double>* speeds) const;

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
      const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times,
      const std::vector<double>& yaws, rclcpp::Time startTime);

private:

  double epsilon_ = 0.001; //small constant to compare double

  // pointer to the actual wm object
  std::string plugin_name_;
  std::string version_id_;
  carma_wm::WorldModelConstPtr wm_;
  StopandWaitConfig config_;
  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;
  
};
}  // namespace stop_and_wait_plugin
