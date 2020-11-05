#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <autoware_msgs/Lane.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <tf2/LinearMath/Matrix3x3.h>  // TODO it should be possible to use tf2_eigen instead which would be more efficient since lanelet2 uses eigen under the hood
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <inlanecruising_plugin/smoothing/SplineI.h>

#include "inlanecruising_config.h"
#include "third_party_library/spline.h"

namespace inlanecruising_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;

struct PointSpeedPair
{
  lanelet::BasicPoint2d point;
  double speed = 0;
};

struct DiscreteCurve
{
  Eigen::Isometry2d frame;
  std::vector<PointSpeedPair> points;
};

class InLaneCruisingPlugin
{
public:
  InLaneCruisingPlugin(carma_wm::WorldModelConstPtr wm_, InLaneCruisingPluginConfig config,
                       PublishPluginDiscoveryCB plugin_discovery_publisher);

  // service callbacks for carma trajectory planning
  bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

  bool onSpin();

  std::vector<double> apply_speed_limits(const std::vector<double> speeds, const std::vector<double> speed_limits);

  Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2);

  std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair>& points, double time_span);

  std::vector<cav_msgs::TrajectoryPlanPoint>
  compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

  std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
      const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times,
      const std::vector<double>& yaws, ros::Time startTime);

  std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                  double max_starting_downtrack,
                                                  const carma_wm::WorldModelConstPtr& wm);

  int getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

  void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                            std::vector<double>* speeds);

  std::unique_ptr<smoothing::SplineI> compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points);

  std::vector<DiscreteCurve> compute_sub_curves(const std::vector<PointSpeedPair>& basic_points);

  Eigen::Isometry2d curvePointInMapTF(const Eigen::Isometry2d& curve_in_map, const lanelet::BasicPoint2d& p, double yaw) const;

private:
  carma_wm::WorldModelConstPtr wm_;
  InLaneCruisingPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;

  cav_msgs::Plugin plugin_discovery_msg_;
};
};  // namespace inlanecruising_plugin
