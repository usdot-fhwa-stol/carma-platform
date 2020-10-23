#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/optional/optional.hpp>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include "inlanecruising_plugin.h"
#include <tuple>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_msgs/LaneFollowingManeuver.h>
#include <limits>

/*
#
# 2d x-axis position of the vehicle center of gravity in meters
# This position is in a fixed inertial frame which vehicle motion is described in
# 
float64 X_pos_global
#
# 2d y-axis position of the vehicle center of gravity in meters
# This position is in a fixed inertial frame which vehicle motion is described in
# 
float64 Y_pos_global
#
# The orientation of the vehicle's longitudinal axis in radians
# This orientation is in a fixed inertial frame which vehicle motion is described in
# 
float64 orientation
#
# longitudinal velocity of the vehicle center of gravity in ms in its body frame
# 
float64 longitudinal_vel

*/

namespace inlanecruising_plugin
{
using PointSpeedPair = std::pair<BasicPoint2d, double>;
std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                const carma_wm::WorldModelConstPtr& wm)
{
  std::vector<PointSpeedPair> points_and_target_speeds;

  for (const auto& manuever : maneuvers) {
    if (manuever.type != cav_msgs::Maneuver::LANE_FOLLOWING) {
      throw std::invalid_argument("In-Lane Cruising does not support this maneuver type");
    }

    cav_msgs::LaneFollowingManeuver lane_following_maneuver = manuever.lane_following_maneuver;

    auto lanelets = wm->getLaneletsBetween(lane_following_maneuver.start_dist, lane_following_maneuver.end_dist, true);
    lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets);
    
    for (auto p : route_geometry) {
      points_and_target_speeds.push_back(std::make_pair(p, lane_following_maneuver.end_speed));
    }
  }

  return points_and_target_speeds;
}

std::vector<PointSpeedPair> downsample_points(const std::vector<PointSpeedPair>& points, int nth_point) {
  std::vector<PointSpeedPair> downsampled_points;
  
  downsampled_points.reserve((points.size() / nth_point) + 1)
  
  for (int i = 0; i < points.size(); i += nth_point) {
    downsampled_points.push_back(points[i]);
  }

  return downsampled_points;
}

int getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state) {
  lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
  double min_distance = std::numeric_limits<double>::max();
  int i = 0;
  int best_index = 0;
  for (const auto& p : points) {
    double distance = lanelet::geometry::distance2d(p, veh_point);
    if (distance < min_distance) {
      best_index = i;
      min_distance = distance;
    }
    i++;
  }

  return best_index;
}

void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points, std::vector<double>* speeds) {
  basic_points->reserve(points.size());
  speeds->reserve(points.size());

  for (const auto& p : points) {
    basic_points->push_back(std::get<0>(p));
    speeds->push_back(std::get<1>(p));
  }

}

std::vector<lanelet::BasicPoint2d> pointSpeedPairsToBasicPoints(const std::vector<PointSpeedPair>& points) {
  std::vector<lanelet::BasicPoint2d> basic_points;
  basic_points.reserve(points.size());

  for (const auto& p : points) {
    basic_points.push_back(std::get<0>(p));
  }

  return basic_points;
}


}  // namespace inlanecruising_plugin