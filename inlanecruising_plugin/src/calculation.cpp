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
#include <carma_wm/WorldModel.h>
#include <carma_wm/Geometry.h>
#include <limits>
#include <unordered_set>

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

std::vector<PointSpeedPair> points_in_time_boundary(const std::vector<PointSpeedPair>& points, double time_span) {
  std::vector<PointSpeedPair> output;
  output.reserve(points.size());
  double current_time = 0;
  PointSpeedPair prev_pair;
  bool first = true;
  for (const auto& p : points) {
    if (current_time > time_span) {
      return output;
    }

   ROS_WARN_STREAM("Evaluating time of point: " << std::get<0>(p).x() << ", " << std::get<0>(p).y());


    if (first) {
      output.push_back(p);
      prev_pair = p;
      first = false;
      continue;
    }

    double delta_d = lanelet::geometry::distance2d(std::get<0>(p), std::get<0>(prev_pair));

    current_time += delta_d / std::get<1>(prev_pair);

    output.push_back(p);
    prev_pair = p;
  }

  return output;
}

std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                double max_starting_downtrack, const carma_wm::WorldModelConstPtr& wm)
{
  std::vector<PointSpeedPair> points_and_target_speeds;
  std::unordered_set<lanelet::Id> visited_lanelets;

  bool first = true;
  ROS_WARN_STREAM("VehDowntrack: " << max_starting_downtrack);
  for (const auto& manuever : maneuvers)
  {
    if (manuever.type != cav_msgs::Maneuver::LANE_FOLLOWING)
    {
      throw std::invalid_argument("In-Lane Cruising does not support this maneuver type");
    }

    cav_msgs::LaneFollowingManeuver lane_following_maneuver = manuever.lane_following_maneuver;

    double starting_downtrack = lane_following_maneuver.start_dist;
    if (first) {
      if (starting_downtrack > max_starting_downtrack) {
        starting_downtrack = max_starting_downtrack;
      }
      first = false;
    }

    ROS_WARN_STREAM("Used downtrack: " << starting_downtrack);

    auto lanelets = wm->getLaneletsBetween(starting_downtrack, lane_following_maneuver.end_dist, true);

    ROS_WARN_STREAM("Maneuver");
    std::vector<lanelet::ConstLanelet> lanelets_to_add;
    for (auto l: lanelets) {
      ROS_WARN_STREAM("Lanelet ID: " << l.id());
      if (visited_lanelets.find(l.id()) == visited_lanelets.end()) {
        lanelets_to_add.push_back(l);
        visited_lanelets.insert(l.id());
      }
    }
    
    lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);

    bool first = true;
    for (auto p : route_geometry)
    {
      if (first && points_and_target_speeds.size() != 0) {
        first = false;
        continue; // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
      }
      points_and_target_speeds.push_back(std::make_pair(p, lane_following_maneuver.end_speed));
    }
  }

  return points_and_target_speeds;
}

std::vector<PointSpeedPair> downsample_points(const std::vector<PointSpeedPair>& points, int nth_point)
{
  std::vector<PointSpeedPair> downsampled_points;

  downsampled_points.reserve((points.size() / nth_point) + 1);

  for (int i = 0; i < points.size(); i += nth_point)
  {
    downsampled_points.push_back(points[i]);
  }

  return downsampled_points;
}

int getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
{
  lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
  ROS_WARN_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
  double min_distance = std::numeric_limits<double>::max();
  int i = 0;
  int best_index = 0;
  for (const auto& p : points)
  {
    double distance = lanelet::geometry::distance2d(std::get<0>(p), veh_point);
    ROS_WARN_STREAM("distance: " << distance);
    ROS_WARN_STREAM("p: " << std::get<0>(p).x() << ", " << std::get<0>(p).y());
    if (distance < min_distance)
    {
      best_index = i;
      min_distance = distance;
    }
    i++;
  }

  return best_index;
}

std::vector<double> moving_average_filter(const std::vector<double> input, int window_size) {
  int i = 0;
  double average;
  std::deque<double> samples;
  std::vector<double> output;
  output.reserve(input.size());
  for (auto value : input)
  {

    if (i < window_size) {
      samples.push_back(value);
    } else {
      samples.pop_front();
      samples.push_back(value);
    }

    double total = 0;
    for (auto s: samples) {
      total += s;
    }

    double average = total / samples.size();
    output.push_back(average);
    i++;
  }

  return output;
}

void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                          std::vector<double>* speeds)
{
  basic_points->reserve(points.size());
  speeds->reserve(points.size());

  for (const auto& p : points)
  {
    basic_points->push_back(std::get<0>(p));
    speeds->push_back(std::get<1>(p));
  }
}

std::vector<lanelet::BasicPoint2d> pointSpeedPairsToBasicPoints(const std::vector<PointSpeedPair>& points)
{
  std::vector<lanelet::BasicPoint2d> basic_points;
  basic_points.reserve(points.size());

  for (const auto& p : points)
  {
    basic_points.push_back(std::get<0>(p));
  }

  return basic_points;
}

boost::optional<tk::spline> compute_fit(std::vector<lanelet::BasicPoint2d> basic_points)
{
  if (basic_points.size() < 3)
  {
    ROS_WARN_STREAM("Insufficient Spline Points");
    return boost::none;
  }

  tk::spline spl;
  std::vector<double> points_x;
  std::vector<double> points_y;

  for (size_t i = 0; i < basic_points.size(); i++)
  {
    ROS_WARN_STREAM("basic_points[i]: " << basic_points[i].x() << ", " << basic_points[i].y());
    points_x.push_back(basic_points[i].x());
    points_y.push_back(basic_points[i].y());
  }

  spl.set_points(points_x, points_y);

  return spl;
}

double calculate_yaw(std::vector<double> cur_point, std::vector<double> next_point)
{
  double dx = next_point[0] - cur_point[0];
  double dy = next_point[1] - cur_point[1];
  double yaw = atan2(dy, dx);
  return yaw;
}

double calculate_curvature(std::vector<double> cur_point, std::vector<double> next_point)
{
  double dist = sqrt(pow(cur_point[0] - next_point[0], 2) + pow(cur_point[1] - next_point[0], 2));

  double angle = calculate_yaw(cur_point, next_point);

  double r = 0.5 * (dist / std::sin(angle));

  double max_curvature = 100000;
  double curvature = std::min(1 / r, max_curvature);

  return curvature;
}

std::vector<double> compute_orientation_from_fit(tk::spline curve, std::vector<lanelet::BasicPoint2d> sampling_points)
{
  std::vector<double> orientations;
  std::vector<double> cur_point{ 0.0, 0.0 };
  std::vector<double> next_point{ 0.0, 0.0 };
  double lookahead = 0.3;
  for (size_t i = 0; i < sampling_points.size() - 1; i++)
  {
    cur_point[0] = sampling_points[i].x();
    cur_point[1] = sampling_points[i].y();
    next_point[0] = sampling_points[i + 1].x();
    next_point[1] = sampling_points[i + 1].y();
    double res = calculate_yaw(cur_point, next_point);
    orientations.push_back(res);
  }
  orientations.push_back(orientations.back());
  return orientations;
}

std::vector<double> compute_curvature_from_fit(tk::spline curve,
                                                                     std::vector<lanelet::BasicPoint2d> sampling_points)
{
  std::vector<double> curvatures;
  std::vector<double> cur_point{ 0.0, 0.0 };
  std::vector<double> next_point{ 0.0, 0.0 };
  double lookahead = 0.3;
  ROS_WARN_STREAM("Computing Curvatures");
  for (size_t i = 0; i < sampling_points.size() - 1; i++)
  {
    cur_point[0] = sampling_points[i].x();
    cur_point[1] = sampling_points[i].y();
    next_point[0] = sampling_points[i + 1].x();
    next_point[1] = sampling_points[i + 1].y();
    double cur = calculate_curvature(cur_point, next_point);
    curvatures.push_back(fabs(cur));  // TODO now using abs think in more detail if this will cause issues
  }
  curvatures.push_back(curvatures.back());
  return curvatures;
}

}  // namespace inlanecruising_plugin