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

#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/optional/optional.hpp>
#include <algorithm>
#include <deque>
#include <tf/transform_datatypes.h>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include "inlanecruising_plugin.h"
#include "calculation.cpp"

using std::ostringstream = oss;
namespace inlanecruising_plugin
{
InLaneCruisingPlugin::InLaneCruisingPlugin(carma_wm::WorldModelConstPtr wm, InLaneCruisingPluginConfig config,
                                           PublishPluginDiscoveryCB plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
{
  plugin_discovery_msg_.name = "InLaneCruisingPlugin";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = false;
  plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
  plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
}

bool InLaneCruisingPlugin::onSpin()
{
  plugin_discovery_publisher_(plugin_discovery_msg_);
  return true;
}

bool InLaneCruisingPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
{
  ros::WallTime start_time = ros::WallTime::now();
  lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
  auto points_and_target_speeds = maneuvers_to_points(req.maneuver_plan.maneuvers, current_downtrack, wm_);

  ROS_DEBUG_STREAM("points_and_target_speeds: " << points_and_target_speeds.size());
  auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds, config_.downsample_ratio);
  ROS_DEBUG_STREAM("downsample_points: " << downsampled_points.size());

  ROS_DEBUG_STREAM("PlanTrajectory");
  cav_msgs::TrajectoryPlan trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = ros::Time::now();
  trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
  ROS_DEBUG_STREAM("1");
  trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points, req.vehicle_state);
  ROS_DEBUG_STREAM("2");

  resp.trajectory_plan = trajectory;
  resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_FOLLOWING);
  resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  ROS_DEBUG_STREAM("3");

  ros::WallTime end_time = ros::WallTime::now();

  ros::WallDuration duration = end_time - start_time;
  ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());

  return true;
}

double compute_speed_for_curvature(double curvature, double lateral_accel_limit)
{
  // Check at compile time for infinity availability
  static_assert(std::numeric_limits<double>::has_infinity, "This code requires compilation using a system that "
                                                           "supports IEEE 754 for access to positive infinity values");

  // Solve a = v^2/r (k = 1/r) for v
  // a = v^2 * k
  // a / k = v^2
  // v = sqrt(a / k)

  if (fabs(curvature) < 0.00000001)
  {  // Check for curvature of 0.
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(fabs(lateral_accel_limit / curvature));
}

std::vector<double> compute_ideal_speeds(std::vector<double> curvatures, double lateral_accel_limit)
{
  std::vector<double> out;
  for (double k : curvatures)
  {
    out.push_back(compute_speed_for_curvature(k, lateral_accel_limit));
  }

  return out;
}

std::vector<double> apply_speed_limits(const std::vector<double> speeds, const std::vector<double> speed_limits)
{
  ROS_ERROR_STREAM("Speeds list size: " << speeds.size());
  ROS_ERROR_STREAM("SpeedLimits list size: " << speed_limits.size());
  if (speeds.size() != speed_limits.size())
  {
    throw std::invalid_argument("Speeds and speed limit lists not same size");
  }
  std::vector<double> out;
  for (int i = 0; i < speeds.size(); i++)
  {
    out.push_back(std::min(speeds[i], speed_limits[i]));
  }

  return out;
}

std::vector<double> compute_downtracks(std::vector<lanelet::BasicPoint2d> basic_points)
{
  std::vector<double> downtracks;
  downtracks.reserve(basic_points.size());
  double current_dt = 0;
  boost::optional<lanelet::BasicPoint2d> prev_p;
  for (const auto& p : basic_points)
  {
    if (!prev_p)
    {
      downtracks.push_back(0);
      continue;
    }
    double dx = p.x() - prev_p->x();
    double dy = p.y() - prev_p->y();
    double dist = sqrt(dx * dx + dy * dy);
    current_dt += dist;
    downtracks.push_back(current_dt);
    prev_p = p;
  }

  return downtracks;
}

struct DiscreteCurve
{
public:
  tf2::Transform frame;
  std::vector<PointSpeedPair> points;
};

tf2::Transform compute_heading_frame(const tf2::Vector3& p1, const tf2::Vector3& p2)
{
  tf2::Matrix3x3 rot_mat = tf2::Matrix3x3::getIdentity();

  double yaw = atan2(p2.y() - p1.y(), p2.x() - p1.x());

  rot_mat.setRPY(0, 0, yaw);
  tf2::Vector3 position(p1.x(), p1.y(), 0);
  tf2::Transform frame(rot_mat, position);
  return frame;
}

tf2::Vector3 point2DToTF2Vec(const lanelet::BasicPoint2d& p)
{
  return tf2::Vector3(p.x(), p.y(), 0);
}

lanelet::BasicPoint2d tf2VecToPoint2D(const tf2::Vector3& p)
{
  return lanelet::BasicPoint2d(p.x(), p.y());
}

bool transformExactMatch(const tf2::Transform& t1, const tf2::Transform& t2)
{
  return t1.getRotation().x() == t2.getRotation().x() && t1.getRotation().y() == t2.getRotation().y() &&
         t1.getRotation().z() == t2.getRotation().z() && t1.getRotation().w() == t2.getRotation().w() &&
         t1.getOrigin().x() == t2.getOrigin().x() && t1.getOrigin().y() == t2.getOrigin().y() &&
         t1.getOrigin().z() == t2.getOrigin().z();
}

std::vector<DiscreteCurve> compute_sub_curves(const std::vector<PointSpeedPair>& basic_points)
{
  if (basic_points.size() < 2)
  {
    throw std::invalid_argument("Not enough points");
  }

  bool x_going_positive = true;  // Since we define the frame to be positive x along line this always starts as true

  std::vector<DiscreteCurve> curves;
  DiscreteCurve curve;
  curve.frame = compute_heading_frame(point2DToTF2Vec(std::get<0>(basic_points[0])),
                                      point2DToTF2Vec(std::get<0>(basic_points[1])));
  tf2::Transform map_in_curve = curve.frame.inverse();

  for (int i = 0; i < basic_points.size() - 1; i++)
  {
    tf2::Vector3 p1 = map_in_curve * point2DToTF2Vec(std::get<0>(basic_points[i]));
    tf2::Vector3 p2 =
        map_in_curve * point2DToTF2Vec(std::get<0>(basic_points[i + 1]));  // TODO Optimization to cache this value

    curve.points.push_back(std::make_pair(tf2VecToPoint2D(p1), std::get<1>(basic_points[i])));

    bool x_dir = (p2.x() - p1.x()) >= 0;
    if (x_going_positive != x_dir)
    {  // TODO this check could be simplified to (!x_dir)
      // New Curve
      curves.push_back(curve);

      curve = DiscreteCurve();
      curve.frame = compute_heading_frame(p1, p2);
      map_in_curve = curve.frame.inverse();
      curve.points.push_back(std::make_pair(tf2VecToPoint2D(p1), std::get<1>(basic_points[i])));  // Include first point
                                                                                                  // in curve
      x_going_positive = true;  // Reset to true because we are using a new frame
    }
  }

  if (curves.size() == 0 || (!transformExactMatch(curves.back().frame, curve.frame)))
  {
    curves.push_back(curve);
  }

  return curves;
}

oss basicPointToStream(const lanelet::BasicPoint2d& point) {
  oss out;
  out << point.x() << ", " << point.y();
  return out;
}

oss pointSpeedPairToStream(const PointSpeedPair& point) {
  oss out;
  out << "Point: " << basicPointToStream(std::get<0>(point) << " Speed: " << std::get<1>(point));
  return out;
}

template<T>
void printDebugPerLine(std::vector<T> values, std::function<oss(T)> func) {
  for (const auto& value : values) {
    ROS_DEBUG_STREAM(func(value));
  }
}

std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
{

  ROS_DEBUG_STREAM("points size: " << points.size());
  printDebugPerLine(points, &pointSpeedPairToStream);

  std::vector<cav_msgs::TrajectoryPlanPoint> final_trajectory;
  int nearest_pt_index = getNearestPointIndex(points, state);

  ROS_DEBUG_STREAM("NearestPtIndex: " << nearest_pt_index);

  std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end());
  auto time_bound_points = points_in_time_boundary(future_points, config_.trajectory_time_length);

  ROS_DEBUG_STREAM("time_bound_points: " << time_bound_points.size());

  printDebugPerLine(time_bound_points, &pointSpeedPairToStream);

  ROS_WARN("Got basic points ");
  std::vector<DiscreteCurve> sub_curves = compute_sub_curves(time_bound_points);

  ROS_DEBUG_STREAM("Got sub_curves " << sub_curves.size());

  std::vector<tf2::Quaternion> final_yaw_values;
  std::vector<double> final_actual_speeds;
  std::vector<lanelet::BasicPoint2d> all_sampling_points;

  for (const auto& discreet_curve : sub_curves)
  {
    ROS_WARN("SubCurve");
    std::vector<double> speed_limits;
    std::vector<lanelet::BasicPoint2d> basic_points;
    splitPointSpeedPairs(discreet_curve.points, &basic_points, &speed_limits);
    boost::optional<tk::spline> fit_curve = compute_fit(basic_points);  // Returned data type TBD

    if (!fit_curve)
    {  // TODO how better to handle this case
      for (auto p : discreet_curve.points)
      {
        final_yaw_values.push_back(final_yaw_values.back());
        final_actual_speeds.push_back(final_actual_speeds.back());
      }
      continue;
    }

    ROS_WARN("Got fit");

    std::vector<lanelet::BasicPoint2d> sampling_points;
    sampling_points.reserve(1 + discreet_curve.points.size() * 2);
    
    std::vector<double> distributed_speed_limits;
    distributed_speed_limits.reserve(1 + discreet_curve.points.size() * 2);

    double totalDist = 0;
    bool firstLoop = true;
    lanelet::BasicPoint2d prev_point(0.0, 0.0);
    std::vector<std::pair<double, double>> limit_distance_pairs;
    limit_distance_pairs.reserve(basic_points.size());
    int current_p_i = 0;
    for (auto p : basic_points)
    {
      if (firstLoop)
      {
        prev_point = p;
        firstLoop = false;
        continue;
      }

      totalDist += lanelet::geometry::distance2d(prev_point, p);
      limit_distance_pairs.push_back(std::make_pair(speed_limits[current_p_i], totalDist));
      current_p_i++;
    }

    double current_dist = 0;
    double step_size = config_.curve_resample_step_size; 
    tk::spline actual_fit_curve = fit_curve.get();
    int current_pair_index = 0;
    while (current_dist < totalDist - step_size)
    {
      double x = current_dist;
      double y = actual_fit_curve(x);
      lanelet::BasicPoint2d p(x, y);
      sampling_points.push_back(p);

      for (size_t i = current_pair_index; i < limit_distance_pairs.size(); i++) {
        if (std::get<1>(limit_distance_pairs[i]) >= current_dist) {
            current_pair_index = i;
            break;
        }
      }

      distributed_speed_limits.push_back(std::get<0>(limit_distance_pairs[current_pair_index]));
      current_dist += step_size;
    }

    ROS_WARN("Sampled points");
    printDebugPerLine(sampling_points, &basicPointToStream);

    
    std::vector<double> yaw_values = compute_orientation_from_fit(actual_fit_curve, sampling_points);

    ROS_WARN("Got yaw");
    std::vector<double> curvatures = compute_curvature_from_fit(actual_fit_curve, sampling_points);
    curvatures = moving_average_filter(curvatures, config_.moving_average_window_size);

    printDebugPerLine(curvatures, [](auto c){oss o; o << "curvatures[i]: " << c; return o;});
    

    ROS_WARN("Got curvatures");

    ROS_WARN("Got speeds limits");
    std::vector<double> ideal_speeds = compute_ideal_speeds(curvatures, config_.lateral_accel_limit);

    printDebugPerLine(ideal_speeds, [](auto s){oss o; o << "ideal_speeds: " << s; return o;});

    ROS_WARN("Got ideal limits");
    std::vector<double> actual_speeds = apply_speed_limits(ideal_speeds, distributed_speed_limits);

    printDebugPerLine(actual_speeds, [](auto s){oss o; o << "actual_speeds: " << s; return o;});
    ROS_WARN("Got actual");

    printDebugPerLine(yaw_values, [](auto y){oss o; o << "yaw_values[i]: " << y; return o;});

    for (int i = 0; i < yaw_values.size() - 1; i++)
    {  // Drop last point
      double yaw = yaw_values[i];
      tf2::Matrix3x3 rot_mat = tf2::Matrix3x3::getIdentity();
      rot_mat.setRPY(0, 0, yaw);
      tf2::Transform c_to_yaw(rot_mat);  // NOTE: I'm pretty certain the origin does not matter here but unit test to
                                         // confirm
      tf2::Transform m_to_yaw = discreet_curve.frame * c_to_yaw;
      final_yaw_values.push_back(m_to_yaw.getRotation());

      tf2::Vector3 vec = point2DToTF2Vec(sampling_points[i]);
      tf2::Vector3 map_frame_vec = discreet_curve.frame * vec;
      all_sampling_points.push_back(tf2VecToPoint2D(map_frame_vec));
    }

    ROS_WARN("Converted yaw to quat");

    final_actual_speeds.insert(final_actual_speeds.end(), actual_speeds.begin(), actual_speeds.end() - 1);

    ROS_WARN("Appended to final");
  }

  ROS_WARN("Processed all curves");

  final_actual_speeds = moving_average_filter(final_actual_speeds, config_.moving_average_window_size);

  printDebugPerLine(final_actual_speeds, [](auto s){oss o; o << "final_actual_speeds[i]: " << s; return o;});
  printDebugPerLine(final_yaw_values, [](auto s){oss o; o << "final_yaw_values[i]: " << s; return o;});

  // Add current vehicle point to front of the trajectory
  std::vector<lanelet::BasicPoint2d> cur_veh_point(state.X_pos_global, state.Y_pos_global);
  all_sampling_points.insert(all_sampling_points.begin(), cur_veh_point); // Add current vehicle position to front of sample points
  final_actual_speeds.insert(final_actual_speeds.begin(), std::max(state.longitudinal_vel, config_.minimum_speed));
  tf2::Quaternion initial_quat;
  initial_quat.setRPY(0,0, state.orientation);
  final_yaw_values.insert(final_yaw_values.begin(), initial_quat)


  // Compute points to local downtracks
  std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

  // Apply lookahead speeds
  final_actual_speeds = trajectory_utils::shift_by_lookahead(final_actual_speeds, config_.lookahead_count)

  // Apply accel limits
  final_actual_speeds = trajectory_utils::apply_accel_limits_by_distance(downtracks, final_actual_speeds, config_.max_accel, config_.max_accel); // TODO write method

  // Convert speeds to times
  std::vector<double> times;
  trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

  // Build trajectory points

  std::vector<cav_msgs::TrajectoryPlanPoint> traj_points = trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values); // TODO need to implement this method

  final_trajectory = post_process_traj_points(traj_points);

  return final_trajectory;
}

std::vector<cav_msgs::TrajectoryPlanPoint>
InLaneCruisingPlugin::post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory)
{
  ros::Time now = ros::Time::now();
  ros::Duration now_duration(now.sec, now.nsec);
  for (int i = 0; i < trajectory.size(); i++)
  {
    trajectory[i].controller_plugin_name = "default";
    trajectory[i].planner_plugin_name = plugin_discovery_msg_.name;
    trajectory[i].target_time += now_duration;
  }

  return trajectory;
}

// compute_fit(points);
// compute_orientation_from_fit(curve, sampling_points)
}  // namespace inlanecruising_plugin
