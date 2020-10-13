/*
 * Copyright (C) 2020 LEIDOS.
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

#include <waypoint_generator/waypoint_generator.hpp>
#include <waypoint_generator/waypoint_generator_config.hpp>
#include <carma_wm/Geometry.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <limits>

namespace waypoint_generator
{
WaypointGenerator::WaypointGenerator(carma_wm::WorldModelConstPtr wm, WaypointGeneratorConfig config, PublishWaypointsCallback waypoint_publisher) 
: _wm(wm), _config(config), _waypoint_publisher(waypoint_publisher) {};

std::vector<int> WaypointGenerator::compute_constant_curvature_regions(std::vector<double> curvatures, double epsilon,
                                                                       int linearity_constraint) const
{
  std::vector<int> regions;
  double cur_min = 0;
  for (int i = 0; i < curvatures.size(); i++)
  {
    if (i == 0)
    {
      cur_min = curvatures[i];
    }

    if (std::fabs(curvatures[i] - cur_min) > epsilon)
    {
      regions.push_back(i - 1);
      cur_min = curvatures[i];
    }
  }

  regions.push_back(curvatures.size() - 1);

  // Post-process for linearly increasing/decreasing regions
  // It is assumed that the underyling dataset is itself linearly increasing
  // or decreasing and that is is just sampling it down to the endpoint
  std::vector<int> out;
  for (int i = 0; i < regions.size() - 1; i++)
  {
    if (regions[i + 1] - regions[i] < linearity_constraint)
    {
      continue;
    }
    else
    {
      out.push_back(regions[i]);
    }
  }

  out.push_back(curvatures.size() - 1);

  return out;
}

std::vector<double> WaypointGenerator::normalize_curvature_regions(std::vector<double> curvatures,
                                                                   std::vector<int> regions) const
{
  int region = 0;
  double min = std::numeric_limits<double>::infinity();
  std::vector<double> mins;
  for (int i = 0; i < curvatures.size(); i++)
  {
    if (i <= regions[region])
    {
      if (curvatures[i] < min)
      {
        min = curvatures[i];
      }
    }
    else
    {
      mins.push_back(min);
      min = std::numeric_limits<double>::infinity();
      region++;
    }
  }
  mins.push_back(min);

  std::vector<double> processed_curvatures;
  for (int i = 0; i < regions.size(); i++)
  {
    if (i == 0)
    {
      for (int j = 0; j <= regions[i]; j++)
      {
        processed_curvatures.push_back(mins[i]);
      }
    }
    else
    {
      for (int j = 0; j <= (regions[i] - regions[i - 1]) - 1; j++)
      {
        processed_curvatures.push_back(mins[i]);
      }
    }
  }

  return processed_curvatures;
}

double WaypointGenerator::compute_speed_for_curvature(double curvature, double lateral_accel_limit) const
{
// Check at compile time for infinity availability
  static_assert(std::numeric_limits<double>::has_infinity, "This code requires compilation using a system that supports IEEE 754 for access to positive infinity values");

  // Solve a = v^2/r (k = 1/r) for v
  // a = v^2 * k
  // a / k = v^2
  // v = sqrt(a / k)
  
  if (fabs(curvature) < 0.00000001) { // Check for curvature of 0.
      return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(fabs(lateral_accel_limit / curvature));
}
std::vector<double> WaypointGenerator::compute_ideal_speeds(std::vector<double> curvatures,
                                                            double lateral_accel_limit) const
{
  std::vector<double> out;
  for (double k : curvatures)
  {
    out.push_back(compute_speed_for_curvature(k, lateral_accel_limit));
  }

  return out;
}

std::vector<double> WaypointGenerator::apply_speed_limits(const std::vector<double> speeds,
                                                          const std::vector<double> speed_limits) const
{
  ROS_ERROR_STREAM("Speeds list size: " << speeds.size());
  ROS_ERROR_STREAM("SpeedLimits list size: " << speed_limits.size());
  if (speeds.size() != speed_limits.size())
  {
    throw std::invalid_argument("Speeds and speed limit lists not same size");
  }

  // Speed limit is increased incrementally
  std::vector<double> out;
  for (int i = 0; i < speeds.size(); i++)

  double threshold_speed = 5.0;
  double speed_increment = 2.0;

  double out_speed = std::min(speeds[0], speed_limits[0]);
  out.push_back(out_speed);
  for (int i = 1; i < speeds.size(); i++)
  {
    out.push_back(std::min(speeds[i], speed_limits[i]));
    double min_speed = std::min(speeds[i], speed_limits[i]);
    if ((min_speed - out[i-1])>threshold_speed){
      out_speed = out[i-1] + speed_increment;  
    }
    else{
      // Deceleration is not required to be smooth, for safety reasons.
      out_speed = min_speed;
    }

    out.push_back(out_speed);

}

std::vector<double> WaypointGenerator::apply_accel_limits(std::vector<double> speeds, std::vector<int> regions,
                                                          lanelet::BasicLineString2d centerline, double accel_limit,
                                                          double decel_limit) const
{
  if (speeds.size() != centerline.size())
  {
    throw std::invalid_argument("Speeds and geometry do not match");
  }

  std::vector<double> out{ speeds };
  double SPEED_INCREMENT = 1.0;

  for (int i = 0; i < regions.size() - 1; i++)
  {
    int idx = regions[i];
    int next_idx = std::min(regions[i+1], int(speeds.size()));
    int prev_idx = std::max(0, regions[i-1]);    
    double initial_v = speeds[idx];
    double final_v = speeds[idx + 1];
    double target_v = final_v;
    double delta_v = abs(speeds[idx+1] - speeds[idx]);
    double avg_v = (final_v + initial_v) / 2.0;
    
    auto tuple1 = carma_wm::geometry::matchSegment(centerline[idx], centerline);
    double initial_x = std::get<0>(tuple1).downtrack;

    if (final_v < initial_v)
    {
      // Slowing down
      // deceleration starts at the previous region
      // to reach the desired velocity in time

      // calclate the length of previous region
      auto tuple2 = carma_wm::geometry::matchSegment(centerline[prev_idx], centerline);
      double previous_x = std::get<0>(tuple2).downtrack;
      double dist = abs(previous_x - initial_x);
      // calculate maximum velocity allowed with the deceleration limit over the region
      double allowed_v = sqrt(initial_v*initial_v - 2*decel_limit*dist);
      target_v = std::max(final_v, allowed_v);
      
      // Interpolate the deceleration over the region
      for (int j = prev_idx+1; j <= idx; j++){
        auto tuple_x = carma_wm::geometry::matchSegment(centerline[j], centerline);
        double progress_x = std::get<0>(tuple_x).downtrack;
        double speed_reduction = abs(((progress_x - previous_x)/dist)*(target_v - initial_v));
        double new_vel = std::max(0.0, out[idx] - speed_reduction);
        out[j] = std::min(speeds[j], new_vel);
      }
    }

    if (final_v > initial_v)
    {
      // Speeding up
      // acceleration starts at the current region
      // to reach the desired velocity begore end of region

      // calclate the length of previous region
      auto tuple2 = carma_wm::geometry::matchSegment(centerline[next_idx], centerline);
      double final_x = std::get<0>(tuple2).downtrack;
      double dist = abs(final_x - initial_x);
      // calculate maximum velocity allowed with the deceleration limit
      double allowed_v = sqrt(initial_v*initial_v + 2*accel_limit*dist);
      target_v = std::min(target_v, allowed_v);
      // Interpolate the acceleration over the region
      for (int j = idx + 1; j <= next_idx; j++){
        auto tuple = carma_wm::geometry::matchSegment(centerline[j], centerline);
        double progress_x = std::get<0>(tuple).downtrack;
        double speed_increment = ((progress_x - initial_x)/dist)*(target_v - initial_v);
        double new_vel = out[idx] + speed_increment;
        out[j] = std::min(speeds[j], new_vel);
      }
    }
  }


    

  
  return out;
}


autoware_msgs::LaneArray WaypointGenerator::generate_lane_array_message(
    std::vector<double> speeds, std::vector<geometry_msgs::Quaternion> orientations,
    const lanelet::BasicLineString2d& centerline) const
{
  autoware_msgs::LaneArray out;

  autoware_msgs::Lane lane;
  lane.lane_id = 1;

  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;
  header.stamp = ros::Time::now();
  lane.header = header;
  std::vector<autoware_msgs::Waypoint> waypoints;

    
  for (int j = 0; j < centerline.size(); j++)
  {
    autoware_msgs::Waypoint wp;
    wp.lane_id = lane.lane_id;
    
    geometry_msgs::Pose p;
    p.position.x = centerline[j].x();
    p.position.y = centerline[j].y();
    p.position.z = 0;
    p.orientation = orientations[j];
    wp.pose.pose = p;
    geometry_msgs::Twist t;
    t.linear.x = speeds[j];  // Vehicle's forward velocity corresponds to x
    wp.twist.twist = t;

    wp.change_flag = 0;
    wp.direction = 0;
    wp.cost = 0;
    wp.time_cost = 0;

    /*
    // left undefined until we understand what we need
    wp.wpstate;
    wp.dtlane;
    wp.gid;
    wp.lid;
    wp.right_lane_id;
    wp.stop_line_id;
    wp.lid;
    */

    waypoints.push_back(wp);
  }

  lane.waypoints = waypoints;
  out.lanes.push_back(lane);

  return out;
}
std::vector<double> WaypointGenerator::get_speed_limits(const lanelet::BasicLineString2d& centerline) const
{
  std::vector<double> out;
  if (!_wm)
  {
    ROS_ERROR_STREAM("get_speed_limit: Invalid WM");
    throw std::invalid_argument("get_speed_limit: Inavlid WM");
  }
  if (centerline.size() == 0)
  {
    ROS_ERROR_STREAM("get_speed_limit: Invalid centerline");
    throw std::invalid_argument("get_speed_limit: Empty centerline passed!");
  }
  for (int i = 0; i < centerline.size(); i++)
  {

    auto nearest_lanelet = _wm->getMap()->laneletLayer.nearest(centerline[i], 1);
    if (nearest_lanelet.size() == 0)
    {
      std::string err_msg = "get_speed_limit: No lanelet found matching point #" + std::to_string(i);
      ROS_ERROR_STREAM(err_msg);
      throw std::invalid_argument(err_msg);
    }
    auto slis = nearest_lanelet[0].regulatoryElementsAs<lanelet::DigitalSpeedLimit>();
    if (slis.size() == 0)
    {
      std::string err_msg = "get_speed_limit: Lanalet Id:" + std::to_string(nearest_lanelet[0].id()) + " has no Digital Speed Limit Regulatory Element!";
      ROS_ERROR_STREAM(err_msg);
      throw std::invalid_argument(err_msg);
    }
    out.push_back(slis[0]->getSpeedLimit().value());
  }

  return out;
}

std::vector<lanelet::ConstLanelet> WaypointGenerator::findSuccessingLanelets() const
{
  std::vector<lanelet::ConstLanelet> out;
  auto shortest_path = _wm->getRoute()->shortestPath();
  ROS_DEBUG_STREAM("Processing " << shortest_path.size() << " lanelets.");
  
  for (size_t i=0; i<shortest_path.size(); i++)
  {
    lanelet::ConstLanelet l = shortest_path[i];
    auto connections = _wm->getMapRoutingGraph()->possiblePaths(l, (uint32_t)2, false);
    std::cerr << "Lanelet: " << l.id() << std::endl;

    std::cerr << "Connections size: " << connections.size() << std::endl;
    bool foundSuccessorOnShortestPath = false;
    for (auto path : connections) {
      auto successor = path[1]; // Each path has length of 2 with the second element being the successor
      std::cerr << "successor: " << successor.id() << std::endl;
      for (auto path_lanelet : shortest_path) {
        if (successor.id() == path_lanelet.id()) {
          foundSuccessorOnShortestPath = true;
          std::cerr << "Found valid successor: " << successor.id() << std::endl;
          break;
        }
      }
      if (foundSuccessorOnShortestPath) {
        break;
      }
    }


    if (!foundSuccessorOnShortestPath && i < shortest_path.size() - 1) {
      throw std::invalid_argument("Route contains lane changes");
    }

    out.push_back(l);


    
    
    // std::cerr << "Lanelet: " << l.id() << std::endl;
    // _wm->getRoute()->forEachSuccessor(l, [&](const lanelet::routing::LaneletVisitInformation& info) {
    //   std::cerr << "Successor: " << info.lanelet.id() << std::endl;
    //   std::cerr << "Pred of Successor: " << info.predecessor.id() << std::endl;

    //   if (info.predecessor.id() == l.id()) { // If the predecessor on the shortestpath was current lanelet
    //     auto right = _wm->getRoute()->rightRelation(l);
    //     auto left = _wm->getRoute()->leftRelation(l);
    //     std::cerr << "Found pred" << std::endl;
    //     std::cerr << "Found pred" << std::endl;
    //     std::cerr << "Found pred" << std::endl;

    //     if ((right && right->lanelet.id() != info.lanelet.id()) &&
    //       (left && left->lanelet.id() != info.lanelet.id())) {
    //         std::cerr << "Found on path" << std::endl;
    //         foundSuccessorOnShortestPath = true;
    //         return false; // Exit loop
    //       }
    //   }
    // });

    // auto following = _wm->getRoute()->followingRelations(shortest_path[i-1]);
    // if (foundSuccessorOnShortestPath) {
    //   out.push_back(l);
    // }
    // else{
    //   // TODO: Find an approach for handling transition between non-successing lanelets
    //   auto right = _wm->getRoute()->rightRelation(shortest_path[i-1]);
    //   auto left = _wm->getRoute()->leftRelation(shortest_path[i-1]);
    //   if (right || left)  throw std::invalid_argument("skipping adjacent lanelet");
    //   else throw std::invalid_argument("unidentified relation to lanelet");
    // }
  }
  return out;

}

void WaypointGenerator::new_route_callback()
{
  ROS_DEBUG_STREAM("Received new route message, processing...");

  std::vector<lanelet::ConstLanelet> tmp = findSuccessingLanelets();

  lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(tmp);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("concatenate_lanelets resulted in data: ");

  for (auto p : route_geometry)
  {
    ROS_DEBUG_STREAM(" Point: ( " << p.x() << ", " << p.y() << " )");
  }

  ROS_DEBUG("Processing curvatures...");
  std::vector<double> curvatures = carma_wm::geometry::getLocalCurvatures(tmp); // TODO the accuracy of this call would be far better if using the non-lane change concatenated set

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Computed Curvatures");

  for (auto p : curvatures)
  {   
    ROS_DEBUG_STREAM(" Curvature: " << p);
  }

  std::vector<int> constant_curvature_regions = compute_constant_curvature_regions(curvatures, _config._curvature_epsilon, _config._linearity_constraint);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Curvature Regions");

  for (auto p : constant_curvature_regions)
  {
    ROS_DEBUG_STREAM(" Region: " << p);
  }

  ROS_DEBUG("Normalizing curvatures...");

  std::vector<double> processed_curvatures = normalize_curvature_regions(curvatures, constant_curvature_regions);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Normalized Curvatures");

  for (auto p : processed_curvatures)
  {
    ROS_DEBUG_STREAM(" Curvature: " << p);
  }

  ROS_DEBUG("Processing speeds...");
  std::vector<double> ideal_speeds = compute_ideal_speeds(processed_curvatures, _config._lateral_accel_limit);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Ideal Speeds");

  for (auto p : ideal_speeds)
  {
    ROS_DEBUG_STREAM("IdealSpeed: " << p);
  }

  std::vector<double> accel_limited_speeds = apply_accel_limits(
      ideal_speeds, constant_curvature_regions, route_geometry, _config._longitudinal_accel_limit, _config._longitudinal_decel_limit);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Accel Limited Speeds");

  for (auto p : accel_limited_speeds)
  {
    ROS_DEBUG_STREAM(" Speed: " << p);
  }

  std::vector<double> speed_limits = get_speed_limits(route_geometry);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Speed Limits");

  for (auto p : speed_limits)
  {
    ROS_DEBUG_STREAM(" SpeedLimits: " << p);
  }

  std::vector<double> final_speeds = this->apply_speed_limits(accel_limited_speeds, speed_limits);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Final Speeds");

  for (auto p : final_speeds)
  {
    ROS_DEBUG_STREAM(" Final Speed: " << p);
  }

  ROS_DEBUG("Processing orientations...");
  lanelet::BasicLineString2d centerline = carma_wm::geometry::concatenate_lanelets(tmp);
  std::vector<geometry_msgs::Quaternion> orientations = carma_wm::geometry::compute_tangent_orientations(centerline);

  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM(" ");
  ROS_DEBUG_STREAM("Orientations");

  for (auto p : orientations)
  {
    ROS_DEBUG_STREAM(" Orientation: ( " << p.x << ", " << p.y << ", " << p.z << ", " << p.w << " )");
  }

  // Update current waypoints
  ROS_DEBUG("Generating final waypoint message.");
  autoware_msgs::LaneArray waypoint_msg;
  waypoint_msg = this->generate_lane_array_message(final_speeds, orientations, centerline);
  waypoint_msg = this->downsample_waypoints(waypoint_msg, _config._downsample_ratio);

  ROS_DEBUG_STREAM("Finished processing route.");

  _waypoint_publisher(waypoint_msg);
  ROS_DEBUG_STREAM("Published waypoints list!");
}

autoware_msgs::LaneArray WaypointGenerator::downsample_waypoints(autoware_msgs::LaneArray waypoints, int ratio) const
{
  autoware_msgs::LaneArray downsampled{waypoints};

  for (int i = 0; i < downsampled.lanes.size(); i++) {
    int idx = 0;
    for (auto j = downsampled.lanes[i].waypoints.begin(); j != downsampled.lanes[i].waypoints.end();) {
      if (idx++ % ratio != 0) {
        j = downsampled.lanes[i].waypoints.erase(j);
      } else {
        ++j;
      }
    }
  }

  return downsampled;
}

};  // namespace waypoint_generator