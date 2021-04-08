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
#include <basic_autonomy/basic_autonomy.h>
#include <basic_autonomy/log/log.h>


namespace basic_autonomy
{
namespace waypoint_generation
{
    std::vector<double> apply_speed_limits(const std::vector<double> speeds,
                                                             const std::vector<double> speed_limits)
    {
        ROS_DEBUG_STREAM("Speeds list size: " << speeds.size());
        ROS_DEBUG_STREAM("SpeedLimits list size: " << speed_limits.size());

        if (speeds.size() != speed_limits.size())
        {
            throw std::invalid_argument("Speeds and speed limit lists not same size");
        }
        std::vector<double> out;
        for (size_t i = 0; i < speeds.size(); i++)
        {
            out.push_back(std::min(speeds[i], speed_limits[i]));
        }

        return out;
    }

    Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2)
    {
        Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

        return carma_wm::geometry::build2dEigenTransform(p1, yaw);
    }

    
    std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,
                                                                             double time_span)
    {
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        split_point_speed_pairs(points, &basic_points, &speeds);

        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(basic_points);

        size_t time_boundary_exclusive_index =
            trajectory_utils::time_boundary_index(downtracks, speeds, time_span);

        if (time_boundary_exclusive_index == 0)
        {
            throw std::invalid_argument("No points to fit in timespan"); 
        }

        std::vector<PointSpeedPair> time_bound_points;
        time_bound_points.reserve(time_boundary_exclusive_index);

        if (time_boundary_exclusive_index == points.size())
        {
            time_bound_points.insert(time_bound_points.end(), points.begin(),
                                    points.end());  // All points fit within time boundary
        }
        else
        {
            time_bound_points.insert(time_bound_points.end(), points.begin(),
                                    points.begin() + time_boundary_exclusive_index - 1);  // Limit points by time boundary
        }

        return time_bound_points;
    }

    std::pair<double, size_t> min_with_exclusions(const std::vector<double>& values, const std::unordered_set<size_t>& excluded) 
    {
        double min = std::numeric_limits<double>::max();
        size_t best_idx = -1;
        for (size_t i = 0; i < values.size(); i++)
        {
            if (excluded.find(i) != excluded.end())
            {
            continue;
            }

            if (values[i] < min) {
            min = values[i];
            best_idx = i;
            }
        }
        return std::make_pair(min, best_idx);
    }

    std::vector<double> optimize_speed(const std::vector<double>& downtracks, const std::vector<double>& curv_speeds, double accel_limit)
    {
        if (downtracks.size() != curv_speeds.size())
        {
            throw std::invalid_argument("Downtracks and speeds do not have the same size");
        }

        if (accel_limit <= 0)
        {
            throw std::invalid_argument("Accel limits should be positive");
        }

        bool optimize = true;
        std::unordered_set<size_t> visited_idx;
        visited_idx.reserve(curv_speeds.size());

        std::vector<double> output = curv_speeds;

        while (optimize)
        {
            auto min_pair = min_with_exclusions(curv_speeds, visited_idx);
            size_t min_idx = std::get<1>(min_pair);
            if (min_idx == -1)
            {
                break;
            }

            visited_idx.insert(min_idx); // Mark this point as visited

            double v_i = std::get<0>(min_pair);
            double x_i = downtracks[min_idx];
            for (int i = min_idx - 1; i > 0; i--) 
            { // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow
                                                    //       First point's speed is left unchanged as it is current speed of the vehicle
                double v_f = curv_speeds[i];
                double dv = v_f - v_i;
                
                double x_f = downtracks[i];
                double dx = x_f - x_i;

                if(dv > 0) {
                    v_f = std::min(v_f, sqrt(v_i * v_i - 2 * accel_limit * dx)); // inverting accel as we are only visiting deceleration case
                    visited_idx.insert(i);
                } else if (dv < 0) {
                    break;
                }
                output[i] = v_f;
                v_i = v_f;
                x_i = x_f;
            }
        }

        log::printDoublesPerLineWithPrefix("only_reverse[i]: ", output);
        
        output = trajectory_utils::apply_accel_limits_by_distance(downtracks, output, accel_limit, accel_limit);
        log::printDoublesPerLineWithPrefix("after_forward[i]: ", output);

        return output;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
    const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
    ros::Time startTime)
    {
        if (points.size() != times.size() || points.size() != yaws.size())
        {
            throw std::invalid_argument("All input vectors must have the same size");
        }

        std::vector<cav_msgs::TrajectoryPlanPoint> traj;
        traj.reserve(points.size());

        for (int i = 0; i < points.size(); i++)
        {
            cav_msgs::TrajectoryPlanPoint tpp;
            ros::Duration relative_time(times[i]);
            tpp.target_time = startTime + relative_time;
            tpp.x = points[i].x();
            tpp.y = points[i].y();
            tpp.yaw = yaws[i];

            tpp.controller_plugin_name = "default";
            tpp.planner_plugin_name;// = plugin_discovery_msg_.name;

            traj.push_back(tpp);
        }

        return traj;
    }

    std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                                      double max_starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm,
                                                                      const GeneralTrajConfig& general_config)
    {
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::unordered_set<lanelet::Id> visited_lanelets;

        bool first = true;
        ROS_DEBUG_STREAM("VehDowntrack: " << max_starting_downtrack);
        for (const auto& manuever : maneuvers)
        {
            if (general_config.trajectory_type == "inlanecruising" && manuever.type != cav_msgs::Maneuver::LANE_FOLLOWING)
            {
                throw std::invalid_argument("In-Lane Cruising does not support this maneuver type");
            }

            cav_msgs::LaneFollowingManeuver lane_following_maneuver = manuever.lane_following_maneuver;

            double starting_downtrack = lane_following_maneuver.start_dist;
            if (first)
            {
                if (starting_downtrack > max_starting_downtrack)
                {
                    starting_downtrack = max_starting_downtrack;
                }
            first = false;
            }

            ROS_DEBUG_STREAM("Used downtrack: " << starting_downtrack);

            auto lanelets = wm->getLaneletsBetween(starting_downtrack, lane_following_maneuver.end_dist, true);

            ROS_DEBUG_STREAM("Maneuver");

            lanelet::BasicLineString2d downsampled_centerline;
            downsampled_centerline.reserve(200);

            for (auto l : lanelets)
            {
                ROS_DEBUG_STREAM("Lanelet ID: " << l.id());
                if (visited_lanelets.find(l.id()) == visited_lanelets.end())
                {
                    bool is_turn = false;
                    if(l.hasAttribute("turn_direction")) 
                    {
                        std::string turn_direction = l.attribute("turn_direction").value();
                        is_turn = turn_direction.compare("left") == 0 || turn_direction.compare("right") == 0;
                    }

                    
                    lanelet::BasicLineString2d centerline = l.centerline2d().basicLineString();
                    lanelet::BasicLineString2d downsampled_points;
                    if (is_turn) {
                        downsampled_points = carma_utils::containers::downsample_vector(centerline, general_config.turn_downsample_ratio);
                    } else {
                        downsampled_points = carma_utils::containers::downsample_vector(centerline, general_config.default_downsample_ratio);
                    }
                    downsampled_centerline = carma_wm::geometry::concatenate_line_strings(downsampled_centerline, downsampled_points);
                    visited_lanelets.insert(l.id());
                }
            }


            first = true;
            for (auto p : downsampled_centerline)
            {
                if (first && points_and_target_speeds.size() != 0)
                {
                    first = false;
                    continue;  // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
                }
                PointSpeedPair pair;
                pair.point = p;
                pair.speed = lane_following_maneuver.end_speed;
                points_and_target_speeds.push_back(pair);
            }
        }

        return points_and_target_speeds;
    }

    int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p, veh_point);
            if (distance < min_distance)
            {
                best_index = i;
                min_distance = distance;
            }
            i++;
        }

        return best_index;
    }

    int get_nearest_point_index(const std::vector<PointSpeedPair>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        ROS_DEBUG_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }

        return best_index;
    }

    void split_point_speed_pairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

    std::vector<PointSpeedPair> attach_back_points(const std::vector<PointSpeedPair>& points, 
                          const int nearest_pt_index, std::vector<PointSpeedPair> future_points, double back_distance)
    {
        std::vector<PointSpeedPair> back_and_future;
        back_and_future.reserve(points.size());
        double total_dist = 0;
        int min_i = 0;
        for (int i = nearest_pt_index; i > 0; --i)
        { 
            min_i = i;
            total_dist += lanelet::geometry::distance2d(points[i].point, points[i-1].point);
        
            if (total_dist > back_distance) {
                break;
            }
        }

        back_and_future.insert(back_and_future.end(), points.begin() + min_i, points.begin() + nearest_pt_index + 1);
        back_and_future.insert(back_and_future.end(), future_points.begin(), future_points.end());
        return back_and_future;
    }

    std::unique_ptr<basic_autonomy::smoothing::SplineI> compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
    {
        if (basic_points.size() < 4)
        {
            ROS_WARN_STREAM("Insufficient Spline Points");
            return nullptr;
        }

        std::unique_ptr<basic_autonomy::smoothing::SplineI> spl = std::make_unique<basic_autonomy::smoothing::BSpline>();
        
        spl->setPoints(basic_points);

        return spl;
    }

    double compute_curvature_at(const basic_autonomy::smoothing::SplineI& fit_curve, double step_along_the_curve)
    {
        lanelet::BasicPoint2d f_prime_pt = fit_curve.first_deriv(step_along_the_curve);
        lanelet::BasicPoint2d f_prime_prime_pt = fit_curve.second_deriv(step_along_the_curve);
        // Convert to 3d vector to do 3d vector operations like cross.
        Eigen::Vector3d f_prime = {f_prime_pt.x(), f_prime_pt.y(), 0};
        Eigen::Vector3d f_prime_prime = {f_prime_prime_pt.x(), f_prime_prime_pt.y(), 0};
        return (f_prime.cross(f_prime_prime)).norm()/(pow(f_prime.norm(),3));
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time, const DetailedTrajConfig& detailed_config)
    {
        ROS_DEBUG_STREAM("VehicleState: "
                        << " x: " << state.X_pos_global << " y: " << state.Y_pos_global << " yaw: " << state.orientation
                        << " speed: " << state.longitudinal_vel);

        log::printDebugPerLine(points, &log::pointSpeedPairToStream);

        int nearest_pt_index = get_nearest_point_index(points, state);

        ROS_DEBUG_STREAM("NearestPtIndex: " << nearest_pt_index);

        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position
        auto time_bound_points = constrain_to_time_boundary(future_points, detailed_config.trajectory_time_length);
        
        ROS_DEBUG_STREAM("Got time_bound_points with size:" << time_bound_points.size());
        log::printDebugPerLine(time_bound_points, &log::pointSpeedPairToStream);


        std::vector<PointSpeedPair> back_and_future = attach_back_points(points,nearest_pt_index,time_bound_points, detailed_config.back_distance);

        ROS_DEBUG_STREAM("Got back_and_future points with size" <<back_and_future.size());
        log::printDebugPerLine(back_and_future, &log::pointSpeedPairToStream);

        std::vector<double> speed_limits;
        std::vector<lanelet::BasicPoint2d> curve_points;
        split_point_speed_pairs(back_and_future, &curve_points, &speed_limits);
        
        std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points); // Compute splines based on curve points
        if (!fit_curve)
        {
            throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
        }

        ROS_DEBUG("Got fit");

        ROS_DEBUG_STREAM("speed_limits.size() " << speed_limits.size());

        std::vector<lanelet::BasicPoint2d> all_sampling_points;
        all_sampling_points.reserve(1 + curve_points.size() * 2);

        std::vector<double> distributed_speed_limits;
        distributed_speed_limits.reserve(1 + curve_points.size() * 2);

        // compute total length of the trajectory to get correct number of points 
        // we expect using curve_resample_step_size
        std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

        int total_step_along_curve = static_cast<int>(downtracks_raw.back() / detailed_config.curve_resample_step_size);

        int current_speed_index = 0;
        size_t total_point_size = curve_points.size();

        double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
        double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
        std::vector<double> better_curvature;
        better_curvature.reserve(1 + curve_points.size() * 2);
            
        for (size_t steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
        {
            lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
            
            all_sampling_points.push_back(p);
            double c = compute_curvature_at((*fit_curve), scaled_steps_along_curve);
            better_curvature.push_back(c);
            if ((double)steps_along_curve > step_threshold_for_next_speed)
            {
            step_threshold_for_next_speed += (double)total_step_along_curve / (double) total_point_size;
            current_speed_index ++;
            }
            distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
            scaled_steps_along_curve += 1.0/total_step_along_curve; //adding steps_along_curve_step_size
        }

        ROS_DEBUG_STREAM("Got sampled points with size:" << all_sampling_points.size());
        log::printDebugPerLine(all_sampling_points, &log::basicPointToStream);

        std::vector<double> final_yaw_values = carma_wm::geometry::compute_tangent_orientations(all_sampling_points);

        log::printDoublesPerLineWithPrefix("raw_curvatures[i]: ", better_curvature);

        std::vector<double> curvatures = smoothing::moving_average_filter(better_curvature, detailed_config.curvature_moving_average_window_size, false);

        std::vector<double> ideal_speeds =
            trajectory_utils::constrained_speeds_for_curvatures(curvatures, detailed_config.lateral_accel_limit);
        
        log::printDoublesPerLineWithPrefix("curvatures[i]: ", curvatures);
        log::printDoublesPerLineWithPrefix("ideal_speeds: ", ideal_speeds);
        log::printDoublesPerLineWithPrefix("final_yaw_values[i]: ", final_yaw_values);

        std::vector<double> final_actual_speeds = apply_speed_limits(ideal_speeds, distributed_speed_limits);
        log::printDoublesPerLineWithPrefix("final_actual_speeds: ", final_actual_speeds);

        ROS_DEBUG("Processed all points in computed fit");

        if (all_sampling_points.size() == 0)
        {
            ROS_WARN_STREAM("No trajectory points could be generated");
            return {};
        }

        // Add current vehicle point to front of the trajectory

        nearest_pt_index = get_nearest_point_index(all_sampling_points, state);
        ROS_DEBUG_STREAM("Curvature right now: " << better_curvature[nearest_pt_index] << ", at x: " << state.X_pos_global << ", y: " << state.Y_pos_global);

        std::vector<lanelet::BasicPoint2d> future_basic_points(all_sampling_points.begin() + nearest_pt_index + 1,
                                                    all_sampling_points.end());  // Points in front of current vehicle position

        std::vector<double> future_speeds(final_actual_speeds.begin() + nearest_pt_index + 1,
                                                    final_actual_speeds.end());  // Points in front of current vehicle position
        std::vector<double> future_yaw(final_yaw_values.begin() + nearest_pt_index + 1,
                                                    final_yaw_values.end());  // Points in front of current vehicle position
        final_actual_speeds = future_speeds;
        all_sampling_points = future_basic_points;
        final_yaw_values = future_yaw;

        lanelet::BasicPoint2d cur_veh_point(state.X_pos_global, state.Y_pos_global);

        all_sampling_points.insert(all_sampling_points.begin(),
                                    cur_veh_point);  // Add current vehicle position to front of sample points

        final_actual_speeds.insert(final_actual_speeds.begin(), state.longitudinal_vel);

        final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

        // Compute points to local downtracks
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

        // Apply accel limits
        final_actual_speeds = optimize_speed(downtracks, final_actual_speeds, detailed_config.max_accel);
        

        log::printDoublesPerLineWithPrefix("postAccel[i]: ", final_actual_speeds);

        final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, detailed_config.speed_moving_average_window_size);
        
        log::printDoublesPerLineWithPrefix("post_average[i]: ", final_actual_speeds);

        for (auto& s : final_actual_speeds)  // Limit minimum speed. TODO how to handle stopping?
        {
            s = std::max(s, detailed_config.minimum_speed);
        }

        log::printDoublesPerLineWithPrefix("post_min_speed[i]: ", final_actual_speeds);

        // Convert speeds to times
        std::vector<double> times;
        trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

        log::printDoublesPerLineWithPrefix("times[i]: ", times);
        
        // Build trajectory points
        // TODO When more plugins are implemented that might share trajectory planning the start time will need to be based
        // off the last point in the plan if an earlier plan was provided
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
            trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values, state_time); 

        return traj_points;
    }

    DetailedTrajConfig compose_detailed_trajectory_config( double trajectory_time_length,
                                                        double curve_resample_step_size,
                                                        double minimum_speed,
                                                        double max_accel,
                                                        double lateral_accel_limit, 
                                                        int speed_moving_average_window_size, 
                                                        int curvature_moving_average_window_size,
                                                        double back_distance)
    {
        DetailedTrajConfig detailed_config;

        detailed_config.trajectory_time_length = trajectory_time_length;
        detailed_config.curve_resample_step_size = curve_resample_step_size;
        detailed_config.minimum_speed = minimum_speed;
        detailed_config.max_accel = max_accel;
        detailed_config.lateral_accel_limit = lateral_accel_limit;
        detailed_config.speed_moving_average_window_size = speed_moving_average_window_size;
        detailed_config.curvature_moving_average_window_size = curvature_moving_average_window_size;
        detailed_config.back_distance = back_distance;

        return detailed_config;
    }

    GeneralTrajConfig compose_general_trajectory_config( std::string trajectory_type,
                                                        int default_downsample_ratio,
                                                        int turn_downsample_ratio)
    {
        GeneralTrajConfig general_config;

        general_config.trajectory_type = trajectory_type;
        general_config.default_downsample_ratio = default_downsample_ratio;
        general_config.turn_downsample_ratio = turn_downsample_ratio;
    
        return general_config;
    }



    
}
}