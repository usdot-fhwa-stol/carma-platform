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

#include <basic_autonomy_ros2/log/log.hpp>
#include <basic_autonomy_ros2/helper_functions.hpp>

namespace basic_autonomy
{
    namespace waypoint_generation
    {
         std::vector<PointSpeedPair> create_geometry_profile(const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers, double max_starting_downtrack,const carma_wm::WorldModelConstPtr &wm,
                                                                   carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,const carma_planning_msgs::msg::VehicleState& state,
                                                                   const GeneralTrajConfig &general_config, const DetailedTrajConfig &detailed_config){
            std::vector<PointSpeedPair> points_and_target_speeds;
            
            bool first = true;
            std::unordered_set<lanelet::Id> visited_lanelets;

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "VehDowntrack:"<<max_starting_downtrack);
            for(const auto &maneuver : maneuvers)
            {
                double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
                
                if(first){
                    starting_downtrack = std::min(starting_downtrack, max_starting_downtrack);
                    first = false;
                }
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Used downtrack: " << starting_downtrack);

                if(maneuver.type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING){
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER),"Creating Lane Follow Geometry");
                    std::vector<PointSpeedPair> lane_follow_points = create_lanefollow_geometry(maneuver, starting_downtrack, wm, general_config, detailed_config, visited_lanelets);
                    points_and_target_speeds.insert(points_and_target_speeds.end(), lane_follow_points.begin(), lane_follow_points.end());
                }
                else if(maneuver.type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE){
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Creating Lane Change Geometry");
                    std::vector<PointSpeedPair> lane_change_points = get_lanechange_points_from_maneuver(maneuver, starting_downtrack, wm, ending_state_before_buffer, state, general_config, detailed_config);
                    points_and_target_speeds.insert(points_and_target_speeds.end(), lane_change_points.begin(), lane_change_points.end());
                }
                else{
                    throw std::invalid_argument("This maneuver type is not supported");
                }
                
            }

            //Add buffer ending to lane follow points at the end of maneuver(s) end dist 
            if(maneuvers.back().type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING){
                points_and_target_speeds = add_lanefollow_buffer(wm, points_and_target_speeds, maneuvers, ending_state_before_buffer, detailed_config);

            }
            return points_and_target_speeds;

        }

        std::vector<PointSpeedPair> create_lanefollow_geometry(const carma_planning_msgs::msg::Maneuver &maneuver, double starting_downtrack,
                                                                const carma_wm::WorldModelConstPtr &wm, const GeneralTrajConfig &general_config, 
                                                                const DetailedTrajConfig &detailed_config, std::unordered_set<lanelet::Id> &visited_lanelets)
        {
            if(maneuver.type != carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING){
                throw std::invalid_argument("Create_lanefollow called on a maneuver type which is not LANE_FOLLOW");
            }
            std::vector<PointSpeedPair> points_and_target_speeds;

            carma_planning_msgs::msg::LaneFollowingManeuver lane_following_maneuver = maneuver.lane_following_maneuver;
            
            auto lanelets = wm->getLaneletsBetween(starting_downtrack, lane_following_maneuver.end_dist + detailed_config.buffer_ending_downtrack, true, true);

            if (lanelets.empty())
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Detected no lanelets between starting downtrack: "<< starting_downtrack << ", and lane_following_maneuver.end_dist: "<< lane_following_maneuver.end_dist);
                throw std::invalid_argument("Detected no lanelets between starting_downtrack and end_dist");
            }

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Maneuver");

            lanelet::BasicLineString2d downsampled_centerline;
            // 400 value here is an arbitrary attempt at improving inlane-cruising performance by reducing copy operations. 
            // Value picked based on annecdotal evidence from STOL system testing
            downsampled_centerline.reserve(400);
            
            //getLaneletsBetween is inclusive of lanelets between its two boundaries
            //which may return lanechange lanelets, so
            //exclude lanechanges and plan for only the straight part
            size_t curr_idx = 0;
            auto following_lanelets = wm->getMapRoutingGraph()->following(lanelets[curr_idx]);
            lanelet::ConstLanelets straight_lanelets;

            if(lanelets.size() <= 1) //no lane change anyways if only size 1
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Detected one straight lanelet Id:" << lanelets[curr_idx].id());
                straight_lanelets = lanelets;
            }
            else
            {
                // skip all lanechanges until lane follow starts
                while (curr_idx + 1 < lanelets.size() && 
                        std::find(following_lanelets.begin(),following_lanelets.end(), lanelets[curr_idx + 1]) == following_lanelets.end())
                {
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "As there were no directly following lanelets after this, skipping lanelet id: " << lanelets[curr_idx].id());
                    curr_idx ++;
                    following_lanelets = wm->getMapRoutingGraph()->following(lanelets[curr_idx]);
                }

                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Added lanelet Id for lane follow: " << lanelets[curr_idx].id());
                // guaranteed to have at least one "straight" lanelet (e.g the last one in the list)
                straight_lanelets.push_back(lanelets[curr_idx]);
                      // add all lanelets on the straight road until next lanechange
                while (curr_idx + 1 < lanelets.size() && 
                        std::find(following_lanelets.begin(),following_lanelets.end(), lanelets[curr_idx + 1]) != following_lanelets.end())
                {
                    curr_idx++;
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Added lanelet Id forlane follow: " << lanelets[curr_idx].id());
                    straight_lanelets.push_back(lanelets[curr_idx]);
                    following_lanelets = wm->getMapRoutingGraph()->following(lanelets[curr_idx]);
                }
                
            }
            
            for (auto l : straight_lanelets)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Processing lanelet ID: " << l.id());
                if (visited_lanelets.find(l.id()) == visited_lanelets.end())
                {

                    bool is_turn = false;
                    if(l.hasAttribute("turn_direction")) {
                        std::string turn_direction = l.attribute("turn_direction").value();
                        is_turn = turn_direction.compare("left") == 0 || turn_direction.compare("right") == 0;
                    }
                    
                    lanelet::BasicLineString2d centerline = l.centerline2d().basicLineString();
                    lanelet::BasicLineString2d downsampled_points;
                    if (is_turn) {
                        downsampled_points = carma_ros2_utils::containers::downsample_vector(centerline, general_config.turn_downsample_ratio);
                    } else {
                        downsampled_points = carma_ros2_utils::containers::downsample_vector(centerline, general_config.default_downsample_ratio);
                    }
                    
                    if(downsampled_centerline.size() != 0 && downsampled_points.size() != 0 // If this is not the first lanelet and the points are closer than 1m drop the first point to prevent overlap
                    && lanelet::geometry::distance2d(downsampled_points.front(), downsampled_centerline.back()) <1.2){
                        downsampled_points = lanelet::BasicLineString2d(downsampled_points.begin() + 1, downsampled_points.end());
                    }

                    downsampled_centerline = carma_wm::geometry::concatenate_line_strings(downsampled_centerline, downsampled_points);
                    visited_lanelets.insert(l.id());
                }
            }

            bool first = true;
            for (auto p : downsampled_centerline)
            {
                if (first && !points_and_target_speeds.empty())
                {
                    first = false;
                    continue; // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
                }
                PointSpeedPair pair;
                pair.point = p;
                pair.speed = lane_following_maneuver.end_speed;
                points_and_target_speeds.push_back(pair);
            }
            
            return points_and_target_speeds;

        }

        std::vector<PointSpeedPair> add_lanefollow_buffer(const carma_wm::WorldModelConstPtr &wm, std::vector<PointSpeedPair>& points_and_target_speeds, const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers,
             carma_planning_msgs::msg::VehicleState &ending_state_before_buffer, const DetailedTrajConfig &detailed_config){
            

            double starting_route_downtrack = wm->routeTrackPos(points_and_target_speeds.front().point).downtrack;

            // Always try to add the maximum buffer. Even if the route ends it may still be possible to add buffered points.
            // This does mean that downstream components might not be able to assume the buffer points are on the route 
            // though this is not likely to be an issue as they are buffer only
            double ending_downtrack = maneuvers.back().lane_following_maneuver.end_dist + detailed_config.buffer_ending_downtrack;

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Add lanefollow buffer: ending_downtrack: " << ending_downtrack << ", maneuvers.back().lane_following_maneuver.end_dist: " << maneuvers.back().lane_following_maneuver.end_dist <<
                            ", detailed_config.buffer_ending_downtrack: " << detailed_config.buffer_ending_downtrack);

            size_t max_i = points_and_target_speeds.size() - 1;
            size_t unbuffered_idx = points_and_target_speeds.size() - 1;
            bool found_unbuffered_idx = false;
            double dist_accumulator = starting_route_downtrack;
            lanelet::BasicPoint2d prev_point;

            boost::optional<lanelet::BasicPoint2d> delta_point;
            for (size_t i = 0; i < points_and_target_speeds.size(); ++i) {
                auto current_point = points_and_target_speeds[i].point;
                
                if (i == 0) {
                    prev_point = current_point;
                    continue;
                }

                double delta_d = lanelet::geometry::distance2d(prev_point, current_point);

                dist_accumulator += delta_d;
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Index i: " << i << ", delta_d: " << delta_d << ", dist_accumulator:" << dist_accumulator <<", current_point.x():" << current_point.x() << 
                "current_point.y():" << current_point.y());
                if (dist_accumulator > maneuvers.back().lane_following_maneuver.end_dist && !found_unbuffered_idx)
                {
                    unbuffered_idx = i - 1;
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Found index unbuffered_idx at: " << unbuffered_idx);
                    found_unbuffered_idx = true;
                }
                
                if (dist_accumulator > ending_downtrack) {
                    max_i = i;
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Max_i breaking at: i: " << i << ", max_i: " << max_i);
                    break;
                }

                // If there are no more points to add but we haven't reached the ending downtrack then
                // construct an extrapolated straight line from the final point and keep adding to this line until the downtrack is met
                // Since this is purely needed to allow for a spline fit edge case, it should have minimal impact on the actual steering behavior of the vehicle
                if (i == points_and_target_speeds.size() - 1) // dist_accumulator < ending_downtrack is guaranteed by earlier conditional
                {

                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Extending trajectory using buffer beyond end of target lanelet");

                    if (!delta_point) { // Set the step size based on last two points
                        delta_point = (current_point - prev_point) * 0.25; // Use a smaller step size then default to help ensure enough points are generated;
                    }

                    // Create an extrapolated new point 
                    auto new_point = current_point + delta_point.get();

                    PointSpeedPair new_pair;
                    new_pair.point = new_point;
                    new_pair.speed = points_and_target_speeds.back().speed;

                    points_and_target_speeds.push_back(new_pair);
                }

                prev_point = current_point;
            }

            ending_state_before_buffer.x_pos_global = points_and_target_speeds[unbuffered_idx].point.x();
            ending_state_before_buffer.y_pos_global = points_and_target_speeds[unbuffered_idx].point.y();
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Here ending_state_before_buffer.x_pos_global: " << ending_state_before_buffer.x_pos_global << 
            ", and y_pos_global" << ending_state_before_buffer.y_pos_global);

            std::vector<PointSpeedPair> constrained_points(points_and_target_speeds.begin(), points_and_target_speeds.begin() + max_i);

            return constrained_points;
        }

        std::vector<lanelet::BasicPoint2d> create_lanechange_geometry(lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double starting_downtrack, double ending_downtrack,
                                                                   const carma_wm::WorldModelConstPtr &wm, int downsample_ratio, double buffer_ending_downtrack)
        {
            std::vector<lanelet::BasicPoint2d> centerline_points;

            //Get starting lanelet and ending lanelets
            lanelet::ConstLanelet starting_lanelet = wm->getMap()->laneletLayer.get(starting_lane_id);
            lanelet::ConstLanelet ending_lanelet = wm->getMap()->laneletLayer.get(ending_lane_id);
            
            lanelet::ConstLanelets starting_lane;
            starting_lane.push_back(starting_lanelet);

            std::vector<lanelet::BasicPoint2d> reference_centerline;
            // 400 value here is an arbitrary attempt at improving performance by reducing copy operations. 
            // Value picked based on annecdotal evidence from STOL system testing
            reference_centerline.reserve(400);
            bool shared_boundary_found = false;
            bool is_lanechange_left = false;
            
            lanelet::BasicLineString2d current_lanelet_centerline = starting_lanelet.centerline2d().basicLineString();
            lanelet::ConstLanelet current_lanelet = starting_lanelet;
            reference_centerline.insert(reference_centerline.end(), current_lanelet_centerline.begin(), current_lanelet_centerline.end());

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Searching for shared boundary with starting lanechange lanelet " << std::to_string(current_lanelet.id()) << " and ending lanelet " << std::to_string(ending_lanelet.id()));
            while(!shared_boundary_found){
                //Assumption- Adjacent lanelets share lane boundary
                if(current_lanelet.leftBound() == ending_lanelet.rightBound()){   
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Lanelet " << std::to_string(current_lanelet.id()) << " shares left boundary with " << std::to_string(ending_lanelet.id()));
                    is_lanechange_left = true;
                    shared_boundary_found = true;
                }

                else if(current_lanelet.rightBound() == ending_lanelet.leftBound()){
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Lanelet " << std::to_string(current_lanelet.id()) << " shares right boundary with " << std::to_string(ending_lanelet.id()));
                    shared_boundary_found = true;
                }

                else{
                    //If there are no following lanelets on route, lanechange should be completing before reaching it
                    if(wm->getMapRoutingGraph()->following(current_lanelet, false).empty())
                    {
                        // Maneuver requires we travel further before completing lane change, but no routable lanelet directly ahead
                        //In this case we have reached a lanelet which does not have a routable lanelet ahead + isn't adjacent to the lanelet where lane change ends
                        //A lane change should have already happened at this point
                        throw(std::invalid_argument("No following lanelets from current lanelet reachable without a lane change, incorrectly chosen end lanelet"));
                    }

                    current_lanelet = wm->getMapRoutingGraph()->following(current_lanelet, false).front(); 
                    if(current_lanelet.id() == starting_lanelet.id()){
                        //Looped back to starting lanelet
                        throw(std::invalid_argument("No lane change in path"));
                    }
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Now checking for shared lane boundary with lanelet " << std::to_string(current_lanelet.id()) << " and ending lanelet " << std::to_string(ending_lanelet.id()));
                    auto current_lanelet_linestring = current_lanelet.centerline2d().basicLineString();   
                    //Concatenate linestring starting from + 1 to avoid overlap 
                    reference_centerline.insert(reference_centerline.end(), current_lanelet_linestring.begin() + 1, current_lanelet_linestring.end());
                    starting_lane.push_back(current_lanelet);
                }
            }

            // Create the target lane centerline using lanelets adjacent to the lanechange lanelets in the starting lane
            std::vector<lanelet::BasicPoint2d> target_lane_centerline;
            for(size_t i = 0;i<starting_lane.size();++i){
                lanelet::ConstLanelet curr_end_lanelet;
                
                if(is_lanechange_left){
                    
                    //get left lanelet
                    if(wm->getMapRoutingGraph()->left(starting_lane[i])){
                        curr_end_lanelet = wm->getMapRoutingGraph()->left(starting_lane[i]).get();
                    }
                    else{
                        curr_end_lanelet = wm->getMapRoutingGraph()->adjacentLeft(starting_lane[i]).get();
                    }
                }
                else{

                    //get right lanelet
                    if(wm->getMapRoutingGraph()->right(starting_lane[i])){
                        curr_end_lanelet = wm->getMapRoutingGraph()->right(starting_lane[i]).get();
                    }
                    else{
                        curr_end_lanelet = wm->getMapRoutingGraph()->adjacentRight(starting_lane[i]).get();
                    }
                }
                
                auto target_lane_linestring = curr_end_lanelet.centerline2d().basicLineString();
                //Concatenate linestring starting from + 1 to avoid overlap
                target_lane_centerline.insert(target_lane_centerline.end(), target_lane_linestring.begin() + 1, target_lane_linestring.end());
                
            }

            //Downsample centerlines
            // 400 value here is an arbitrary attempt at improving performance by reducing copy operations. 
            // Value picked based on annecdotal evidence from STOL system testing

            std::vector<lanelet::BasicPoint2d> downsampled_starting_centerline;
            downsampled_starting_centerline.reserve(400);
            downsampled_starting_centerline = carma_ros2_utils::containers::downsample_vector(reference_centerline, downsample_ratio);

            std::vector<lanelet::BasicPoint2d> downsampled_target_centerline;
            downsampled_target_centerline.reserve(400);
            downsampled_target_centerline = carma_ros2_utils::containers::downsample_vector(target_lane_centerline, downsample_ratio);

            //If points are not the same size - resample to ensure same size along both centerlines
            if(downsampled_starting_centerline.size() != downsampled_target_centerline.size())
            {
                auto centerlines = resample_linestring_pair_to_same_size(downsampled_starting_centerline, downsampled_target_centerline);
                downsampled_starting_centerline = centerlines[0];
                downsampled_target_centerline = centerlines[1];

            }

            //Constrain to starting and ending downtrack
            int start_index_starting_centerline = waypoint_generation::get_nearest_index_by_downtrack(downsampled_starting_centerline, wm, starting_downtrack);
            carma_planning_msgs::msg::VehicleState start_state;
            start_state.x_pos_global = downsampled_starting_centerline[start_index_starting_centerline].x();
            start_state.y_pos_global = downsampled_starting_centerline[start_index_starting_centerline].y();
            int start_index_target_centerline = waypoint_generation::get_nearest_point_index(downsampled_target_centerline, start_state);

            int end_index_target_centerline = waypoint_generation::get_nearest_index_by_downtrack(downsampled_target_centerline, wm, ending_downtrack);
            carma_planning_msgs::msg::VehicleState end_state;
            end_state.x_pos_global = downsampled_target_centerline[end_index_target_centerline].x();
            end_state.y_pos_global = downsampled_target_centerline[end_index_target_centerline].y();
            int end_index_starting_centerline = waypoint_generation::get_nearest_point_index(downsampled_starting_centerline, end_state);

            std::vector<lanelet::BasicPoint2d> constrained_start_centerline(downsampled_starting_centerline.begin() + start_index_starting_centerline, downsampled_starting_centerline.begin() + end_index_starting_centerline);
            std::vector<lanelet::BasicPoint2d> constrained_target_centerline(downsampled_target_centerline.begin() + start_index_target_centerline, downsampled_target_centerline.begin() + end_index_target_centerline);

            //Create Trajectory geometry
            double delta_step = 1.0 / constrained_start_centerline.size();

            for (size_t i = 0; i < constrained_start_centerline.size(); ++i)
            {
                lanelet::BasicPoint2d current_position;
                lanelet::BasicPoint2d start_lane_pt = constrained_start_centerline[i];
                lanelet::BasicPoint2d target_lane_pt = constrained_target_centerline[i];
                double delta = delta_step * i;
                current_position.x() = target_lane_pt.x() * delta + (1 - delta) * start_lane_pt.x();
                current_position.y() = target_lane_pt.y() * delta + (1 - delta) * start_lane_pt.y();

                centerline_points.push_back(current_position);
            }

            // Add points from the remaining length of the target lanelet to provide sufficient distance for adding buffer
            double dist_to_target_lane_end = lanelet::geometry::distance2d(centerline_points.back(), downsampled_target_centerline.back());
            centerline_points.insert(centerline_points.end(), downsampled_target_centerline.begin() + end_index_target_centerline, downsampled_target_centerline.end());
            
            // If the additional distance from the remaining length of the target lanelet does not provide more than the required
            // buffer_ending_downtrack, then also add points from the lanelet following the target lanelet
            if (dist_to_target_lane_end < buffer_ending_downtrack) {
                auto following_lanelets = wm->getMapRoutingGraph()->following(ending_lanelet, false);
                if(!following_lanelets.empty()){
                    //Arbitrarily choosing first following lanelet for buffer since points are only being used to fit spline
                    auto following_lanelet_centerline = following_lanelets.front().centerline2d().basicLineString();
                    centerline_points.insert(centerline_points.end(), following_lanelet_centerline.begin(), 
                                                                                following_lanelet_centerline.end());
                }
            }

            return centerline_points;
        }

       std::vector<std::vector<lanelet::BasicPoint2d>> resample_linestring_pair_to_same_size(std::vector<lanelet::BasicPoint2d>& line_1, std::vector<lanelet::BasicPoint2d>& line_2){
            
            auto start_time = std::chrono::high_resolution_clock::now(); // Start timing the execution time for planning so it can be logged

            std::vector<std::vector<lanelet::BasicPoint2d>> output;
            
            //Fit centerlines to a spline
            std::unique_ptr<smoothing::SplineI> fit_curve_1 = compute_fit(line_1); // Compute splines based on curve points
            if (!fit_curve_1)
            {
                throw std::invalid_argument("Could not fit a spline curve along the starting_lane centerline points!");
            }

            std::unique_ptr<smoothing::SplineI> fit_curve_2 = compute_fit(line_2); // Compute splines based on curve points
            if (!fit_curve_2)
            {
                throw std::invalid_argument("Could not fit a spline curve along the ending_lane centerline points!");
            }

            //Sample spline to get centerlines of equal size
            std::vector<lanelet::BasicPoint2d> all_sampling_points_line1;
            std::vector<lanelet::BasicPoint2d> all_sampling_points_line2;

            size_t total_point_size = std::min(line_1.size(), line_2.size());

            all_sampling_points_line1.reserve(1 + total_point_size * 2);
            std::vector<double> downtracks_raw_line1 = carma_wm::geometry::compute_arc_lengths(line_1);
            //int total_step_along_curve1 = static_cast<int>(downtracks_raw_line1.back() / 2.0);
            //double step_threshold_line1 = (double)total_step_along_curve1 / (double)total_point_size;
            //TODO: are we missing some computation here?  step_threshold_line1 and step_threshold_line2 are not used anywhere
            //      and these calcs can be deleted (see below also).
            
            all_sampling_points_line2.reserve(1 + total_point_size * 2);
            std::vector<double> downtracks_raw_line2 = carma_wm::geometry::compute_arc_lengths(line_2);
            //TODO: unused variable: int total_step_along_curve2 = static_cast<int>(downtracks_raw_line2.back() / 2.0);
            //TODO: unused variable: double step_threshold_line2 = (double)total_step_along_curve2 / (double)total_point_size;

            double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
            
            
            all_sampling_points_line2.reserve(1 + total_point_size * 2);
            
            for(size_t i = 0;i<total_point_size; ++i){
                lanelet::BasicPoint2d p1 = (*fit_curve_1)(scaled_steps_along_curve);
                lanelet::BasicPoint2d p2 = (*fit_curve_2)(scaled_steps_along_curve);
                all_sampling_points_line1.push_back(p1);
                all_sampling_points_line2.push_back(p2);

                scaled_steps_along_curve += 1.0 / total_point_size;  //adding steps_along_curve_step_size
            }

            output.push_back(all_sampling_points_line1);
            output.push_back(all_sampling_points_line2);

            auto end_time = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "ExecutionTime for resample lane change centerlines: " << duration.count() << " milliseconds");

            return output;
        }
        
        std::vector<PointSpeedPair> get_lanechange_points_from_maneuver(const carma_planning_msgs::msg::Maneuver &maneuver, double starting_downtrack,
                                                                   const carma_wm::WorldModelConstPtr &wm, carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,
                                                                    const carma_planning_msgs::msg::VehicleState &state, const GeneralTrajConfig &general_config,const DetailedTrajConfig &detailed_config)
        {
            if(maneuver.type != carma_planning_msgs::msg::Maneuver::LANE_CHANGE){
                throw std::invalid_argument("Create_lanechange called on a maneuver type which is not LANE_CHANGE");
            }
            std::vector<PointSpeedPair> points_and_target_speeds;
            std::unordered_set<lanelet::Id> visited_lanelets;

            carma_planning_msgs::msg::LaneChangeManeuver lane_change_maneuver = maneuver.lane_change_maneuver;
            double ending_downtrack = lane_change_maneuver.end_dist;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Maneuver ending downtrack:"<<ending_downtrack);
            if(starting_downtrack >= ending_downtrack)
            {
                throw(std::invalid_argument("Start distance is greater than or equal to ending distance"));
            }

            //get route between starting and ending downtracks - downtracks should be constant for complete length of maneuver
            std::vector<lanelet::BasicPoint2d> route_geometry = create_lanechange_geometry(std::stoi(lane_change_maneuver.starting_lane_id),std::stoi(lane_change_maneuver.ending_lane_id),
                                                                                        starting_downtrack, ending_downtrack, wm, general_config.default_downsample_ratio, detailed_config.buffer_ending_downtrack);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Route geometry size:"<<route_geometry.size());

            lanelet::BasicPoint2d state_pos(state.x_pos_global, state.y_pos_global);
            double current_downtrack = wm->routeTrackPos(state_pos).downtrack;
            int nearest_pt_index = get_nearest_index_by_downtrack(route_geometry, wm, current_downtrack);
            int ending_pt_index = get_nearest_index_by_downtrack(route_geometry, wm, ending_downtrack);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Nearest pt index in maneuvers to points: "<< nearest_pt_index);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Ending pt index in maneuvers to points: "<< ending_pt_index);

            ending_state_before_buffer.x_pos_global = route_geometry[ending_pt_index].x();
            ending_state_before_buffer.y_pos_global = route_geometry[ending_pt_index].y();

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "ending_state_before_buffer_:"<<ending_state_before_buffer.x_pos_global << 
                    ", ending_state_before_buffer_.y_pos_global" << ending_state_before_buffer.y_pos_global);

            
            double route_length = wm->getRouteEndTrackPos().downtrack;

            if (ending_downtrack + detailed_config.buffer_ending_downtrack < route_length)
            {
                ending_pt_index = get_nearest_index_by_downtrack(route_geometry, wm, ending_downtrack + detailed_config.buffer_ending_downtrack);
            }
            else
            {
                ending_pt_index = route_geometry.size() - 1;
            }

            lanelet::BasicLineString2d future_route_geometry(route_geometry.begin() + nearest_pt_index, route_geometry.begin() + ending_pt_index);
            bool first = true;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Future geom size:"<< future_route_geometry.size());

            for (auto p : future_route_geometry)
            {
                if (first && !points_and_target_speeds.empty())
                {
                    first = false;
                    continue; // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
                }
                PointSpeedPair pair;
                pair.point = p;
                //If current speed is above min speed, keep at current speed. Otherwise use end speed from maneuver.
                pair.speed = (state.longitudinal_vel > detailed_config.minimum_speed) ? state.longitudinal_vel : lane_change_maneuver.end_speed;
                points_and_target_speeds.push_back(pair);
                
            }
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Const speed assigned:"<<points_and_target_speeds.back().speed);
            return points_and_target_speeds;
            

        } 

        std::vector<double> apply_speed_limits(const std::vector<double> speeds,
                                               const std::vector<double> speed_limits)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Speeds list size: " << speeds.size());
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "SpeedLimits list size: " << speed_limits.size());

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

        Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d &p1,
                                                const lanelet::BasicPoint2d &p2)
        {
            Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

            return carma_wm::geometry::build2dEigenTransform(p1, yaw);
        }

        std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair> &points,
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
                                         points.end()); // All points fit within time boundary
            }
            else
            {
                time_bound_points.insert(time_bound_points.end(), points.begin(),
                                         points.begin() + time_boundary_exclusive_index - 1); // Limit points by time boundary
            }

            return time_bound_points;
        }

        std::pair<double, size_t> min_with_exclusions(const std::vector<double> &values, const std::unordered_set<size_t> &excluded)
        {
            double min = std::numeric_limits<double>::max();
            size_t best_idx = -1;
            for (size_t i = 0; i < values.size(); i++)
            {
                if (excluded.find(i) != excluded.end())
                {
                    continue;
                }

                if (values[i] < min)
                {
                    min = values[i];
                    best_idx = i;
                }
            }
            return std::make_pair(min, best_idx);
        }

        std::vector<double> optimize_speed(const std::vector<double> &downtracks, const std::vector<double> &curv_speeds, double accel_limit)
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
                int min_idx = std::get<1>(min_pair);
                if (min_idx == -1)
                {
                    break;
                }

                visited_idx.insert(min_idx); // Mark this point as visited

                double v_i = std::get<0>(min_pair);
                double x_i = downtracks[min_idx];
                for (int i = min_idx - 1; i > 0; i--)
                {   // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow
                    //       First point's speed is left unchanged as it is current speed of the vehicle
                    double v_f = curv_speeds[i];
                    double dv = v_f - v_i;

                    double x_f = downtracks[i];
                    double dx = x_f - x_i;

                    if (dv > 0)
                    {
                        v_f = std::min(v_f, sqrt(v_i * v_i - 2 * accel_limit * dx)); // inverting accel as we are only visiting deceleration case
                        visited_idx.insert(i);
                    }
                    else if (dv < 0)
                    {
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

        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
            const std::vector<lanelet::BasicPoint2d> &points, const std::vector<double> &times, const std::vector<double> &yaws,
            rclcpp::Time startTime)
        {
            if (points.size() != times.size() || points.size() != yaws.size())
            {
                throw std::invalid_argument("All input vectors must have the same size");
            }

            std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj;
            traj.reserve(points.size());

            for (size_t i = 0; i < points.size(); i++)
            {
                carma_planning_msgs::msg::TrajectoryPlanPoint tpp;
                rclcpp::Duration relative_time(times[i] * 1e9); // Conversion of times[i] from seconds to nanoseconds
                tpp.target_time = startTime + relative_time;
                tpp.x = points[i].x();
                tpp.y = points[i].y();
                tpp.yaw = yaws[i];

                tpp.controller_plugin_name = "default";
                //tpp.planner_plugin_name        //Planner plugin name is filled in the tactical plugin

                traj.push_back(tpp);
            }

            return traj;
        }


        std::vector<PointSpeedPair> attach_past_points(const std::vector<PointSpeedPair> &points_set, std::vector<PointSpeedPair> future_points,
                                                       const int nearest_pt_index,  double back_distance)
        {
            std::vector<PointSpeedPair> back_and_future;
            back_and_future.reserve(points_set.size());
            double total_dist = 0;
            int min_i = 0;

            // int must be used here to avoid overflow when i = 0
            for (int i = nearest_pt_index; i >= 0; --i)
            {
                min_i = i;
                total_dist += lanelet::geometry::distance2d(points_set[i].point, points_set[i - 1].point);

                if (total_dist > back_distance)
                {
                    break;
                }
            }

            back_and_future.insert(back_and_future.end(), points_set.begin() + min_i, points_set.begin() + nearest_pt_index + 1);
            back_and_future.insert(back_and_future.end(), future_points.begin(), future_points.end());
            return back_and_future;
        }

        std::unique_ptr<basic_autonomy::smoothing::SplineI> compute_fit(const std::vector<lanelet::BasicPoint2d> &basic_points)
        {
            if (basic_points.size() < 4)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Insufficient Spline Points");
                return nullptr;
            }

            std::unique_ptr<basic_autonomy::smoothing::SplineI> spl = std::make_unique<basic_autonomy::smoothing::BSpline>();

            spl->setPoints(basic_points);

            return spl;
        }

        double compute_curvature_at(const basic_autonomy::smoothing::SplineI &fit_curve, double step_along_the_curve)
        {
            lanelet::BasicPoint2d f_prime_pt = fit_curve.first_deriv(step_along_the_curve);
            lanelet::BasicPoint2d f_prime_prime_pt = fit_curve.second_deriv(step_along_the_curve);
            // Convert to 3d vector to do 3d vector operations like cross.
            Eigen::Vector3d f_prime = {f_prime_pt.x(), f_prime_pt.y(), 0};
            Eigen::Vector3d f_prime_prime = {f_prime_prime_pt.x(), f_prime_prime_pt.y(), 0};
            return (f_prime.cross(f_prime_prime)).norm() / (pow(f_prime.norm(), 3));
        }

        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> compose_lanefollow_trajectory_from_path(
            const std::vector<PointSpeedPair> &points, const carma_planning_msgs::msg::VehicleState &state, const rclcpp::Time &state_time, const carma_wm::WorldModelConstPtr &wm,
            const carma_planning_msgs::msg::VehicleState &ending_state_before_buffer, carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds& debug_msg, const DetailedTrajConfig &detailed_config)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "VehicleState: "
                             << " x: " << state.x_pos_global << " y: " << state.y_pos_global << " yaw: " << state.orientation
                             << " speed: " << state.longitudinal_vel);

            log::printDebugPerLine(points, &log::pointSpeedPairToStream);

            int nearest_pt_index = get_nearest_point_index(points, state);

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "NearestPtIndex: " << nearest_pt_index);

            std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position
            auto time_bound_points = constrain_to_time_boundary(future_points, detailed_config.trajectory_time_length);

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Got time_bound_points with size:" << time_bound_points.size());
            log::printDebugPerLine(time_bound_points, &log::pointSpeedPairToStream);

            std::vector<PointSpeedPair> back_and_future = attach_past_points(points, time_bound_points, nearest_pt_index, detailed_config.back_distance);

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Got back_and_future points with size" << back_and_future.size());
            log::printDebugPerLine(back_and_future, &log::pointSpeedPairToStream);

            std::vector<double> speed_limits;
            std::vector<lanelet::BasicPoint2d> curve_points;
            split_point_speed_pairs(back_and_future, &curve_points, &speed_limits);

            std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points); // Compute splines based on curve points
            if (!fit_curve)
            {
                throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
            }

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Got fit");

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "speed_limits.size() " << speed_limits.size());

            std::vector<lanelet::BasicPoint2d> all_sampling_points;
            all_sampling_points.reserve(1 + curve_points.size() * 2);

            std::vector<double> distributed_speed_limits;
            distributed_speed_limits.reserve(1 + curve_points.size() * 2);

            // compute total length of the trajectory to get correct number of points
            // we expect using curve_resample_step_size
            std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

            auto total_step_along_curve = static_cast<int>(downtracks_raw.back() / detailed_config.curve_resample_step_size);

            int current_speed_index = 0;
            size_t total_point_size = curve_points.size();

            double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
            double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
            std::vector<double> better_curvature;
            better_curvature.reserve(1 + curve_points.size() * 2);

            for (int steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
            {
                lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);

                all_sampling_points.push_back(p);
                double c = compute_curvature_at((*fit_curve), scaled_steps_along_curve);
                better_curvature.push_back(c);
                if ((double)steps_along_curve > step_threshold_for_next_speed)
                {
                    step_threshold_for_next_speed += (double)total_step_along_curve / (double)total_point_size;
                    current_speed_index++;
                }
                distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
                scaled_steps_along_curve += 1.0 / total_step_along_curve;              //adding steps_along_curve_step_size
            }

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Got sampled points with size:" << all_sampling_points.size());
            log::printDebugPerLine(all_sampling_points, &log::basicPointToStream);

            std::vector<double> final_yaw_values = carma_wm::geometry::compute_tangent_orientations(all_sampling_points);

            log::printDoublesPerLineWithPrefix("raw_curvatures[i]: ", better_curvature);

            std::vector<double> curvatures = smoothing::moving_average_filter(better_curvature, detailed_config.curvature_moving_average_window_size, false);
            std::vector<double> ideal_speeds =
                trajectory_utils::constrained_speeds_for_curvatures(curvatures, detailed_config.lateral_accel_limit);

            log::printDoublesPerLineWithPrefix("curvatures[i]: ", curvatures);
            log::printDoublesPerLineWithPrefix("ideal_speeds: ", ideal_speeds);
            log::printDoublesPerLineWithPrefix("final_yaw_values[i]: ", final_yaw_values);

            std::vector<double> constrained_speed_limits = apply_speed_limits(ideal_speeds, distributed_speed_limits);

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Processed all points in computed fit");

            if (all_sampling_points.empty())
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "No trajectory points could be generated");
                return {};
            }

            // Add current vehicle point to front of the trajectory

            nearest_pt_index = get_nearest_index_by_downtrack(all_sampling_points, wm, state);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Current state's nearest_pt_index: " << nearest_pt_index);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Curvature right now: " << better_curvature[nearest_pt_index] << ", at state x: " << state.x_pos_global << ", state y: " << state.y_pos_global);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Corresponding to point: x: " << all_sampling_points[nearest_pt_index].x() << ", y:" << all_sampling_points[nearest_pt_index].y());

            int buffer_pt_index = get_nearest_index_by_downtrack(all_sampling_points, wm, ending_state_before_buffer);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Ending state's index before applying buffer (buffer_pt_index): " << buffer_pt_index);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Corresponding to point: x: " << all_sampling_points[buffer_pt_index].x() << ", y:" << all_sampling_points[buffer_pt_index].y());

            if(nearest_pt_index + 1 >= buffer_pt_index){
                
                lanelet::BasicPoint2d current_pos(state.x_pos_global, state.y_pos_global);
                lanelet::BasicPoint2d ending_pos(ending_state_before_buffer.x_pos_global, ending_state_before_buffer.y_pos_global);

                if(wm->routeTrackPos(ending_pos).downtrack < wm->routeTrackPos(current_pos).downtrack ){

                    RCLCPP_WARN_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Current state is at or past the planned end distance. Couldn't generate trajectory");
                    return {};
                }
                else{
                    //Current point is behind the ending state of maneuver and a valid trajectory is possible
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Returning the two remaining points in the maneuver");

                    std::vector<lanelet::BasicPoint2d> remaining_traj_points = {current_pos, ending_pos};

                    std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(remaining_traj_points);
                    std::vector<double> speeds = {state.longitudinal_vel, state.longitudinal_vel};//Keep current speed
                    std::vector<double> times;
                    trajectory_utils::conversions::speed_to_time(downtracks, speeds, &times);
                    std::vector<double> yaw = {state.orientation, state.orientation}; //Keep current orientation

                    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj_points =
                    trajectory_from_points_times_orientations(remaining_traj_points, times, yaw, state_time);

                    return traj_points;

                }
            }

            //drop buffer points here

             std::vector<lanelet::BasicPoint2d> future_basic_points(all_sampling_points.begin() + nearest_pt_index + 1,
                                            all_sampling_points.begin()+ buffer_pt_index);  // Points in front of current vehicle position

            std::vector<double> future_speeds(constrained_speed_limits.begin() + nearest_pt_index + 1,
                                                        constrained_speed_limits.begin() + buffer_pt_index);  // Points in front of current vehicle position
            std::vector<double> future_yaw(final_yaw_values.begin() + nearest_pt_index + 1,
                                                        final_yaw_values.begin() + buffer_pt_index);  // Points in front of current vehicle position
            std::vector<double>  final_actual_speeds = future_speeds;
            all_sampling_points = future_basic_points;
            final_yaw_values = future_yaw;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Trimmed future points to size: "<< future_basic_points.size());

            lanelet::BasicPoint2d cur_veh_point(state.x_pos_global, state.y_pos_global);

            all_sampling_points.insert(all_sampling_points.begin(),
                                       cur_veh_point); // Add current vehicle position to front of sample points

            final_actual_speeds.insert(final_actual_speeds.begin(), state.longitudinal_vel);

            final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

            // Compute points to local downtracks
            std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

            // Apply accel limits
            final_actual_speeds = optimize_speed(downtracks, final_actual_speeds, detailed_config.max_accel);

            log::printDoublesPerLineWithPrefix("postAccel[i]: ", final_actual_speeds);

            final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, detailed_config.speed_moving_average_window_size);

            log::printDoublesPerLineWithPrefix("post_average[i]: ", final_actual_speeds);

            for (auto &s : final_actual_speeds) // Limit minimum speed. TODO how to handle stopping?
            {
                s = std::max(s, detailed_config.minimum_speed);
            }

            log::printDoublesPerLineWithPrefix("post_min_speed[i]: ", final_actual_speeds);

            // Convert speeds to times
            std::vector<double> times;
            trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

            log::printDoublesPerLineWithPrefix("times[i]: ", times);

            // Build trajectory points
            std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj_points =
                trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values, state_time);

            //debug msg
            carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds msg;
            msg.velocity_profile = final_actual_speeds;
            msg.relative_downtrack = downtracks;
            msg.tangent_headings = final_yaw_values;
            std::vector<double> aligned_speed_limits(constrained_speed_limits.begin() + nearest_pt_index,
                                                    constrained_speed_limits.end());

            msg.speed_limits = aligned_speed_limits;
            std::vector<double> aligned_curvatures(curvatures.begin() + nearest_pt_index,
                                                    curvatures.end());
            msg.curvatures = aligned_curvatures;
            msg.lat_accel_limit = detailed_config.lateral_accel_limit;
            msg.lon_accel_limit = detailed_config.max_accel;
            msg.starting_state = state;
            debug_msg = msg;                                        
            

            return traj_points;
        }

        DetailedTrajConfig compose_detailed_trajectory_config(double trajectory_time_length,
                                                              double curve_resample_step_size,
                                                              double minimum_speed,
                                                              double max_accel,
                                                              double lateral_accel_limit,
                                                              int speed_moving_average_window_size,
                                                              int curvature_moving_average_window_size,
                                                              double back_distance,
                                                              double buffer_ending_downtrack)
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
            detailed_config.buffer_ending_downtrack = buffer_ending_downtrack;

            return detailed_config;
        }

        GeneralTrajConfig compose_general_trajectory_config(const std::string& trajectory_type,
                                                            int default_downsample_ratio,
                                                            int turn_downsample_ratio)
        {
            GeneralTrajConfig general_config;

            general_config.trajectory_type = trajectory_type;
            general_config.default_downsample_ratio = default_downsample_ratio;
            general_config.turn_downsample_ratio = turn_downsample_ratio;


            return general_config;
        }


        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> compose_lanechange_trajectory_from_path(
            const std::vector<PointSpeedPair> &points, const carma_planning_msgs::msg::VehicleState &state, const rclcpp::Time &state_time,
            const carma_wm::WorldModelConstPtr &wm, const carma_planning_msgs::msg::VehicleState &ending_state_before_buffer, const DetailedTrajConfig &detailed_config)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Input points size in compose traj from centerline: "<< points.size());
            int nearest_pt_index = get_nearest_index_by_downtrack(points, wm, state);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "nearest_pt_index: "<< nearest_pt_index);

            std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end());
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "future_points size: "<< future_points.size());

            //Compute yaw values from original trajectory.
            std::vector<lanelet::BasicPoint2d> future_geom_points;
            std::vector<double> final_actual_speeds;
            split_point_speed_pairs(future_points, &future_geom_points, &final_actual_speeds);

            std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(future_geom_points);
            if(!fit_curve){
                throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
            }
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Got fit");
            std::vector<lanelet::BasicPoint2d> all_sampling_points;
            all_sampling_points.reserve(1 + future_geom_points.size() * 2);

            lanelet::BasicPoint2d current_vehicle_point(state.x_pos_global, state.y_pos_global);

            future_geom_points.insert(future_geom_points.begin(),
                                       current_vehicle_point); // Add current vehicle position to front of future geometry points

            final_actual_speeds.insert(final_actual_speeds.begin(), state.longitudinal_vel);
        
            //Compute points to local downtracks
            std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(future_geom_points);

            auto total_step_along_curve = static_cast<int>(downtracks.back() /detailed_config.curve_resample_step_size);

            double scaled_steps_along_curve = 0.0; //from 0 (start) to 1 (end) for the whole trajectory

            for(int steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++){
                lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);

                all_sampling_points.push_back(p);

                scaled_steps_along_curve += 1.0 / total_step_along_curve; 
            }
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Got sampled points with size:" << all_sampling_points.size());

            std::vector<double> final_yaw_values = carma_wm::geometry::compute_tangent_orientations(future_geom_points);
            if(final_yaw_values.size() > 0) {
                final_yaw_values[0] = state.orientation; // Set the initial yaw value based on the initial state
            }

            final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, detailed_config.speed_moving_average_window_size);

            //Convert speeds to time
            std::vector<double> times;
            trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

            //Remove extra points
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Before removing extra buffer points, future_geom_points.size()"<< future_geom_points.size());
            int end_dist_pt_index = get_nearest_index_by_downtrack(future_geom_points, wm, ending_state_before_buffer);
            future_geom_points.resize(end_dist_pt_index + 1);
            times.resize(end_dist_pt_index + 1);
            final_yaw_values.resize(end_dist_pt_index + 1);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "After removing extra buffer points, future_geom_points.size():"<< future_geom_points.size());

            std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj_points =
                trajectory_from_points_times_orientations(future_geom_points, times, final_yaw_values, state_time);

            return traj_points;
        }


    }
}