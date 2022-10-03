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
#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"

namespace light_controlled_intersection_tactical_plugin
{


    LightControlledIntersectionTacticalPlugin::LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm, const Config& config,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger)
        :wm_(wm), config_(config), logger_(logger)
    {
    }

    void LightControlledIntersectionTacticalPlugin::planTrajectoryCB( 
        carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
        carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
    {
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Starting light controlled intersection trajectory planning");
        
        if(req->maneuver_index_to_plan >= req->maneuver_plan.maneuvers.size())
        {
            throw std::invalid_argument(
                "Light Control Intersection Plugin asked to plan invalid maneuver index: " + std::to_string(req->maneuver_index_to_plan) + 
                " for plan of size: " + std::to_string(req->maneuver_plan.maneuvers.size()));
        }

        std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
        // Expecting only one maneuver for an intersection, which corresponds to the maneuver_index_to_plan value provided in the request
        if(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING
            && GET_MANEUVER_PROPERTY(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan], parameters.string_valued_meta_data.front()) == light_controlled_intersection_strategy_)
        {
            maneuver_plan.push_back(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan]);
            resp->related_maneuvers.push_back(req->maneuver_index_to_plan);
        }
        else 
        {
            throw std::invalid_argument("Light Control Intersection Plugin asked to plan unsupported maneuver");
        }

        lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global);
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Planning state x:" << req->vehicle_state.x_pos_global << " , y: " << req->vehicle_state.y_pos_global);

        current_downtrack_ = wm_->routeTrackPos(veh_pos).downtrack;

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Current_downtrack: "<< current_downtrack_);

        auto current_lanelets = wm_->getLaneletsFromPoint({req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global});
        lanelet::ConstLanelet current_lanelet;
        
        if (current_lanelets.empty())
        {
            RCLCPP_ERROR_STREAM(logger_->get_logger(), "Given vehicle position is not on the road! Returning...");
            return;
        }

        // get the lanelet that is on the route in case overlapping ones found
        for (auto llt : current_lanelets)
        {
            auto route = wm_->getRoute()->shortestPath();
            if (std::find(route.begin(), route.end(), llt) != route.end())
            {
                current_lanelet = llt;
                break;
            }
        }

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Current_lanelet: " << current_lanelet.id());

        speed_limit_ = findSpeedLimit(current_lanelet, wm_);

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "speed_limit_: " << speed_limit_);

        DetailedTrajConfig wpg_detail_config;
        GeneralTrajConfig wpg_general_config;

        wpg_general_config = basic_autonomy:: waypoint_generation::compose_general_trajectory_config("intersection_transit",
                                                                                    config_.default_downsample_ratio,
                                                                                    config_.turn_downsample_ratio);

        wpg_detail_config = basic_autonomy:: waypoint_generation::compose_detailed_trajectory_config(config_.trajectory_time_length, 
                                                                                config_.curve_resample_step_size, config_.minimum_speed, 
                                                                                config_.vehicle_accel_limit,
                                                                                config_.lateral_accel_limit, 
                                                                                config_.speed_moving_average_window_size, 
                                                                                config_.curvature_moving_average_window_size, config_.back_distance,
                                                                                config_.buffer_ending_downtrack);


        // Create curve-fitting compatible trajectories (with extra back and front attached points) with raw speed limits from maneuver 
        auto points_and_target_speeds = createGeometryProfile(maneuver_plan, std::max((double)0, current_downtrack_ - config_.back_distance),
                                                                                wm_, ending_state_before_buffer_, req->vehicle_state, wpg_general_config, wpg_detail_config);

        // Change raw speed limit values to target speeds specified by the algorithm
        applyOptimizedTargetSpeedProfile(maneuver_plan.front(), req->vehicle_state.longitudinal_vel, points_and_target_speeds);

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "points_and_target_speeds: " << points_and_target_speeds.size());

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "PlanTrajectory");

        //Trajectory Plan
        carma_planning_msgs::msg::TrajectoryPlan trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = req->header.stamp;
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        // Compose smooth trajectory/speed by resampling
        trajectory.trajectory_points = basic_autonomy::waypoint_generation::compose_lanefollow_trajectory_from_path(points_and_target_speeds, 
                                                                                    req->vehicle_state, req->header.stamp, wm_, ending_state_before_buffer_, debug_msg_, 
                                                                                    wpg_detail_config); // Compute the trajectory

        // Set the planning plugin field name
        for (auto& p : trajectory.trajectory_points) {
            p.planner_plugin_name = plugin_discovery_msg_.name;
        }
        
        bool is_new_case_successful = GET_MANEUVER_PROPERTY(maneuver_plan.front(), parameters.int_valued_meta_data[1]);
        TSCase new_case = static_cast<TSCase>GET_MANEUVER_PROPERTY(maneuver_plan.front(), parameters.int_valued_meta_data[0]);

        if (is_last_case_successful_ != boost::none && last_case_ != boost::none)
        {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "all variables are set!");
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "is_last_case_successful_.get(): " << (int)is_last_case_successful_.get());
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "evaluation distance: " << last_successful_ending_downtrack_ - current_downtrack_);

            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "evaluation time: " << std::to_string(last_successful_scheduled_entry_time_ - rclcpp::Time(req->header.stamp).seconds()));
        }
        else
        {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Not all variables are set...");
        }

        carma_planning_msgs::msg::TrajectoryPlan reduced_last_traj;
        std::vector<double> reduced_final_speeds;

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "traj points size: " << last_trajectory_.trajectory_points.size() << ", last_final_speeds_ size: " <<
                            last_final_speeds_.size() );

        for (size_t i = 0; i < last_trajectory_.trajectory_points.size(); i++)
        {
            if (rclcpp::Time(last_trajectory_.trajectory_points[i].target_time) > rclcpp::Time(req->header.stamp) - rclcpp::Duration(0.1 * 1e9)) // Duration is in nanoseconds
            {
                reduced_last_traj.trajectory_points.emplace_back(last_trajectory_.trajectory_points[i]);
                reduced_final_speeds.emplace_back(last_final_speeds_[i]);
            }
        }

        last_final_speeds_ = reduced_final_speeds;
        last_trajectory_.trajectory_points = reduced_last_traj.trajectory_points;

        if (is_last_case_successful_ != boost::none && last_case_ != boost::none
            && last_case_.get() == new_case
            && is_new_case_successful == true
            && !last_trajectory_.trajectory_points.empty()
            && rclcpp::Time(last_trajectory_.trajectory_points.back().target_time) > rclcpp::Time(req->header.stamp) + rclcpp::Duration(1 * 1e9)) // Duration is in nanoseconds
        {
            resp->trajectory_plan = last_trajectory_;
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Last TRAJ's target time: " << rclcpp::Time(last_trajectory_.trajectory_points.back().target_time).seconds() << ", and stamp:" << rclcpp::Time(req->header.stamp).seconds());
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "USING LAST TRAJ: " << (int)last_case_.get());
        }
        else if (is_last_case_successful_ != boost::none && last_case_ != boost::none
            && is_last_case_successful_.get() == true
            && is_new_case_successful == false
            && last_successful_ending_downtrack_ - current_downtrack_ < config_.algorithm_evaluation_distance
            && last_successful_scheduled_entry_time_ - rclcpp::Time(req->header.stamp).seconds() < config_.algorithm_evaluation_period
            && !last_trajectory_.trajectory_points.empty()
            && rclcpp::Time(last_trajectory_.trajectory_points.back().target_time).seconds() > rclcpp::Time(req->header.stamp).seconds())
        {
            resp->trajectory_plan = last_trajectory_;
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Last Traj's target time: " << rclcpp::Time(last_trajectory_.trajectory_points.back().target_time).seconds() << ", and stamp:" << rclcpp::Time(req->header.stamp).seconds() << ", and scheduled: " << std::to_string(last_successful_scheduled_entry_time_));
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "EDGE CASE: USING LAST TRAJ: " << (int)last_case_.get());
        }  
        else
        {
            last_trajectory_ = trajectory;
            resp->trajectory_plan = trajectory;
            last_case_ = new_case;
            last_final_speeds_ = debug_msg_.velocity_profile;
            is_last_case_successful_ = is_new_case_successful;
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "USING NEW: Target time: " << rclcpp::Time(last_trajectory_.trajectory_points.back().target_time).seconds() << ", and stamp:" << rclcpp::Time(req->header.stamp).seconds());
            if (is_new_case_successful)
            {
                last_successful_ending_downtrack_ = GET_MANEUVER_PROPERTY(maneuver_plan.front(), end_dist);              // if algorithm was successful, this is traffic_light_downtrack
                last_successful_scheduled_entry_time_ = rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver_plan.front(), end_time)).seconds();  // if algorithm was successful, this is also scheduled entry time (ET in TSMO UC2 Algo)
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "last_successful_ending_downtrack_:" << last_successful_ending_downtrack_ << ", last_successful_scheduled_entry_time_: " << std::to_string(last_successful_scheduled_entry_time_));
            }
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "USING NEW CASE!!! : " << (int)last_case_.get());
        
        }
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Debug: new case:" << (int) new_case << ", is_new_case_successful: " << is_new_case_successful);

        resp->trajectory_plan.initial_longitudinal_velocity = last_final_speeds_.front();

        resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return;
    }

    void LightControlledIntersectionTacticalPlugin::applyTrajectorySmoothingAlgorithm(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double remaining_dist, 
                                                double starting_speed, double departure_speed, TrajectoryParams tsp)
    {
        if (points_and_target_speeds.empty())
        {
            throw std::invalid_argument("Point and target speed list is empty! Unable to apply case one speed profile...");
        }
        
        // Checking route geometry start against start_dist and adjust profile
        double planning_downtrack_start = wm->routeTrackPos(points_and_target_speeds[0].point).downtrack; // this can include buffered points earlier than maneuver start_dist

        //Check calculated total dist against maneuver limits
        double total_distance_needed = remaining_dist;
        double dist1 = tsp.x1_ - start_dist;
        double dist2 = tsp.x2_ - start_dist;
        double dist3 = tsp.x3_ - start_dist;

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "total_distance_needed: " << total_distance_needed << "\n" <<
                        "dist1: " << dist1 << "\n" <<
                        "dist2: " << dist2 << "\n" <<
                        "dist3: " << dist3);
        double algo_min_speed = std::min({tsp.v1_,tsp.v2_,tsp.v3_});
        double algo_max_speed = std::max({tsp.v1_,tsp.v2_,tsp.v3_});

        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "found algo_minimum_speed: " << algo_min_speed << "\n" <<
                        "algo_max_speed: " << algo_max_speed);

        double total_dist_planned = 0; //Starting dist for maneuver treated as 0.0

        if (planning_downtrack_start < start_dist)
        {
            //Account for the buffer distance that is technically not part of this maneuver
            
            total_dist_planned = planning_downtrack_start - start_dist;
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "buffered section is present. Adjusted total_dist_planned to: " << total_dist_planned);      
        }
        
        double prev_speed = starting_speed;
        auto prev_point = points_and_target_speeds.front();
        
        for(auto& p : points_and_target_speeds)
        {
            double delta_d = lanelet::geometry::distance2d(prev_point.point, p.point);
            total_dist_planned += delta_d;  

            //Apply the speed from algorithm at dist covered
            //Kinematic: v_f = sqrt(v_o^2 + 2*a*d)
            double speed_i;
            if (total_dist_planned <= epsilon_) 
            {
                //Keep target speed same for buffer distance portion
                speed_i = starting_speed;
            }
            else if(total_dist_planned <= dist1 + epsilon_){
                //First segment
                speed_i = sqrt(pow(starting_speed, 2) + 2 * tsp.a1_ * total_dist_planned);
            }
            else if(total_dist_planned > dist1 && total_dist_planned <= dist2 + epsilon_){
                //Second segment
                speed_i = sqrt(std::max(pow(tsp.v1_, 2) + 2 * tsp.a2_ * (total_dist_planned - dist1), 0.0)); //std::max to ensure negative value is not sqrt
            }
            else if (total_dist_planned > dist2 && total_dist_planned <= dist3 + epsilon_)
            {
                //Third segment
                speed_i = sqrt(std::max(pow(tsp.v2_, 2) + 2 * tsp.a3_ * (total_dist_planned - dist2), 0.0)); //std::max to ensure negative value is not sqrt
            }
            else 
            {
                //buffer points that will be cut
                speed_i = prev_speed;
            }

            if (isnan(speed_i))
            {
                speed_i = std::max(config_.minimum_speed, algo_min_speed);
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Detected nan number from equations. Set to " << speed_i);
            }

            p.speed = std::max({speed_i, config_.minimum_speed, algo_min_speed});
            p.speed = std::min({p.speed, speed_limit_, algo_max_speed}); 
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Applied speed: " << p.speed << ", at dist: " << total_dist_planned);

            prev_point = p;
            prev_speed = p.speed;
        }
    }

    void LightControlledIntersectionTacticalPlugin::applyOptimizedTargetSpeedProfile(const carma_planning_msgs::msg::Maneuver& maneuver, const double starting_speed, std::vector<PointSpeedPair>& points_and_target_speeds)
    {
        if(GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data).size() < 9 || 
            GET_MANEUVER_PROPERTY(maneuver, parameters.int_valued_meta_data).size() < 2 ){
            throw std::invalid_argument("There must be 9 float_valued_meta_data and 2 int_valued_meta_data to apply algorithm's parameters.");
        }

        TrajectoryParams tsp;

        tsp.a1_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
        tsp.v1_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[1]);
        tsp.x1_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[2]);

        tsp.a2_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[3]);
        tsp.v2_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[4]);
        tsp.x2_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[5]);

        tsp.a3_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[6]);
        tsp.v3_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[7]);
        tsp.x3_ = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[8]);

        double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
        double ending_downtrack = GET_MANEUVER_PROPERTY(maneuver, end_dist);
        double departure_speed = GET_MANEUVER_PROPERTY(maneuver, end_speed);
        double scheduled_entry_time = rclcpp::Time(GET_MANEUVER_PROPERTY(maneuver, end_time)).seconds();
        double entry_dist = ending_downtrack - starting_downtrack;

        // change speed profile depending on algorithm case starting from maneuver start_dist
        applyTrajectorySmoothingAlgorithm(wm_, points_and_target_speeds, starting_downtrack, entry_dist, starting_speed, 
                                                departure_speed, tsp);
    }

    double LightControlledIntersectionTacticalPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt, const carma_wm::WorldModelConstPtr &wm) const
    {
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm->getTrafficRules();
        if (traffic_rules)
        {
            return (*traffic_rules)->speedLimit(llt).speedLimit.value();
        }
        else
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }
    }

    std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::createGeometryProfile(const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers, double max_starting_downtrack,const carma_wm::WorldModelConstPtr &wm,
                                                                        carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,const carma_planning_msgs::msg::VehicleState& state,
                                                                        const GeneralTrajConfig &general_config, const DetailedTrajConfig &detailed_config)
    {
        std::vector<PointSpeedPair> points_and_target_speeds;
        
        bool first = true;
        std::unordered_set<lanelet::Id> visited_lanelets;
        std::vector<carma_planning_msgs::msg::Maneuver> processed_maneuvers;
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "VehDowntrack: "<<max_starting_downtrack);

        // Only one maneuver is expected in the received maneuver plan
        if(maneuvers.size() == 1)
        {
            auto maneuver = maneuvers.front();

            double starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
            
            starting_downtrack = std::min(starting_downtrack, max_starting_downtrack);

            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Used downtrack: " << starting_downtrack);

            // check if required parameter from strategic planner is present
            if(GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data).empty())
            {
                throw std::invalid_argument("No time_to_schedule_entry is provided in float_valued_meta_data");
            }

            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Creating Lane Follow Geometry");
            std::vector<PointSpeedPair> lane_follow_points = basic_autonomy::waypoint_generation::create_lanefollow_geometry(maneuver, starting_downtrack, wm, general_config, detailed_config, visited_lanelets);
            points_and_target_speeds.insert(points_and_target_speeds.end(), lane_follow_points.begin(), lane_follow_points.end());   
            processed_maneuvers.push_back(maneuver);         
        }
        else 
        {
            throw std::invalid_argument("Light Control Intersection Plugin can only create a geometry profile for one maneuver");
        }

        //Add buffer ending to lane follow points at the end of maneuver(s) end dist 
        if(!processed_maneuvers.empty() && processed_maneuvers.back().type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING){
            points_and_target_speeds = add_lanefollow_buffer(wm, points_and_target_speeds, processed_maneuvers, ending_state_before_buffer, detailed_config);
        }
        
        return points_and_target_speeds;
    }

    void LightControlledIntersectionTacticalPlugin::setConfig(const Config& config) 
    {
        config_ = config;
    }


} // light_controlled_intersection_tactical_plugin