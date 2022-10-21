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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <string>
#include <algorithm>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "route_following_plugin.hpp"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <chrono>

namespace route_following_plugin
{
 namespace std_ph = std::placeholders;

namespace {
/**
 * \brief Anonymous function to extract maneuver end speed which can not be optained with GET_MANEUVER_PROPERY calls due to it missing in stop and wait plugin
 */ 
double getManeuverEndSpeed(const carma_planning_msgs::msg::Maneuver& mvr) { 
    switch(mvr.type) {
        case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
            return mvr.lane_following_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
            return mvr.lane_change_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            return mvr.intersection_transit_straight_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            return mvr.intersection_transit_left_turn_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            return mvr.intersection_transit_right_turn_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
            return 0;
        default:
            return 0;
    }
}

/**
 * \brief Anonymous function to set the lanelet ids for all maneuver types except lane following
 */ 
void setManeuverLaneletIds(carma_planning_msgs::msg::Maneuver& mvr, lanelet::Id start_id, lanelet::Id end_id) {

    switch(mvr.type) {
        case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
            mvr.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
            mvr.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            mvr.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(start_id);
            mvr.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(end_id);
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            mvr.intersection_transit_left_turn_maneuver.starting_lane_id = std::to_string(start_id);
            mvr.intersection_transit_left_turn_maneuver.ending_lane_id = std::to_string(end_id);
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            mvr.intersection_transit_right_turn_maneuver.starting_lane_id = std::to_string(start_id);
            mvr.intersection_transit_right_turn_maneuver.ending_lane_id = std::to_string(end_id);
            break;
        case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
            mvr.stop_and_wait_maneuver.starting_lane_id = std::to_string(start_id);
            mvr.stop_and_wait_maneuver.ending_lane_id = std::to_string(end_id);
            break;
        default:
            throw std::invalid_argument("Maneuver type does not have start,end lane ids");
    }
}

}

  RouteFollowingPlugin::RouteFollowingPlugin(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::StrategicPlugin(options), tf2_buffer_(get_clock()), config_(Config())
  {
    // Declare parameters
    config_.min_plan_duration_ = declare_parameter<double>("minimal_plan_duration", config_.min_plan_duration_);
    config_.lane_change_plugin_= declare_parameter<std::string>("lane_change_plugin", config_.lane_change_plugin_);
    config_.stop_and_wait_plugin_ = declare_parameter<std::string>("stop_and_wait_plugin", config_.stop_and_wait_plugin_);
    config_.lanefollow_planning_tactical_plugin_ = declare_parameter<std::string>("lane_following_plugin", config_.lanefollow_planning_tactical_plugin_);
    config_.route_end_point_buffer_ = declare_parameter<double>("guidance/route/destination_downtrack_range", config_.route_end_point_buffer_);
    config_.accel_limit_ = declare_parameter<double>("vehicle_acceleration_limit", config_.accel_limit_);
    config_.lateral_accel_limit_ = declare_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit_);
    config_.stopping_accel_limit_multiplier_ = declare_parameter<double>("stopping_accel_limit_multiplier", config_.stopping_accel_limit_multiplier_);
    config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
    config_.min_maneuver_length_ = declare_parameter<double>("min_maneuver_length", config_.min_maneuver_length_);
  }

  carma_ros2_utils::CallbackReturn RouteFollowingPlugin::on_configure_plugin()
  {
    config_ = Config();

    get_parameter<double>("minimal_plan_duration", config_.min_plan_duration_);
    get_parameter<std::string>("lane_change_plugin", config_.lane_change_plugin_);
    get_parameter<std::string>("stop_and_wait_plugin", config_.stop_and_wait_plugin_);
    get_parameter<std::string>("lane_following_plugin", config_.lanefollow_planning_tactical_plugin_);
    get_parameter<double>("guidance/route/destination_downtrack_range", config_.route_end_point_buffer_);
    get_parameter<double>("vehicle_acceleration_limit", config_.accel_limit_);
    get_parameter<double>("vehicle_lateral_accel_limit", config_.lateral_accel_limit_);
    get_parameter<double>("stopping_accel_limit_multiplier", config_.stopping_accel_limit_multiplier_);
    get_parameter<double>("min_maneuver_length", config_.min_maneuver_length_);
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("route_following_plugin"), "RouteFollowingPlugin Config: " << config_);

    // Setup publishers
    upcoming_lane_change_status_pub_ = create_publisher<carma_planning_msgs::msg::UpcomingLaneChangeStatus>("upcoming_lane_change_status", 1);

    // Setup subscribers
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 50,
                                                              std::bind(&RouteFollowingPlugin::twist_cb,this,std_ph::_1));
    current_maneuver_plan_sub_ = create_subscription<carma_planning_msgs::msg::ManeuverPlan>("maneuver_plan", 50,
                                                              std::bind(&RouteFollowingPlugin::current_maneuver_plan_cb,this,std_ph::_1));
    
    // set world model point form wm listener
    wml_ = get_world_model_listener();

    wm_ = get_world_model();

    //set a route callback to update route and calculate maneuver
    wml_->setRouteCallback([this]() {
        RCLCPP_INFO_STREAM(get_logger(),"Recomputing maneuvers due to a route update");
        this->latest_maneuver_plan_ = routeCb(wm_->getRoute()->shortestPath());
    });

    wml_->setMapCallback([this]() {
        if (wm_->getRoute()) { // If this map update occured after a route was provided we need to regenerate maneuvers
            RCLCPP_INFO_STREAM(get_logger(),"Recomputing maneuvers due to map update");
            this->latest_maneuver_plan_ = routeCb(wm_->getRoute()->shortestPath());
        }
    });

    initializeBumperTransformLookup();
    
    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

    void RouteFollowingPlugin::twist_cb(geometry_msgs::msg::TwistStamped::UniquePtr msg)
    {
        current_speed_ = msg->twist.linear.x;
    }
    
    void RouteFollowingPlugin::current_maneuver_plan_cb(carma_planning_msgs::msg::ManeuverPlan::UniquePtr msg) {
        current_maneuver_plan_ = std::move(msg);
    }

    std::vector<carma_planning_msgs::msg::Maneuver> RouteFollowingPlugin::routeCb(const lanelet::routing::LaneletPath &route_shortest_path)
    {
        
        
        for (const auto& ll:route_shortest_path)
        {
            shortest_path_set_.insert(ll.id());
        }
        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        //This function calculates the maneuver plan every time the route is set
        RCLCPP_DEBUG_STREAM(get_logger(),"New route created");

        //Go through entire route - identify lane changes and fill in the spaces with lane following
        auto nearest_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc_, 10); //Return 10 nearest lanelets
        if (nearest_lanelets.empty())
        {
            RCLCPP_WARN_STREAM(get_logger(),"Cannot find any lanelet in map!");

            return maneuvers;
        }

        maneuvers.reserve(route_shortest_path.size());

        double route_length = wm_->getRouteEndTrackPos().downtrack;
        double start_dist = 0.0;
        double end_dist = 0.0;
        double start_speed = 0.0;

        size_t shortest_path_index;
        //Find lane changes in path - up to the second to last lanelet in path (till lane change is possible)
        for (shortest_path_index = 0; shortest_path_index < route_shortest_path.size() - 1; ++shortest_path_index)
        {
            RCLCPP_DEBUG_STREAM(get_logger(),"current shortest_path_index:" << shortest_path_index);

            auto following_lanelets = wm_->getRoute()->followingRelations(route_shortest_path[shortest_path_index]);
            RCLCPP_DEBUG_STREAM(get_logger(),"following_lanelets.size():" << following_lanelets.size());

            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path[shortest_path_index]);

            //update start distance and start speed from previous maneuver if it exists
            start_dist = (maneuvers.empty()) ? wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().front()).downtrack : GET_MANEUVER_PROPERTY(maneuvers.back(), end_dist); // TODO_REFAC if there is no initial maneuver start distance and start speed should be derived from current state. Current state ought to be provided in planning request
            start_speed = (maneuvers.empty()) ? 0.0 : getManeuverEndSpeed(maneuvers.back());
            RCLCPP_DEBUG_STREAM(get_logger(),"start_dist:" << start_dist << ", start_speed:" << start_speed);

            end_dist = wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().back()).downtrack;
            RCLCPP_DEBUG_STREAM(get_logger(),"end_dist:" << end_dist);
            end_dist = std::min(end_dist, route_length);
            RCLCPP_DEBUG_STREAM(get_logger(),"min end_dist:" << end_dist);

            if (std::fabs(start_dist - end_dist) < 0.1) //TODO: edge case that was not recreatable. Sometimes start and end dist was same which crashes inlanecruising
            {
                RCLCPP_WARN_STREAM(get_logger(),"start and end dist are equal! shortest path id" << shortest_path_index << ", lanelet id:" << route_shortest_path[shortest_path_index].id() <<
                    ", start and end dist:" << start_dist);
                continue;
            }



            if (isLaneChangeNeeded(following_lanelets, route_shortest_path[shortest_path_index + 1].id()))
            {
                RCLCPP_DEBUG_STREAM(get_logger(),"LaneChangeNeeded");
    
                // Determine the Lane Change Status
                RCLCPP_DEBUG_STREAM(get_logger(),"Recording lanechange start_dist <<" << start_dist  << ", from llt id:" << route_shortest_path[shortest_path_index].id() << " to llt id: " << 
                    route_shortest_path[shortest_path_index+ 1].id());
                upcoming_lane_change_status_msg_map_.push({start_dist, ComposeLaneChangeStatus(route_shortest_path[shortest_path_index],route_shortest_path[shortest_path_index + 1])});

                maneuvers.push_back(composeLaneChangeManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path[shortest_path_index].id(), route_shortest_path[shortest_path_index + 1].id()));
                ++shortest_path_index; //Since lane change covers 2 lanelets - skip planning for the next lanelet

            }
            else
            {
                RCLCPP_DEBUG_STREAM(get_logger(),"Lanechange NOT Needed ");
                maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, { route_shortest_path[shortest_path_index].id() } ));
            }
        }

        // Add stop and wait maneuver as last maneuver if there is a lanelet unplanned for in path
        if (shortest_path_index < route_shortest_path.size())
        {

            // Compute target deceleration for stopping
            double stopping_accel_limit = config_.accel_limit_ * config_.stopping_accel_limit_multiplier_;

            // Estimate the entry speed for the stopping maneuver
            double stopping_entry_speed = maneuvers.empty() ? current_speed_ : getManeuverEndSpeed(maneuvers.back());

            // Add stop and wait maneuver based on deceleration target and entry speed
            maneuvers = addStopAndWaitAtRouteEnd( maneuvers, route_length, stopping_entry_speed, stopping_accel_limit, config_.lateral_accel_limit_, config_.min_maneuver_length_ );
        
        }
        ////------------------
        RCLCPP_DEBUG_STREAM(get_logger(),"Maneuver plan along route successfully generated");
        return maneuvers;
    }

    std::vector<carma_planning_msgs::msg::Maneuver> RouteFollowingPlugin::addStopAndWaitAtRouteEnd (
        const std::vector<carma_planning_msgs::msg::Maneuver>& input_maneuvers, 
        double route_end_downtrack, double stopping_entry_speed, double stopping_logitudinal_accel,
        double lateral_accel_limit, double min_maneuver_length
    ) const
    {
        RCLCPP_INFO_STREAM(get_logger(),"Attempting to plan Stop and Wait Maneuver");
        /**
         * Alogirthm in this block is as follows
         * 1. Compute distance to slowdown to stop at acceleration limit
         * 2. Drop maneuvers which are wholly encompassed in stopping distance
         * 3. If end downtrack - stop distance is > last maneuver end distance then fill delta with lane follow followed by stop and wait
         * 4. If end downtrack - stop distance is < last maneuver end distance then reduce the existing maneuver to allow for stopping maneuver to be planned
         * 5. Add maneuver to list
         */ 
        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers = input_maneuvers; // Output maneuvers which will be modified


        // Compute stopping distance where v_f = 0
        // (v_f^2 - v_i^2) / (2*a) = d 
        double stopping_distance = 0.5 * (stopping_entry_speed * stopping_entry_speed) / stopping_logitudinal_accel;

        // Compute required starting downtrack for maneuver
        double required_start_downtrack = std::max(0.0, route_end_downtrack - stopping_distance);

        // Loop to drop any maneuvers which fully overlap our stopping maneuver while accounting for minimum length maneuver buffers
        while ( !maneuvers.empty() && maneuverWithBufferStartsAfterDowntrack(maneuvers.back(), required_start_downtrack, lateral_accel_limit, config_.min_maneuver_length_) ) { 
            
            RCLCPP_WARN_STREAM(get_logger(),"Dropping maneuver with id: " <<  GET_MANEUVER_PROPERTY(maneuvers.back(), parameters.maneuver_id) );

            if (maneuvers.back().type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE) {

                // TODO develop more robust approach for this case per: https://github.com/usdot-fhwa-stol/carma-platform/issues/1350
                throw std::invalid_argument("Stopping at the end of the route requires replanning a lane change. RouteFollowing cannot yet handle this case");
            }

            maneuvers.pop_back(); // Drop maneuver

        }

        double last_maneuver_end_downtrack =  required_start_downtrack; // Set default starting location for stop and wait maneuver 

        if ( !maneuvers.empty() ) { // If there are existing maneuvers we need to make sure stop and wait does not overwrite them
            
            last_maneuver_end_downtrack = GET_MANEUVER_PROPERTY(maneuvers.back(), end_dist);
            
            // If our stopping maneuver does not intersect with existing maneuvers
            if ( required_start_downtrack >=  last_maneuver_end_downtrack ) {  
                
                // If the delta is under minimum_maneuver_length we can just extend the stopping maneuver
                // Otherwise add a new lane follow maneuver
                if (required_start_downtrack - last_maneuver_end_downtrack > min_maneuver_length) {
                 
                    // Identify the lanelets which will be crossed by this lane follow maneuver
                    std::vector<lanelet::ConstLanelet> crossed_lanelets = wm_->getLaneletsBetween(last_maneuver_end_downtrack, required_start_downtrack, true, false); 

                    if (crossed_lanelets.empty()) {
                        throw std::invalid_argument("The new lane follow maneuver does not cross any lanelets going from: " + std::to_string(last_maneuver_end_downtrack) + " to: " + std::to_string(required_start_downtrack));
                    }

                    // Create the lane follow maneuver
                    maneuvers.push_back(composeLaneFollowingManeuverMessage(last_maneuver_end_downtrack, required_start_downtrack, stopping_entry_speed, stopping_entry_speed, lanelet::utils::transform(crossed_lanelets, [](auto ll) { return ll.id(); })));
                    
                    // Update last maneuver end downtrack so the stop and wait maneuver can be properly formulated
                    last_maneuver_end_downtrack = required_start_downtrack; 
                } else {
                    RCLCPP_DEBUG_STREAM(get_logger(),"Stop and wait maneuver being extended to nearest maneuver which is closer than the minimum maneuver length");
                }

            } else { // If our stopping maneuver intersects with existing maneuvers

                                    
                SET_MANEUVER_PROPERTY(maneuvers.back(), end_dist, required_start_downtrack);

                // Identify the lanelets which will be crossed by this updated maneuver
                std::vector<lanelet::ConstLanelet> crossed_lanelets = wm_->getLaneletsBetween(GET_MANEUVER_PROPERTY(maneuvers.back(), start_dist), required_start_downtrack, true, false);

                if (crossed_lanelets.empty()) {
                    throw std::invalid_argument("Updated maneuver does not cross any lanelets going from: " + std::to_string(GET_MANEUVER_PROPERTY(maneuvers.back(), start_dist)) + " to: " + std::to_string(required_start_downtrack));
                }

                // Set the impact lane ids for maneuvers
                if (maneuvers.back().type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING) {

                    maneuvers.back().lane_following_maneuver.lane_ids = lanelet::utils::transform(crossed_lanelets, [](auto ll) { return std::to_string(ll.id()); });

                } else {

                    setManeuverLaneletIds(maneuvers.back(), crossed_lanelets.front().id(), crossed_lanelets.back().id());

                }

                last_maneuver_end_downtrack = required_start_downtrack;

            }   
        }
        
        // Identify the lanelets which will be crossed by this stop and wait maneuver
        std::vector<lanelet::ConstLanelet> crossed_lanelets = wm_->getLaneletsBetween(last_maneuver_end_downtrack, route_end_downtrack, true, false);

        if (crossed_lanelets.empty()) {

            throw std::invalid_argument("Stopping maneuver does not cross any lanelets going from: " + std::to_string(last_maneuver_end_downtrack) + " to: " + std::to_string(route_end_downtrack));

        }

        lanelet::Id start_lane = crossed_lanelets.front().id();
        lanelet::Id end_lane = crossed_lanelets.back().id();

        // Build stop and wait maneuver
        maneuvers.push_back(composeStopAndWaitManeuverMessage(last_maneuver_end_downtrack, route_end_downtrack, stopping_entry_speed, start_lane, end_lane,stopping_logitudinal_accel));

        return maneuvers;

    }

    bool RouteFollowingPlugin::maneuverWithBufferStartsAfterDowntrack(const carma_planning_msgs::msg::Maneuver& maneuver, double downtrack, double lateral_accel, double min_maneuver_length) const {
       
        if (maneuver.type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE) {
                
            // Compute the time it takes to move laterally to the next lane
            double lane_change_time = sqrt(0.5 * MAX_LANE_WIDTH / lateral_accel);
            
            // Compute logitudinal distance covered in lane change time
            double min_lane_change_distance = std::max(
                min_maneuver_length, 
                lane_change_time * (GET_MANEUVER_PROPERTY(maneuver, start_speed) + getManeuverEndSpeed(maneuver)) / 2.0 // dist = v_avg * t
            );

            return GET_MANEUVER_PROPERTY(maneuver, start_dist) + min_lane_change_distance > downtrack;

        } else { 
            
            return GET_MANEUVER_PROPERTY(maneuver, start_dist) + min_maneuver_length > downtrack; 
        
        }
    }

        bool RouteFollowingPlugin::get_availability()
        {
         return true;
        }
    
        std::string RouteFollowingPlugin::get_version_id()
        {
                return "v1.0";
        }

    void  RouteFollowingPlugin::plan_maneuvers_callback(
      std::shared_ptr<rmw_request_id_t> srv_header, 
      carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
      {

        if (latest_maneuver_plan_.empty())
        {
           RCLCPP_ERROR_STREAM(get_logger(),"A maneuver plan has not been generated");
            return;
        }

        double current_downtrack;
        
        if (!req->prior_plan.maneuvers.empty())
        {
            current_downtrack = GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_dist);
            RCLCPP_DEBUG_STREAM(get_logger(),"Detected a prior plan! Using back maneuver's end_dist:"<< current_downtrack);            
        }
        else
        {
            current_downtrack = req->veh_downtrack;
            RCLCPP_DEBUG_STREAM(get_logger(),"Detected NO prior plan! Using req.veh_downtrack: "<< current_downtrack);
        }
        
        //Return the set of maneuvers which intersect with min_plan_duration
        size_t i = 0;
        double planned_time = 0.0;

        std::vector<carma_planning_msgs::msg::Maneuver> new_maneuvers;

        while (planned_time < config_.min_plan_duration_ && i < latest_maneuver_plan_.size())
        {
            RCLCPP_DEBUG_STREAM(get_logger(),"Checking maneuver id " << i);
            //Ignore plans for distance already covered
            if (GET_MANEUVER_PROPERTY(latest_maneuver_plan_[i], end_dist) <= current_downtrack)
            {
                RCLCPP_DEBUG_STREAM(get_logger(),"Skipping maneuver id " << i);

                ++i;
                continue;
            }
            if(planned_time == 0.0){
                //update start distance of first maneuver
                setManeuverStartDist(latest_maneuver_plan_[i], current_downtrack);
            }
            planned_time += getManeuverDuration(latest_maneuver_plan_[i], epsilon_).seconds();

            new_maneuvers.push_back(latest_maneuver_plan_[i]);
            ++i;
        }

        if (new_maneuvers.empty())
        {
            RCLCPP_WARN_STREAM(get_logger(),"Cannot plan maneuver because no route is found");
            return;
        }

        //Update time progress for maneuvers
        if (!req->prior_plan.maneuvers.empty())
        {
            updateTimeProgress(new_maneuvers, GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_time));
           RCLCPP_DEBUG_STREAM(get_logger(),"Detected a prior plan! Using back maneuver's end time:"<< std::to_string(rclcpp::Time(GET_MANEUVER_PROPERTY(req->prior_plan.maneuvers.back(), end_time)).seconds()));    
           RCLCPP_DEBUG_STREAM(get_logger(),"Where plan_completion_time was:"<< std::to_string(rclcpp::Time(req->prior_plan.planning_completion_time).seconds()));            
        }
        else
        {
            updateTimeProgress(new_maneuvers, rclcpp::Time(req->header.stamp));
           RCLCPP_DEBUG_STREAM(get_logger(),"Detected NO prior plan! Using this->now():"<< std::to_string(this->now().seconds()));   
        }

        //update starting speed of first maneuver
        if (!req->prior_plan.maneuvers.empty())
        {
            double start_speed;
            switch (req->prior_plan.maneuvers.back().type)
            {
                case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
                    start_speed = req->prior_plan.maneuvers.back().lane_following_maneuver.end_speed;
                    break;
                case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
                    start_speed = req->prior_plan.maneuvers.back().lane_change_maneuver.end_speed;
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                    start_speed = req->prior_plan.maneuvers.back().intersection_transit_straight_maneuver.end_speed;
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                    start_speed = req->prior_plan.maneuvers.back().intersection_transit_left_turn_maneuver.end_speed;
                    break;
                case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                    start_speed = req->prior_plan.maneuvers.back().intersection_transit_right_turn_maneuver.end_speed;
                    break;
                default:
                    throw std::invalid_argument("Invalid maneuver type, cannot update starting speed for maneuver");
            }
            
            updateStartingSpeed(new_maneuvers.front(), start_speed);
            RCLCPP_DEBUG_STREAM(get_logger(),"Detected a prior plan! Using back maneuver's end speed:"<< start_speed);    
        }
        else 
        {
            updateStartingSpeed(new_maneuvers.front(), req->veh_logitudinal_velocity);
            RCLCPP_DEBUG_STREAM(get_logger(),"Detected NO prior plan! Using req->veh_logitudinal_velocity:"<< req->veh_logitudinal_velocity);    
        }
        //update plan
        resp->new_plan = req->prior_plan;
        RCLCPP_DEBUG_STREAM(get_logger(),"Updating maneuvers before returning... Prior plan size:" << req->prior_plan.maneuvers.size());
        for (const auto& mvr : new_maneuvers)
        {
            resp->new_plan.maneuvers.push_back(mvr);
        }

        RCLCPP_DEBUG_STREAM(get_logger(),"Returning total of maneuver size: " << resp->new_plan.maneuvers.size());
        resp->new_plan.planning_completion_time = this->now();

        return;
    }

    carma_planning_msgs::msg::UpcomingLaneChangeStatus RouteFollowingPlugin::ComposeLaneChangeStatus(lanelet::ConstLanelet starting_lanelet,lanelet::ConstLanelet ending_lanelet)
    {
        carma_planning_msgs::msg::UpcomingLaneChangeStatus upcoming_lanechange_status_msg;
        // default to right lane change
        upcoming_lanechange_status_msg.lane_change = carma_planning_msgs::msg::UpcomingLaneChangeStatus::RIGHT; 
        // change to left if detected
        for (const auto &relation : wm_->getRoute()->leftRelations(starting_lanelet))
        {
           RCLCPP_DEBUG_STREAM(get_logger(),"Checking relation.lanelet.id()" <<relation.lanelet.id());
            if (relation.lanelet.id() == ending_lanelet.id())
            {
               RCLCPP_DEBUG_STREAM(get_logger(),"relation.lanelet.id()" << relation.lanelet.id() << " is LEFT");
                upcoming_lanechange_status_msg.lane_change = carma_planning_msgs::msg::UpcomingLaneChangeStatus::LEFT;
                break;
            }  
        }
        RCLCPP_DEBUG_STREAM(get_logger(),"ComposeLaneChangeStatus Exiting now");
        return upcoming_lanechange_status_msg;
    }


    void RouteFollowingPlugin::bumper_pose_cb()
    {
        RCLCPP_DEBUG_STREAM(get_logger(),"Entering pose_cb");

        RCLCPP_DEBUG_STREAM(get_logger(),"Looking up front bumper pose...");
        
        try
        {
            tf_ = tf2_buffer_.lookupTransform("map", "vehicle_front", rclcpp::Time(0), rclcpp::Duration(1, 0)); //save to local copy of transform 1 sec timeout
            tf2::fromMsg(tf_, frontbumper_transform_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_STREAM(get_logger(),std::string(ex.what()));
        }

        geometry_msgs::msg::Pose front_bumper_pose;
        front_bumper_pose.position.x = frontbumper_transform_.getOrigin().getX();
        front_bumper_pose.position.y = frontbumper_transform_.getOrigin().getY();
        
        if (!wm_->getRoute())
            return;

        lanelet::BasicPoint2d current_loc(front_bumper_pose.position.x, front_bumper_pose.position.y);
        current_loc_ = current_loc;
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        
        RCLCPP_DEBUG_STREAM(get_logger(),"pose_cb : current_progress" << current_progress << ", and upcoming_lane_change_status_msg_map_.size(): " << upcoming_lane_change_status_msg_map_.size());
        while (!upcoming_lane_change_status_msg_map_.empty() && current_progress > upcoming_lane_change_status_msg_map_.front().first)
        {
            RCLCPP_DEBUG_STREAM(get_logger(),"pose_cb : the vehicle has passed the lanechange point at downtrack" << upcoming_lane_change_status_msg_map_.front().first);
            upcoming_lane_change_status_msg_map_.pop();
        }

        if (!upcoming_lane_change_status_msg_map_.empty() && upcoming_lane_change_status_msg_map_.front().second.lane_change != carma_planning_msgs::msg::UpcomingLaneChangeStatus::NONE)
        {
            RCLCPP_DEBUG_STREAM(get_logger(),"upcoming_lane_change_status_msg_map_.lane_change : " << static_cast<int>(upcoming_lane_change_status_msg_map_.front().second.lane_change) << 
           ", downtrack until that lanechange: " << upcoming_lane_change_status_msg_map_.front().first);
            upcoming_lane_change_status_msg_map_.front().second.downtrack_until_lanechange=upcoming_lane_change_status_msg_map_.front().first-current_progress;
           RCLCPP_DEBUG_STREAM(get_logger(),"upcoming_lane_change_status_msg_map_.front().second.downtrack_until_lanechange: " <<static_cast<double>(upcoming_lane_change_status_msg_map_.front().second.downtrack_until_lanechange));
            upcoming_lane_change_status_pub_->publish(upcoming_lane_change_status_msg_map_.front().second); 
        }
        
        // Check if we need to return to route shortest path.
        // Step 1. Check if another plugin aside from RFP has been in control
        if (current_maneuver_plan_  != nullptr &&
            GET_MANEUVER_PROPERTY(current_maneuver_plan_->maneuvers[0], parameters.planning_strategic_plugin) \
            != planning_strategic_plugin_) {
            // If another plugin may have brought us off shortest path, check our current lanelet
            auto llts = wm_->getLaneletsFromPoint(current_loc, 10);                                          
            // Remove any candidate lanelets not on the route
            llts.erase(std::remove_if(llts.begin(), llts.end(),
                [&](auto lanelet) -> bool { return !wm_->getRoute()->contains(lanelet); }));

            // !!! ASSUMPTION !!!:
            // Once non-route lanelets have been removed, it is assumed that our actual current lanelet is the only one that can remain.
            // TODO: Verify that this assumption is true in all cases OR implement more robust logic to track current lanelet when there are overlaps

            if (llts.size() > 1) {
                // Assumed that:
                // 1. Vehicle is in a lanelet on the route.
                // 2. The route does not contain overlapping lanelets.
               RCLCPP_WARN_STREAM(get_logger(),"ANOMALOUS SIZE DETECTED FOR CURRENT LANELET CANDIDATES! SIZE: " << llts.size());
            } else if (llts.size() < 1) {
                //  We've left the route entirely.
                RCLCPP_ERROR_STREAM(get_logger(),"Vehicle has left the route entirely. Unable to compute new shortest path.");
                throw std::domain_error("Vehicle not on route, unable to compute shortest path.");
            }

            const auto& current_lanelet = llts[0];

            // if the current lanelet is not on the shortest path
            if (shortest_path_set_.find(current_lanelet.id()) == shortest_path_set_.end())
            {
                returnToShortestPath(current_lanelet);
    
            }
        }
    }

    rclcpp::Duration RouteFollowingPlugin::getManeuverDuration(carma_planning_msgs::msg::Maneuver &maneuver, double epsilon) const
    {
        double maneuver_start_speed = GET_MANEUVER_PROPERTY(maneuver, start_speed);
        double manever_end_speed = getManeuverEndSpeed(maneuver);
        double cur_plus_target = maneuver_start_speed + manever_end_speed;
        if(cur_plus_target < epsilon){
            throw std::invalid_argument("Maneuver start and ending speed is zero");
        }
        rclcpp::Duration duration{0,0};
        double maneuver_start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
        double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);

        RCLCPP_DEBUG_STREAM(get_logger(),"maneuver_end_dist: " << maneuver_end_dist << ", maneuver_start_dist: " << maneuver_start_dist << ", cur_plus_target: " << cur_plus_target);

        duration = rclcpp::Duration((maneuver_end_dist - maneuver_start_dist) / (0.5 * cur_plus_target) * 1e9);

        return duration;
    }

    void RouteFollowingPlugin::updateTimeProgress(std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers, rclcpp::Time start_time) const
    {
        rclcpp::Time time_progress = start_time;
        rclcpp::Time prev_time = time_progress;

        for (auto &maneuver : maneuvers)
        {
            time_progress += getManeuverDuration(maneuver, epsilon_);
            switch (maneuver.type)
            {
            case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
                maneuver.lane_following_maneuver.start_time = prev_time;
                maneuver.lane_following_maneuver.end_time = time_progress;
                break;
            case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
                maneuver.lane_change_maneuver.start_time = prev_time;
                maneuver.lane_change_maneuver.end_time = time_progress;
                break;
            case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                maneuver.intersection_transit_straight_maneuver.start_time = prev_time;
                maneuver.intersection_transit_straight_maneuver.end_time = time_progress;
                break;
            case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                maneuver.intersection_transit_left_turn_maneuver.start_time = prev_time;
                maneuver.intersection_transit_left_turn_maneuver.end_time = time_progress;
                break;
            case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                maneuver.intersection_transit_right_turn_maneuver.start_time = prev_time;
                maneuver.intersection_transit_right_turn_maneuver.end_time = time_progress;
                break;
            case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
                maneuver.stop_and_wait_maneuver.start_time = prev_time;
                maneuver.stop_and_wait_maneuver.start_time = time_progress;
                maneuver.stop_and_wait_maneuver.end_time = start_time + rclcpp::Duration(86400, 0); // Set maneuver time period as 24hrs since this is the end of the route
                break;
            default:
                throw std::invalid_argument("Invalid maneuver type, cannot update time progress for maneuver");
            }
            prev_time = time_progress;
        }
    }

    void RouteFollowingPlugin::updateStartingSpeed(carma_planning_msgs::msg::Maneuver &maneuver, double start_speed) const
    {
        switch (maneuver.type)
        {
        case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
            maneuver.lane_following_maneuver.start_speed = start_speed;
            break;
        case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
            maneuver.lane_change_maneuver.start_speed = start_speed;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            maneuver.intersection_transit_straight_maneuver.start_speed = start_speed;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            maneuver.intersection_transit_left_turn_maneuver.start_speed = start_speed;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            maneuver.intersection_transit_right_turn_maneuver.start_speed = start_speed;
            break;
        case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
            maneuver.stop_and_wait_maneuver.start_speed = start_speed;
            break;
        default:
            throw std::invalid_argument("Invalid maneuver type, cannot update starting speed for maneuver");
        }
    }

    void RouteFollowingPlugin::setManeuverStartDist(carma_planning_msgs::msg::Maneuver &maneuver, double start_dist) const
    {
        switch (maneuver.type)
        {
        case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
            maneuver.lane_following_maneuver.start_dist = start_dist;
            break;
        case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
            maneuver.lane_change_maneuver.start_dist = start_dist;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            maneuver.intersection_transit_straight_maneuver.start_dist = start_dist;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            maneuver.intersection_transit_left_turn_maneuver.start_dist = start_dist;
            break;
        case carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            maneuver.intersection_transit_right_turn_maneuver.start_dist = start_dist;
            break;
        case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
            maneuver.stop_and_wait_maneuver.start_dist = start_dist;
            break;
        default:
            throw std::invalid_argument("Invalid maneuver type");
        }
    }

    carma_planning_msgs::msg::Maneuver RouteFollowingPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, const std::vector<lanelet::Id>& lane_ids) const
    {
        carma_planning_msgs::msg::Maneuver maneuver_msg;
        maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = lanefollow_planning_tactical_plugin_;
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_following_maneuver.start_dist = start_dist;
        maneuver_msg.lane_following_maneuver.start_speed = start_speed;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        maneuver_msg.lane_following_maneuver.lane_ids = lanelet::utils::transform(lane_ids, [](auto id) { return std::to_string(id); });
        //Start time and end time for maneuver are assigned in updateTimeProgress

        // NOTE: The maneuver id is set here because maneuvers are regenerated once per route, so it is acceptable to regenerate them on route updates.
        //       If maneuvers were not generated only on route updates we would want to preserve the ids across plans
        maneuver_msg.lane_following_maneuver.parameters.maneuver_id = getNewManeuverId();

        std::stringstream ss;
        for (const auto& id : maneuver_msg.lane_following_maneuver.lane_ids)
            ss << " " << id;

        RCLCPP_DEBUG_STREAM(get_logger(),"Creating lane follow id: " << maneuver_msg.lane_following_maneuver.parameters.maneuver_id 
                        << " start dist: " << start_dist << " end dist: " << end_dist << "lane_ids: " << ss.str());
        
        return maneuver_msg;
    }

    carma_planning_msgs::msg::Maneuver RouteFollowingPlugin::composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id) const
    {
        carma_planning_msgs::msg::Maneuver maneuver_msg;
        maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver_msg.lane_change_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = lane_change_plugin_;
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_change_maneuver.start_dist = start_dist;
        maneuver_msg.lane_change_maneuver.start_speed = start_speed;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;
        maneuver_msg.lane_change_maneuver.starting_lane_id = std::to_string(starting_lane_id);
        maneuver_msg.lane_change_maneuver.ending_lane_id = std::to_string(ending_lane_id);
        //Start time and end time for maneuver are assigned in updateTimeProgress

        // NOTE: The maneuver id is set here because maneuvers are regenerated once per route, so it is acceptable to regenerate them on route updates.
        //       If maneuvers were not generated only on route updates we would want to preserve the ids across plans
        maneuver_msg.lane_change_maneuver.parameters.maneuver_id = getNewManeuverId();

        RCLCPP_DEBUG_STREAM(get_logger(),"Creating lane change id: "  << maneuver_msg.lane_change_maneuver.parameters.maneuver_id << "start dist: " << start_dist << " end dist: " << end_dist << " Starting llt: " << starting_lane_id << " Ending llt: " << ending_lane_id);

        return maneuver_msg;
    }

    std::string RouteFollowingPlugin::getNewManeuverId() const {
        static auto gen = boost::uuids::random_generator(); // Initialize uuid generator
        
        return boost::lexical_cast<std::string>(gen()); // generate uuid and convert to string
    }

    carma_planning_msgs::msg::Maneuver RouteFollowingPlugin::composeStopAndWaitManeuverMessage(double start_dist, double end_dist, double start_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double stopping_accel) const
    {
        carma_planning_msgs::msg::Maneuver maneuver_msg;
        maneuver_msg.type = carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN
                                                                        | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = stop_and_wait_plugin_;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.stop_and_wait_maneuver.start_dist = start_dist;
        maneuver_msg.stop_and_wait_maneuver.start_speed = start_speed;
        maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(starting_lane_id);
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(ending_lane_id);
        //Start time and end time for maneuver are assigned in updateTimeProgress

        // Set the meta-data for the StopAndWait Maneuver to define the buffer in the route end point stopping location
        maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(config_.route_end_point_buffer_);
        maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(stopping_accel);
        
        // NOTE: The maneuver id is set here because maneuvers are regenerated once per route, so it is acceptable to regenerate them on route updates.
        //       If maneuvers were not generated only on route updates we would want to preserve the ids across plans
        maneuver_msg.stop_and_wait_maneuver.parameters.maneuver_id = getNewManeuverId();

        RCLCPP_DEBUG_STREAM(get_logger(),"Creating stop and wait maneuver id:" << maneuver_msg.stop_and_wait_maneuver.parameters.maneuver_id << "start dist: " << start_dist << " end dist: " << end_dist << " start_speed: " << start_speed << " Starting llt: " << starting_lane_id << " Ending llt: " << ending_lane_id);

        return maneuver_msg;
    }

    bool RouteFollowingPlugin::isLaneChangeNeeded(lanelet::routing::LaneletRelations relations, lanelet::Id target_id) const
    {
        //This method is constrained to the lanelet being checked against being accessible. A non-accessible target lanelet would result in unspecified behavior
        for (auto &relation : relations)
        {
            if (relation.lanelet.id() == target_id && relation.relationType == lanelet::routing::RelationType::Successor)
            {
                return false;
            }
        }
        return true;
    }

    double RouteFollowingPlugin::findSpeedLimit(const lanelet::ConstLanelet &llt)
    {
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        if (traffic_rules)
        {
            return (*traffic_rules)->speedLimit(llt).speedLimit.value();
        }
        else
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }
    }

    void RouteFollowingPlugin::initializeBumperTransformLookup() 
    {
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
        tf2_buffer_.setUsingDedicatedThread(true);
    }

    void RouteFollowingPlugin::returnToShortestPath(const lanelet::ConstLanelet &current_lanelet)
    {
        auto original_shortestpath = wm_->getRoute()->shortestPath();
        RCLCPP_DEBUG_STREAM(get_logger(),"The vehicle has left the shortest path");
        auto routing_graph = wm_->getMapRoutingGraph();

        // In order to return to the shortest path, the closest future lanelet on the shortest path needs to be found.
        // That is the following lanelet of the adjacent lanelet of the current lanelet.
        auto adjacent_lanelets = routing_graph->besides(current_lanelet);
        if (!adjacent_lanelets.empty())
        {
            for (const auto& adjacent:adjacent_lanelets)
            {
                if (shortest_path_set_.find(adjacent.id())!=shortest_path_set_.end())
                {
                    auto following_lanelets = routing_graph->following(adjacent);
                    const auto& target_following_lanelet = following_lanelets[0];
                    RCLCPP_DEBUG_STREAM(get_logger(),"The target_following_lanelet id is: " << target_following_lanelet.id());
                    lanelet::ConstLanelets interm;
                    interm.push_back(static_cast<lanelet::ConstLanelet>(target_following_lanelet));
                    // a new shortest path, via the target_following_lanelet is calculated and used an alternative shortest path
                    auto new_shortestpath = routing_graph->shortestPathVia(current_lanelet, interm, original_shortestpath.back());
                    RCLCPP_DEBUG_STREAM(get_logger(),"a new shortestpath is generated to return to original shortestpath");
                    // routeCb is called to update latest_maneuver_plan_
                    if (new_shortestpath) this->latest_maneuver_plan_ = routeCb(new_shortestpath.get());
                    break;
                }
                else
                {
                    // a new shortest path, via the current_lanelet is calculated and used an alternative shortest path
                    lanelet::ConstLanelets new_interm;
                    new_interm.push_back(static_cast<lanelet::ConstLanelet>(current_lanelet));
                    auto new_shortestpath = routing_graph->shortestPathVia(current_lanelet, new_interm, original_shortestpath.back());
                    RCLCPP_DEBUG_STREAM(get_logger(),"Cannot return to the original shortestpath from adjacent lanes, so a new shortestpath is generated");
                    // routeCb is called to update latest_maneuver_plan_
                    if (new_shortestpath) this->latest_maneuver_plan_ = routeCb(new_shortestpath.get());
                }
            }
            
        }
        else
        {
           RCLCPP_WARN_STREAM(get_logger(),"Alternative shortest path cannot be generated");
        }

    }
              
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(route_following_plugin::RouteFollowingPlugin)