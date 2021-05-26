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
#include "route_following_plugin.h"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>

namespace route_following_plugin
{
    RouteFollowingPlugin::RouteFollowingPlugin() : current_speed_(0.0) , min_plan_duration_(16.0) { }

    void RouteFollowingPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh2_.reset(new ros::CARMANodeHandle("/"));

        plan_maneuver_srv_ = nh_->advertiseService("plugins/RouteFollowing/plan_maneuvers", &RouteFollowingPlugin::plan_maneuver_cb, this);
                
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "RouteFollowing";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

        pose_sub_ = nh_->subscribe("current_pose", 1, &RouteFollowingPlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteFollowingPlugin::twist_cb, this);

        // read ros parameters
        pnh_->param<double>("minimal_plan_duration", min_plan_duration_, 16.0);
        pnh_->param<std::string>("lane_change_plugin", lane_change_plugin_);

        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();

        //set a route callback to update route and calculate maneuver
        wml_->setRouteCallback([this]() { 
            this->latest_maneuver_plan_ = route_cb(wm_->getRoute()->shortestPath());
        });
        
        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { plugin_discovery_pub_.publish(plugin_discovery_msg_); });
    }

    void RouteFollowingPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void RouteFollowingPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
        lanelet::BasicPoint2d curr_loc(pose_msg_->pose.position.x, pose_msg_->pose.position.y);
        current_loc_ = curr_loc;
    }
    void RouteFollowingPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    std::vector<cav_msgs::Maneuver> RouteFollowingPlugin::route_cb(const lanelet::routing::LaneletPath& route_shortest_path){
        std::vector <cav_msgs::Maneuver> maneuvers;
        maneuvers.reserve(route_shortest_path.size());
        //This function calculates the maneuver plan every time the route is set
        ROS_DEBUG_STREAM("New route created");
        //Go through entire route - identify lane changes and fill in the spaces with lane following
        auto nearest_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc_, 10);   //Return 10 nearest lanelets
        if(nearest_lanelets.empty())
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return maneuvers; 
        }

        double route_length = wm_->getRouteEndTrackPos().downtrack;
        double start_dist, end_dist, start_speed;
        size_t shortest_path_index;
        //Find lane changes in path - up to the second to last lanelet in path (till lane change is possible)
        for(shortest_path_index = 0; shortest_path_index < route_shortest_path.size() - 1; ++shortest_path_index){
            auto following_lanelets = wm_->getRoute()->followingRelations(route_shortest_path[shortest_path_index]);
            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path[shortest_path_index]);

            //update start distance and start speed from previous maneuver if it exists
            start_dist = (maneuvers.empty()) ?  wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().front()).downtrack :
                                                        GET_MANEUVER_PROPERTY(maneuvers.back(), end_dist);
            start_speed = (maneuvers.empty()) ? 0.0 : GET_MANEUVER_PROPERTY(maneuvers.back(), end_speed);

            end_dist = wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().back()).downtrack; 
            end_dist = std::min(end_dist, route_length);

            if(isLaneChangeNeeded(following_lanelets, route_shortest_path[shortest_path_index + 1].id())){
                maneuvers.push_back(composeLaneChangeManeuverMessage(start_dist, end_dist, start_speed , target_speed_in_lanelet, route_shortest_path[shortest_path_index].id(), route_shortest_path[shortest_path_index+1].id(), ros::Time::now()));
                ++shortest_path_index; //Since lane change covers 2 lanelets - skip planning for the next lanelet
            }
            else{
                maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path[shortest_path_index].id(),ros::Time::now()));
            }
            
        }
        //TO DO  - Update this block so that Stop and wait is the last maneuver ---- 
        //add lane follow as last maneuver if there is a lanelet unplanned for in path
        if(shortest_path_index < route_shortest_path.size() -1){
            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path.back());
            start_dist = wm_->routeTrackPos(route_shortest_path.back().centerline2d().front()).downtrack;
            end_dist = wm_->routeTrackPos(route_shortest_path.back().centerline2d().back()).downtrack; 
            maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path.back().id(),ros::Time::now()));
        }
        ////------------------
        ROS_DEBUG_STREAM("Maneuver plan along route successfully generated");
        return maneuvers;
    }

    bool RouteFollowingPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    { 
        if(latest_maneuver_plan_.empty()){
            ROS_ERROR_STREAM("A maneuver plan has not been generated");
            return false;
        }
        
        double current_downtrack = wm_->routeTrackPos(current_loc_).downtrack;
        
        //Return the set of maneuvers which intersect with min_plan_duration
        int i=0;
        double planned_time = 0.0;
        while(planned_time < min_plan_duration_ && i < latest_maneuver_plan_.size()){
            //Ignore plans for distance already covered
            if(GET_MANEUVER_PROPERTY(latest_maneuver_plan_[i], end_dist) < current_downtrack){
                ++i;
                continue;
            }
            double maneuver_start_time = GET_MANEUVER_PROPERTY(latest_maneuver_plan_[i], start_time).toSec();
            double maneuver_end_time = GET_MANEUVER_PROPERTY(latest_maneuver_plan_[i],end_time).toSec();
            planned_time += maneuver_end_time - maneuver_start_time;
            
            resp.new_plan.maneuvers.push_back(latest_maneuver_plan_[i]);
            ++i;
        }
        
        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }
        //update plan

        //update start distance of first maneuver and time for all maneuvers
        setManeuverStartDist(resp.new_plan.maneuvers.front(), current_downtrack);
        //Update time progress for maneuvers
        updateTimeProgress(resp.new_plan.maneuvers, ros::Time::now());
        //update starting speed of first maneuver
        updateStartingSpeed(resp.new_plan.maneuvers.front(), current_speed_);

        return true;
    }

    void RouteFollowingPlugin::updateTimeProgress(std::vector<cav_msgs::Maneuver>& maneuvers, ros::Time start_time) const {
        ros::Time time_progress = start_time;
        ros::Time prev_time = time_progress;

        for(auto& maneuver : maneuvers){
            double maneuver_start_speed = GET_MANEUVER_PROPERTY(maneuver,start_speed);
            double manever_end_speed = GET_MANEUVER_PROPERTY(maneuver,end_speed);
            double cur_plus_target = maneuver_start_speed + manever_end_speed;
            ros::Duration maneuver_duration;
            double maneuver_start_dist = GET_MANEUVER_PROPERTY(maneuver,start_dist);
            double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuver,end_dist);
            maneuver_duration = ros::Duration((maneuver_end_dist - maneuver_start_dist) / (0.5 * cur_plus_target));

            time_progress += maneuver_duration;
            switch(maneuver.type){
                case cav_msgs::Maneuver::LANE_FOLLOWING :
                maneuver.lane_following_maneuver.start_time = prev_time;
                maneuver.lane_following_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE :
                maneuver.lane_change_maneuver.start_time = prev_time;
                maneuver.lane_change_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT :
                maneuver.intersection_transit_straight_maneuver.start_time = prev_time;
                maneuver.intersection_transit_straight_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN :
                maneuver.intersection_transit_left_turn_maneuver.start_time = prev_time;
                maneuver.intersection_transit_left_turn_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN :
                maneuver.intersection_transit_right_turn_maneuver.start_time = prev_time;
                maneuver.intersection_transit_right_turn_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::STOP_AND_WAIT :
                maneuver.stop_and_wait_maneuver.start_time = prev_time;
                maneuver.stop_and_wait_maneuver.start_time = time_progress;
                break;
            default :
                throw std::invalid_argument("Invalid maneuver type, cannot update time progress for maneuver");
            }
            prev_time = time_progress;
        }
    }

    void RouteFollowingPlugin::updateStartingSpeed(cav_msgs::Maneuver& maneuver, double start_speed) const {
                switch(maneuver.type){
                case cav_msgs::Maneuver::LANE_FOLLOWING :
                maneuver.lane_following_maneuver.start_speed = start_speed;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE :
                maneuver.lane_change_maneuver.start_speed = start_speed;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT :
                maneuver.intersection_transit_straight_maneuver.start_speed = start_speed;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN :
                maneuver.intersection_transit_left_turn_maneuver.start_speed = start_speed;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN :
                maneuver.intersection_transit_right_turn_maneuver.start_speed = start_speed;
                break;
            case cav_msgs::Maneuver::STOP_AND_WAIT :
                maneuver.stop_and_wait_maneuver.start_speed = start_speed;
                break;
            default :
                throw std::invalid_argument("Invalid maneuver type, cannot update starting speed for maneuver");
            }
    }

    void RouteFollowingPlugin::setManeuverStartDist(cav_msgs::Maneuver& maneuver, double start_dist) const { 
        switch(maneuver.type){
            case cav_msgs::Maneuver::LANE_FOLLOWING :
                maneuver.lane_following_maneuver.start_dist = start_dist;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE :
                maneuver.lane_change_maneuver.start_dist = start_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT :
                maneuver.intersection_transit_straight_maneuver.start_dist = start_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN :
                maneuver.intersection_transit_left_turn_maneuver.start_dist = start_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN :
                maneuver.intersection_transit_right_turn_maneuver.start_dist = start_dist;
                break;
            case cav_msgs::Maneuver::STOP_AND_WAIT :
                maneuver.stop_and_wait_maneuver.start_dist = start_dist;
                break;
            default :
                throw std::invalid_argument("Invalid maneuver type");
        }

    }
    

    int RouteFollowingPlugin::findLaneletIndexFromPath(int target_id,const lanelet::routing::LaneletPath& path) const
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }
    
    cav_msgs::Maneuver RouteFollowingPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id, ros::Time start_time) const {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = lanefollow_planning_tactical_plugin_;
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_following_maneuver.start_dist = start_dist;
        maneuver_msg.lane_following_maneuver.start_speed = start_speed;
        maneuver_msg.lane_following_maneuver.start_time = start_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = start_speed + target_speed;
        if (cur_plus_target < 0.00001) {
            throw std::invalid_argument("Requested zero speed lane following.");
        } else {
            maneuver_msg.lane_following_maneuver.end_time = start_time + ros::Duration((end_dist - start_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        ROS_INFO_STREAM("Creating lane follow start dist: "<<start_dist<<" end dist: "<<end_dist);
        ROS_INFO_STREAM("Duration: "<<maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id,lanelet::Id ending_lane_id, ros::Time start_time) const{
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        maneuver_msg.lane_change_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = lane_change_plugin_;
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_change_maneuver.start_dist = start_dist;
        maneuver_msg.lane_change_maneuver.start_speed = start_speed;
        maneuver_msg.lane_change_maneuver.start_time = start_time;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = start_speed + target_speed;
        if (cur_plus_target < 0.00001) {
            throw std::invalid_argument("Requested zero speed lane change.");
        } else {
            maneuver_msg.lane_change_maneuver.end_time = start_time + ros::Duration((end_dist - start_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_change_maneuver.starting_lane_id = std::to_string(starting_lane_id);
        maneuver_msg.lane_change_maneuver.ending_lane_id = std::to_string(ending_lane_id);
        
        ROS_INFO_STREAM("Creating lane change start dist: "<<start_dist<<" end dist: "<<end_dist<<" Starting llt: "<<starting_lane_id<<" Ending llt: "<<ending_lane_id);
        ROS_INFO_STREAM("Duration: "<<maneuver_msg.lane_change_maneuver.end_time.toSec() - maneuver_msg.lane_change_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    bool RouteFollowingPlugin::isLaneChangeNeeded(lanelet::routing::LaneletRelations relations, int target_id)
    {
        //This method is constrained to the lanelet being checked against being accessible. A non-accessible target lanelet would result in unspecified behavior
        for(auto& relation : relations)
        {
            if(relation.lanelet.id() == target_id && relation.relationType == lanelet::routing::RelationType::Successor)
            {
                return false;
            }
        }
        return true;
    }

    double RouteFollowingPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        try
        {
           lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
           return (*traffic_rules)->speedLimit(llt).speedLimit.value();
        } 
        catch(const std::exception& e)
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }
        
    }
}