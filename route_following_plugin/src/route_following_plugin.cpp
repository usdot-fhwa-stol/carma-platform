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
    RouteFollowingPlugin::RouteFollowingPlugin() : mvr_duration_(16.0), current_speed_(0.0) { }
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
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteFollowingPlugin::twist_cd, this);

        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();
        //set a route callback to update route and calculate maneuver
        wml_->setRouteCallback(route_set);
        if(route_set)    
        {
            route_cb();
        }
        
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
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    void RouteFollowingPlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    void RouteFollowingPlugin::route_cb(){
        //This function calculates the maneuver every time a route is set
        //Go through route - identify lane changes and fill in the spaces with lane following
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10); 
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
        }

        auto shortest_path = wm_->getRoute()->shortestPath();

        lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d()))
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path);
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    current_lanelet = shortest_path[last_lanelet_index];
                    break;
                }
            }
        }
        if(last_lanelet_index == -1)
        {
            ROS_ERROR_STREAM("Current position is not on the shortest path! Returning an empty maneuver");
        }

        double route_length = wm_->getRouteEndTrackPos().downtrack;;

        //Find lane changes in path - up to the second to last lanelet in path (till lane change is possible)
        for(int i = 0;i < shortest_path.size() - 2; i++){
            auto following_lanelets = wm_->getRoute()->followingRelations(shortest_path[i]);
            double target_speed_in_lanelet = findSpeedLimit(shortest_path[i]);
            double start_dist = wm_->routeTrackPos(shortest_path[i].centerline2d().front()).downtrack;
            double end_dist = wm_->routeTrackPos(shortest_path[i].centerline2d().back()).downtrack; 

            if(end_dist > route_length){
                end_dist = route_length;
            }

            if(identifyLaneChange(following_lanelets, shortest_path[i].id())){
                maneuvers.push_back(composeLaneChangeManeuverMessage(start_dist, end_dist, 0.0 , target_speed_in_lanelet, shortest_path[i].id(), shortest_path[i+1].id(), ros::Time::now()));
            }
            else{
                maneuvers.push_back(composeManeuverMessage(start_dist, end_dist, 0.0, target_speed_in_lanelet, shortest_path[i].id(),ros::Time::now()));
            }
        }
        //add lane follow as last maneuver
        if(maneuvers.size() < shortest_path.size()){
            double target_speed_in_lanelet = findSpeedLimit(shortest_path.back());
            double start_dist = wm_->routeTrackPos(shortest_path.back().centerline2d().front()).downtrack;
            double end_dist = wm_->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack; 
            maneuvers.push_back(composeManeuverMessage(start_dist, end_dist, 0.0, target_speed_in_lanelet, shortest_path.back().id(),ros::Time::now()));
        }
        

    }

    bool RouteFollowingPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    { 
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        
        //Estimate 15s downtrack distance from end_speed of current maneuver
        double maneuver_target_speed = get_final_speed(maneuvers.front());
        double total_maneuver_length = current_progress + maneuver_target_speed * mvr_duration_;
        double route_length=  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        //update start distance of first maneuver and time for all maneuvers
           set_maneuver_start_dist(maneuvers.front(), current_progress);

        //Return the set of maneuvers which intersect those 15s
        int i=0;
        while(current_progress < total_maneuver_length && i < maneuvers.size()){
           double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuvers[i],end_dist);
           current_progress += maneuver_end_dist;
           //Update time progress for maneuvers
           resp.new_plan.maneuvers.push_back(maneuvers[i]);
           i++;
        }
        
        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }
        return true;
    }

    void RouteFollowingPlugin::set_maneuver_start_dist(cav_msgs::Maneuver& maneuver, double start_dist){
        switch(maneuver.type){
            case cav_msgs::Maneuver::LANE_FOLLOWING :
                maneuver.lane_following_maneuver.start_dist = start_dist;
            case cav_msgs::Maneuver::LANE_CHANGE :
                maneuver.lane_change_maneuver.start_dist = start_dist;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT :
                maneuver.intersection_transit_straight_maneuver.start_dist = start_dist;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN :
                maneuver.intersection_transit_left_turn_maneuver.start_dist = start_dist;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN :
                maneuver.intersection_transit_right_turn_maneuver.start_dist = start_dist;
            case cav_msgs::Maneuver::STOP_AND_WAIT :
                maneuver.stop_and_wait_maneuver.start_dist = start_dist;
            default :
                ROS_DEBUG_STREAM("Invalid maneuver type");
        }

    }
    
    double RouteFollowingPlugin::get_final_speed(cav_msgs::Maneuver maneuver){
        double speed;
        if(maneuver.type == cav_msgs::Maneuver::STOP_AND_WAIT){
            speed =0.0;
        }
        else if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
        }
        else{
            speed = GET_MANEUVER_PROPERTY(maneuver,end_speed);
        }
        return speed;
    }

    int RouteFollowingPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
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
    
    cav_msgs::Maneuver RouteFollowingPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InLaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(mvr_duration_);
        } else {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        current_time = maneuver_msg.lane_following_maneuver.end_time;
        ROS_INFO_STREAM("Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_INFO_STREAM("Duration:"<<maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeLaneChangeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int starting_lane_id,int ending_lane_id, ros::Time current_time){
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        maneuver_msg.lane_change_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = lane_change_plugin_;
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.lane_change_maneuver.start_dist = current_dist;
        maneuver_msg.lane_change_maneuver.start_speed = current_speed;
        maneuver_msg.lane_change_maneuver.start_time = current_time;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) {
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(mvr_duration_);
        } else {
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_change_maneuver.starting_lane_id = std::to_string(starting_lane_id);
        maneuver_msg.lane_change_maneuver.ending_lane_id = std::to_string(ending_lane_id);
        current_time = maneuver_msg.lane_change_maneuver.end_time;
        ROS_INFO_STREAM("Creating lane change start dist:"<<current_dist<<" end dist:"<<end_dist<<" Starting llt:"<<starting_lane_id<<" Ending llt:"<<ending_lane_id);
        ROS_INFO_STREAM("Duration:"<<maneuver_msg.lane_change_maneuver.end_time.toSec() - maneuver_msg.lane_change_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    bool RouteFollowingPlugin::identifyLaneChange(lanelet::routing::LaneletRelations relations, int target_id)
    {
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
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        double target_speed = 0.0, traffic_speed =0.0, param_speed =0.0;
        double hardcoded_max=lanelet::Velocity(hardcoded_params::control_limits::MAX_LONGITUDINAL_VELOCITY_MPS * lanelet::units::MPS()).value();

        if (traffic_rules)
        {
            traffic_speed=(*traffic_rules)->speedLimit(llt).speedLimit.value();
            
        }
        else{
            ROS_WARN(" Valid traffic rules object could not be built.");
        }

        if(config_limit > 0.0 && config_limit < hardcoded_max)
        {
            param_speed = config_limit;
            ROS_DEBUG("Using Configurable value");
        }
        else 
        {
            param_speed = hardcoded_max;
            ROS_DEBUG(" Using Hardcoded maximum");
        }
        //If either value is 0, use the other valid limit
        if(traffic_speed <= epislon_ || param_speed <= epislon_){
            target_speed = std::max(traffic_speed, param_speed);
        }
        else{
            target_speed = std::min(traffic_speed,param_speed);
        }
        
        return target_speed;
    }
}