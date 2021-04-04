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

/**
 * TODO remove code duplication with arbitrator where this method is copied from
 * \brief Macro definition to enable easier access to fields shared across the maneuver typees
 * 
 * TODO: Implement a better system for handling Maneuver objects such that this
 *       macro isn't needed.
 * 
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */

#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                    ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        ((mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                        ((mvr).type == cav_msgs::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :\
                            throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))))

namespace {
double getManeuverEndSpeed(const cav_msgs::Maneuver& mvr) {
    switch(mvr.type) {
        case cav_msgs::Maneuver::LANE_FOLLOWING:
            return mvr.lane_following_maneuver.end_speed;
        case cav_msgs::Maneuver::LANE_CHANGE:
            return mvr.lane_change_maneuver.end_speed;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            return mvr.intersection_transit_straight_maneuver.end_speed;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            return mvr.intersection_transit_left_turn_maneuver.end_speed;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            return mvr.intersection_transit_right_turn_maneuver.end_speed;
        case cav_msgs::Maneuver::STOP_AND_WAIT:
            return 0;
        default:
            ROS_ERROR_STREAM("Requested end speed from unsupported maneuver type");
            return 0;
    }
}
}

namespace route_following_plugin
{
    RouteFollowingPlugin::RouteFollowingPlugin() : mvr_duration_(16.0), current_speed_(0.0) { }
    void RouteFollowingPlugin::initialize()
    {
        
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        gnh_.reset(new ros::CARMANodeHandle("/"));
        
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
        
        pnh_->param<double>("minimal_maneuver_duration", mvr_duration_, mvr_duration_);
        pnh_->param<double>("/guidance/route/destination_downtrack_range", route_end_point_buffer_, route_end_point_buffer_);
        pnh_->param<double>("/vehicle_acceleration_limit", accel_limit_, accel_limit_);
        pnh_->param<double>("stopping_accel_limit_multiplier", stopping_accel_limit_multiplier_, stopping_accel_limit_multiplier_);

        wml_.reset(new carma_wm::WMListener());

        // set world model point form wm listener
        wm_ = wml_->getWorldModel();
        ros::CARMANodeHandle::setSpinCallback([this]() -> bool 
        {
           plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });
    }
    void RouteFollowingPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }
    bool RouteFollowingPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {        
        if (!wm_->getRoute()) {
            throw std::invalid_argument("RouteFollowing asked to plan maneuver before route is available. ");
        }



        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        double current_progress = 0;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();

        if (!req.prior_plan.maneuvers.empty()) { // If there is an existing maneuver plan then set the start state based on that end state
            double start_dist = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
            double start_speed = getManeuverEndSpeed(req.prior_plan.maneuvers.back());
            ros::Time start_time = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
            auto optional_point = wm_->pointFromRouteTrackPos(start_dist);
            if (!optional_point) {
                throw std::invalid_argument("Could not get starting point for plan. Is maneuver plan beyond end of route?");
            }
            current_loc = *optional_point;
            current_progress = start_dist;
            speed_progress = start_speed;
            time_progress = start_time;
        } else {
            current_progress = wm_->routeTrackPos(current_loc).downtrack;            
        }
        
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10);       
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
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
            return true;
        }

        
        double target_speed=findSpeedLimit(current_lanelet);   //get Speed Limit

        double total_maneuver_length = current_progress + mvr_duration_ * target_speed;
        double route_length=  wm_->getRouteEndTrackPos().downtrack - (route_end_point_buffer_ * 0.5); 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        bool approaching_route_end = false;
        double time_req_to_stop,stopping_dist;

        double accel_target = stopping_accel_limit_multiplier_ * accel_limit_; 
        double min_time_to_stop = 4.4704 / accel_target; // 10mph is the minimum stopping distance we will compute for. Once below that it will be a fixed value. // TODO make parameter
        double min_stopping_dist =  0.5 * 4.4704 * min_time_to_stop;
        time_req_to_stop = std::max(speed_progress / accel_target, min_time_to_stop); 

        stopping_dist = std::max(0.5 * speed_progress * time_req_to_stop, min_stopping_dist);
        
        if(route_length - current_progress <= stopping_dist){
            approaching_route_end = true;
        }

        ROS_DEBUG_STREAM("Starting Loop");
        ROS_DEBUG_STREAM("Time Required To Stop: " << time_req_to_stop << " stopping_dist: " << stopping_dist<<" target_speed: "<< target_speed);
        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);
        
        while(current_progress < total_maneuver_length && !approaching_route_end)
        {
            ROS_DEBUG_STREAM("Lanlet: " << shortest_path[last_lanelet_index].id());
            ROS_DEBUG_STREAM("current_progress: "<< current_progress);
            ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
            ROS_DEBUG_STREAM("target_speed: " << target_speed);
            ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());

            auto p = shortest_path[last_lanelet_index].centerline2d().back();
            double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
            end_dist = std::min(end_dist, total_maneuver_length);
            ROS_DEBUG_STREAM("end_dist: " << end_dist);
            double dist_diff = end_dist - current_progress;
            ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
            
            if(route_length - end_dist <= stopping_dist){
                end_dist = route_length - stopping_dist; 
                dist_diff = end_dist - current_progress;
                approaching_route_end = true;
            }
            
            if(end_dist < current_progress){
                break;
            }

            resp.new_plan.maneuvers.push_back(
                composeManeuverMessage(current_progress, end_dist,  
                                    speed_progress, target_speed, 
                                    shortest_path[last_lanelet_index].id(), time_progress));
            
            current_progress += dist_diff;
            time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
            speed_progress = target_speed;


            time_req_to_stop = std::max(speed_progress / accel_target, min_time_to_stop); // the time required to stop should be updated after each maneuver is generated so that it can be evaluated before planning the next maneuver

            stopping_dist = std::max(0.5 * speed_progress * time_req_to_stop, min_stopping_dist);
            

            if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
            {
                break;
            }
            auto following_lanelets = wm_->getRoute()->followingRelations(shortest_path[last_lanelet_index]);
            if(following_lanelets.size() == 0)
            {
                ROS_WARN_STREAM("Cannot find the following lanelet.");
                return true;
            }
            if(identifyLaneChange(following_lanelets, shortest_path[last_lanelet_index + 1].id()))
            {
                ++last_lanelet_index;
            }
            else
            {
                ROS_WARN_STREAM("Cannot find the next lanelet in the current lanelet's successor list!");
                return true;
            }
            //get speed limit
            current_lanelet=shortest_path[last_lanelet_index];  //update current lanelet
            target_speed=findSpeedLimit(current_lanelet); 
        }
        ROS_DEBUG_STREAM("Done Loop: approaching_route_end: " << approaching_route_end);
        if(approaching_route_end){
            ros::Time end_time = ros::Time::now();
            if (resp.new_plan.maneuvers.size() > 0) {
                end_time = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
            }
            
            resp.new_plan.maneuvers.push_back(
                composeStopandWaitManeuverMessage(current_progress,total_maneuver_length,
                speed_progress,shortest_path[last_lanelet_index].id(),
                shortest_path[last_lanelet_index].id(),end_time,time_req_to_stop)); 
        }
        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }
        return true;
    }
    void RouteFollowingPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    void RouteFollowingPlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
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
        return maneuver_msg;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeStopandWaitManeuverMessage(double current_dist, double end_dist, double current_speed, int start_lane_id, int end_lane_id, ros::Time current_time, double end_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN
                                                                            | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = "StopandWaitPlugin";
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
        maneuver_msg.stop_and_wait_maneuver.start_speed = current_speed;
        maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
        maneuver_msg.stop_and_wait_maneuver.start_time = current_time;
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(start_lane_id);
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(end_lane_id);
        // Receiving plugins can identify the index of the route end buffer by reaching index 1 of the meta data field
        maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(route_end_point_buffer_ * 0.5);
        if(end_time < mvr_duration_){ // TODO double check these durations
            end_time = mvr_duration_;
        } 
        maneuver_msg.stop_and_wait_maneuver.end_time = current_time + ros::Duration(end_time);
        
        return maneuver_msg;
    }

    bool RouteFollowingPlugin::identifyLaneChange(lanelet::routing::LaneletRelations relations, int target_id)
    {
        for(auto& relation : relations)
        {
            if(relation.lanelet.id() == target_id && relation.relationType == lanelet::routing::RelationType::Successor)
            {
                return true;
            }
        }
        return false;
    }

    double RouteFollowingPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        double traffic_speed =0.0;

        if (traffic_rules)
        {
            traffic_speed=(*traffic_rules)->speedLimit(llt).speedLimit.value();
            
        }
        else {
            throw std::invalid_argument("Could not get traffic rules and therefore can't get speed limit");
        }
        
        return traffic_speed;
    }
}