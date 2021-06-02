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
    RouteFollowingPlugin::RouteFollowingPlugin() : current_speed_(0.0), min_plan_duration_(16.0) {}

    void RouteFollowingPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh2_.reset(new ros::CARMANodeHandle("/"));

        plan_maneuver_srv_ = nh_->advertiseService("plugins/RouteFollowing/plan_maneuvers", &RouteFollowingPlugin::planManeuverCb, this);

        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "RouteFollowing";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

        pose_sub_ = nh_->subscribe("current_pose", 1, &RouteFollowingPlugin::pose_cb, this);
<<<<<<< HEAD
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteFollowingPlugin::twist_cd, this);
        
        pnh_->param<double>("minimal_maneuver_duration", mvr_duration_, 16.0);
        pnh2_->param<double>("config_speed_limit",config_limit);
        pnh_->param<double>("buffer_time_lanechange",buffer_lanechange_time_);
        nh_->param<double>("route_end_jerk", jerk_, 100.0);
        nh_->param<std::string>("lane_change_plugin", lane_change_plugin_);
=======
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteFollowingPlugin::twist_cb, this);

        // read ros parameters
        pnh_->param<double>("minimal_plan_duration", min_plan_duration_, 16.0);
        pnh_->param<std::string>("lane_change_plugin", lane_change_plugin_);

>>>>>>> feature/route_following_update
        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();

        //set a route callback to update route and calculate maneuver
        wml_->setRouteCallback([this]() {
            this->latest_maneuver_plan_ = routeCb(wm_->getRoute()->shortestPath());
        });

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto &) { plugin_discovery_pub_.publish(plugin_discovery_msg_); });
    }

    void RouteFollowingPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void RouteFollowingPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        pose_msg_ = msg;
        lanelet::BasicPoint2d curr_loc(pose_msg_->pose.position.x, pose_msg_->pose.position.y);
        current_loc_ = curr_loc;
    }
    void RouteFollowingPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr &msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    std::vector<cav_msgs::Maneuver> RouteFollowingPlugin::routeCb(const lanelet::routing::LaneletPath &route_shortest_path)
    {
        std::vector<cav_msgs::Maneuver> maneuvers;
        //This function calculates the maneuver plan every time the route is set
        ROS_DEBUG_STREAM("New route created");
        //Go through entire route - identify lane changes and fill in the spaces with lane following
        auto nearest_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc_, 10); //Return 10 nearest lanelets
        if (nearest_lanelets.empty())
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
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
            auto following_lanelets = wm_->getRoute()->followingRelations(route_shortest_path[shortest_path_index]);
            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path[shortest_path_index]);

            //update start distance and start speed from previous maneuver if it exists
            start_dist = (maneuvers.empty()) ? wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().front()).downtrack : GET_MANEUVER_PROPERTY(maneuvers.back(), end_dist);
            start_speed = (maneuvers.empty()) ? 0.0 : GET_MANEUVER_PROPERTY(maneuvers.back(), end_speed);

            end_dist = wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().back()).downtrack;
            end_dist = std::min(end_dist, route_length);

            if (isLaneChangeNeeded(following_lanelets, route_shortest_path[shortest_path_index + 1].id()))
            {
                maneuvers.push_back(composeLaneChangeManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path[shortest_path_index].id(), route_shortest_path[shortest_path_index + 1].id()));
                ++shortest_path_index; //Since lane change covers 2 lanelets - skip planning for the next lanelet
            }
            else
            {
                maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path[shortest_path_index].id()));
            }
        }
        //TO DO  - Update this block so that Stop and wait is the last maneuver ----
        //add lane follow as last maneuver if there is a lanelet unplanned for in path
        if (shortest_path_index < route_shortest_path.size() - 1)
        {
<<<<<<< HEAD
            ROS_ERROR_STREAM("Current position is not on the shortest path! Returning an empty maneuver");
            return true;
        }
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();
        double target_speed=findSpeedLimit(current_lanelet);   //get Speed Limit

        double total_maneuver_length = current_progress + mvr_duration_ * target_speed;
        double route_length=  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        //Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!=0){
            time_progress = req.prior_plan.planning_completion_time;
            if (time_progress.toSec() == 0.0)
            {
                ROS_DEBUG_STREAM("given time_progress (planning_completion_time) was zero!, defaulting to current time");
                time_progress =  ros::Time::now();
            }

            int end_lanelet =0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(),speed_progress,current_progress,end_lanelet);
            ROS_DEBUG_STREAM("was >> last_lanelet_index:" << last_lanelet_index);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet,shortest_path);
            ROS_DEBUG_STREAM("after updateCurrentStatus >> last_lanelet_index:" << last_lanelet_index);
=======
            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path.back());
            start_dist = wm_->routeTrackPos(route_shortest_path.back().centerline2d().front()).downtrack;
            end_dist = wm_->routeTrackPos(route_shortest_path.back().centerline2d().back()).downtrack;
            maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path.back().id()));
>>>>>>> feature/route_following_update
        }
        ////------------------
        ROS_DEBUG_STREAM("Maneuver plan along route successfully generated");
        return maneuvers;
    }

    bool RouteFollowingPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {
        if (latest_maneuver_plan_.empty())
        {
            ROS_ERROR_STREAM("A maneuver plan has not been generated");
            return false;
        }

<<<<<<< HEAD
            //if not already on last lanelet in path, check relation with next lanelet- follow lane change procedure if req, else lane follow
            
            // de facto lanechanging checker
            if ((last_lanelet_index + 1) < shortest_path.size() && identifyLaneChange(following_lanelets, shortest_path[last_lanelet_index + 1].id()))
            {
                ROS_DEBUG_STREAM("Lane change detected for index:" << last_lanelet_index);
                is_lanechanging_lanelet_[last_lanelet_index]= true;
                is_lanechanging_lanelet_[last_lanelet_index + 1] = true; //next lanelet is also technically lanechange
                is_first_lanechange_lanelet_[last_lanelet_index] = true;
            }

            if(is_lanechanging_lanelet_[last_lanelet_index])
            {
                //calculate required distance for lane change
                ROS_DEBUG_STREAM("Found lanechange at last_lanelet_index:" << last_lanelet_index);
                double longl_travel_dist = (target_speed*(LANE_CHANGE_TIME_MAX + buffer_lanechange_time_));
                double lane_change_start_dist; 
                double starting_lanelet_id;
                double ending_lanelet_id;
                int last_lanelet_index_temp; // use this index to handle 2 lc llts end_dist where they have to be same
                lane_change_start_dist = current_progress;

                if (is_first_lanechange_lanelet_[last_lanelet_index]) //handle lanelet mismatch if either of lc llts
                {
                    starting_lanelet_id = shortest_path[last_lanelet_index].id();
                    ending_lanelet_id = shortest_path[last_lanelet_index+1].id();
                    last_lanelet_index_temp = last_lanelet_index + 1;
                    ROS_DEBUG_STREAM("This is first lanechange lanelet, so starting_lanelet_id:" << starting_lanelet_id <<
                        ", ending_lanelet_id" << ending_lanelet_id << ", last_lanelet_index_temp" << last_lanelet_index_temp);

                }
                else  //if not duplicate, yet not first lanehcange lanelet (because later, we are not composing for duplicate lc lanelet anyway)
                {
                    starting_lanelet_id = shortest_path[last_lanelet_index -1].id();
                    ending_lanelet_id = shortest_path[last_lanelet_index].id();
                    last_lanelet_index_temp = last_lanelet_index;
                    ROS_DEBUG_STREAM("This is second lanechange lanelet, so starting_lanelet_id:" << starting_lanelet_id <<
                        ", ending_lanelet_id" << ending_lanelet_id << ", last_lanelet_index_temp" << last_lanelet_index_temp);
                }

                /////////                            
                //TO DO- This if else condition might not be needed
                if(wm_->routeTrackPos(shortest_path[last_lanelet_index_temp].centerline2d().back()).downtrack >= route_length){
                    //lane_change_start_dist = route_length - longl_travel_dist;
                    lanechange_end_dist_map_[last_lanelet_index] = route_length;

                }
                else{
                    //lane_change_start_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index + 1].centerline2d().back()).downtrack - longl_travel_dist;
                    lanechange_end_dist_map_[last_lanelet_index]  = wm_->routeTrackPos(shortest_path[last_lanelet_index_temp].centerline2d().back()).downtrack;
                }
                ////////////////               
                dist_diff = lanechange_end_dist_map_[last_lanelet_index] - current_progress; //this could be zero if it is the second lanelet for lanechange, which is consistent for the rest of the logic
                ROS_DEBUG_STREAM("mish: dist_diff:" << dist_diff <<" at last_lanelet_index" << last_lanelet_index);
                ROS_DEBUG_STREAM("lanechange_end_dist_map_[last_lanelet_index]" << lanechange_end_dist_map_[last_lanelet_index] <<
                                    "current_progress:" << current_progress);
                
                if (std::fabs(lane_change_start_dist - lanechange_end_dist_map_[last_lanelet_index]) > 0.1) // check if this is duplicate lanechange maneuver (occurs for 2nd lanelet of the lanechange)
                {
                    resp.new_plan.maneuvers.push_back(
                composeLaneChangeManeuverMessage(lane_change_start_dist, lanechange_end_dist_map_[last_lanelet_index], speed_progress, target_speed, 
                                    starting_lanelet_id, ending_lanelet_id,
                                    time_progress));
                }
            }
            else
            {
                resp.new_plan.maneuvers.push_back(
                composeManeuverMessage(current_progress, end_dist,  
                                    speed_progress, target_speed, 
                                    shortest_path[last_lanelet_index].id(), time_progress));
                
            }
            current_progress += dist_diff;
            //time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
            speed_progress = target_speed;
            
=======
        double current_downtrack = wm_->routeTrackPos(current_loc_).downtrack;

        //Return the set of maneuvers which intersect with min_plan_duration
        int i = 0;
        double planned_time = 0.0;
>>>>>>> feature/route_following_update

        while (planned_time < min_plan_duration_ && i < latest_maneuver_plan_.size())
        {
            //Ignore plans for distance already covered
            if (GET_MANEUVER_PROPERTY(latest_maneuver_plan_[i], end_dist) < current_downtrack)
            {
                ++i;
                continue;
            }
<<<<<<< HEAD

            ++last_lanelet_index;

        }
        
        ROS_DEBUG_STREAM("Done Loop: approaching_route_end: " << approaching_route_end);
        if(approaching_route_end){
            ros::Time end_time = ros::Time::now();
            if (resp.new_plan.maneuvers.size() > 0) {
                end_time = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
=======
            if(planned_time == 0.0){
                //update start distance of first maneuver
                setManeuverStartDist(latest_maneuver_plan_[i], current_downtrack);
>>>>>>> feature/route_following_update
            }
            planned_time += getManeuverDuration(latest_maneuver_plan_[i], 0.0001).toSec();

            resp.new_plan.maneuvers.push_back(latest_maneuver_plan_[i]);
            ++i;
        }

        if (resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }
        //update plan

        //Update time progress for maneuvers
        updateTimeProgress(resp.new_plan.maneuvers, ros::Time::now());
        //update starting speed of first maneuver
        updateStartingSpeed(resp.new_plan.maneuvers.front(), current_speed_);

        return true;
    }

    ros::Duration RouteFollowingPlugin::getManeuverDuration(cav_msgs::Maneuver &maneuver, double epsilon) const
    {
        double maneuver_start_speed = GET_MANEUVER_PROPERTY(maneuver, start_speed);
        double manever_end_speed = GET_MANEUVER_PROPERTY(maneuver, end_speed);
        double cur_plus_target = maneuver_start_speed + manever_end_speed;
        if(cur_plus_target < epsilon){
            throw std::invalid_argument("Maneuver start and ending speed is zero");
        }
        ros::Duration duration;
        double maneuver_start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
        double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
        duration = ros::Duration((maneuver_end_dist - maneuver_start_dist) / (0.5 * cur_plus_target));

        return duration;
    }

    void RouteFollowingPlugin::updateTimeProgress(std::vector<cav_msgs::Maneuver> &maneuvers, ros::Time start_time) const
    {
        ros::Time time_progress = start_time;
        ros::Time prev_time = time_progress;

        for (auto &maneuver : maneuvers)
        {
            time_progress += getManeuverDuration(maneuver, 0.001);
            switch (maneuver.type)
            {
            case cav_msgs::Maneuver::LANE_FOLLOWING:
                maneuver.lane_following_maneuver.start_time = prev_time;
                maneuver.lane_following_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE:
                maneuver.lane_change_maneuver.start_time = prev_time;
                maneuver.lane_change_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                maneuver.intersection_transit_straight_maneuver.start_time = prev_time;
                maneuver.intersection_transit_straight_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                maneuver.intersection_transit_left_turn_maneuver.start_time = prev_time;
                maneuver.intersection_transit_left_turn_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                maneuver.intersection_transit_right_turn_maneuver.start_time = prev_time;
                maneuver.intersection_transit_right_turn_maneuver.end_time = time_progress;
                break;
            case cav_msgs::Maneuver::STOP_AND_WAIT:
                maneuver.stop_and_wait_maneuver.start_time = prev_time;
                maneuver.stop_and_wait_maneuver.start_time = time_progress;
                break;
            default:
                throw std::invalid_argument("Invalid maneuver type, cannot update time progress for maneuver");
            }
            prev_time = time_progress;
        }
    }

    void RouteFollowingPlugin::updateStartingSpeed(cav_msgs::Maneuver &maneuver, double start_speed) const
    {
        switch (maneuver.type)
        {
        case cav_msgs::Maneuver::LANE_FOLLOWING:
            maneuver.lane_following_maneuver.start_speed = start_speed;
            break;
        case cav_msgs::Maneuver::LANE_CHANGE:
            maneuver.lane_change_maneuver.start_speed = start_speed;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            maneuver.intersection_transit_straight_maneuver.start_speed = start_speed;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            maneuver.intersection_transit_left_turn_maneuver.start_speed = start_speed;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            maneuver.intersection_transit_right_turn_maneuver.start_speed = start_speed;
            break;
        case cav_msgs::Maneuver::STOP_AND_WAIT:
            maneuver.stop_and_wait_maneuver.start_speed = start_speed;
            break;
        default:
            throw std::invalid_argument("Invalid maneuver type, cannot update starting speed for maneuver");
        }
    }

    void RouteFollowingPlugin::setManeuverStartDist(cav_msgs::Maneuver &maneuver, double start_dist) const
    {
        switch (maneuver.type)
        {
        case cav_msgs::Maneuver::LANE_FOLLOWING:
            maneuver.lane_following_maneuver.start_dist = start_dist;
            break;
        case cav_msgs::Maneuver::LANE_CHANGE:
            maneuver.lane_change_maneuver.start_dist = start_dist;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            maneuver.intersection_transit_straight_maneuver.start_dist = start_dist;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            maneuver.intersection_transit_left_turn_maneuver.start_dist = start_dist;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            maneuver.intersection_transit_right_turn_maneuver.start_dist = start_dist;
            break;
        case cav_msgs::Maneuver::STOP_AND_WAIT:
            maneuver.stop_and_wait_maneuver.start_dist = start_dist;
            break;
        default:
            throw std::invalid_argument("Invalid maneuver type");
        }
    }

    int RouteFollowingPlugin::findLaneletIndexFromPath(int target_id, const lanelet::routing::LaneletPath &path) const
    {
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id) const
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = lanefollow_planning_tactical_plugin_;
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_following_maneuver.start_dist = start_dist;
        maneuver_msg.lane_following_maneuver.start_speed = start_speed;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        //Start time and end time for maneuver are assigned in updateTimeProgress

        ROS_INFO_STREAM("Creating lane follow start dist: " << start_dist << " end dist: " << end_dist);
        
        return maneuver_msg;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id) const
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        maneuver_msg.lane_change_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = lane_change_plugin_;
<<<<<<< HEAD
        ROS_DEBUG_STREAM("Plugin used for lane change:"<<lane_change_plugin_);
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.lane_change_maneuver.start_dist = current_dist;
        maneuver_msg.lane_change_maneuver.start_speed = current_speed;
        maneuver_msg.lane_change_maneuver.start_time = current_time;
=======
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_change_maneuver.start_dist = start_dist;
        maneuver_msg.lane_change_maneuver.start_speed = start_speed;
>>>>>>> feature/route_following_update
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;
        maneuver_msg.lane_change_maneuver.starting_lane_id = std::to_string(starting_lane_id);
        maneuver_msg.lane_change_maneuver.ending_lane_id = std::to_string(ending_lane_id);
        //Start time and end time for maneuver are assigned in updateTimeProgress

        ROS_INFO_STREAM("Creating lane change start dist: " << start_dist << " end dist: " << end_dist << " Starting llt: " << starting_lane_id << " Ending llt: " << ending_lane_id);

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

<<<<<<< HEAD
    void RouteFollowingPlugin::updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id){
        if(maneuver.type == cav_msgs::Maneuver::STOP_AND_WAIT){
            speed =0.0;
            current_progress = maneuver.stop_and_wait_maneuver.end_dist;
            lane_id = stoi(maneuver.stop_and_wait_maneuver.ending_lane_id);
        }
        else if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
            current_progress =  maneuver.lane_following_maneuver.end_dist;
            lane_id =  stoi(maneuver.lane_following_maneuver.lane_id);
        }
        else{
            speed = GET_MANEUVER_PROPERTY(maneuver,end_speed);
            current_progress =GET_MANEUVER_PROPERTY(maneuver,end_dist);
            lane_id = stoi(GET_MANEUVER_PROPERTY(maneuver,ending_lane_id));
        }
        ROS_INFO_STREAM("updateCurrentStatus: speed:"<<speed<<", current_progress:"<<current_progress << ", lane_id:"<<lane_id);
    }

    double RouteFollowingPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
=======
    double RouteFollowingPlugin::findSpeedLimit(const lanelet::ConstLanelet &llt)
>>>>>>> feature/route_following_update
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
}