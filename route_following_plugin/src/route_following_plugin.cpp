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
        upcoming_lane_change_status_pub_ = nh_->advertise<cav_msgs::UpcomingLaneChangeStatus>("upcoming_lane_change_status", 1);
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
        pnh_->param<std::string>("lane_change_plugin", lane_change_plugin_, "CooperativeLaneChangePlugin");

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
            ROS_DEBUG_STREAM("current shortest_path_index:" << shortest_path_index);

            auto following_lanelets = wm_->getRoute()->followingRelations(route_shortest_path[shortest_path_index]);
            ROS_DEBUG_STREAM("following_lanelets.size():" << following_lanelets.size());

            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path[shortest_path_index]);

            //update start distance and start speed from previous maneuver if it exists
            start_dist = (maneuvers.empty()) ? wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().front()).downtrack : GET_MANEUVER_PROPERTY(maneuvers.back(), end_dist);
            start_speed = (maneuvers.empty()) ? 0.0 : GET_MANEUVER_PROPERTY(maneuvers.back(), end_speed);
            ROS_DEBUG_STREAM("start_dist:" << start_dist << ", start_speed:" << start_speed);

            end_dist = wm_->routeTrackPos(route_shortest_path[shortest_path_index].centerline2d().back()).downtrack;
            ROS_DEBUG_STREAM("end_dist:" << end_dist);
            end_dist = std::min(end_dist, route_length);
            ROS_DEBUG_STREAM("min end_dist:" << end_dist);

            if (std::fabs(start_dist - end_dist) < 0.1) //TODO: edge case that was not recreatable. Sometimes start and end dist was same which crashes inlanecruising
            {
                ROS_WARN_STREAM("start and end dist are equal! shortest path id" << shortest_path_index << ", lanelet id:" << route_shortest_path[shortest_path_index].id() <<
                    ", start and end dist:" << start_dist);
                continue;
            }



            if (isLaneChangeNeeded(following_lanelets, route_shortest_path[shortest_path_index + 1].id()))
            {
                ROS_DEBUG_STREAM("LaneCHangeNeeded");
    
                maneuvers.push_back(composeLaneChangeManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path[shortest_path_index].id(), route_shortest_path[shortest_path_index + 1].id()));
                ++shortest_path_index; //Since lane change covers 2 lanelets - skip planning for the next lanelet

                //Determine the Lane Change Status
                ROS_DEBUG_STREAM("Recording lanechange start_dist <<" << start_dist  << ", from llt id:" << route_shortest_path[shortest_path_index].id() << " to llt id: " << 
                    route_shortest_path[shortest_path_index+ 1].id());
                upcoming_lane_change_status_msg_map_.push({start_dist, ComposeLaneChangeStatus(route_shortest_path[shortest_path_index],route_shortest_path[shortest_path_index + 1])});
            }
            else
            {
                ROS_DEBUG_STREAM("Lanechange NOT Needed ");
                maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path[shortest_path_index].id()));
            }
        }
        //TO DO  - Update this block so that Stop and wait is the last maneuver ----
        //add lane follow as last maneuver if there is a lanelet unplanned for in path
        if (shortest_path_index < route_shortest_path.size())
        {
            double target_speed_in_lanelet = findSpeedLimit(route_shortest_path.back());
            start_dist = wm_->routeTrackPos(route_shortest_path.back().centerline2d().front()).downtrack;
            end_dist = wm_->routeTrackPos(route_shortest_path.back().centerline2d().back()).downtrack;
            maneuvers.push_back(composeLaneFollowingManeuverMessage(start_dist, end_dist, start_speed, target_speed_in_lanelet, route_shortest_path.back().id()));
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

        double current_downtrack = wm_->routeTrackPos(current_loc_).downtrack;

        //Return the set of maneuvers which intersect with min_plan_duration
        int i = 0;
        double planned_time = 0.0;

        while (planned_time < min_plan_duration_ && i < latest_maneuver_plan_.size())
        {
            ROS_DEBUG_STREAM("Checking maneuver id " << i);
            //Ignore plans for distance already covered
            if (GET_MANEUVER_PROPERTY(latest_maneuver_plan_[i], end_dist) < current_downtrack)
            {
                ROS_DEBUG_STREAM("Skipping maneuver id " << i);

                ++i;
                continue;
            }
            if(planned_time == 0.0){
                //update start distance of first maneuver
                setManeuverStartDist(latest_maneuver_plan_[i], current_downtrack);
            }
            planned_time += getManeuverDuration(latest_maneuver_plan_[i], epsilon_).toSec();

            resp.new_plan.maneuvers.push_back(latest_maneuver_plan_[i]);
            ++i;
        }

        if (resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
            return false;
        }
        //update plan

        //Update time progress for maneuvers
        updateTimeProgress(resp.new_plan.maneuvers, ros::Time::now());
        //update starting speed of first maneuver
        updateStartingSpeed(resp.new_plan.maneuvers.front(), current_speed_);
        //TO DO - replace block below with better method to update ending speed if speed limit changes
        //update End distance for current maneuver based on speed limit of current lanelet
        auto shortest_path = wm_->getRoute()->shortestPath();
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc_, 10);
        lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc_, llt.second.polygon2d()))
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
        updateEndingSpeed(resp.new_plan.maneuvers.front(), findSpeedLimit(current_lanelet));

        return true;
    }

    cav_msgs::UpcomingLaneChangeStatus RouteFollowingPlugin::ComposeLaneChangeStatus(lanelet::ConstLanelet starting_lanelet,lanelet::ConstLanelet ending_lanelet)
    {
        cav_msgs::UpcomingLaneChangeStatus upcoming_lanechange_status_msg;
        // default to right lane change
        upcoming_lanechange_status_msg.lane_change = cav_msgs::UpcomingLaneChangeStatus::RIGHT; 
        // change to left if detected
        for (auto &relation : wm_->getRoute()->leftRelations(starting_lanelet))
        {
            ROS_DEBUG_STREAM("Checking relation.lanelet.id()" <<relation.lanelet.id());
            if (relation.lanelet.id() == ending_lanelet.id())
            {
                ROS_DEBUG_STREAM("relation.lanelet.id()" << relation.lanelet.id() << " is LEFT");
                upcoming_lanechange_status_msg.lane_change = cav_msgs::UpcomingLaneChangeStatus::LEFT;
                break;
            }  
        }
        ROS_DEBUG_STREAM("==== ComposeLaneChangeStatus Exiting now");
        return upcoming_lanechange_status_msg;
    }

    void RouteFollowingPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {

        ROS_DEBUG_STREAM("Entering pose_cb");
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        if (!wm_->getRoute())
            return;

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        current_loc_ = current_loc;
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        
        auto llts = wm_->getLaneletsFromPoint(current_loc, 10);                                          

        ROS_DEBUG_STREAM("pose_cb : current_progress" << current_progress << ", and upcoming_lane_change_status_msg_map_.size(): " << upcoming_lane_change_status_msg_map_.size());

        while (!upcoming_lane_change_status_msg_map_.empty() && current_progress > upcoming_lane_change_status_msg_map_.front().first)
        {
            ROS_DEBUG_STREAM("pose_cb : the vehicle has passed the lanechange point at downtrack" << upcoming_lane_change_status_msg_map_.front().first);
            upcoming_lane_change_status_msg_map_.pop();
        }

        if (!upcoming_lane_change_status_msg_map_.empty() && upcoming_lane_change_status_msg_map_.front().second.lane_change != cav_msgs::UpcomingLaneChangeStatus::NONE)
        {
            ROS_DEBUG_STREAM("upcoming_lane_change_status_msg_map_.lane_change : " << static_cast<int>(upcoming_lane_change_status_msg_map_.front().second.lane_change) << 
            ", downtrack until that lanechange: " << upcoming_lane_change_status_msg_map_.front().first);
            upcoming_lane_change_status_msg_map_.front().second.downtrack_until_lanechange=upcoming_lane_change_status_msg_map_.front().first-current_progress;
            ROS_DEBUG_STREAM("upcoming_lane_change_status_msg_map_.front().second.downtrack_until_lanechange: " <<static_cast<double>(upcoming_lane_change_status_msg_map_.front().second.downtrack_until_lanechange));
            upcoming_lane_change_status_pub_.publish(upcoming_lane_change_status_msg_map_.front().second); 
        }
      
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

        ROS_DEBUG_STREAM("maneuver_end_dist: " << maneuver_end_dist << ", maneuver_start_dist: " << maneuver_start_dist << ", cur_plus_target: " << cur_plus_target);

        duration = ros::Duration((maneuver_end_dist - maneuver_start_dist) / (0.5 * cur_plus_target));

        return duration;
    }

    void RouteFollowingPlugin::updateTimeProgress(std::vector<cav_msgs::Maneuver> &maneuvers, ros::Time start_time) const
    {
        ros::Time time_progress = start_time;
        ros::Time prev_time = time_progress;

        for (auto &maneuver : maneuvers)
        {
            time_progress += getManeuverDuration(maneuver, epsilon_);
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

    void RouteFollowingPlugin::updateEndingSpeed(cav_msgs::Maneuver &maneuver, double end_speed) const
    {
        ROS_DEBUG_STREAM("end_speed: " << end_speed);
        switch (maneuver.type)
        {
        case cav_msgs::Maneuver::LANE_FOLLOWING:
            maneuver.lane_following_maneuver.end_speed = end_speed;
            ROS_DEBUG_STREAM("end_speed updated for lane following: " << end_speed);
            break;
        case cav_msgs::Maneuver::LANE_CHANGE:
            maneuver.lane_change_maneuver.end_speed = end_speed;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            maneuver.intersection_transit_straight_maneuver.end_speed = end_speed;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            maneuver.intersection_transit_left_turn_maneuver.end_speed = end_speed;
            break;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            maneuver.intersection_transit_right_turn_maneuver.end_speed = end_speed;
            break;
        case cav_msgs::Maneuver::STOP_AND_WAIT:
            //Do nothing
            break;
        default:
            throw std::invalid_argument("Invalid maneuver type, cannot update starting speed for maneuver");
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
        ROS_DEBUG_STREAM("Returning -1 because could not find lanelet id" << target_id);

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

        ROS_DEBUG_STREAM("Creating lane follow start dist: " << start_dist << " end dist: " << end_dist << "lane_id" << maneuver_msg.lane_following_maneuver.lane_id);
        
        return maneuver_msg;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeLaneChangeManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id) const
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        maneuver_msg.lane_change_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = lane_change_plugin_;
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_change_maneuver.start_dist = start_dist;
        maneuver_msg.lane_change_maneuver.start_speed = start_speed;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;
        maneuver_msg.lane_change_maneuver.starting_lane_id = std::to_string(starting_lane_id);
        maneuver_msg.lane_change_maneuver.ending_lane_id = std::to_string(ending_lane_id);
        //Start time and end time for maneuver are assigned in updateTimeProgress

        ROS_DEBUG_STREAM("Creating lane change start dist: " << start_dist << " end dist: " << end_dist << " Starting llt: " << starting_lane_id << " Ending llt: " << ending_lane_id);

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
}