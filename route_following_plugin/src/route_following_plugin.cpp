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


namespace route_following_plugin
{
    RouteFollowingPlugin::RouteFollowingPlugin() : current_speed_(0.0), mvr_duration_(16.0) {}

    void RouteFollowingPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        plan_maneuver_srv_ = nh_->advertiseService("plugins/RouteFollowingPlugin/plan_maneuvers", &RouteFollowingPlugin::plan_maneuver_cb, this);
                
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "RouteFollowingPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

        pose_sub_ = nh_->subscribe("current_pose", 1, &RouteFollowingPlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteFollowingPlugin::twist_cd, this);
        
        pnh_->param<double>("minimal_maneuver_duration", mvr_duration_, 16.0);

        // set world model point form wm listener
        wm_ = wml_.getWorldModel();

        ros::CARMANodeHandle::setSpinCallback([this]() {
            plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });
    }

    void RouteFollowingPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    bool RouteFollowingPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp){        

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 1);
        if(current_lanelets.size() == 0) {
            ROS_WARN_STREAM("Cannot find current lanelet");
            return true;
        }
        auto current_lanelet = current_lanelets[0];
        double current_progress = wm_->routeTrackPos(lanelet::BasicPoint2d(pose_msg_.pose.position.x, pose_msg_.pose.position.y)).downtrack;
        double speed_progress = current_speed_;
        double total_maneuver_length = current_progress + mvr_duration_ * RouteFollowingPlugin::TWENTY_FIVE_MPH_IN_MS;
        auto shortest_path = wm_->getRoute()->shortestPath();
        int last_lanelet_index = -1;
        for(int i = 0; i < shortest_path.size(); ++i)
        {
            if(shortest_path[i].id() == current_lanelet.second.id())
            {
                last_lanelet_index = i;
                break;
            }
        }
        while(current_progress < total_maneuver_length && last_lanelet_index < shortest_path.size())
        {
            cav_msgs::Maneuver maneuver_msg;
            maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
            maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
            maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
            maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InlaneCruisingPlugin";
            maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
            
            double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
            double dist_diff = end_dist - current_progress;

            maneuver_msg.lane_following_maneuver.start_dist = current_progress;
            maneuver_msg.lane_following_maneuver.start_speed = speed_progress;
            maneuver_msg.lane_following_maneuver.start_time = ros::Time::now();
            maneuver_msg.lane_following_maneuver.end_dist = end_dist;
            maneuver_msg.lane_following_maneuver.end_speed = TWENTY_FIVE_MPH_IN_MS;
            maneuver_msg.lane_following_maneuver.end_time = ros::Time::now() + ros::Duration(dist_diff / (0.5 * (current_speed_ + TWENTY_FIVE_MPH_IN_MS)));
            maneuver_msg.lane_following_maneuver.lane_id = std::to_string(shortest_path[last_lanelet_index].id());

            resp.new_plan.maneuvers.push_back(maneuver_msg);

            current_progress += dist_diff;
            speed_progress = TWENTY_FIVE_MPH_IN_MS;
            if(current_progress >= total_maneuver_length || last_lanelet_index + 1 == shortest_path.size())
            {
                break;
            }

            auto following_lanelets = wm_->getRoute()->followingRelations(shortest_path[last_lanelet_index]);
            if(following_lanelets.size() == 0)
            {
                ROS_WARN_STREAM("Cannot find the following lanelet.");
                return true;
            }
            bool successor_found = false;
            for(auto& relation : following_lanelets)
            {
                if(relation.lanelet.id() == shortest_path[last_lanelet_index + 1].id())
                {
                    if(relation.relationType == lanelet::routing::RelationType::Successor) {
                        ++last_lanelet_index;
                        successor_found = true;
                        break;
                    } else
                    {
                        ROS_WARN_STREAM("Next lanelet requires a lane change maneuver. It is not supported yet.");
                        return true;
                    }
                }
            }
            if(!successor_found) {
                ROS_WARN_STREAM("Cannot find the next lanelet in the current lanelet's successor list!");
                return true;
            }
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

}
