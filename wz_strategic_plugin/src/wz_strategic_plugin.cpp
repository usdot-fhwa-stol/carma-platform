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
#include "wz_strategic_plugin/wz_strategic_plugin.h"

namespace wz_strategic_plugin
{

    void WzStrategicPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        plan_maneuver_srv_ = nh_->advertiseService("plugins/RouteFollowing/plan_maneuvers", &WzStrategicPlugin::planManeuverCb, this);

        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "WzStrategic";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

        pose_sub_ = nh_->subscribe("current_pose", 1, &WzStrategicPlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &WzStrategicPlugin::twist_cb, this);

        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto &) { plugin_discovery_pub_.publish(plugin_discovery_msg_); });

    }

    void WzStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        pose_msg_ = msg;
        lanelet::BasicPoint2d curr_loc(pose_msg_->pose.position.x, pose_msg_->pose.position.y);
        current_loc_ = curr_loc;
    }

    void WzStrategicPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr &msg)
    {
        current_speed_ = msg->twist.linear.x;
    }


    bool WzStrategicPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {

        // find car_down_track
        //

        double current_car_down_track = wm_->routeTrackPos(current_loc_).downtrack;

        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc_, 1);

        // find traffic_light_down_track
        //

        // auto regems = wml.getWorldModel()->getMap()->laneletLayer.get(current_lanelets[0].id()).regulatoryElements();
        auto traffic_list = current_lanelets[0].regulatoryElementsAs<lanelet::CarmaTrafficLight>();

        double traffic_light_down_track = 0;

        if(!traffic_list.empty()) {
            traffic_light_down_track = traffic_list[0].stopLine().down_track;
        }

        double distance_remaining_to_traffic_light  = traffic_light_down_track - current_car_down_track;
        double time_remaining_to_traffic_light = distance_remaining_to_traffic_light / current_speed_;

        // get traffic light timeming 

        auto traffic_light_current_state = traffic_list[0].getState();
        auto traffic_light_next_state = "";

        double current_traffic_light_state_remaining_time = 0;
        double time_remaining_to_traffic_light = 0;      


        // enum class CarmaTrafficLightState {UNAVAILABLE=0,DARK=1,STOP_THEN_PROCEED=2,STOP_AND_REMAIN=3,PRE_MOVEMENT=4,PERMISSIVE_MOVEMENT_ALLOWED=5,PROTECTED_MOVEMENT_ALLOWED=6,PERMISSIVE_CLEARANCE=7,PROTECTED_CLEARANCE=8,CAUTION_CONFLICTING_TRAFFIC=9};

        if(distance_remaining_to_traffic_light > min_distance_to_traffic_light ) {        

            if(time_remaining_to_traffic_light > current_traffic_light_state_remaining_time) {

                if(traffic_light_next_state == "red") {

                    // stop_and_wait
                    composeStopandWaitManeuverMessage( current_car_down_track, traffic_light_down_track, current_speed_, start_lane_id, end_lane_id, current_time, end_time)

                    // workzone_tactical

                }

                if(traffic_light_next_state == "green") {

                    // LaneFollowing
                    composeLaneFollowingManeuverMessage(current_car_down_track, traffic_light_down_track, current_speed_, current_speed_, lane_id)

                    // workzone_tactical

                }

            }

            if(time_remaining_to_traffic_light < current_traffic_light_state_remaining_time) {

                if(traffic_light_current_state == "red") {

                    // stop_and_wait
                    composeStopandWaitManeuverMessage( current_car_down_track, traffic_light_down_track, current_speed_, start_lane_id, end_lane_id, current_time, end_time)
                    // workzone_tactical

                }

                if(traffic_light_current_state == "green") {

                    // LaneFollowing
                    composeLaneFollowingManeuverMessage(current_car_down_track, traffic_light_down_track, current_speed_, current_speed_, lane_id)
                    // workzone_tactical

                }

            }

        }

        
        return true;
    }

    cav_msgs::Maneuver WzStrategicPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id) const
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

    cav_msgs::Maneuver WzStrategicPlugin::composeStopandWaitManeuverMessage(double current_dist, double end_dist, double current_speed, int start_lane_id, int end_lane_id, ros::Time current_time, double end_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = "StopandWaitPlugin";
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
        maneuver_msg.stop_and_wait_maneuver.start_speed = current_speed;
        maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
        maneuver_msg.stop_and_wait_maneuver.start_time = current_time;
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(start_lane_id);
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(end_lane_id);
        if(end_time < mvr_duration_){
            end_time = mvr_duration_;
        }

        maneuver_msg.stop_and_wait_maneuver.end_time = current_time + ros::Duration(end_time);

        ROS_INFO_STREAM("Creating StopandWait start dist: " << current_dist << " end dist: " << end_dist);

        return maneuver_msg;
    }

    cav_msgs::Maneuver WzStrategicPlugin::composeWorkZoneManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id) const
    {
    }

    void WzStrategicPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

}