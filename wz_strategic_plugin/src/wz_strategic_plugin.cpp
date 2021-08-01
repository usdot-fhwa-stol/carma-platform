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
#include "wz_strategic_plugin/wz_strategic_plugin.h"

#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                    ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        (mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                        throw new std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))

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

    bool WzStrategicPlugin::supportedLightState(lanelet::CarmaTrafficLightState state) const {
        switch(state) {
            // NOTE: Following cases are intentional fall through.
            // Supported light states
            case lanelet::CarmaTrafficLightState::PERMISSIVE_MOVEMENT_ALLOWED: // Solid Green there could be conflict traffic
            case lanelet::CarmaTrafficLightState::PERMISSIVE_CLEARANCE: // Yellow Solid there is a chance of conflicting traffic
            case lanelet::CarmaTrafficLightState::STOP_AND_REMAIN: // Solid Red
                return true;
            
            // Unsupported light states
            case lanelet::CarmaTrafficLightState::UNAVAILABLE: // No data available
            case lanelet::CarmaTrafficLightState::DARK: // Light is non-functional
            case lanelet::CarmaTrafficLightState::STOP_THEN_PROCEED: // Flashing Red
            case lanelet::CarmaTrafficLightState::PRE_MOVEMENT: // Yellow Red transition (Found only in the EU)
            case lanelet::CarmaTrafficLightState::PROTECTED_MOVEMENT_ALLOWED: // Solid Green no chance of conflict traffic (normally used with arrows)
            case lanelet::CarmaTrafficLightState::PROTECTED_CLEARANCE: // Yellow Solid no chance of conflicting traffic (normally used with arrows)
            case lanelet::CarmaTrafficLightState::CAUTION_CONFLICTING_TRAFFIC: // Yellow Flashing
            default:
                return false;
        }
    }

    double WzStrategicPlugin::estimate_distance_to_stop(double v, double a) {
        return (v*v)/2*a;
    }

    double WzStrategicPlugin::estimate_time_to_stop(double d, double v) {
        return 2*d/v;
    };

    bool WzStrategicPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {
        ROS_DEBUG("Finding car infomrmation");

        if (wm_->getRoute()) {
            ROS_ERROR_STREAM("Could not plan maneuvers as route was not available");
            return true;
        }

        // find car_down_track
        double current_car_down_track = wm_->routeTrackPos(current_loc_).downtrack;
        ROS_DEBUG("current_car_down_track %d", current_car_down_track);

        
        // Algorithm should be 
        // 1. Get vehicle state (downtrack, speed, time) based on 
        // 2. Identify distance required to stop
        // 3. Get lanelets from current location to either distance required to stop or end of maneuver planning distance
        // 4. Check retrieved lanelts for traffic lights. 
        // 5. Calculate stopping 






        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc_, 1);
        if(current_lanelets.size() == 0) {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return false;
        }

        auto current_lanelet = current_lanelets[0];

        ros::Time start_time;
        double current_progress = 0;

        if(req.prior_plan.maneuvers.size() > 0) {
            current_progress = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
            start_time = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
        } else {
            start_time = ros::Time::now();
            current_progress = current_car_down_track;
        }


        ROS_DEBUG("\n\nFinding traffic_light information");
        auto traffic_list = wm_->predictTrafficLight(current_loc_);

        if(!traffic_list.empty()) {

            auto nearest_traffic_light = traffic_list.front();
            double traffic_light_down_track = wm_->routeTrackPos(nearest_traffic_light->stopLine().front().front().basicPoint2d()).downtrack;
            ROS_DEBUG("traffic_light_down_track %d", traffic_light_down_track);

            double workzone_end_point_down_track = wm_->routeTrackPos(nearest_traffic_light->getControlledLanelets().back().centerline2d().back()).downtrack;

            auto traffic_light_lanelet = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, nearest_traffic_light->stopLine().front().front().basicPoint2d(), 1)[0];

            double distance_remaining_to_traffic_light  = traffic_light_down_track - current_progress;
            ROS_DEBUG("distance_remaining_to_traffic_light %d", distance_remaining_to_traffic_light);

            if(distance_remaining_to_traffic_light <= max_distance_to_traffic_light ) {        

                ROS_DEBUG("distance_remaining_to_traffic_light is smaller than max_distance_to_traffic_light %d %d", distance_remaining_to_traffic_light, max_distance_to_traffic_light);

                ROS_DEBUG("current_speed_ %d", current_speed_);

                ROS_DEBUG("time_remaining_to_traffic_light %d", distance_remaining_to_traffic_light / current_speed_);
                ros::Duration time_remaining_to_traffic_light(distance_remaining_to_traffic_light / current_speed_);

                lanelet::Id start_lane_id = current_lanelet.second.id();
                lanelet::Id end_lane_id = traffic_light_lanelet.second.id();

                auto traffic_light_current_state = nearest_traffic_light->predictState(start_time); 
                
                if (!traffic_light_current_state) { // Check if light state was available
                    ROS_WARN_STREAM("Traffic light data no available for start time " << start_time.toSec());
                    // TODO what to return here?
                }     

                if (!supportedLightState(traffic_light_current_state.get())) { // Check if current state is supported type
                    ROS_ERROR_STREAM("WorkZone Plugin asked to handle CarmaTrafficLightState: " << traffic_light_current_state.get() << " which is not supported.");
                    // TODO what to return here?
                }

                ROS_DEBUG_STREAM("traffic_light_current_state" << traffic_light_current_state.get());
                



                auto traffic_light_next_predicted_state = nearest_traffic_light->predictState(start_time + time_remaining_to_traffic_light);

                if (!traffic_light_next_predicted_state) { // Check if light state was available
                    ROS_WARN_STREAM("Traffic light data no available for start time " << start_time.toSec());
                    // TODO what to return here?
                }     

                if (!supportedLightState(traffic_light_next_predicted_state.get())) { // Check if current state is supported type
                    ROS_ERROR_STREAM("WorkZone Plugin asked to handle CarmaTrafficLightState: " << traffic_light_next_predicted_state.get() << " which is not supported.");
                    // TODO what to return here?
                }

                ROS_DEBUG_STREAM("traffic_light_next_predicted_state" << traffic_light_next_predicted_state.get());


                if(traffic_light_current_state.get() != traffic_light_next_predicted_state.get()) {
                    
                    // Treat both red and yellow lights the same
                    if(traffic_light_next_predicted_state.get() == lanelet::CarmaTrafficLightState::STOP_AND_REMAIN 
                        || traffic_light_next_predicted_state.get() == lanelet::CarmaTrafficLightState::PERMISSIVE_CLEARANCE) {

                        double estimated_distance_to_stop = estimate_distance_to_stop(current_speed_,declaration);
                        double estimated_time_to_stop = estimate_time_to_stop(estimated_distance_to_stop,current_speed_);

                        double lane_following_distance = traffic_light_down_track - estimated_distance_to_stop;

                        // fix time
                        // stop_and_wait
                        cav_msgs::Maneuver stop_and_wait = composeStopAndWaitManeuverMessage(current_progress + lane_following_distance, 
                                                traffic_light_down_track, 
                                                current_speed_, 
                                                start_lane_id,
                                                end_lane_id,
                                                start_time,
                                                estimated_time_to_stop);

                        resp.new_plan.maneuvers.push_back(stop_and_wait);

                        // workzone_tactical
                        cav_msgs::Maneuver workzone_tactical = composeIntersectionTransitMessage( traffic_light_down_track,
                                                                                                  workzone_end_point_down_track, 
                                                                                                  current_speed_, 
                                                                                                  current_speed_,
                                                                                                  start_time + ros::Duration(std::max(15.0,estimated_time_to_stop)),
                                                                                                  start_lane_id);
                        resp.new_plan.maneuvers.push_back(workzone_tactical);

                    }

                    // Green light
                    if(traffic_light_next_predicted_state.get() == lanelet::CarmaTrafficLightState::PERMISSIVE_MOVEMENT_ALLOWED) {

                        // LaneFollowing
                        cav_msgs::Maneuver Lane_Following = composeLaneFollowingManeuverMessage(current_progress, 
                                                                                                traffic_light_down_track, 
                                                                                                current_speed_, 
                                                                                                current_speed_, 
                                                                                                start_lane_id);
                        resp.new_plan.maneuvers.push_back(Lane_Following);


                        // workzone_tactical
                        cav_msgs::Maneuver workzone_tactical = composeIntersectionTransitMessage(traffic_light_down_track, 
                                                                                                 workzone_end_point_down_track, 
                                                                                                 current_speed_, 
                                                                                                 current_speed_, 
                                                                                                 start_time + time_remaining_to_traffic_light, 
                                                                                                 start_lane_id);
                        resp.new_plan.maneuvers.push_back(workzone_tactical);

                    }

                }
                else {

                    // Treat both red and yellow lights the same
                    if(traffic_light_current_state.get() == lanelet::CarmaTrafficLightState::STOP_AND_REMAIN 
                        || traffic_light_current_state.get() == lanelet::CarmaTrafficLightState::PERMISSIVE_CLEARANCE) {

                        double estimated_distance_to_stop = estimate_distance_to_stop(current_speed_,declaration);
                        double estimated_time_to_stop = estimate_time_to_stop(estimated_distance_to_stop,current_speed_);

                        double lane_following_distance = traffic_light_down_track - estimated_distance_to_stop;

                        // stop_and_wait
                        cav_msgs::Maneuver stop_and_wait = composeStopAndWaitManeuverMessage(current_progress + lane_following_distance, 
                                                traffic_light_down_track, 
                                                current_speed_, 
                                                start_lane_id,
                                                end_lane_id,
                                                start_time,
                                                estimated_time_to_stop);

                        resp.new_plan.maneuvers.push_back(stop_and_wait);

                        // workzone_tactical
                        cav_msgs::Maneuver workzone_tactical = composeIntersectionTransitMessage( traffic_light_down_track,
                                                                                                  workzone_end_point_down_track, 
                                                                                                  current_speed_, 
                                                                                                  current_speed_, 
                                                                                                  start_time + ros::Duration(std::max(15.0,estimated_time_to_stop)), 
                                                                                                  start_lane_id);
                        resp.new_plan.maneuvers.push_back(workzone_tactical);

                    }

                    // Green light
                    if(traffic_light_current_state.get() == lanelet::CarmaTrafficLightState::PERMISSIVE_MOVEMENT_ALLOWED) {

                        // LaneFollowing
                        cav_msgs::Maneuver Lane_Following = composeLaneFollowingManeuverMessage(current_progress, 
                                                                                                traffic_light_down_track, 
                                                                                                current_speed_, 
                                                                                                current_speed_, 
                                                                                                start_lane_id);
                        resp.new_plan.maneuvers.push_back(Lane_Following);

                        // workzone_tactical
                        cav_msgs::Maneuver workzone_tactical = composeIntersectionTransitMessage(traffic_light_down_track, 
                                                                                                 workzone_end_point_down_track, 
                                                                                                 current_speed_, 
                                                                                                 current_speed_, 
                                                                                                 start_time + time_remaining_to_traffic_light,
                                                                                                 start_lane_id);
                        resp.new_plan.maneuvers.push_back(workzone_tactical);

                    }
                
                }
            }
            else {
                ROS_DEBUG("distance_remaining_to_traffic_light is larger than max_distance_to_traffic_light ");
            }
        }

        return true;
    }

    cav_msgs::Maneuver WzStrategicPlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InLaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.lane_following_maneuver.start_dist = start_dist;
        maneuver_msg.lane_following_maneuver.start_speed = start_speed;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lane_id) }; // TODO might not be valid assumption
        //Start time and end time for maneuver are assigned in updateTimeProgress

        ROS_INFO_STREAM("Creating lane follow start dist: " << start_dist << " end dist: " << end_dist);
        
        return maneuver_msg;
    }

    cav_msgs::Maneuver WzStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double& end_dist, double start_speed, lanelet::Id& starting_lane_id, lanelet::Id& ending_lane_id, ros::Time time, double& time_to_stop)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = "StopAndWaitPlugin";
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
        maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
        maneuver_msg.stop_and_wait_maneuver.start_speed = start_speed;
        // maneuver_msg.stop_and_wait_maneuver.target_speed = target_speed;
        maneuver_msg.stop_and_wait_maneuver.start_time = time;
        maneuver_msg.stop_and_wait_maneuver.end_time = time + ros::Duration(std::max(15.0,time_to_stop));
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = starting_lane_id;
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = ending_lane_id;
        ROS_INFO_STREAM("Creating lane follow start dist: " << current_dist << " end dist: " << end_dist);

        return maneuver_msg;
    }

    cav_msgs::Maneuver WzStrategicPlugin::composeIntersectionTransitMessage(double& start_dist, double& end_dist, double& start_speed, double& target_speed, ros::Time start_time, lanelet::Id& starting_lane_id)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
        maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin = "IntersectionTransitPlugin";
        maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin = planning_strategic_plugin_;
        maneuver_msg.intersection_transit_straight_maneuver.start_dist = start_dist;
        maneuver_msg.intersection_transit_straight_maneuver.start_speed = start_speed;
        maneuver_msg.intersection_transit_straight_maneuver.end_dist = end_dist;
        maneuver_msg.intersection_transit_straight_maneuver.end_speed = target_speed;
        maneuver_msg.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(starting_lane_id);

        //Start time and end time for maneuver are assigned in updateTimeProgress

        ROS_INFO_STREAM("Creating WORKZONE start dist: " << start_dist << " end dist: " << end_dist);
        
        return maneuver_msg;

    }

    void WzStrategicPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

}