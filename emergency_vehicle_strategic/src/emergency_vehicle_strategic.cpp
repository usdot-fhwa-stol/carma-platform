/*
 * Copyright (C) 2019-2021 LEIDOS.
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

// Developed by the Fang Zhou - JFL Solutions LLC.

#include <ros/ros.h>
#include <string>
#include "emergency_vehicle_strategic.h"
#include <array>
#include <stdlib.h> 

namespace emergency_vehicle_strategic
{
    // EmergencyVehicleStrategicPlugin::EmergencyVehicleStrategicPlugin();
    // -------------------------------- Plug-in constructor --------------------------------
    EmergencyVehicleStrategicPlugin::EmergencyVehicleStrategicPlugin(carma_wm::WorldModelConstPtr wm, 
                                                                       EmergencyVehicleStrategicPluginConfig config,
                                                                       PublishPluginDiscoveryCB plugin_discovery_publisher,
                                                                       StopVehicleCB stop_vehicle_publisher,
                                                                       TurningSignalCB turn_signal_publisher
                                                                       ):
    plugin_discovery_publisher_(plugin_discovery_publisher),
    stop_vehicle_publisher_(stop_vehicle_publisher),
    turn_signal_publisher_(turn_signal_publisher), 
    wm_(wm),
    config_(config)
    {
        ROS_DEBUG_STREAM("Top of EmergencyVehicleStrategicPlugin actor.");
        std::string hostStaticId = config_.vehicleID; //static ID for this vehicle
        long cur_t = ros::Time::now().toNSec()/1000000; // current time in millisecond
        
        plugin_discovery_msg_.name = "EmergencyVehicleStrategicPlugin";
        plugin_discovery_msg_.version_id = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
    }
    
    // -------------------------------- Data extraction --------------------------------
    // Build map projector from projection string (georefernce).
    void EmergencyVehicleStrategicPlugin::georeference_cb(const std_msgs::StringConstPtr& msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str()); 
    }

    void EmergencyVehicleStrategicPlugin::controller_setting_cb(std_msgs::Float32 msg)
    {
        controler_look_ahead_distance_=msg.data;
    }

    // Find ecef point based on pose message
    cav_msgs::LocationECEF EmergencyVehicleStrategicPlugin::pose_to_ecef(geometry_msgs::PoseStamped pose_msg)
    {
        // check map projector 
        if (!map_projector_) 
        {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
        
        // read host location
        cav_msgs::LocationECEF location;
        // note: ecef point read from map projector is in m.
        lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({pose_msg.pose.position.x, pose_msg.pose.position.y, 0.0}, 1);
        location.ecef_x = ecef_point.x() * 100.0;
        location.ecef_y = ecef_point.y() * 100.0;
        location.ecef_z = ecef_point.z() * 100.0;    
        
        // note: the returned ecef is in cm.
        return location;
    }

    // Function to assign host pose_ecef_point_
    void EmergencyVehicleStrategicPlugin::setHostECEF(cav_msgs::LocationECEF pose_ecef_point)
    {
        // Note, the ecef here is in cm. 
        pose_ecef_point_ = pose_ecef_point;
    }
    
    // Callback to update downtrack and crosstrack based on pose message.
    void EmergencyVehicleStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        if (!wm_->getRoute())
        return;
    
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        // read current location based on pose msg
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        carma_wm::TrackPos tc = wm_->routeTrackPos(current_loc);

        // update host's DtD and CtD
        current_downtrack_ = tc.downtrack;
        current_crosstrack_ = tc.crosstrack;

        // set host ecef
        // note: the ecef read from "pose_ecef_point" is in cm.
        cav_msgs::LocationECEF pose_ecef_point = pose_to_ecef(pose_msg_);
        setHostECEF(pose_ecef_point);
        
    }

    // callback to update the command speed on x direction, in m/s.
    void EmergencyVehicleStrategicPlugin::cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        cmd_speed_ = msg->twist.linear.x;
    }
   
    // twist command, read linear speed on x direction, in m/s.
    void EmergencyVehicleStrategicPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
        if (current_speed_ < STOPPED_SPEED)     // note: stopped_speed (threshold of determining stop) is defined in the header file.
        {
            current_speed_ = 0.0;
        }
    }

    // twist command, read linear speed on x direction, in m/s.
    void EmergencyVehicleStrategicPlugin::route_cb(const cav_msgs::RouteConstPtr& msg)
    {
        route_msg_ = cav_msgs::Route(*msg.get());
    }

    // Find the speed limit for the current road (also used as the desired speed).
    double EmergencyVehicleStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        double target_speed = 0.0;

        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        if (traffic_rules)
        {
            target_speed =(*traffic_rules)->speedLimit(llt).speedLimit.value();
        }
        else
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }        
        return target_speed;
    }

    // Return the ecef point projected to local map point.
    lanelet::BasicPoint2d EmergencyVehicleStrategicPlugin::ecef_to_map_point(cav_msgs::LocationECEF ecef_point)
    {
        if (!map_projector_) 
        {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }

        lanelet::BasicPoint3d map_point = map_projector_->projectECEF(
            {(double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0 }, -1);
        
        lanelet::BasicPoint2d output {map_point.x(), map_point.y()};
        return output;
    } 
    
    // -------------------------------- Helper functions --------------------------------

    // callback function to go through external object list and check if emergency vehicle exist
    void EmergencyVehicleStrategicPlugin::external_objects_cb(const cav_msgs::ExternalObjectListConstPtr& msg)
    {
        /**
         * Note: 
         * This function is called in the emergency pullover strategic plugin node, which is running consistently with the node. 
         * So, it is necessary to check for the already detected emergency vehicle. Once detected, host vehicle should
         *  stop searching and send lane change plan right away. It is assumed only one emergency vehicle will appear.
         */
        
        // loop through objects to check for emergency vehicle 
        for (auto obj : msg->objects) 
        {
            // Note: add "emergency vehicle" type to cav_msgs/external_object
            if (obj.object_type == cav_msgs::ExternalObject::EMERGENCY_VEHICLE)
            {
                // first time detect 
                if (emergency_vehicle_detected_) 
                {
                    // update bool indicator if 1st detected
                    emergency_vehicle_detected_ = true;
                }

                // init a new msg to handle data type
                geometry_msgs::PoseStamped msg;
                geometry_msgs::PoseWithCovariance pose_msg = obj.pose;
                msg.pose = pose_msg.pose;
                
                // update emergency vehicle 
                emergency_vehicle_pose_ = msg;
                // update emergency vehicle ID
                emergency_vehicle_id_ = obj.id;

                if (emergency_vehicle_pose_.pose.position.x>2)
                {
                    front_emergency_vehicle_detected_=true;
                    rear_emergency_vehicle_detected_=!front_emergency_vehicle_detected_;
                }else if(emergency_vehicle_pose_.pose.position.x<-2)
                {
                    rear_emergency_vehicle_detected_=true;
                    front_emergency_vehicle_detected_=!rear_emergency_vehicle_detected_;
                }
                
                return;
            }
        }    
        return;    
    }

    void EmergencyVehicleStrategicPlugin::lane_change_finish_manual_cb(const std_msgs::BoolConstPtr& msg)
    {
        lane_change_finished_manual=true;
        ROS_DEBUG_STREAM("lane_change_finished_manual received");
    }
    
    bool EmergencyVehicleStrategicPlugin::isSameLaneWithEmergencyVehicle(geometry_msgs::PoseStamped pose_msg)
    {
        // find ecef location of the emergency vehicle
        emergency_ecef_location_ = pose_to_ecef(pose_msg);
        // find dtd and ctd
        lanelet::BasicPoint2d current_loc(pose_msg.pose.position.x, pose_msg.pose.position.y);
        double emergency_dtd = wm_->routeTrackPos(current_loc).downtrack;
        double emergency_ctd = wm_->routeTrackPos(current_loc).crosstrack;

        ROS_DEBUG_STREAM("Emergency vehicle down track from ecef: " << emergency_dtd << ", cross track from ecef: " << emergency_ctd);

        // check lane position 
        bool is_emergency_same_lane = abs(current_crosstrack_-emergency_ctd) <= config_.maxCrosstrackError;

        return is_emergency_same_lane;
    }

    // check if lane change is feasible 
    bool EmergencyVehicleStrategicPlugin::is_lanechange_possible(lanelet::Id start_lanelet_id, lanelet::Id target_lanelet_id)
    {
        lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(start_lanelet_id);
        lanelet::ConstLanelet ending_lanelet = wm_->getMap()->laneletLayer.get(target_lanelet_id);
        lanelet::ConstLanelet current_lanelet = starting_lanelet;
        bool shared_boundary_found = false;
        
        while(!shared_boundary_found)
        {
            //Assumption: Adjacent lanelets share lane boundary
            if(current_lanelet.leftBound() == ending_lanelet.rightBound())
            {   
                ROS_DEBUG_STREAM("Lanelet " << std::to_string(current_lanelet.id()) << " shares left boundary with " << std::to_string(ending_lanelet.id()));
                shared_boundary_found = true;
            }
            else if(current_lanelet.rightBound() == ending_lanelet.leftBound())
            {
                ROS_DEBUG_STREAM("Lanelet " << std::to_string(current_lanelet.id()) << " shares right boundary with " << std::to_string(ending_lanelet.id()));
                shared_boundary_found = true;
            }

            else
            {
                //If there are no following lanelets on route, lanechange should be completing before reaching it
                if(wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
                {
                    //In this case we have reached a lanelet which does not have a routable lanelet ahead + isn't adjacent to the lanelet where lane change ends
                    return false;
                }

                current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front(); 
                if(current_lanelet.id() == starting_lanelet.id())
                {
                    //Looped back to starting lanelet
                    return false;
                }
            }
        }

        return true;
    }

    // Return the lanelet id.
    int EmergencyVehicleStrategicPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
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

    // compose maneuver message 
    cav_msgs::Maneuver EmergencyVehicleStrategicPlugin::composeManeuverMessage(double current_dist, 
                                                                                                double end_dist, 
                                                                                                double current_speed, 
                                                                                                double target_speed, 
                                                                                                int lane_id, 
                                                                                                ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        // note: need to modify "cav_msgs/msg/Maneuver_parameters --> negotiation_type" to add enum "PULLOVER"
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::PULLOVER;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        // use default lane following tactical plug-in
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InLaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "EmergencyVehicleStrategicPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(config_.time_step);

        if (pull_in_flag_ && lane_id==3241)//normal road for prepass
        {
            lane_id=2997;
        }

        if (pull_in_flag_ && lane_id==4793)//ramp map
        {
            lane_id=5202;
        }else if (pull_in_flag_ && lane_id==5202)
        {
        }else if (lane_id==5202)
        {
            lane_id=4793;
        }

        maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lane_id) };

        lanelet::ConstLanelet current_lanelet = wm_->getMap()->laneletLayer.get(lane_id);
        if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
        {
            auto next_lanelet_id = wm_->getMapRoutingGraph()->following(current_lanelet, false).front().id();
  
            if (pull_in_flag_ && lane_id==1985)//normal road for prepass
            {
                next_lanelet_id=2997;
            }else if (lane_id==1985)
            {
                next_lanelet_id=3241;
            }

            if (pull_in_flag_ && lane_id==2109)//ramp map
            {
                next_lanelet_id=5202;
            }else if (lane_id==2109)
            {
                next_lanelet_id=4793;
            }
            maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(next_lanelet_id));

            for (int i = 0; i < config_.following_lanelets_number; ++i)
            {
                lanelet::ConstLanelet temp_lanelet = wm_->getMap()->laneletLayer.get(next_lanelet_id);
                if (!wm_->getMapRoutingGraph()->following(temp_lanelet, false).empty())
                {
                    next_lanelet_id = wm_->getMapRoutingGraph()->following(temp_lanelet, false).front().id();
                    maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(next_lanelet_id));
                }
                else
                {
                    ROS_DEBUG_STREAM("No following lanelets in the added for loop");
                }
            }
        }
        else
        {
            ROS_DEBUG_STREAM("No following lanelets");
        }

        current_time = maneuver_msg.lane_following_maneuver.end_time;
        ROS_DEBUG_STREAM(
                    "Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM(
                    "Duration:"<< maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    //compose maneuver message for lane change 
    cav_msgs::Maneuver EmergencyVehicleStrategicPlugin::composeLaneChangeManeuverMessage(double current_dist, 
                                                                                                          double end_dist, 
                                                                                                          double current_speed, 
                                                                                                          double target_speed, 
                                                                                                          int starting_lane_id, 
                                                                                                          int ending_lane_id, 
                                                                                                          ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        // change to lane change maneuvers
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        // note: need to modify "cav_msgs/msg/Maneuver_parameters --> negotiation_type" to add enum "PULLOVER"
        maneuver_msg.lane_change_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::PULLOVER;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = "CooperativeLaneChangePlugin";
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = "EmergencyVehicleStrategicPlugin";
        maneuver_msg.lane_change_maneuver.start_dist = current_dist;
        maneuver_msg.lane_change_maneuver.start_speed = current_speed;
        maneuver_msg.lane_change_maneuver.start_time = current_time;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;        
        // Generate a new maneuver ID for the lane change maneuver (not needed for lane following maneuver).
        maneuver_msg.lane_change_maneuver.parameters.maneuver_id = boost::uuids::to_string(boost::uuids::random_generator()());
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) 
        {
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(config_.time_step);
        } 
        else 
        {
            // maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(config_.time_step+controler_look_ahead_distance_/target_speed);

        }
        // need both start laneID and end laneID  for lane change
        maneuver_msg.lane_change_maneuver.starting_lane_id = { std::to_string(starting_lane_id) };
        maneuver_msg.lane_change_maneuver.ending_lane_id = { std::to_string(ending_lane_id) };

        current_time = maneuver_msg.lane_change_maneuver.end_time;
        ROS_DEBUG_STREAM(
                    "Creating lane change start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM(
                    "Duration:"<< maneuver_msg.lane_change_maneuver.end_time.toSec() - maneuver_msg.lane_change_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    // update current status based on maneuver 
    void EmergencyVehicleStrategicPlugin::updateCurrentStatus(cav_msgs::Maneuver maneuver, 
                                                               double& speed, 
                                                               double& current_progress, 
                                                               int& lane_id)
    {
        if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
            current_progress =  maneuver.lane_following_maneuver.end_dist;
            if (maneuver.lane_following_maneuver.lane_ids.empty()) 
            {
                ROS_DEBUG_STREAM("Lane id of lane following maneuver not set. Using 0");
                lane_id = 0;
            } 
            else 
            {
                lane_id =  stoi(maneuver.lane_following_maneuver.lane_ids[0]);
            }
        }
    }

    void EmergencyVehicleStrategicPlugin::check_lane_change_option(int lane_change_intention,
																	lanelet::ConstLanelet starting_lanelet, 
                                                                  int& lane_change_option, 
                                                                  lanelet::ConstLanelet& left_target_lanelet,
                                                                  lanelet::ConstLanelet& right_target_lanelet,
                                                                  int& target_lanelet_id) 
    {
        if (!route_msg_.route_path_lanelet_ids.empty())
        {
            int ite=0;
            for(auto id : route_msg_.route_path_lanelet_ids)
            {
                // find lanelet relation
                auto llt = wm_->getMap()->laneletLayer.get(id);
                auto relation = wm_->getMapRoutingGraph()->routingRelation(starting_lanelet, llt);
                ite++;

                if (lane_change_intention==2)
                {
	                // if right lane exist
	                if (relation == lanelet::routing::RelationType::Right) //currently, we only consider that there are two lanes!
	                {
	                    // select target lanelet
	                    target_lanelet_id = id;
	                    ROS_DEBUG_STREAM("Right lane change possible, right side target_lanelet_id: " << id);
	                    lane_change_option=2;
	                    right_target_lanelet = llt;
	                    break;
	                }                
            	}else if (lane_change_intention==1)
            	{
	                if(relation == lanelet::routing::RelationType::Left)
	                {
	                    // select target lanelet
	                    target_lanelet_id = id;
	                    ROS_DEBUG_STREAM("Left lane change possible, left side target_lanelet_id: " << id);
	                    lane_change_option=1;
	                    left_target_lanelet = llt;
	                    break;
	                }
            	}
                else
                {
                    lane_change_option=0;
                }
                // if loop ends with no right lane, mark lane change impossible and continue lane following
                if (ite == route_msg_.route_path_lanelet_ids.size())
                {
                    ROS_DEBUG_STREAM("No lane change lanelet found, mark lane change impossible...");
                    lane_change_option=0; 
                }
            }
        }
        else
        {
            lane_change_option=4;
            ROS_DEBUG_STREAM("Route has not been loaded yet EmergencyVehicleStrategicPlugin");
        }
    }

    void EmergencyVehicleStrategicPlugin::prepass_decision_cb(const std_msgs::BoolConstPtr& msg)
    {        
        pull_in_flag_=msg->data;
    }

    void EmergencyVehicleStrategicPlugin::right_turn_signal()
    {
        autoware_msgs::LampCmd msg;
        msg.header.stamp=ros::Time::now();
        msg.l=0;
        msg.r=1;
        turn_signal_publisher_(msg);
    }

    void EmergencyVehicleStrategicPlugin::left_turn_signal()
    {
        autoware_msgs::LampCmd msg;
        msg.header.stamp=ros::Time::now();
        msg.l=1;
        msg.r=0;
        turn_signal_publisher_(msg);
    }

    void EmergencyVehicleStrategicPlugin::no_turn_signal()
    {
        autoware_msgs::LampCmd msg;
        msg.header.stamp=ros::Time::now();
        msg.l=0;
        msg.r=0;
        turn_signal_publisher_(msg);
    }

    // -------------------------------- Generate maneuver plan --------------------------------
    bool EmergencyVehicleStrategicPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, 
                                                            cav_srvs::PlanManeuversResponse &resp)
    {
        // current position and lanelets
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10); 

        // raise warn if no path was found
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }

        // locate lanelet on shortest path
        auto shortest_path = wm_->getRoute()->shortestPath(); // find path among route
        lanelet::ConstLanelet target_lanelet; 

        // lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d())) 
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path); // usage: findLaneletIndexFromPath(target_id, lanelet2_path)
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    break;
                }
            }
        }

        if (current_lanelets.empty())
        {
            ROS_DEBUG_STREAM("The current laneletssssss are empty. So no lane change!");
        } 
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;


        // read status data
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double current_downtrack=current_progress;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();
        
        // generate target data
        double target_speed;  
        target_speed = findSpeedLimit(current_lanelet);   //get Speed Limit
        double total_maneuver_length = current_progress + config_.time_step * target_speed;
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);
        
        ROS_DEBUG_STREAM("Starting Loop");

        // Note: Use current_lanelet list (which was determined based on vehicle pose) to find current lanelet ID. 
        long current_lanelet_id = current_lanelets[0].second.id();

        // rear emergency vehicle detected: right lane change maneuver
        if (rear_emergency_vehicle_detected_ || left_lane_change_finished_)
        {   
            ROS_DEBUG_STREAM("Start to handle rear emergence vehicle detected case");
            // init right lane indicator 
            lanelet::BasicPoint2d target_lane_center_loc;
            
            ROS_DEBUG_STREAM("Planning lane change maneuver with right lane!");          
            
            // send out lane change plan
            while (current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Lane Change Maneuver ! ");

                // lane change maneuver plan variables 
                double end_dist = total_maneuver_length;
                double dist_diff = end_dist - current_progress;
                double lc_end_dist = total_maneuver_length;

                // find target lanelet for lane change
                lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(current_lanelet_id);
                lanelet::ConstLanelet target_lanelet;
                bool lanechangePossible=false;
                int target_lanelet_id;
                int ite = 0;
                int lane_change_option=0;
                lanelet::ConstLanelet left_target_lanelet,right_target_lanelet;
                int lane_change_intention=2;
                check_lane_change_option(lane_change_intention,starting_lanelet,lane_change_option,left_target_lanelet,right_target_lanelet,target_lanelet_id);

                ROS_DEBUG_STREAM("found target_lanelet_id: " << target_lanelet_id);


                if (lane_change_option==2)
                {
                    target_lanelet=right_target_lanelet;
                    // check if the lanelet is enough to do the lane change
                    target_lane_center_loc = target_lanelet.centerline2d().back();
                    double target_downtrack = wm_->routeTrackPos(target_lane_center_loc).downtrack;
                   // look for connecting lanelet if the current right lanelet is about to end
                    while(target_downtrack<lc_end_dist)
                    {
                        if (!wm_->getMapRoutingGraph()->following(target_lanelet, false).empty())
                        {
                            target_lanelet = wm_->getMapRoutingGraph()->following(target_lanelet, false).front();
                            target_lane_center_loc = target_lanelet.centerline2d().back();
                            target_downtrack=wm_->routeTrackPos(target_lane_center_loc).downtrack;
                            target_lanelet_id=target_lanelet.id();
                        }else
                        {
                            ROS_DEBUG_STREAM(" Can not find target lanelet with enough downtrack! ");
                            lanechangePossible=false;
                            break;
                        }
                    }
                    lanechangePossible = true;
                }        

                // send plan 
                if (lanechangePossible && (!right_lane_change_sent_))
                {
                    ROS_DEBUG_STREAM("Right lane change possible, planning it.. " );
                    cav_msgs::Maneuver maneuver_msg_=composeLaneChangeManeuverMessage(current_progress, lc_end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, target_lanelet_id , time_progress);
                    resp.new_plan.maneuvers.push_back(maneuver_msg_);
                    resp.new_plan.planning_completion_time=maneuver_msg_.lane_change_maneuver.end_time;
                    right_lane_change_sent_=true;
                    lane_change_maneuverplan_=resp.new_plan;
                    ROS_DEBUG_STREAM("Saved lane change maneuver plan and waiting to be finished ");
                    right_lane_change_finished_downtrack_=total_maneuver_length+controler_look_ahead_distance_;
                    turn_signal_option=2;
                }

                else if (right_lane_change_sent_  &&  (!right_lane_change_finished_))
                {
                    resp.new_plan=lane_change_maneuverplan_;
                    ROS_DEBUG_STREAM("Repeat lane change maneuver plan. Awaiting lane change finished ");
                    turn_signal_option=2;
                }
                else
                {
                    ROS_DEBUG_STREAM("Right lane change finished and reset right_lane_change_sent_ as false");
                    right_lane_change_sent_=false;
                    // use reduced target speed after lane change
                    target_speed = target_speed*config_.lane_change_speed_adjustment;   //get Speed Limit
                    total_maneuver_length = current_progress + config_.time_step * target_speed;
                    total_maneuver_length = std::min(total_maneuver_length, route_length);
                    double end_dist = total_maneuver_length*2;
                    // mark lane change finish position
                    if (right_lane_change_finish_dtd_ == 0.0) {right_lane_change_finish_dtd_ = current_downtrack_;}

                    // log maneuver data
                    ROS_DEBUG_STREAM("Same Lane Maneuver after lane change (right lane not available)! ");

                    // calculate distance increment
                    double dist_diff = end_dist - current_progress;

                    ROS_DEBUG_STREAM("Lane change impossible, planning lane following instead ... " );
                    cav_msgs::Maneuver maneuver_msg_=composeManeuverMessage(current_progress, end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, time_progress);
                    resp.new_plan.maneuvers.push_back(maneuver_msg_);
                    resp.new_plan.planning_completion_time=maneuver_msg_.lane_following_maneuver.end_time;

                    turn_signal_option=0;

                    // send stop notice once lane following dtd in emergency lane pass threshold (vehicle length)
                    if (current_downtrack_ - right_lane_change_finish_dtd_ >= config_.vehicle_length)
                    {
                        // publish stop notice 
                        std_msgs::Bool stop_v_msg;
                        stop_v_msg.data = true;
                        stop_vehicle_publisher_(stop_v_msg);
                        ROS_DEBUG_STREAM("Publish notice to stop vehicle... ");
                    }
                } // send plan (lane change vs in lane cruising)

                if (current_downtrack_>right_lane_change_finished_downtrack_)
                {
                    right_lane_change_finished_=true;
                    ROS_DEBUG_STREAM("Right lane change has finished and set right_lane_change_finished_ as true!");
                }

                // update lane change variables 
                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_change_maneuver.end_time;
                speed_progress = target_speed;

                if(current_progress >= total_maneuver_length)
                {
                    break;
                }
            }// while loop send out lane change plan

            // Note: lane change finished was determined by checking right lane existance. 
        
        } // rear emergency vehicle detected


        else if(front_emergency_vehicle_detected_)
        {
            ROS_DEBUG_STREAM("Start to handle front emergence vehicle detected case");
            // init right lane indicator 
            lanelet::BasicPoint2d target_lane_center_loc;
            
            ROS_DEBUG_STREAM("Planning lane change maneuver with left lane!");
            
            // send out lane change plan
            while (current_progress < total_maneuver_length)
            { 
                ROS_DEBUG_STREAM("Lane Change Maneuver ! ");

                // lane change maneuver plan variables 
                double end_dist = total_maneuver_length;
                double dist_diff = end_dist - current_progress;
                double lc_end_dist = total_maneuver_length;

                // find target lanelet for lane change
                lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(current_lanelet_id);
                lanelet::ConstLanelet target_lanelet;
                bool lanechangePossible=false;
                int target_lanelet_id;
                int ite = 0;
                int lane_change_option=0;
                lanelet::ConstLanelet left_target_lanelet,right_target_lanelet;
                int lane_change_intention=1;
                check_lane_change_option(lane_change_intention,starting_lanelet,lane_change_option,left_target_lanelet,right_target_lanelet,target_lanelet_id);

                if (lane_change_option==1)
                { 
                    target_lanelet=left_target_lanelet;
                    // check if the lanelet is enough to do the lane change
                    target_lane_center_loc = target_lanelet.centerline2d().back();
                    double target_downtrack = wm_->routeTrackPos(target_lane_center_loc).downtrack;
                    // look for connecting lanelet if the current right lanelet is about to end
                    while(target_downtrack<lc_end_dist)
                    {
                        if (!wm_->getMapRoutingGraph()->following(target_lanelet, false).empty())
                        {
                            target_lanelet = wm_->getMapRoutingGraph()->following(target_lanelet, false).front();
                            target_lane_center_loc = target_lanelet.centerline2d().back();
                            target_downtrack=wm_->routeTrackPos(target_lane_center_loc).downtrack;
                            target_lanelet_id=target_lanelet.id();
                        }else
                        {
                            ROS_DEBUG_STREAM(" Can not find target lanelet with enough downtrack! ");
                            lanechangePossible=false;
                            break;
                        }
                    }
                    lanechangePossible = true;
                }
                // send plan 
                if (lanechangePossible && (!left_lane_change_sent_))
                {
                    ROS_DEBUG_STREAM("Left lane change possible, planning it.. " );
                    cav_msgs::Maneuver maneuver_msg_=composeLaneChangeManeuverMessage(current_progress, lc_end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, target_lanelet_id , time_progress);
                    resp.new_plan.maneuvers.push_back(maneuver_msg_);
                    resp.new_plan.planning_completion_time=maneuver_msg_.lane_change_maneuver.end_time;
                    left_lane_change_sent_=true;
                    lane_change_maneuverplan_=resp.new_plan;
                    left_lane_change_finished_downtrack_=total_maneuver_length+controler_look_ahead_distance_;

                    turn_signal_option=1;
                }
                else if (left_lane_change_sent_  &&  (!left_lane_change_finished_))
                {
                    resp.new_plan=lane_change_maneuverplan_;
                    ROS_DEBUG_STREAM("Repeat left lane change maneuver plan. Awaiting left lane change finished ");
                    turn_signal_option=1;
                }
                
                else
                {
                    ROS_DEBUG_STREAM("Left lane change finished and reset left_lane_change_sent_ as false");
                    left_lane_change_sent_=false;
                    // use reduced target speed after lane change
                    target_speed = target_speed*config_.lane_change_speed_adjustment;   //get Speed Limit
                    total_maneuver_length = current_progress + config_.time_step * target_speed;
                    total_maneuver_length = std::min(total_maneuver_length, route_length);
                    double end_dist = total_maneuver_length*2;
                    // mark lane change finish position
                    if (left_lane_change_finish_dtd_ == 0.0) {left_lane_change_finish_dtd_ = current_downtrack_;}

                    // log maneuver data
                    ROS_DEBUG_STREAM("Same Lane Maneuver after lane change (right lane not available)! ");

                    // calculate distance increment
                    double dist_diff = end_dist - current_progress;

                    ROS_DEBUG_STREAM("Lane change impossible, planning lane following instead ... " );
                    cav_msgs::Maneuver maneuver_msg_=composeManeuverMessage(current_progress, end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, time_progress);
                    resp.new_plan.maneuvers.push_back(maneuver_msg_);
                    resp.new_plan.planning_completion_time=maneuver_msg_.lane_following_maneuver.end_time;

                    turn_signal_option=0;
                    no_turn_signal();

                } // send plan (lane change vs in lane cruising)

                if (current_downtrack_>left_lane_change_finished_downtrack_+config_.left_path_safety_distance)
                {
                    left_lane_change_finished_=true;
                    ROS_DEBUG_STREAM("Left lane change has finished and set left_lane_change_finished_ as true!");
                }

                // update lane change variables 
                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_change_maneuver.end_time;
                speed_progress = target_speed;
                if(current_progress >= total_maneuver_length)
                {
                    break;
                }

                ++last_lanelet_index;
            }// while loop send out lane change plan

        }// front emergency vehicle detected
        
        // emergency vehicle not detected: same-lane maneuver  
        else 
        {
            ROS_DEBUG_STREAM("Planning Same Lane Maneuver! ");
            while (current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Same Lane Maneuver before emergency vehicle detection! ");
                double end_dist = total_maneuver_length;
                double dist_diff = end_dist - current_progress;
                if (end_dist < current_progress)
                {
                    break;
                }

                cav_msgs::Maneuver maneuver_msg_=composeManeuverMessage(current_progress, end_dist,  
                                            speed_progress, target_speed, current_lanelet_id, time_progress);
                resp.new_plan.maneuvers.push_back(maneuver_msg_);
                resp.new_plan.planning_completion_time=maneuver_msg_.lane_following_maneuver.end_time;
                                    
                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                speed_progress = target_speed;

                turn_signal_option=0;
                no_turn_signal();

                if(current_progress >= total_maneuver_length)
                {
                    break;
                }
                ++last_lanelet_index;
            }
        } //emergency vehicle not detected

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }  
        
        return true;
    }

    // -------------------------------- Spin Emergency Pullover plugin --------------------------------
    bool EmergencyVehicleStrategicPlugin::onSpin() 
    {  
        plugin_discovery_publisher_(plugin_discovery_msg_);      
        if (turn_signal_option==0)
        {
            no_turn_signal();
        }else if (turn_signal_option==1)
        {
            left_turn_signal();
        }else if (turn_signal_option==2)
        {
            right_turn_signal();
        }

        if (!emergency_vehicle_detected_)
        {
        }
        else
        {
            if(is_pull_over_initiated)
            {
            }
            else
            {
            }
        }

        return true;
    }
    
} //emergency_vehicle_strategic