/*
 * Copyright (C) 2021 LEIDOS.
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
#include <carma_utils/CARMAUtils.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/WMListener.h>
#include "sci_strategic_plugin.h"
#include "sci_strategic_plugin_config.h"

int main(int argc, char** argv)
{
  // Initialize ros connection
  ros::init(argc, argv, "sci_strategic_plugin");

  // Setup node handles
  ros::CARMANodeHandle nh;
  ros::CARMANodeHandle pnh("~");

  // Initialize world model
  carma_wm::WMListener wml;

  // Load Parameters
  sci_strategic_plugin::SCIStrategicPluginConfig config;

  // clang-format off
  
  pnh.param<double>("vehicle_decel_limit_multiplier",   config.vehicle_decel_limit_multiplier, config.vehicle_decel_limit_multiplier);
  pnh.param<double>("vehicle_accel_limit_multiplier",   config.vehicle_accel_limit_multiplier, config.vehicle_accel_limit_multiplier);
  pnh.param<double>("stop_line_buffer",   config.stop_line_buffer, config.stop_line_buffer);
  pnh.param<double>("delta_t",   config.delta_t, config.delta_t);
  pnh.param<double>("min_gap",   config.min_gap, config.min_gap);
  pnh.param<double>("reaction_time",   config.reaction_time, config.reaction_time);
  pnh.param<std::string>("strategic_plugin_name",            config.strategic_plugin_name, config.strategic_plugin_name);
  pnh.param<std::string>("lane_following_plugin_name",       config.lane_following_plugin_name, config.lane_following_plugin_name);
  pnh.param<std::string>("intersection_transit_plugin_name", config.intersection_transit_plugin_name, config.intersection_transit_plugin_name);
  pnh.getParam("/vehicle_id", config.vehicle_id);
  pnh.getParam("/vehicle_length", config.veh_length);
  pnh.getParam("/vehicle_deceleration_limit", config.vehicle_decel_limit);
  pnh.getParam("/vehicle_acceleration_limit", config.vehicle_accel_limit);
  // clang-format on

  // Construct plugin
  sci_strategic_plugin::SCIStrategicPlugin plugin(wml.getWorldModel(), config);

  // Setup callback connections
  ros::ServiceServer plan_maneuver_srv =
      nh.advertiseService("plugins/" + config.strategic_plugin_name + "/plan_maneuvers", &sci_strategic_plugin::SCIStrategicPlugin::planManeuverCb, &plugin);
  
  // Mobility Operation Subscriber
  ros::Subscriber mob_operation_sub = nh.subscribe("incoming_mobility_operation", 1, &sci_strategic_plugin::SCIStrategicPlugin::mobilityOperationCb, &plugin);
  
  // Current Pose Subscriber
  ros::Subscriber current_pose_sub = nh.subscribe("current_pose", 1, &sci_strategic_plugin::SCIStrategicPlugin::currentPoseCb, &plugin);

  ros::Subscriber bsm_sub = nh.subscribe("bsm_outbound", 1, &sci_strategic_plugin::SCIStrategicPlugin::BSMCb, &plugin);

  // Create publishers
  ros::Publisher plugin_discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
  plugin.mobility_operation_pub = nh.advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 1);

  ros::Timer pub_timer_ = nh.createTimer(
            ros::Duration(ros::Rate(10.0)),
            [&plugin, &plugin_discovery_pub](const auto&) {
                                    plugin_discovery_pub.publish(plugin.getDiscoveryMsg());
                                    plugin.onSpin();
                                    });
            
 
  ROS_INFO("Successfully launched node.");
  // Start
  ros::CARMANodeHandle::spin();

  return 0;
};
