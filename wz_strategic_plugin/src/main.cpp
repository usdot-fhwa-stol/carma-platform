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
#include "wz_strategic_plugin/wz_strategic_plugin.h"
#include "wz_strategic_plugin/wz_strategic_plugin_config.h"

int main(int argc, char** argv)
{
  // Initialize ros connection
  ros::init(argc, argv, "wz_strategic_plugin");

  // Setup node handles
  ros::CARMANodeHandle nh;
  ros::CARMANodeHandle pnh("~");

  // Create publishers
  ros::Publisher plugin_discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);

  // Initialize world model
  carma_wm::WMListener wml;

  // Load Parameters
  wz_strategic_plugin::WzStrategicPluginConfig config;

  // clang-format off
  pnh.param<double>("/vehicle_acceleration_limit",      config.vehicle_decel_limit, config.vehicle_decel_limit);
  pnh.param<double>("vehicle_decel_limit_multiplier",   config.vehicle_decel_limit_multiplier, config.vehicle_decel_limit_multiplier);
  pnh.param<double>("min_approach_distance",            config.min_approach_distance, config.min_approach_distance);
  pnh.param<double>("stopping_location_buffer",         config.stopping_location_buffer, config.stopping_location_buffer);
  pnh.param<double>("green_light_time_buffer",          config.green_light_time_buffer, config.green_light_time_buffer);
  pnh.param<double>("min_maneuver_planning_period",     config.min_maneuver_planning_period, config.min_maneuver_planning_period);
  pnh.param<std::string>("strategic_plugin_name",            config.strategic_plugin_name, config.strategic_plugin_name);
  pnh.param<std::string>("lane_following_plugin_name",       config.lane_following_plugin_name, config.lane_following_plugin_name);
  pnh.param<std::string>("stop_and_wait_plugin_name",        config.stop_and_wait_plugin_name, config.stop_and_wait_plugin_name);
  pnh.param<std::string>("intersection_transit_plugin_name", config.intersection_transit_plugin_name, config.intersection_transit_plugin_name);
  // pnh.param<double>("/vehicle_length", config.vehicle_length, config.vehicle_length);
  // clang-format on

  // Construct plugin
  wz_strategic_plugin::WzStrategicPlugin wzp(wml.getWorldModel(), config);

  // Setup callback connections
  ros::ServiceServer plan_maneuver_srv =
      nh.advertiseService("plugins/" + config.strategic_plugin_name + "/plan_maneuvers", &wz_strategic_plugin::WzStrategicPlugin::planManeuverCb, &wzp);

  ros::Timer discovery_pub_timer =
      nh.createTimer(ros::Duration(ros::Rate(10.0)), [&wzp, &plugin_discovery_pub](const auto&) {
        plugin_discovery_pub.publish(wzp.getDiscoveryMsg());
      });

  // Start
  ros::CARMANodeHandle::spin();

  return 0;
};