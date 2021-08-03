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
#include "wz_strategic_plugin/wz_strategic_plugin.h"
#include "wz_strategic_plugin/wz_strategic_plugin_config.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wz_strategic_plugin");

  ros::CARMANodeHandle nh;
  ros::CARMANodeHandle pnh("~");

  ros::Publisher plugin_discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);

  carma_wm::WMListener wml;

  WzStrategicPluginConfig config;

  // clang-format off
  pnh.param<double>("/vehicle_deceleration_limit",      config.vehicle_decel_limit, config.vehicle_decel_limit);
  pnh.param<double>("vehicle_decel_limit_multiplier",   config.vehicle_decel_limit_multiplier, config.vehicle_decel_limit_multiplier);
  pnh.param<double>("min_approach_distance",            config.min_approach_distance, config.min_approach_distance);
  pnh.param<double>("stopping_location_buffer",         config.stopping_location_buffer, config.stopping_location_buffer);
  pnh.param<double>("green_light_time_buffer",          config.green_light_time_buffer, config.green_light_time_buffer);
  pnh.param<double>("min_maneuver_planning_period",     config.min_maneuver_planning_period, config.min_maneuver_planning_period);
  pnh.param<double>("strategic_plugin_name",            config.strategic_plugin_name, config.strategic_plugin_name);
  pnh.param<double>("lane_following_plugin_name",       config.lane_following_plugin_name, config.lane_following_plugin_name);
  pnh.param<double>("stop_and_wait_plugin_name",        config.stop_and_wait_plugin_name, config.stop_and_wait_plugin_name);
  pnh.param<double>("intersection_transit_plugin_name", config.intersection_transit_plugin_name, config.intersection_transit_plugin_name);
  // clang-format on

  wz_strategic_plugin::WzStrategicPlugin wzp(
      wml.getWorldModel(), config, [&plugin_discovery_pub](const auto& msg) { plugin_discovery_pub.publish(msg); });

  ros::Service plan_maneuver_srv =
      nh.advertiseService("plugins/WzStrategic/plan_maneuvers", &WzStrategicPlugin::planManeuverCb, &wzp);

  ros::Timer discovery_pub_timer =
      nh.createTimer(ros::Duration(ros::Rate(10.0)), [&wzp, &plugin_discovery_pub](const auto&) {
        plugin_discovery_pub.publish(wzp.getDiscoveryMsg());
      });

  ros::CARMANodeHandle::spin();
  return 0;
};