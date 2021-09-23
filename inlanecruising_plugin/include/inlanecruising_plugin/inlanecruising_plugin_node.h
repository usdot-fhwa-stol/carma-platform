#pragma once

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

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <autoware_msgs/Lane.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include "inlanecruising_plugin.h"
#include "inlanecruising_config.h"

namespace inlanecruising_plugin
{
/**
 * \brief ROS node for the InLaneCruisingPlugin
 */ 
class InLaneCruisingPluginNode
{
public:

  /**
   * \brief Entrypoint for this node
   */ 
  void run()
  {
    ros::CARMANodeHandle nh;
    ros::CARMANodeHandle pnh("~");

    carma_wm::WMListener wml;
    auto wm_ = wml.getWorldModel();

    ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
    ros::Publisher trajectory_debug_pub = pnh.advertise<carma_debug_msgs::TrajectoryCurvatureSpeeds>("debug/trajectory_planning", 1);
    
    InLaneCruisingPluginConfig config;

    pnh.param<double>("trajectory_time_length", config.trajectory_time_length, config.trajectory_time_length);
    pnh.param<double>("curve_resample_step_size", config.curve_resample_step_size, config.curve_resample_step_size);
    pnh.param<int>("default_downsample_ratio", config.default_downsample_ratio, config.default_downsample_ratio);
    pnh.param<int>("turn_downsample_ratio", config.turn_downsample_ratio, config.turn_downsample_ratio);
    pnh.param<double>("minimum_speed", config.minimum_speed, config.minimum_speed);
    pnh.param<double>("max_accel_multiplier", config.max_accel_multiplier, config.max_accel_multiplier);
    pnh.param<double>("lat_accel_multiplier", config.lat_accel_multiplier, config.lat_accel_multiplier);
    pnh.param<double>("back_distance", config.back_distance, config.back_distance);
    pnh.param<int>("speed_moving_average_window_size", config.speed_moving_average_window_size,
                     config.speed_moving_average_window_size);
    pnh.param<int>("curvature_moving_average_window_size", config.curvature_moving_average_window_size,
                     config.curvature_moving_average_window_size);
    pnh.param<double>("/vehicle_acceleration_limit", config.max_accel, config.max_accel);
    pnh.param<double>("/vehicle_lateral_accel_limit", config.lateral_accel_limit, config.lateral_accel_limit);
    pnh.param<bool>("enable_object_avoidance", config.enable_object_avoidance, config.enable_object_avoidance);
    pnh.param<double>("buffer_ending_downtrack", config.buffer_ending_downtrack, config.buffer_ending_downtrack);

    ROS_INFO_STREAM("InLaneCruisingPlugin Params" << config);
    
    config.lateral_accel_limit = config.lateral_accel_limit * config.lat_accel_multiplier;
    config.max_accel = config.max_accel *  config.max_accel_multiplier;
    
    // Determine if we will enable debug publishing by checking the current log level of the node
    std::map< std::string, ros::console::levels::Level> logger;
    ros::console::get_loggers(logger);
    config.publish_debug = logger[ROSCONSOLE_DEFAULT_NAME] == ros::console::levels::Debug;

    ROS_INFO_STREAM("InLaneCruisingPlugin Params After Accel Change" << config);
    
    InLaneCruisingPlugin worker(wm_, config, [&discovery_pub](const auto& msg) { discovery_pub.publish(msg); },
                                             [&trajectory_debug_pub](const auto& msg) { trajectory_debug_pub.publish(msg); });

    ros::ServiceServer trajectory_srv_ = nh.advertiseService("plugins/InLaneCruisingPlugin/plan_trajectory",
                                            &InLaneCruisingPlugin::plan_trajectory_cb, &worker);

    //TODO: Update yield client to use the Plugin Manager capabilities query, in case someone else wants to add an alternate yield implementation 
    ros::ServiceClient yield_client = nh.serviceClient<cav_srvs::PlanTrajectory>("plugins/YieldPlugin/plan_trajectory");
    worker.set_yield_client(yield_client);
    ROS_INFO_STREAM("Yield Client Set");

    ros::Timer discovery_pub_timer_ = nh.createTimer(
            ros::Duration(ros::Rate(10.0)),
            [&worker](const auto&) {worker.onSpin();});
    ros::CARMANodeHandle::spin();
  }
};

}  // namespace inlanecruising_plugin