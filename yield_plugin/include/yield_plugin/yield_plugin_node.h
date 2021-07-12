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
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <functional>

#include "yield_plugin.h"
#include "yield_config.h"

namespace yield_plugin
{
/**
 * \brief ROS node for the YieldPlugin
 */ 
class YieldPluginNode
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
    ros::Publisher mob_resp_pub = nh.advertise<cav_msgs::MobilityResponse>("outgoing_mobility_response", 1);
    ros::Publisher lc_status_pub = nh.advertise<cav_msgs::LaneChangeStatus>("cooperative_lane_change_status", 10);

    YieldPluginConfig config;

    pnh.param<double>("acceleration_adjustment_factor", config.acceleration_adjustment_factor, config.acceleration_adjustment_factor);
    pnh.param<double>("collision_horizon", config.collision_horizon, config.collision_horizon);
    pnh.param<double>("min_obstacle_speed", config.min_obstacle_speed, config.min_obstacle_speed);
    pnh.param<double>("tpmin", config.tpmin, config.tpmin);
    pnh.param<double>("yield_max_deceleration", config.yield_max_deceleration, config.yield_max_deceleration);
    pnh.param<double>("x_gap", config.x_gap, config.x_gap);
    pnh.param<double>("max_stop_speed", config.max_stop_speed, config.max_stop_speed);
    pnh.getParam("/vehicle_length", config.vehicle_length);
    pnh.getParam("/vehicle_height", config.vehicle_height);
    pnh.getParam("/vehicle_width", config.vehicle_width);
    pnh.getParam("/vehicle_id", config.vehicle_id);
    pnh.param<bool>("always_accept_mobility_request", config.always_accept_mobility_request, config.always_accept_mobility_request);
    pnh.param<int>("acceptable_passed_timesteps", config.acceptable_passed_timesteps, config.acceptable_passed_timesteps);
    pnh.param<double>("intervehicle_collision_distance", config.intervehicle_collision_distance, config.intervehicle_collision_distance);
    pnh.param<double>("safety_collision_time_gap", config.safety_collision_time_gap, config.safety_collision_time_gap);
    pnh.param<bool>("enable_adjustable_gap", config.enable_adjustable_gap, config.enable_adjustable_gap);
    pnh.param<int>("acceptable_urgency", config.acceptable_urgency, config.acceptable_urgency);
    pnh.param<double>("speed_moving_average_window_size", config.speed_moving_average_window_size, config.speed_moving_average_window_size);
    ROS_INFO_STREAM("YieldPlugin Params" << config);

    YieldPlugin worker(wm_, config, [&discovery_pub](auto msg) { discovery_pub.publish(msg); }, [&mob_resp_pub](auto msg) { mob_resp_pub.publish(msg); },
                                    [&lc_status_pub](auto msg) { lc_status_pub.publish(msg); });
      
    ros::ServiceServer trajectory_srv_ = nh.advertiseService("plugins/YieldPlugin/plan_trajectory",
                                            &YieldPlugin::plan_trajectory_cb, &worker);
    ros::Subscriber mob_request_sub = nh.subscribe("incoming_mobility_request", 5, &YieldPlugin::mobilityrequest_cb,  &worker);
    ros::Subscriber bsm_sub = nh.subscribe("bsm_outbound", 1, &YieldPlugin::bsm_cb,  &worker);
    ros::Subscriber georeference_sub = nh.subscribe("georeference", 1, &YieldPlugin::georeferenceCallback, &worker);
    
    ros::Timer discovery_pub_timer_ = nh.createTimer(
            ros::Duration(ros::Rate(10.0)),
            [&worker](const auto&) { worker.onSpin(); });

    ros::CARMANodeHandle::spin();
    
  }
};

}  // namespace yield_plugin
