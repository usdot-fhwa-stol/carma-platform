#pragma once

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

/*
 * Developed by the UCLA Mobility Lab, 10/20/2021. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
 */

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/PlatooningInfo.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include "platoon_strategic_ihp.h"
#include "platoon_config_ihp.h"

namespace platoon_strategic_ihp
{
/**
 * \brief ROS node for the YieldPlugin
 */ 
class PlatoonStrategicIHPPluginNode
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
    ros::Publisher mob_response_pub = nh.advertise<cav_msgs::MobilityResponse>("outgoing_mobility_response", 1);
    ros::Publisher mob_request_pub = nh.advertise<cav_msgs::MobilityRequest>("outgoing_mobility_request", 1);
    ros::Publisher mob_operation_pub = nh.advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 1);
    ros::Publisher platoon_info_pub = nh.advertise<cav_msgs::PlatooningInfo>("platoon_info", 1);


    PlatoonPluginConfig config;

    pnh.param<int>("maxPlatoonSize", config.maxPlatoonSize, config.maxPlatoonSize);
    pnh.param<int>("algorithmType", config.algorithmType, config.algorithmType);
    pnh.param<int>("statusMessageInterval", config.statusMessageInterval, config.statusMessageInterval);
    pnh.param<int>("infoMessageInterval", config.infoMessageInterval, config.infoMessageInterval);
    pnh.param<double>("timeHeadway", config.timeHeadway, config.timeHeadway);
    pnh.param<double>("standStillHeadway", config.standStillHeadway, config.standStillHeadway);
    pnh.param<double>("maxAllowedJoinTimeGap", config.maxAllowedJoinTimeGap, config.maxAllowedJoinTimeGap);
    pnh.param<double>("maxAllowedJoinGap", config.maxAllowedJoinGap, config.maxAllowedJoinGap);
    pnh.param<double>("minAllowedJoinGap", config.minAllowedJoinGap, config.minAllowedJoinGap);
    pnh.param<double>("desiredJoinTimeGap", config.desiredJoinTimeGap, config.desiredJoinTimeGap);
    pnh.param<double>("desiredJoinGap", config.desiredJoinGap, config.desiredJoinGap);
    pnh.param<double>("waitingStateTimeout", config.waitingStateTimeout, config.waitingStateTimeout);
    pnh.param<double>("cmdSpeedMaxAdjustment", config.cmdSpeedMaxAdjustment, config.cmdSpeedMaxAdjustment);
    pnh.param<double>("minAllowableHeadaway", config.minAllowableHeadaway, config.minAllowableHeadaway);
    pnh.param<double>("maxAllowableHeadaway", config.maxAllowableHeadaway, config.maxAllowableHeadaway);
    pnh.param<double>("headawayStableLowerBond", config.headawayStableLowerBond, config.headawayStableLowerBond);
    pnh.param<double>("headawayStableUpperBond", config.headawayStableUpperBond, config.headawayStableUpperBond);
    pnh.param<double>("minCutinGap", config.minCutinGap, config.minCutinGap);
    pnh.param<double>("maxCutinGap", config.maxCutinGap, config.maxCutinGap);
    pnh.param<double>("maxCrosstrackError", config.maxCrosstrackError, config.maxCrosstrackError);
    pnh.param<bool>("test_front_join", config.test_front_join, config.test_front_join);
    pnh.param<double>("minPlatooningSpeed", config.minPlatooningSpeed, config.minPlatooningSpeed);
    pnh.param<bool>("allowCutinJoin", config.allowCutinJoin, config.allowCutinJoin);
    pnh.param<double>("significantDTDchange", config.significantDTDchange, config.significantDTDchange);
    pnh.param<double>("longitudinalCheckThresold", config.longitudinalCheckThresold, config.longitudinalCheckThresold);
    pnh.param<bool>("test_cutin_join", config.test_cutin_join, config.test_cutin_join);
    pnh.param<int>("join_index", config.join_index, config.join_index);
    pnh.getParam("/vehicle_length", config.vehicleLength);
    pnh.getParam("/vehicle_id", config.vehicleID);
    
    ROS_INFO_STREAM("PlatoonPluginConfig Params" << config);

    PlatoonStrategicIHPPlugin worker(wm_, config, [&discovery_pub](auto msg) { discovery_pub.publish(msg); }, [&mob_response_pub](auto msg) { mob_response_pub.publish(msg); },
                                    [&mob_request_pub](auto msg) { mob_request_pub.publish(msg); }, [&mob_operation_pub](auto msg) { mob_operation_pub.publish(msg); },
                                    [&platoon_info_pub](auto msg) { platoon_info_pub.publish(msg); } );
  
    ros::ServiceServer maneuver_srv_ = nh.advertiseService("plugins/PlatooningStrategicIHPPlugin/plan_maneuvers",
                                            &PlatoonStrategicIHPPlugin::plan_maneuver_cb, &worker);
    ros::Subscriber mob_request_sub = nh.subscribe("incoming_mobility_request", 1, &PlatoonStrategicIHPPlugin::mob_req_cb,  &worker);
    ros::Subscriber mob_response_sub = nh.subscribe("incoming_mobility_response", 1, &PlatoonStrategicIHPPlugin::mob_resp_cb,  &worker);
    ros::Subscriber mob_operation_sub = nh.subscribe("incoming_mobility_operation", 1, &PlatoonStrategicIHPPlugin::mob_op_cb,  &worker);
    ros::Subscriber current_pose_sub = nh.subscribe("current_pose", 1, &PlatoonStrategicIHPPlugin::pose_cb,  &worker);
    ros::Subscriber current_twist_sub = nh.subscribe("current_velocity", 1, &PlatoonStrategicIHPPlugin::twist_cb,  &worker);
    // not use BSMID, consider delete
    // ros::Subscriber bsm_sub = nh.subscribe("bsm_outbound", 1, &PlatoonStrategicIHPPlugin::bsm_cb,  &worker);
    ros::Subscriber cmd_sub = nh.subscribe("twist_raw", 1, &PlatoonStrategicIHPPlugin::cmd_cb,  &worker);
    ros::Subscriber georeference_sub = nh.subscribe("georeference", 1, &PlatoonStrategicIHPPlugin::georeference_cb, &worker);

    
    ros::Timer discovery_pub_timer_ = nh.createTimer(
            ros::Duration(ros::Rate(10.0)),
            [&worker](const auto&) { worker.onSpin(); });

    ros::CARMANodeHandle::spin();
  }
};

}  // namespace platoon_strategicIHP
