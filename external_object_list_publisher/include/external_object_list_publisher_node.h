/*
 * Copyright (C) 2021-2022 LEIDOS.
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
 * Developed by the JFL Solutions LLC.
 * Author: Fang Zhou
 */

#pragma once

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityOperation.h>

#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/ExternalObject.h>

#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <functional>

#include "external_object_list_publisher.h"
#include "external_object_list_publisher_config.h"


namespace external_object_list_publisher
{

  /**
   * \brief ROS Node to for External Object List Publisher
   */ 
  class ExternalObjectListPublisherNode 
  { 
    // public variables and functions
    public:
      /**
       * \brief Entrypoint for this node
       */
      void run()
      {
        ros::CARMANodeHandle nh;
        ros::CARMANodeHandle pnh("~");

        // external object list publisher involved
        ros::Publisher external_objecets_pub = nh.advertise<cav_msgs::ExternalObjectList>("external_objects", 1);
        // load config file
        ExternalObjectListPublisherConfig config;

        // get parameters
        pnh.param<double>("emergency_vehicle_distance", config.emergency_vehicle_distance, config.emergency_vehicle_distance);
        
        ROS_INFO_STREAM("ExternalObjectListPublisher Params" << config);

        // init worker
        ExternalObjectListPublisher worker(config, [&external_objecets_pub](auto msg) {external_objecets_pub.publish(msg);});
      
        // vehicle state subs
        ros::Subscriber emergncy_detection_manual_sub=nh.subscribe("emergncy_detection_manual",1,&ExternalObjectListPublisher::emergncy_detection_cb, &worker);
       
        ros::Timer discovery_pub_timer_ = nh.createTimer(
                ros::Duration(ros::Rate(10.0)),
                [&worker](const auto&) { worker.onSpin(); });

        ros::CARMANodeHandle::spin();
      }
  };
}
