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

/*
 * Developed by the JFL Solutions LLC.
 * Author: Fang Zhou
 */

#pragma once

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/format.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanManeuvers.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>

#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include "external_object_list_publisher_config.h"

#include <cav_msgs/PlanType.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <carma_wm/TrafficControl.h>


namespace external_object_list_publisher
{
    using ExternalObjectListCB = std::function<void(const cav_msgs::ExternalObjectList&)>;

    class ExternalObjectListPublisher
    {
        public: 

            /**
            * \brief Constructor
            * 
            * \param wm Pointer to initalized instance of the carma world model for accessing semantic map data
            * \param config The configuration to be used for this object
            */ 
            ExternalObjectListPublisher(ExternalObjectListPublisherConfig config, 
                                        ExternalObjectListCB external_object_list_publisher);


            void emergncy_detection_cb(const std_msgs::BoolConstPtr& msg);

            /**
            * \brief Generate and publish the sythetic message for emergency vehicles
            */
            void publish_external_object_list(bool front_or_rear);

            /**
            * \brief Spin callback function
            */
            bool onSpin();
            
            // public global variable

            // ECEF position of the host vehicle
            cav_msgs::LocationECEF pose_ecef_point_;

            // Speed below which platooning will not be attempted; non-zero value allows for sensor noise
            const double STOPPED_SPEED = 0.5; // m/s

        private: 

            // private global variables 

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;
            
            // publisher varaible 
            ExternalObjectListCB external_object_list_publisher_; 

            // local copy of configuration file
            ExternalObjectListPublisherConfig config_;

            // pointer for enabling lanechange (send emergency vehicle detection result, default is false)
            bool is_emergency_vehicle_detected_ = false; 

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            // Current vehicle downtrack distance in route, m
            double current_downtrack_ = 0;

            // Current vehicle crosstrack distance in route, m
            double current_crosstrack_ = 0;

            // Current vehicle measured speed, m/s
            double current_speed_ = 0;
    };
}
