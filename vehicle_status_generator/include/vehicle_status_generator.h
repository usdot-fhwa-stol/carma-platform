#pragma once

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

#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/MobilityOperation.h>
#include <automotive_platform_msgs/VelocityAccelCov.h>
#include <pacmod_msgs/YawRateRpt.h>
#include <j2735_msgs/TransmissionState.h>
#include <std_msgs/Float64.h>
#include <wgs84_utils/wgs84_utils.h>
#include <novatel_gps_msgs/NovatelDualAntennaHeading.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include "vehicle_status_generator_worker.h"

#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>

namespace vehicle_status_generator
{

    class VehicleStatusGenerator
    {

    public:

        VehicleStatusGenerator();

        // general starting point of this node
        void run();

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        double vehicle_deceleration_limit = 3.0;

        // worker class
        VehicleStatusGeneratorWorker worker;

        // publisher for generated BSMs
        ros::Publisher  vehicle_status_pub;

        // subscribers for gathering data from the platform
        ros::Subscriber speed_sub_;

        // frequency for vehicle_status generation
        double vehicle_status_generation_frequency_;

        // size of the vehicle
        double vehicle_length_, vehicle_width_;

        // acceleration limit
        double vehicle_acceleration_limit_;
        
        // minimum safety gap in m
        double x_gap = 2.0;

        // timer to run the bsm generation task
        ros::Timer timer_;

        // the Mobility Operation object that all subscribers make updates to
        cav_msgs::MobilityOperation mo_;

        double current_speed_;

        // initialize this node
        void initialize();

        // callbacks for the subscribers
        void speedCallback(const std_msgs::Float64ConstPtr& msg);

        // callback for the timer
        void generateMobilityOperation(const ros::TimerEvent& event);
    };

}
