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
#include <automotive_platform_msgs/VelocityAccelCov.h>
#include <pacmod_msgs/YawRateRpt.h>
#include <sensor_msgs/Imu.h>
#include <j2735_msgs/TransmissionState.h>
#include <std_msgs/Float64.h>
#include <wgs84_utils/wgs84_utils.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include <vector>
#include <ros/ros.h>
#include <stdint.h>
#include <algorithm>
#include <stdlib.h>
#include <random>
#include <gps_common/GPSFix.h>
#include "bsm_generator_worker.h"

namespace bsm_generator
{

    class BSMGenerator
    {

    public:

        BSMGenerator();

        // general starting point of this node
        void run();

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // worker class
        BSMGeneratorWorker worker;

        // publisher for generated BSMs
        ros::Publisher  bsm_pub_;

        // subscribers for gathering data from the platform
        ros::Subscriber pose_sub_;
        ros::Subscriber accel_sub_;
        ros::Subscriber yaw_sub_;
        ros::Subscriber gear_sub_;
        ros::Subscriber speed_sub_;
        ros::Subscriber steer_wheel_angle_sub_;
        ros::Subscriber brake_sub_;
        ros::Subscriber heading_sub_;
        ros::Subscriber georeference_sub_;

        std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

        // frequency for bsm generation
        double bsm_generation_frequency_;

        //Enable/disable rotation of BSM ID during vehicle operations
        bool bsm_id_rotation_enabled_;

        std::vector<uint8_t> bsm_message_id_;
        

        // size of the vehicle
        double vehicle_length_, vehicle_width_;

        // timer to run the bsm generation task
        ros::Timer timer_;

        // the BSM object that all subscribers make updates to
        cav_msgs::BSM bsm_;

        // initialize this node
        void initialize();

        // fill some default data in BSM
        void initializeBSM();

        // callbacks for the subscribers
        void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
        void accelCallback(const automotive_platform_msgs::VelocityAccelCovConstPtr& msg);
        void yawCallback(const sensor_msgs::ImuConstPtr& msg);
        void gearCallback(const j2735_msgs::TransmissionStateConstPtr& msg);
        void speedCallback(const std_msgs::Float64ConstPtr& msg);
        void steerWheelAngleCallback(const std_msgs::Float64ConstPtr& msg);
        void brakeCallback(const std_msgs::Float64ConstPtr& msg);
        void headingCallback(const gps_common::GPSFixConstPtr& msg);

        /**
         * \brief Callback for map projection string to define lat/lon -> map conversion
         * \brief msg The proj string defining the projection.
         */ 
        void georeferenceCallback(const std_msgs::StringConstPtr& msg);

        // callback for the timer
        void generateBSM(const ros::TimerEvent& event);
    };

}