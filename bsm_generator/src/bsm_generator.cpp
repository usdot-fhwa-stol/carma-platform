/*
 * Copyright (C) 2019 LEIDOS.
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


#include "bsm_generator.h"

namespace bsm_generator
{
    BSMGenerator::BSMGenerator() : bsm_generation_frequency_(10.0) {}

    void BSMGenerator::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("bsm_generation_frequency", bsm_generation_frequency_, 10.0);
        bsm_pub_ = nh_->advertise<cav_msgs::BSM>("bsm_outbound", 5);
        timer_ = nh_->createTimer(ros::Duration(1.0 / bsm_generation_frequency_), &BSMGenerator::generateBSM, this);
        gear_sub_ = nh_->subscribe("transmission_state", 1, &BSMGenerator::gearCallback, this);
        speed_sub_ = nh_->subscribe("vehicle_speed", 1, &BSMGenerator::speedCallback, this);
        steer_wheel_angle_sub_ = nh_->subscribe("steering_wheel_angle", 1, &BSMGenerator::steerWheelAngleCallback, this);

        // waypoints_sub_ = nh_->subscribe("final_waypoints", 1, &AutowarePlugin::waypoints_cb, this);
        //pose_sub_ = nh_->subscribe("current_pose", 1, &BSMGenerator::pose_cb, this);
        // twist_sub_ = nh_->subscribe("current_velocity", 1, &AutowarePlugin::twist_cd, this);
        
    }

    void BSMGenerator::initializeBSM()
    {
        bsm_.core_data.presence_vector = 0;
        bsm_.core_data.transmission.transmission_state = bsm_.core_data.transmission.UNAVAILABLE;
    }

    void BSMGenerator::run()
    {
        initializeBSM();
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void BSMGenerator::speedCallback(const std_msgs::Float64ConstPtr& msg)
    {
        bsm_.core_data.speed = worker.getSpeedInRange(msg->data);
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SPEED_AVAILABLE;
    }

    void BSMGenerator::gearCallback(const j2735_msgs::TransmissionStateConstPtr& msg)
    {
        bsm_.core_data.transmission.transmission_state = msg->transmission_state;
    }

    void BSMGenerator::steerWheelAngleCallback(const std_msgs::Float64ConstPtr& msg)
    {
        
    }

    void BSMGenerator::generateBSM(const ros::TimerEvent& event)
    {
        bsm_.header.stamp = ros::Time::now();
        bsm_.core_data.msg_count = worker.getNextMsgCount();
        bsm_.core_data.id = worker.getMsgId(ros::Time::now());
        bsm_.core_data.sec_mark = worker.getSecMark(ros::Time::now());
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SEC_MARK_AVAILABLE;
        // currently the accuracy is not available because ndt_matching does not provide accuracy measurement
        bsm_.core_data.accuracy.presence_vector = 0;

        bsm_pub_.publish(bsm_);
    }
}
