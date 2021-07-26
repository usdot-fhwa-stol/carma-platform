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


#include "vehicle_status_generator.h"

namespace vehicle_status_generator
{
    VehicleStatusGenerator::VehicleStatusGenerator() : vehicle_status_generation_frequency_(10.0) {}

    void VehicleStatusGenerator::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        pnh_->param<double>("vehicle_status_generation_frequency_", vehicle_status_generation_frequency_, 10.0);
        nh_->param<double>("vehicle_length", vehicle_length_, 5.0);
        nh_->param<double>("vehicle_width", vehicle_width_, 2.0);

        nh_->param<double>("vehicle_acceleration_limit", vehicle_acceleration_limit_, 2.0);
        nh_->param<double>("yield_max_deceleration", yield_max_deceleration, 2.0);
        nh_->param<double>("x_gap", x_gap, 2.0);

        vehicle_status_pub = nh_->advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 5);
        timer_ = nh_->createTimer(ros::Duration(1.0 / vehicle_status_generation_frequency_), &VehicleStatusGenerator::generateMobilityOperation, this);
        speed_sub_ = nh_->subscribe("vehicle_speed_cov", 1, &VehicleStatusGenerator::speedCallback, this);

    }

    void VehicleStatusGenerator::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void VehicleStatusGenerator::speedCallback(const std_msgs::Float64ConstPtr& msg)
    {
        current_speed_ = msg->data;
    }

    void VehicleStatusGenerator::generateMobilityOperation(const ros::TimerEvent& event)
    {
        mo_.header.timestamp = ros::Time::now().toNSec();
        mo_.header.sender_bsm_id = std::string(reinterpret_cast<const char*>(&worker.getMsgId(ros::Time::now())[0]), worker.getMsgId(ros::Time::now()).size());
        mo_.strategy_params = std::to_string(x_gap) + "," + std::to_string(vehicle_acceleration_limit_) + "," + std::to_string(yield_max_deceleration);

        vehicle_status_pub.publish(mo_);
    }
}
