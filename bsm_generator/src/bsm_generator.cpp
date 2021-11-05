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


#include "bsm_generator.h"

namespace bsm_generator
{
    BSMGenerator::BSMGenerator() : bsm_generation_frequency_(10.0) {}

    void BSMGenerator::initialize()
    {
        int bsmid;
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("bsm_generation_frequency", bsm_generation_frequency_, 10.0);
        pnh_->param<bool>("/bsm_id_rotation_enabled", bsm_id_rotation_enabled_, true);
        pnh_->param<int>("/bsm_message_id", bsmid, 0);
        for(size_t i = 0; i < 4; ++i ) //As the BSM Messsage ID is a four-element vector, the loop should iterate four times.
        {
            bsm_message_id_.emplace_back( bsmid >> (8 * i) );
        }

        nh_->param<double>("vehicle_length", vehicle_length_, 5.0);
        nh_->param<double>("vehicle_width", vehicle_width_, 2.0);
        bsm_pub_ = nh_->advertise<cav_msgs::BSM>("bsm_outbound", 5);
        timer_ = nh_->createTimer(ros::Duration(1.0 / bsm_generation_frequency_), &BSMGenerator::generateBSM, this);
        gear_sub_ = nh_->subscribe("transmission_state", 1, &BSMGenerator::gearCallback, this);
        speed_sub_ = nh_->subscribe("vehicle_speed_cov", 1, &BSMGenerator::speedCallback, this);
        steer_wheel_angle_sub_ = nh_->subscribe("steering_wheel_angle", 1, &BSMGenerator::steerWheelAngleCallback, this);
        accel_sub_ = nh_->subscribe("velocity_accel_cov", 1, &BSMGenerator::accelCallback, this);
        yaw_sub_ = nh_->subscribe("imu_raw", 1, &BSMGenerator::yawCallback, this);
        brake_sub_ = nh_->subscribe("brake_position", 1, &BSMGenerator::brakeCallback, this);
        pose_sub_ = nh_->subscribe("pose", 1, &BSMGenerator::poseCallback, this);
        heading_sub_ = nh_->subscribe("gnss_fix_fused", 1, &BSMGenerator::headingCallback, this);
        georeference_sub_ = nh_->subscribe("georeference", 1, &BSMGenerator::georeferenceCallback, this);
    }

    void BSMGenerator::initializeBSM()
    {
        bsm_.core_data.presence_vector = 0;
        bsm_.core_data.size.vehicle_width = vehicle_width_;
        bsm_.core_data.size.vehicle_length = vehicle_length_;
        bsm_.core_data.size.presence_vector = bsm_.core_data.size.presence_vector | bsm_.core_data.size.VEHICLE_LENGTH_AVAILABLE;
        bsm_.core_data.size.presence_vector = bsm_.core_data.size.presence_vector | bsm_.core_data.size.VEHICLE_WIDTH_AVAILABLE;
    }

    void BSMGenerator::run()
    {
        initialize();
        initializeBSM();
        ros::CARMANodeHandle::spin();
    }

    void BSMGenerator::georeferenceCallback(const std_msgs::StringConstPtr& msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  // Build projector from proj string
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
        bsm_.core_data.angle = worker.getSteerWheelAngleInRnage(msg->data);
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.STEER_WHEEL_ANGLE_AVAILABLE;
    }

    void BSMGenerator::accelCallback(const automotive_platform_msgs::VelocityAccelCovConstPtr& msg)
    {
        bsm_.core_data.accelSet.longitudinal = worker.getLongAccelInRange(msg->accleration);
        bsm_.core_data.accelSet.presence_vector = bsm_.core_data.accelSet.presence_vector | bsm_.core_data.accelSet.ACCELERATION_AVAILABLE;
    }

    void BSMGenerator::yawCallback(const sensor_msgs::ImuConstPtr& msg)
    {
        bsm_.core_data.accelSet.yaw_rate = worker.getYawRateInRange(static_cast<float>(msg->angular_velocity.z));
        bsm_.core_data.accelSet.presence_vector = bsm_.core_data.accelSet.presence_vector | bsm_.core_data.accelSet.YAWRATE_AVAILABLE;
    }

    void BSMGenerator::brakeCallback(const std_msgs::Float64ConstPtr& msg)
    {
        bsm_.core_data.brakes.wheelBrakes.brake_applied_status = worker.getBrakeAppliedStatus(msg->data);
    }

    void BSMGenerator::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        // Use pose message as an indicator of new location updates
        if (!map_projector_) {
            ROS_DEBUG_STREAM("Ignoring pose message as projection string has not been defined");
            return;
        }
        
        lanelet::GPSPoint coord = map_projector_->reverse( { msg->pose.position.x, msg->pose.position.y, msg->pose.position.z } );
      
        bsm_.core_data.longitude = coord.lon;
        bsm_.core_data.latitude = coord.lat;
        bsm_.core_data.elev = coord.ele;
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.LONGITUDE_AVAILABLE;
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.LATITUDE_AVAILABLE;
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.ELEVATION_AVAILABLE;

    }

    void BSMGenerator::headingCallback(const gps_common::GPSFixConstPtr& msg)
    {
        bsm_.core_data.heading = worker.getHeadingInRange(static_cast<float>(msg->track));
    }

    void BSMGenerator::generateBSM(const ros::TimerEvent& event)
    {
        bsm_.header.stamp = ros::Time::now();
        bsm_.core_data.msg_count = worker.getNextMsgCount();
        
        if(bsm_id_rotation_enabled_)
        {
            bsm_.core_data.id = worker.getMsgId(ros::Time::now());
        }
        else
        {
            bsm_.core_data.id = bsm_message_id_;
        }
        bsm_.core_data.sec_mark = worker.getSecMark(ros::Time::now());
        bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SEC_MARK_AVAILABLE;
        // currently the accuracy is not available because ndt_matching does not provide accuracy measurement
        bsm_.core_data.accuracy.presence_vector = 0;
        bsm_pub_.publish(bsm_);
    }
}
