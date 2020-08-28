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

#include "rosbag_mock_drivers/MockCANDriver.h"

namespace mock_drivers{

    bool MockCANDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockCANDriver";
        discovery_msg.status = 1;

        discovery_msg.can = true;
        discovery_msg.radar = false;
        discovery_msg.gnss = false;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = false;
        discovery_msg.comms = false;
        discovery_msg.controller = false;
        discovery_msg.camera = false;
        discovery_msg.imu = false;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("driver_discovery", discovery_msg);

        return true;
    }

    MockCANDriver::MockCANDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

    }

    int MockCANDriver::run(){

        mock_driver_node_.init();

        // data topic publishers

            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + acc_engaged_topic_, acc_engaged_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + acceleration_topic_, acceleration_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + antilock_brakes_topic_, antilock_brakes_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + brake_position_topic_, brake_position_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + engine_speed_topic_, engine_speed_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + fuel_flow_topic_, fuel_flow_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + odometer_topic_, odometer_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + parking_brake_topic_, parking_brake_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + speed_topic_, speed_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + stability_ctrl_active_topic_, stability_ctrl_active_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + stability_ctrl_enabled_topic_, stability_ctrl_enabled_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + steering_wheel_angle_topic_, steering_wheel_angle_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Float64>(bag_prefix_ + throttle_position_topic_, throttle_position_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + traction_ctrl_active_topic_, traction_ctrl_active_topic_, false, 10);
            addPassthroughPubNoHeader<std_msgs::Bool>(bag_prefix_ + traction_ctrl_enabled_topic_, traction_ctrl_enabled_topic_, false, 10);
            addPassthroughPubNoHeader<j2735_msgs::TransmissionState>(bag_prefix_ + transmission_state_topic_, transmission_state_topic_, false, 10);
            addPassthroughPubNoHeader<cav_msgs::TurnSignal>(bag_prefix_ + turn_signal_state_topic_, turn_signal_state_topic_, false, 10);

            addPassthroughPub<geometry_msgs::TwistStamped>(bag_prefix_ + vehicle_twist, vehicle_twist, false, 10);
            addPassthroughPub<autoware_msgs::VehicleStatus>(bag_prefix_ + vehicle_status_topic_, vehicle_status_topic_, false, 10);
            addPassthroughPub<automotive_platform_msgs::VelocityAccel>(bag_prefix_ + velocity_accel_topic_, velocity_accel_topic_, false, 10);


        // driver discovery publisher
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockCANDriver::driverDiscovery, this));

        mock_driver_node_.spin(50);

        return 0;
    }

}