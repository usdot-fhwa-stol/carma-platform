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
    
    void MockCANDriver::parserCB(const cav_simulation_msgs::BagData::ConstPtr& msg){

        // generate messages from bag data
        ros::Time curr_time = ros::Time::now();
        
        if(msg->acc_engaged_flag){
            std_msgs::Bool acc_engaged = msg->acc_engaged;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/acc_engaged", acc_engaged);
        }
        
        if(msg->acceleration_flag){
            std_msgs::Float64 acceleration = msg->acceleration;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/acceleration", acceleration);
        }
        
        if(msg->antilock_brakes_active_flag){
            std_msgs::Bool antilock_brakes_active = msg->antilock_brakes_active;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/antilock_brakes_active", antilock_brakes_active);
        }
        
        if(msg->brake_lights_flag){
            std_msgs::Bool brake_lights = msg->brake_lights;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/brake_lights", brake_lights);
        }
        
        if(msg->brake_position_flag){
            std_msgs::Float64 brake_position = msg->brake_position;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/brake_position", brake_position);
        }
        
        if(msg->engine_speed_flag){
            std_msgs::Float64 engine_speed = msg->engine_speed;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/engine_speed", engine_speed);
        }
        
        if(msg->fuel_flow_flag){
            std_msgs::Float64 fuel_flow = msg->fuel_flow;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/fuel_flow", fuel_flow);
        }
        
        if(msg->odometer_flag){
            std_msgs::Float64 odometer = msg->odometer;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/odometer", odometer);
        }
        
        if(msg->parking_brake_flag){
            std_msgs::Bool parking_brake = msg->parking_brake;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/parking_brake", parking_brake);
        }
        
        if(msg->speed_flag){
            std_msgs::Float64 speed = msg->speed;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/speed", speed);
        }
        
        if(msg->stability_ctrl_active_flag){
            std_msgs::Bool stability_ctrl_active = msg->stability_ctrl_active;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/stability_ctrl_active", stability_ctrl_active);
        }
        
        if(msg->stability_ctrl_enabled_flag){
            std_msgs::Bool stability_ctrl_enabled = msg->stability_ctrl_enabled;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/stability_ctrl_enabled", stability_ctrl_enabled);
        }
        
        if(msg->steering_wheel_angle_flag){
            std_msgs::Float64 steering_wheel_angle = msg->steering_wheel_angle;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/steering_wheel_angle", steering_wheel_angle);
        }
        
        if(msg->throttle_position_flag){
            std_msgs::Float64 throttle_position = msg->throttle_position;
            mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("can/throttle_position", throttle_position);
        }
        
        if(msg->traction_ctrl_active_flag){
            std_msgs::Bool traction_ctrl_active = msg->traction_ctrl_active;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/traction_ctrl_active", traction_ctrl_active);
        }
        
        if(msg->traction_ctrl_enabled_flag){
            std_msgs::Bool traction_ctrl_enabled = msg->traction_ctrl_enabled;
            mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("can/traction_ctrl_enabled", traction_ctrl_enabled);
        }
        
        if(msg->transmission_state_flag){
            j2735_msgs::TransmissionState transmission_state = msg->transmission_state;
            mock_driver_node_.publishDataNoHeader<j2735_msgs::TransmissionState>("can/transmission_state", transmission_state);
        }
        
        if(msg->turn_signal_state_flag){
            cav_msgs::TurnSignal turn_signal_state = msg->turn_signal_state;
            mock_driver_node_.publishDataNoHeader<cav_msgs::TurnSignal>("can/turn_signal_state", turn_signal_state);
        }
        
        if(msg->vehicle_twist_flag){
            geometry_msgs::TwistStamped vehicle_twist = msg->vehicle_twist;
            vehicle_twist.header.stamp = curr_time;
            mock_driver_node_.publishData<geometry_msgs::TwistStamped>("can/vehicle/twist", vehicle_twist);
        }
        
        if(msg->vehicle_status_flag){
            autoware_msgs::VehicleStatus vehicle_status = msg->vehicle_status;
            vehicle_status.header.stamp = curr_time;
            mock_driver_node_.publishData<autoware_msgs::VehicleStatus>("can/vehicle_status", vehicle_status);
        }
        
        if(msg->velocity_accel_flag){
            automotive_platform_msgs::VelocityAccel velocity_accel = msg->velocity_accel;
            velocity_accel.header.stamp = curr_time;
            mock_driver_node_.publishData<automotive_platform_msgs::VelocityAccel>("can/velocity_accel", velocity_accel);
        }
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

        mock_driver_node_.spin(100);

        return 0;
    }

}