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

#include "cpp_mock_drivers/BagParser.h"

namespace mock_drivers{

    BagParser::BagParser(bool dummy){
        mock_driver_node_ = MockDriverNode(dummy);
        bag_data_pub_ptr_ = boost::make_shared<ROSComms<carma_simulation_msgs::BagData>>(ROSComms<carma_simulation_msgs::BagData>(CommTypes::pub, false, 10, "bag_data"));
    }

    BagParser::BagParser(std::string file_path){
        file_path_ = file_path;
        bag_data_pub_ptr_ = boost::make_shared<ROSComms<carma_simulation_msgs::BagData>>(ROSComms<carma_simulation_msgs::BagData>(CommTypes::pub, false, 10, "bag_data"));
    }

    bool BagParser::publishCallback() {
        static ros::Time startTime;
        static ros::Duration timeFrame = ros::Duration(1.0/rate_);
        carma_simulation_msgs::BagData message;
        
        bag_.open(file_path_, rosbag::bagmode::Read);

        if (startTime.isZero()){
            startTime = rosbag::View(bag_).getBeginTime();
        }

        for(rosbag::MessageInstance const m: rosbag::View(bag_, startTime, (startTime + timeFrame))){

            // camera
            // if (m.getTopic() == "/hardware_interface/camera/1/camera_info"){message.camera_info_1 = *m.instantiate<sensor_msgs::CameraInfo>();}
            // if (m.getTopic() == "/hardware_interface/camera/1/image_raw"){message.image_raw_1 = *m.instantiate<sensor_msgs::Image>();}
            if (m.getTopic() == "/hardware_interface/camera/camera_info"){message.camera_info = *m.instantiate<sensor_msgs::CameraInfo>();}
            if (m.getTopic() == "/hardware_interface/camera/image_raw"){message.image_raw = *m.instantiate<sensor_msgs::Image>();}
            if (m.getTopic() == "/hardware_interface/camera/image_rects"){message.image_rects = *m.instantiate<sensor_msgs::Image>();}
            if (m.getTopic() == "/hardware_interface/camera/projection_matrix"){message.projection_matrix = *m.instantiate<autoware_msgs::ProjectionMatrix>();}
            
            // can
            if (m.getTopic() == "/hardware_interface/can/acc_engaged"){message.acc_engaged = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/acceleration"){message.acceleration = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/antilock_brakes_active"){message.antilock_brakes_active = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/brake_lights"){message.brake_lights = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/brake_position"){message.brake_position = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/engine_speed"){message.engine_speed = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/fuel_flow"){message.fuel_flow = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/odometer"){message.odometer = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/parking_brake"){message.parking_brake = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/speed"){message.speed = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/stability_ctrl_active"){message.stability_ctrl_active = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/stability_ctrl_enabled"){message.stability_ctrl_enabled = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/steering_wheel_angle"){message.steering_wheel_angle = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/throttle_position"){message.throttle_position = *m.instantiate<std_msgs::Float64>();}
            if (m.getTopic() == "/hardware_interface/can/traction_ctrl_active"){message.traction_ctrl_active = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/traction_ctrl_enabled"){message.traction_ctrl_enabled = *m.instantiate<std_msgs::Bool>();}
            if (m.getTopic() == "/hardware_interface/can/transmission_state"){message.transmission_state = *m.instantiate<j2735_msgs::TransmissionState>();}
            if (m.getTopic() == "/hardware_interface/can/turn_signal_state"){message.turn_signal_state = *m.instantiate<cav_msgs::TurnSignal>();}
            if (m.getTopic() == "/hardware_interface/can/vehicle/twist"){message.vehicle_twist = *m.instantiate<geometry_msgs::TwistStamped>();}
            if (m.getTopic() == "/hardware_interface/can/vehicle_status"){message.vehicle_status = *m.instantiate<autoware_msgs::VehicleStatus>();}
            if (m.getTopic() == "/hardware_interface/can/velocity_accel"){message.velocity_accel = *m.instantiate<automotive_platform_msgs::VelocityAccel>();}
            
            // comms
            if (m.getTopic() == "/hardware_interface/comms/inbound_binary_msg"){message.inbound_binary_msg = *m.instantiate<cav_msgs::ByteArray>();}
            
            // controller
            if (m.getTopic() == "/hardware_interface/controller/robot_status"){message.robot_status = *m.instantiate<cav_msgs::RobotEnabled>();}
            
            // gnss
            if (m.getTopic() == "/hardware_interface/gnss/gnss_fixed_fused"){message.gnss_fixed_fused = *m.instantiate<gps_common::GPSFix>();}
            
            // imu
            if (m.getTopic() == "/hardware_interface/imu/raw_data"){message.raw_data = *m.instantiate<sensor_msgs::Imu>();}
            
            // lidar
            if (m.getTopic() == "/hardware_interface/lidar/points_raw"){message.points_raw = *m.instantiate<sensor_msgs::PointCloud2>();}
            
            // radar
            if (m.getTopic() == "/hardware_interface/radar/status"){message.status = *m.instantiate<radar_msgs::RadarStatus>();}
            if (m.getTopic() == "/hardware_interface/radar/tracks_raw"){message.tracks_raw = *m.instantiate<radar_msgs::RadarTrackArray>();}
            
            // roadway_sensor
            if (m.getTopic() == "/hardware_interface/roadway_sensor/detected_objects"){message.detected_objects = *m.instantiate<derived_object_msgs::ObjectWithCovariance>();}
            if (m.getTopic() == "/hardware_interface/roadway_sensor/lane_models"){message.lane_models = *m.instantiate<derived_object_msgs::LaneModels>();}

        }

        startTime += timeFrame;

        mock_driver_node_.publishData<carma_simulation_msgs::BagData>("/bag_data", message);

        bag_.close();

        return true;

    }
    
    int BagParser::run(){

        mock_driver_node_.init();

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<carma_simulation_msgs::BagData>>>(bag_data_pub_ptr_);

        mock_driver_node_.setSpinCallback(std::bind(&BagParser::publishCallback, this));
        mock_driver_node_.spin(rate_);

        return 0;
    }
}