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

#include "rosbag_mock_drivers/MockCameraDriver.h"

namespace mock_drivers{

    bool MockCameraDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockCameraDriver";
        discovery_msg.status = 1;

        discovery_msg.can = false;
        discovery_msg.radar = false;
        discovery_msg.gnss = false;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = false;
        discovery_msg.comms = false;
        discovery_msg.controller = false;
        discovery_msg.camera = true;
        discovery_msg.imu = false;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("driver_discovery", discovery_msg);

        return true;
    }

    MockCameraDriver::MockCameraDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);
    }

    int MockCameraDriver::run(){

        mock_driver_node_.init();

        addPassthroughPub<sensor_msgs::CameraInfo>(bag_prefix_ + camera_info_topic_, camera_info_topic_, false, 10);
        addPassthroughPub<sensor_msgs::Image>(bag_prefix_ + image_raw_topic_, image_raw_topic_, false, 10);
        addPassthroughPub<sensor_msgs::Image>(bag_prefix_ + image_rects_topic_, image_rects_topic_, false, 10);
        addPassthroughPub<autoware_msgs::ProjectionMatrix>(bag_prefix_ + projection_matrix_topic_, projection_matrix_topic_, false, 10);

        // driver discovery publisher
        mock_driver_node_.addPub(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockCameraDriver::driverDiscovery, this));

        mock_driver_node_.spin(20);

        return 0;
    }

}