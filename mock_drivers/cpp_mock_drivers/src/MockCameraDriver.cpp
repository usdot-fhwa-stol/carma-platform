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

#include "cpp_mock_drivers/MockCameraDriver.h"

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

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("/hardware_interface/driver_discovery", discovery_msg);

        return true;
    }

    void MockCameraDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        sensor_msgs::CameraInfo camera_info = msg->camera_info;
        sensor_msgs::Image image_raw = msg->image_raw;
        // sensor_msgs::CameraInfo camera_info_1 = msg->camera_info_1;
        // sensor_msgs::Image image_raw_1 = msg->image_raw_1;
        sensor_msgs::Image image_rects = msg->image_rects;
        autoware_msgs::ProjectionMatrix projection_matrix = msg->projection_matrix;

        ros::Time curr_time = ros::Time::now();
        
        camera_info.header.stamp = curr_time;
        image_raw.header.stamp = curr_time;
        // camera_info_1.header.stamp = curr_time;
        // image_raw_1.header.stamp = curr_time;
        image_rects.header.stamp = curr_time;
        projection_matrix.header.stamp = curr_time;

        mock_driver_node_.publishData<sensor_msgs::CameraInfo>("camera_info", camera_info);
        mock_driver_node_.publishData<sensor_msgs::Image>("image_raw", image_raw);
        // mock_driver_node_.publishData<sensor_msgs::CameraInfo>("/1/camera_info", camera_info_1);
        // mock_driver_node_.publishData<sensor_msgs::Image>("/1/image_raw", image_raw_1);
        mock_driver_node_.publishData<sensor_msgs::Image>("image_rects", image_rects);
        mock_driver_node_.publishData<autoware_msgs::ProjectionMatrix>("projection_matrix", projection_matrix);
    }

    MockCameraDriver::MockCameraDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        camera_info_ptr_ = boost::make_shared<ROSComms<sensor_msgs::CameraInfo>>(ROSComms<sensor_msgs::CameraInfo>(CommTypes::pub, false, 10, "camera_info"));
        image_raw_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Image>>(ROSComms<sensor_msgs::Image>(CommTypes::pub, false, 10, "image_raw"));
        // camera_info_1_ptr_ = boost::make_shared<ROSComms<sensor_msgs::CameraInfo>>(ROSComms<sensor_msgs::CameraInfo>(CommTypes::pub, false, 10, "1/camera_info"));
        // image_raw_1_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Image>>(ROSComms<sensor_msgs::Image>(CommTypes::pub, false, 10, "1/image_raw"));
        image_rects_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Image>>(ROSComms<sensor_msgs::Image>(CommTypes::pub, false, 10, "image_rects"));
        projection_matrix_ptr_ = boost::make_shared<ROSComms<autoware_msgs::ProjectionMatrix>>(ROSComms<autoware_msgs::ProjectionMatrix>(CommTypes::pub, false, 10, "projection_matrix"));
        
    }

    int MockCameraDriver::run(){

        mock_driver_node_.init();

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::CameraInfo>>>(camera_info_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Image>>>(image_raw_ptr_);
        // mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::CameraInfo>>>(camera_info_1_ptr_);
        // mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Image>>>(image_raw_1_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Image>>>(image_rects_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<autoware_msgs::ProjectionMatrix>>>(projection_matrix_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockCameraDriver::driverDiscovery, this));

        mock_driver_node_.spin(1);

        return 0;
    }

}