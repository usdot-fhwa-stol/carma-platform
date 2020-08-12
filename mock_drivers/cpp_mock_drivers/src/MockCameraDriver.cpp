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

    void MockCameraDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        
    }

    MockCameraDriver::MockCameraDriver(){

        camera_info_ptr_ = boost::make_shared<ROSComms<sensor_msgs::CameraInfo>>(ROSComms<sensor_msgs::CameraInfo>(CommTypes::pub, false, 10, "camera_info"));
        image_raw_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Image>>(ROSComms<sensor_msgs::Image>(CommTypes::pub, false, 10, "image_raw"));
        camera_info_1_ptr_ = boost::make_shared<ROSComms<sensor_msgs::CameraInfo>>(ROSComms<sensor_msgs::CameraInfo>(CommTypes::pub, false, 10, "1/camera_info"));
        image_raw_1_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Image>>(ROSComms<sensor_msgs::Image>(CommTypes::pub, false, 10, "1/image_raw"));
        image_rects_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Image>>(ROSComms<sensor_msgs::Image>(CommTypes::pub, false, 10, "image_rects"));
        projection_matrix_ptr_ = boost::make_shared<ROSComms<autoware_msgs::ProjectionMatrix>>(ROSComms<autoware_msgs::ProjectionMatrix>(CommTypes::pub, false, 10, "projection_matrix"));
        
    }

    int MockCameraDriver::run(){

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::CameraInfo>>>(camera_info_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Image>>>(image_raw_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::CameraInfo>>>(camera_info_1_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Image>>>(image_raw_1_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Image>>>(image_rects_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<autoware_msgs::ProjectionMatrix>>>(projection_matrix_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}