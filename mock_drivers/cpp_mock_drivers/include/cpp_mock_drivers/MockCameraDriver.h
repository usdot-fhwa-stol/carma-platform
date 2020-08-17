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

#pragma once

#include "cpp_mock_drivers/MockDriver.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <autoware_msgs/ProjectionMatrix.h>

namespace mock_drivers{

    class MockCameraDriver : public MockDriver {

        private:

            boost::shared_ptr<ROSComms<sensor_msgs::CameraInfo>> camera_info_ptr_;
            boost::shared_ptr<ROSComms<sensor_msgs::Image>> image_raw_ptr_;
            boost::shared_ptr<ROSComms<sensor_msgs::CameraInfo>> camera_info_1_ptr_;
            boost::shared_ptr<ROSComms<sensor_msgs::Image>> image_raw_1_ptr_;
            boost::shared_ptr<ROSComms<sensor_msgs::Image>> image_rects_ptr_;
            boost::shared_ptr<ROSComms<autoware_msgs::ProjectionMatrix>> projection_matrix_ptr_;

        public:

            MockCameraDriver(bool dummy = false);
            int run();
            void parserCB(const cav_msgs::BagData::ConstPtr& msg);
            bool driverDiscovery();

    };

}