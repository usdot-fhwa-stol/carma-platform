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

#include <carma_simulation_msgs/BagData.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <autoware_msgs/ProjectionMatrix.h>
#include <j2735_msgs/TransmissionState.h>
#include <cav_msgs/TurnSignal.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <automotive_platform_msgs/VelocityAccel.h>
#include <cav_msgs/ByteArray.h>
#include <cav_msgs/RobotEnabled.h>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <radar_msgs/RadarStatus.h>
#include <radar_msgs/RadarTrackArray.h>
#include <derived_object_msgs/ObjectWithCovariance.h>
#include <derived_object_msgs/LaneModels.h>
#include <tf2_msgs/TFMessage.h>

#include "cpp_mock_drivers/ROSComms.h"
#include "cpp_mock_drivers/MockDriverNode.h"


namespace mock_drivers{

    class BagParser {

        private:

            boost::shared_ptr<ROSComms<carma_simulation_msgs::BagData>> bag_data_pub_ptr_;
            MockDriverNode mock_driver_node_;
            rosbag::Bag bag_;
            std::string file_path_;
            double rate_ = 10.0;

        public:

            BagParser(bool dummy = false);
            BagParser(std::string file_path);
            bool publishCallback();
            int run();

            MockDriverNode getMockDriverNode() {return mock_driver_node_;}


    };
    
}