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

#include "cpp_mock_drivers/MockGNSSDriver.h"

namespace mock_drivers{

    void MockGNSSDriver::parserCB(const cav_msgs::BagParserMsg::ConstPtr& msg){
        
    }

    MockGNSSDriver::MockGNSSDriver(){

        GPS_fix_ptr_ = boost::make_shared<ROSComms<gps_common::GPSFix>>(ROSComms<gps_common::GPSFix>(CommTypes::pub, false, 10, "gnss_fixed_fused"));
    
    }

    int MockGNSSDriver::run(){

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagParserMsg::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<gps_common::GPSFix>>>(GPS_fix_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}