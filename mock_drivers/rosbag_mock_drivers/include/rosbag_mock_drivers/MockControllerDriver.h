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

#include "rosbag_mock_drivers/MockDriver.h"
#include <autoware_msgs/VehicleCmd.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_srvs/SetEnableRobotic.h>

namespace mock_drivers{

    class MockControllerDriver : public MockDriver {

        private:

            boost::shared_ptr<ROSComms<cav_msgs::RobotEnabled>> robot_status_ptr_;
            boost::shared_ptr<ROSComms<const autoware_msgs::VehicleCmd::ConstPtr&>> vehicle_cmd_ptr_;
            boost::shared_ptr<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>> enable_robotic_ptr_;

        public:

            MockControllerDriver(bool dummy = false);
            int run();
            void parserCB(const cav_simulation_msgs::BagData::ConstPtr& msg);
            void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg);
            bool enableRoboticSrv(cav_srvs::SetEnableRobotic::Request& req, cav_srvs::SetEnableRobotic::Response& res);
            bool driverDiscovery();

    };

}