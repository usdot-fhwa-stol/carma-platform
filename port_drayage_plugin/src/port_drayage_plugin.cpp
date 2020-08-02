/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include "port_drayage_plugin/port_drayage_plugin.h"
#include "port_drayage_plugin/port_drayage_worker.h"

namespace port_drayage_plugin
{

    int PortDrayagePlugin::run() {
        if (_nh == nullptr || _pnh == nullptr) {
            ROS_ERROR("Port Drayage Plugin not properly initialized, node handles are null!");
            return -1;
        }

        double speed_epsilon = _pnh->param("stop_speed_epsilon", 1.0);
        std::string cmv_id = _pnh->param("cmv_id", "");
        std::string cargo_id = _pnh->("cargo_id", "");

        _outbound_mobility_operations_publisher = std::make_shared<ros::Publisher>("outbound_mobility_operation");
        PortDrayageWorker pdw{
            cmv_id,
            cargo_id,
            "HOST_ID",
            [this](cav_msgs::MobilityOperation msg) {
               _outbound_mobility_operations_publisher->publish<cav_msgs::MobilityOperation>(msg);
            },
            speed_epsilon
        };

        _maneuver_plan_subscriber = std::make_shared<ros::Subscriber>(
            _nh->subscribe<cav_msgs::ManeuverPlan>("final_Maneuver_plan", 5, 
            &PortDrayageWorker::set_maneuver_plan, 
            &pdw);
        );

        _cur_speed_subscriber = std::make_shared<ros::Subscriber>(
            _nh->subscribe<geometry_msgs::TwistStamped>("speed", 5, 
            &PortDrayageWorker::set_current_speed, 
            &pdw);
        );

        ros::CARMANodeHandle::spin();

        return 0;
    }
} // namespace port_drayage_plugin
