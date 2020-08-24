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
        std::string cmv_id;
        _pnh->param<std::string>("cmv_id", cmv_id, "");
        std::string cargo_id;
        _pnh->param<std::string>("cargo_id", cargo_id, "");

        ros::Publisher outbound_mob_op = _nh->advertise<cav_msgs::MobilityOperation>("outbound_mobility_operation", 5);
        _outbound_mobility_operations_publisher = std::make_shared<ros::Publisher>(outbound_mob_op);
        PortDrayageWorker pdw{
            cmv_id,
            cargo_id,
            "HOST_ID",
            [this](cav_msgs::MobilityOperation msg) {
               _outbound_mobility_operations_publisher->publish<cav_msgs::MobilityOperation>(msg);
            },
            speed_epsilon
        };
        
        ros::Subscriber maneuver_sub = _nh->subscribe<cav_msgs::ManeuverPlan>("final_Maneuver_plan", 5, 
            [&](const cav_msgs::ManeuverPlanConstPtr& plan) {
                pdw.set_maneuver_plan(plan);
        });
        _maneuver_plan_subscriber = std::make_shared<ros::Subscriber>(maneuver_sub);

        ros::Subscriber twist_sub = _nh->subscribe<geometry_msgs::TwistStamped>("localization/ekf_twist", 5, 
            [&](const geometry_msgs::TwistStampedConstPtr& speed) {
                pdw.set_current_speed(speed);
        });
        _cur_speed_subscriber = std::make_shared<ros::Subscriber>(twist_sub);

        std::function<bool()> spin_cb = [&]() {
            return pdw.spin();
        };
        _nh->setSpinCallback(spin_cb);

        ros::CARMANodeHandle::spin();

        return 0;
    }
} // namespace port_drayage_plugin
