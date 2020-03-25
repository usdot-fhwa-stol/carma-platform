/*
 * Copyright (C) 2020 LEIDOS.
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

#include <ros/ros.h>
#include <waypoint_generator/waypoint_generator.hpp>
#include <string>
#include <carma_utils/CARMAUtils.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <cav_msgs/Route.h>

namespace waypoint_generator
{
    class WaypointGeneratorNode
    {
        public:
            WaypointGeneratorNode(int argc, char** argv, std::string node_name):
                argc(argc),
                argv(argv),
                node_name(node_name) {};

            void initialize();
            void run();
            void new_route_callback();
            ~WaypointGeneratorNode();
        private:
            int argc;
            char** argv;
            std::string node_name;
            std::shared_ptr<ros::CARMANodeHandle> _nh, _pnh;
            ros::Subscriber _route_sub;
            ros::Publisher _waypoints_pub;
            double _curvature_epsilon = 3.0;
            int _linearity_constraint = 2;
            double _lateral_accel_limit = 1.5;
            double _longitudinal_accel_limit = 1.5;
            double _longitudinal_decel_limit = 1.5;
            double _max_speed = 10.0;
            autoware_msgs::LaneArray _cur_waypoints;
            WaypointGenerator _wpg;
            carma_wm::WMListener _wml;
            carma_wm::WorldModelConstPtr _wm;
    };
}