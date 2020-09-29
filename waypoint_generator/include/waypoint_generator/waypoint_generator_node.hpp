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
            void publishWaypoints(const autoware_msgs::LaneArray& msg);
        private:
            int argc;
            char** argv;
            std::string node_name;
            std::shared_ptr<ros::CARMANodeHandle> _nh{nullptr}; 
            std::shared_ptr<ros::CARMANodeHandle> _pnh{nullptr};
            ros::Subscriber _route_sub;
            ros::Publisher _waypoints_pub;
            autoware_msgs::LaneArray _cur_waypoints;
            std::shared_ptr<WaypointGenerator> _wpg;
            std::shared_ptr<carma_wm::WMListener> _wml{nullptr};
            carma_wm::WorldModelConstPtr _wm;
    };
}