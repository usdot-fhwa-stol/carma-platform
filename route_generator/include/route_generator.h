#pragma once

/*
 * Copyright (C) 2019 LEIDOS.
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

#include <string>
#include <iostream>
#include <vector>
#include <fstream>

#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>

#include <cav_msgs/Route.h>
#include <cav_msgs/RoutePath.h>
#include <cav_srvs/GetAvailableRoutes.h>
#include <cav_srvs/SetActiveRoute.h>
#include <cav_srvs/StartActiveRoute.h>
#include <cav_srvs/AbortActiveRoute.h>
#include <carma_utils/CARMAUtils.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>

#include "boost/filesystem.hpp"
#include "wgs84_utils/wgs84_utils.h"

class RouteGenerator
{

public:

    RouteGenerator();
    virtual ~RouteGenerator();

    // general starting point of this node
    void run();

    // read file names in the given route path
    static std::vector<std::string> read_route_names(std::string route_path);

private:

    // node handles
    std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

    // wm listener and pointer
    carma_wm::WMListener wml;
    carma_wm::WorldModelConstPtr wm;

    // Buffer which holds the tree of transforms
    tf2_ros::Buffer tfBuffer_;
    // tf2 listeners. Subscribes to the /tf and /tf_static topics
    tf2_ros::TransformListener tfListener_;

    // status of the selected route file
    bool route_is_active_;

    // directory of the routes path
    std::string route_file_path_;

    // publisher for waypoint loader full file path
    ros::Publisher route_file_path_pub_;
    ros::Publisher route_bin_pub_;
    
    // route service servers
    ros::ServiceServer get_available_route_srv_;
    ros::ServiceServer set_active_route_srv_;
    ros::ServiceServer start_active_route_srv_;
    ros::ServiceServer abort_active_route_srv_;

    // service callbacks
    bool get_available_route_cb(cav_srvs::GetAvailableRoutesRequest &req, cav_srvs::GetAvailableRoutesResponse &resp);
    bool set_active_route_cb(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp);
    bool start_active_route_cb(cav_srvs::StartActiveRouteRequest &req, cav_srvs::StartActiveRouteResponse &resp);
    bool abort_active_route_cb(cav_srvs::AbortActiveRouteRequest &req, cav_srvs::AbortActiveRouteResponse &resp);

    // new service callbacks, will replace old service call later
    bool set_active_route_cb_new(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp);

    // initialize this node
    void initialize();

};
