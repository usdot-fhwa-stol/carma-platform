#pragma once

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

#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>
#include <cav_msgs/Route.h>
#include <cav_msgs/RouteEvent.h>
#include <cav_msgs/RouteState.h>
#include <cav_srvs/GetAvailableRoutes.h>
#include <cav_srvs/SetActiveRoute.h>
#include <cav_srvs/StartActiveRoute.h>
#include <cav_srvs/AbortActiveRoute.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <wgs84_utils/wgs84_utils.h>
#include <boost/filesystem.hpp>

#include "route_state_worker.h"

namespace route {

    class RouteGeneratorWorker
    {

    public:

        RouteGeneratorWorker(tf2_ros::Buffer& tf_buffer, carma_wm::WorldModelConstPtr wm);
        
        // generate a route using Lanelet2 library
        lanelet::Optional<lanelet::routing::Route> routing(const lanelet::BasicPoint2d start,
                                                        const std::vector<lanelet::BasicPoint2d>& via,
                                                        const lanelet::BasicPoint2d end,
                                                        const lanelet::LaneletMapConstPtr map_pointer,
                                                        const carma_wm::LaneletRoutingGraphConstPtr graph_pointer) const;

        // server callbacks
        bool get_available_route_cb(cav_srvs::GetAvailableRoutesRequest &req, cav_srvs::GetAvailableRoutesResponse &resp);
        bool set_active_route_cb(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp);
        bool abort_active_route_cb(cav_srvs::AbortActiveRouteRequest &req, cav_srvs::AbortActiveRouteResponse &resp);

        void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

        void set_route_file_path(const std::string& path);

        std::vector<tf2::Vector3> load_route_destinationsin_ecef(const std::string& route_id) const;

        std::vector<lanelet::BasicPoint2d> transform_to_map_frame(const std::vector<tf2::Vector3>& ecef_points, const tf2::Transform& map_in_earth) const;

        cav_msgs::Route compose_route_msg(const lanelet::Optional<lanelet::routing::Route>& route) const;

        void set_publishers(ros::Publisher route_event_pub, ros::Publisher route_state_pub, ros::Publisher route_pub);

        void set_ctdt_param(double ct_max_error, double dt_dest_range);

        bool spin_ballback();

    private:

        // route state worker
        RouteStateWorker rs_worker_;

        // param for the directory of the routes path
        std::string route_file_path_;

        tf2_ros::Buffer& tf_tree_;

        carma_wm::WorldModelConstPtr world_model_;

        cav_msgs::Route      route_msg_;
        cav_msgs::RouteEvent route_event_msg_;
        cav_msgs::RouteState route_state_msg_;

        double cross_track_max_, down_track_target_range_;

        double current_crosstrack_distance_, current_downtrack_distance_;

        ros::Publisher route_event_pub_, route_state_pub_, route_pub_;

        bool new_route_msg_generated_;

        std::queue<uint8_t> route_event_queue;

        void publish_route_event(uint8_t event_type);

    };
}

