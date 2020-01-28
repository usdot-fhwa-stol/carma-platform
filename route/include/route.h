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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <carma_utils/CARMAUtils.h>

#include "route_generator_worker.h"

namespace route {

    class Route
    {

    public:

        Route();

        // general starting point of this node
        void run();

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // wm listener and pointer
        carma_wm::WMListener wml_;
        carma_wm::WorldModelConstPtr wm_;

        // route generator worker
        RouteGeneratorWorker rg_worker_;

        // Buffer which holds the tree of transforms
        tf2_ros::Buffer tf_buffer_;
        // tf2 listeners. Subscribes to the /tf and /tf_static topics
        tf2_ros::TransformListener tf_listener_;

        // route related publishers
        ros::Publisher route_pub_;
        ros::Publisher route_state_pub_;
        ros::Publisher route_event_pub_;

        // current pose subscriber
        ros::Subscriber pose_sub_;
        
        // route service servers
        ros::ServiceServer get_available_route_srv_;
        ros::ServiceServer set_active_route_srv_;
        ros::ServiceServer abort_active_route_srv_;

        // initialize this node
        void initialize();

        // spin callback
        bool spin_callback();

    };
}


