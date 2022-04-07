/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include "route.h"
#include <carma_wm/WMListener.h>

namespace route {

    using std::placeholders::_1;
 
    void Route::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        // init publishers
        route_pub_ = nh_->advertise<cav_msgs::Route>("route", 1, true);
        route_event_pub_ = nh_->advertise<cav_msgs::RouteEvent>("route_event", 1);
        route_state_pub_ = nh_->advertise<cav_msgs::RouteState>("route_state", 1, true);
        route_marker_pub_= nh_->advertise<visualization_msgs::Marker>("route_marker", 1, true);
        // init subscribers
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteGeneratorWorker::twist_cb, &rg_worker_);
        geo_sub_ = nh_->subscribe("georeference", 1, &RouteGeneratorWorker::georeference_cb, &rg_worker_);
        // init service server
        get_available_route_srv_ = nh_->advertiseService("get_available_routes", &RouteGeneratorWorker::get_available_route_cb, &rg_worker_);
        set_active_route_srv_ = nh_->advertiseService("set_active_route", &RouteGeneratorWorker::set_active_route_cb, &rg_worker_);
        abort_active_route_srv_ = nh_->advertiseService("abort_active_route", &RouteGeneratorWorker::abort_active_route_cb, &rg_worker_);
        // set world model point form wm listener
        wm_ = wml_.getWorldModel();
        wml_.enableUpdatesWithoutRouteWL();
        rg_worker_.setWorldModelPtr(wm_);
        rg_worker_.setReroutingChecker(std::bind(&carma_wm::WMListener::checkIfReRoutingNeededWL,&wml_));
        // load params and pass to route generator worker
        double ct_error, dt_range;
        int cte_count_max;
        pnh_->getParam("max_crosstrack_error", ct_error);
        pnh_->getParam("destination_downtrack_range", dt_range);
        pnh_->getParam("cte_max_count", cte_count_max);
        rg_worker_.set_ctdt_param(ct_error, dt_range);
        rg_worker_.set_CTE_dist(ct_error);
        rg_worker_.set_CTE_count_max(cte_count_max);
        std::string route_file_location;
        pnh_->getParam("route_file_path", route_file_location);
        rg_worker_.set_route_file_path(route_file_location);
        rg_worker_.set_publishers(route_event_pub_, route_state_pub_, route_pub_,route_marker_pub_);
        rg_worker_.initializeBumperTransformLookup();
    }

    void Route::run()
    {
        initialize();

        // spin with spin_callback function from RouteGeneratorWorker
        ros::Timer spin_timer = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { rg_worker_.spin_callback(); });

        ros::CARMANodeHandle::spin();
    }

}



