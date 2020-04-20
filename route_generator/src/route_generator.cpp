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

#include <ros/ros.h>

#include "route_generator.h"

RouteGenerator::RouteGenerator(){}

RouteGenerator::~RouteGenerator() {}

void RouteGenerator::initialize()
{
    nh_.reset(new ros::CARMANodeHandle());
    pnh_.reset(new ros::CARMANodeHandle("~"));
    pnh_->getParam("route_file_path", route_file_path_);
    route_file_path_pub_ = nh_->advertise<std_msgs::String>("selected_route_path", 1);
    get_available_route_srv_ = nh_->advertiseService("get_available_routes", &RouteGenerator::get_available_route_cb, this);
    set_active_route_srv_ = nh_->advertiseService("set_active_route", &RouteGenerator::set_active_route_cb, this);
    start_active_route_srv_ = nh_->advertiseService("start_active_route", &RouteGenerator::start_active_route_cb, this);
    abort_active_route_srv_ = nh_->advertiseService("abort_active_route", &RouteGenerator::abort_active_route_cb, this);
}

void RouteGenerator::run()
{
    initialize();   
    ros::CARMANodeHandle::spin();
}

bool RouteGenerator::get_available_route_cb(cav_srvs::GetAvailableRoutesRequest &req, cav_srvs::GetAvailableRoutesResponse &resp)
{
    std::vector<std::string> route_ids = RouteGenerator::read_route_names(route_file_path_);
    for(int i = 0; i < route_ids.size(); ++i)
    {
        std::string route_name = route_ids[i].substr(0, route_ids[i].find(".csv"));
        cav_msgs::Route route_msg;
        route_msg.routeID = route_name;
        route_msg.routeName = route_name;
        route_msg.valid = true;
        resp.availableRoutes.push_back(route_msg);
    }
    return true;
}

bool RouteGenerator::set_active_route_cb(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp)
{
    if(!route_is_active_)
    {
        std::string route_file_name = req.routeID;

        // Check if the route file name contains spaces and if so log an error as waypoint_loader does not support spaces in file names
        // TODO once the waypoint_loader is no longer required it may be possible to support files with spaces
        if (route_file_name.find(" ") != std::string::npos) {
            ROS_WARN_STREAM("Route file name cannot contain spaces");
            resp.errorStatus = cav_srvs::SetActiveRouteResponse::NO_ROUTE;
            return true;
        }

        route_file_name = route_file_name + ".csv";

        std_msgs::String selected_route_file_path;
        selected_route_file_path.data = route_file_path_ + route_file_name;
        route_file_path_pub_.publish(selected_route_file_path);
        resp.errorStatus = cav_srvs::SetActiveRouteResponse::NO_ERROR;
    }
    else
    {
        ROS_WARN_STREAM("A route has already been started.");
        resp.errorStatus = cav_srvs::SetActiveRouteResponse::ALREADY_FOLLOWING_ROUTE;
    }
    
    return true;
}

bool RouteGenerator::start_active_route_cb(cav_srvs::StartActiveRouteRequest &req, cav_srvs::StartActiveRouteResponse &resp)
{
    if(!route_is_active_)
    {
        route_is_active_ = true;
        resp.errorStatus = cav_srvs::StartActiveRouteResponse::NO_ERROR;
    }
    else
    {
        ROS_WARN_STREAM("A route has already been started.");
        resp.errorStatus = cav_srvs::StartActiveRouteResponse::ALREADY_FOLLOWING_ROUTE;
    }
    
    return true;
}

bool RouteGenerator::abort_active_route_cb(cav_srvs::AbortActiveRouteRequest &req, cav_srvs::AbortActiveRouteResponse &resp)
{
    if(route_is_active_)
    {
        route_is_active_ = false;
        resp.error_status = cav_srvs::AbortActiveRouteResponse::NO_ERROR;
    }
    else
    {
        ROS_WARN_STREAM("No active route to abort!");
        resp.error_status = cav_srvs::AbortActiveRouteResponse::NO_ACTIVE_ROUTE;
    }
    return true;
}

std::vector<std::string> RouteGenerator::read_route_names(const std::string& route_path)
{
    boost::filesystem::path route_path_object(route_path);
    std::vector<std::string> route_names;
    if(boost::filesystem::exists(route_path_object))
    {
        boost::filesystem::directory_iterator end_point;
        for(boost::filesystem::directory_iterator itr(route_path_object); itr != end_point; ++itr)
        {
            if(!boost::filesystem::is_directory(itr->status()))
            {
                route_names.push_back(itr->path().filename().generic_string());
            }
        }
    }
    return route_names;
}

bool RouteGenerator::is_route_active ()
{
    return route_is_active_;
}