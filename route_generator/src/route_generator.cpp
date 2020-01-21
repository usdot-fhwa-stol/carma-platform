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

#include "route_generator.h"

RouteGenerator::RouteGenerator() : route_is_active_(false),  tf_listener_(tf_buffer_) {}

RouteGenerator::~RouteGenerator() {}

void RouteGenerator::initialize()
{
    nh_.reset(new ros::CARMANodeHandle());
    pnh_.reset(new ros::CARMANodeHandle("~"));
    pnh_->getParam("route_file_path", route_file_path_);
    route_pub_ = nh_->advertise<cav_msgs::RoutePath>("route", 1);
    get_available_route_srv_ = nh_->advertiseService("get_available_routes", &RouteGenerator::get_available_route_cb, this);
    set_active_route_srv_ = nh_->advertiseService("set_active_route", &RouteGenerator::set_active_route_cb, this);
    start_active_route_srv_ = nh_->advertiseService("start_active_route", &RouteGenerator::start_active_route_cb, this);
    abort_active_route_srv_ = nh_->advertiseService("abort_active_route", &RouteGenerator::abort_active_route_cb, this);
    wm_ = wml_.getWorldModel();
}

void RouteGenerator::run()
{
    initialize();   
    ros::CARMANodeHandle::spin();
}

bool RouteGenerator::get_available_route_cb(cav_srvs::GetAvailableRoutesRequest &req, cav_srvs::GetAvailableRoutesResponse &resp)
{
    std::vector<std::string> route_ids = rg_worker_.read_route_names(route_file_path_);
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
    auto destination_points = rg_worker_.load_route_destinationsin_ecef(route_file_path_, req.routeID);
    if(destination_points.size() < 2) {
        ROS_ERROR_STREAM("Selected route file conatins 1 or less points. Routing cannot be completed.");
        resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTE_FILE_ERROR;
        return false;
    }
    // transform to local map frame
    tf2::Transform map_in_earth;
    try{
        tf2::convert(tf_buffer_.lookupTransform("earth", "map", ros::Time(0)).transform, map_in_earth);
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Could not lookup transform with exception " << ex.what());
        resp.errorStatus = cav_srvs::SetActiveRouteResponse::TRANSFORM_ERROR;
        return false;
    }
    // Compose routing function inputs
    std::vector<lanelet::BasicPoint2d> destination_points_in_map;
    for(tf2::Vector3 point : destination_points) {
        tf2::Transform point_in_earth;
        point_in_earth.setOrigin(point);
        auto point_in_map = map_in_earth.inverse() * point_in_earth;
        destination_points_in_map.push_back(lanelet::BasicPoint2d(point_in_map.getOrigin().getX(), point_in_map.getOrigin().getY()));
    }
    carma_wm::LaneletRoutingGraphConstPtr p = wm_->getMapRoutingGraph();
    // generate a route
    auto route = rg_worker_.routing(destination_points_in_map.front(),
                         std::vector<lanelet::BasicPoint2d>(destination_points_in_map.begin() + 1, destination_points_in_map.end() - 1),
                         destination_points_in_map.back(),
                         wm_->getMap(), wm_->getMapRoutingGraph());
    if(!route) {
        ROS_ERROR_STREAM("Cannot find a route passing all destinations.");
        resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
        return false;
    }
    // publish route to carma_wm
    cav_msgs::RoutePath msg;
    msg.route_name = req.routeID;
    for(const auto& ll : route.get().shortestPath()) {
        msg.shortest_path_lanelet_ids.push_back(ll.id());
    }
    route_pub_.publish(msg);
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