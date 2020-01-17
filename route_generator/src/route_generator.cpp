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

#include <ros/ros.h>

#include "route_generator.h"

RouteGenerator::RouteGenerator() : route_is_active_(false),  tfListener_(tfBuffer_) {}

RouteGenerator::~RouteGenerator() {}

void RouteGenerator::initialize()
{
    nh_.reset(new ros::CARMANodeHandle());
    pnh_.reset(new ros::CARMANodeHandle("~"));
    pnh_->getParam("route_file_path", route_file_path_);
    route_file_path_pub_ = nh_->advertise<std_msgs::String>("selected_route_path", 1);
    route_bin_pub_ = nh_->advertise<cav_msgs::RoutePath>("selected_route_bin", 1);
    get_available_route_srv_ = nh_->advertiseService("get_available_routes", &RouteGenerator::get_available_route_cb, this);
    set_active_route_srv_ = nh_->advertiseService("set_active_route", &RouteGenerator::set_active_route_cb_new, this);
    start_active_route_srv_ = nh_->advertiseService("start_active_route", &RouteGenerator::start_active_route_cb, this);
    abort_active_route_srv_ = nh_->advertiseService("abort_active_route", &RouteGenerator::abort_active_route_cb, this);
    carma_wm::WorldModelConstPtr wm = wml.getWorldModel();
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
        std::string route_file_name = req.routeID.append(".csv");
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

bool RouteGenerator::set_active_route_cb_new(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp)
{
    // load destination points
    std::string route_file_name = route_file_path_ + req.routeID.append(".csv");
    std::ifstream fs(route_file_name);
    std::string line;
    std::vector<tf2::Vector3> destination_points;
    while(std::getline(fs, line))
    {
        wgs84_utils::wgs84_coordinate coordinate;
        auto comma = line.find(",");
        coordinate.lon = std::stod(line.substr(0, comma));
        line.erase(0, comma + 1);
        comma = line.find(",");
        coordinate.lat = std::stod(line.substr(0, comma));
        coordinate.elevation = std::stod(line.substr(comma + 1));
        destination_points.emplace_back(wgs84_utils::geodesic_to_ecef(coordinate, tf2::Transform()));
    }
    // transform to local map frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2::Transform map_in_earth;
    try{
        // TODO This line gives me an error.
        //tf2::convert(tfBuffer_.lookupTransform("earth", "map", ros::Time(0)).transform, map_in_earth);
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Could not lookup transform with exception " << ex.what());
    }
    // Compose routing function inputs
    std::vector<lanelet::BasicPoint2d> destination_points_in_map;
    for(tf2::Vector3 point : destination_points) {
        tf2::Transform point_in_earth;
        point_in_earth.setOrigin(point);
        auto point_in_map = map_in_earth.inverse() * point_in_earth;
        destination_points_in_map.push_back(lanelet::BasicPoint2d(point_in_map.getOrigin().getX(), point_in_map.getOrigin().getY()));
    }
    carma_wm::LaneletRoutingGraphConstPtr p = wm->getMapRoutingGraph();
    if(destination_points_in_map.size() < 2) {
        return false;
    }
    // generate a route
    auto route = routing(destination_points_in_map.front(),
                         std::vector<lanelet::BasicPoint2d>(destination_points_in_map.begin() + 1, destination_points_in_map.end() - 1),
                         destination_points_in_map.back(),
                         wm->getMap(), wm->getMapRoutingGraph());
    if(!route) {
        return false;
    }
    // publish route to carma_wm
    cav_msgs::RoutePath msg;
    msg.route_name = req.routeID;
    for(const auto& ll : route.get().shortestPath()) {
        msg.shortest_path_lanelet_ids.push_back(ll.id());
    }
    route_bin_pub_.publish(msg);
    return true;
}

lanelet::Optional<lanelet::routing::Route> RouteGenerator::routing(lanelet::BasicPoint2d start, std::vector<lanelet::BasicPoint2d> via, lanelet::BasicPoint2d end, lanelet::LaneletMapConstPtr map_pointer, carma_wm::LaneletRoutingGraphConstPtr graph_pointer)
{
    // find start lanelet
    auto start_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, start, 1);
    // check if there are any lanelets in the map
    if(start_lanelet_vector.size() == 0) {
        ROS_ERROR_STREAM("Found no lanelets in the map. Routing cannot be done.");
        return lanelet::Optional<lanelet::routing::Route>();
    }
    // extract starting lanelet
    auto start_lanelet = lanelet::ConstLanelet(start_lanelet_vector[0].second.constData());
    // find end lanelet
    auto end_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, end, 1);
    // extract end lanelet
    auto end_lanelet = lanelet::ConstLanelet(end_lanelet_vector[0].second.constData());
    // find all via lanelets
    lanelet::ConstLanelets via_lanelets_vector;
    for(lanelet::BasicPoint2d point : via) {
        auto via_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, point, 1);
        via_lanelets_vector.push_back(lanelet::ConstLanelet(via_lanelet_vector[0].second.constData()));
    }
    auto route = graph_pointer->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);
    return route;
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

std::vector<std::string> RouteGenerator::read_route_names(std::string route_path)
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