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

#include "route_generator_worker.h"

lanelet::Optional<lanelet::routing::Route> RouteGeneratorWorker::routing(lanelet::BasicPoint2d start, std::vector<lanelet::BasicPoint2d> via, lanelet::BasicPoint2d end, lanelet::LaneletMapConstPtr map_pointer, carma_wm::LaneletRoutingGraphConstPtr graph_pointer)
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

std::vector<std::string> RouteGeneratorWorker::read_route_names(std::string route_path)
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

std::vector<tf2::Vector3> RouteGeneratorWorker::load_route_destinationsin_ecef(std::string route_file_path, std::string route_id)
{
    std::string route_file_name = route_file_path + route_id.append(".csv");
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
    return destination_points;
}