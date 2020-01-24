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
#include <iostream>
#include <fstream>

#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>

#include <tf2_ros/transform_listener.h>
#include <wgs84_utils/wgs84_utils.h>
#include <boost/filesystem.hpp>

class RouteGeneratorWorker
{

public:
    
    // generate a route using Lanelet2 library
    lanelet::Optional<lanelet::routing::Route> routing(lanelet::BasicPoint2d start,
                                                       std::vector<lanelet::BasicPoint2d> via,
                                                       lanelet::BasicPoint2d end,
                                                       lanelet::LaneletMapConstPtr map_pointer,
                                                       carma_wm::LaneletRoutingGraphConstPtr graph_pointer);

    // read file names in the given route path
    std::vector<std::string> read_route_names(std::string route_path);

    std::vector<tf2::Vector3> load_route_destinationsin_ecef(std::string route_file_path, std::string route_id);

};