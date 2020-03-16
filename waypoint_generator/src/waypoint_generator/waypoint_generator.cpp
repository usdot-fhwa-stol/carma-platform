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

#include <waypoint_generator/waypoint_generator.hpp>

namespace waypoint_generator
{
void WaypointGenerator::initialize()
{

}

void WaypointGenerator::process_route(cav_msgs::Route route_msg)
{

}

void WaypointGenerator::publish_waypoints(autoware_msgs::LaneArray waypoints)
{

}

void WaypointGenerator::run()
{

}

std::vector<int> WaypointGenerator::compute_constant_curvature_regions(
    std::vector<double> curvatures, 
    double epsilon,
    int linearity_constraint)
{
    std::vector<int> regions;
    double cur_min = 0;
    for (int i = 0; i < curvatures.size(); i++) {
        if (i == 0) {
            cur_min = curvatures[i];
        }

        if (std::fabs(curvatures[i] - cur_min) > epsilon) {
            regions.push_back(i - 1);
            cur_min = curvatures[i];
        }
    }

    regions.push_back(curvatures.size() - 1);

    // Post-process for linearly increasing/decreasing regions
    // It is assumed that the underyling dataset is itself linearly increasing
    // or decreasing and that is is just sampling it down to the endpoint
    std::vector<int> out;
    for (int i = 0; i < regions.size() - 1; i++) {
        if (regions[i + 1] - regions[i] < linearity_constraint) {
            continue;
        } else {
            out.push_back(regions[i]);
        }
    }

    out.push_back(curvatures.size() - 1);

    return out;
}

std::vector<double> WaypointGenerator::normalize_curvature_regions(std::vector<double> curvatures, 
    std::vector<int> regions)
{

    int region = 0;
    double min = std::numeric_limits<double>::infinity();
    std::vector<double> mins;
    for (int i = 0; i < curvatures.size(); i++) {
        if (i <= regions[region]) {
            if (curvatures[i] < min) { 
                min = curvatures[i];
            }
        } else {
            mins.push_back(min);
            min = std::numeric_limits<double>::infinity();
            region++;
        }
    }
    mins.push_back(min);

    std::vector<double> processed_curvatures;
    for (int i = 0; i < regions.size(); i++) {
        if (i == 0) {
            for (int j = 0; j <= regions[i]; j++) {
                processed_curvatures.push_back(mins[i]);
            }
        } else {
            for (int j = 0; j <= (regions[i] - regions[i - 1]) - 1; j++) {
                processed_curvatures.push_back(mins[i]);
            }
        }
    }

    return processed_curvatures;
}

double WaypointGenerator::compute_speed_for_curvature(double curvature, double lateral_accel_limit)
{
    // Solve a = v^2/r (k = 1/r) for v
    // a = v^2 * k
    // a * k = v^2
    // v = sqrt(a * k)
    return std::sqrt(lateral_accel_limit * curvature);
}
std::vector<double> WaypointGenerator::compute_ideal_speeds(std::vector<double> curvatures, 
    double lateral_accel_limit)
{
    std::vector<double> out;
    for (double k : curvatures) {
        out.push_back(compute_speed_for_curvature(k, lateral_accel_limit));
    }

    return out;
}

std::vector<double> WaypointGenerator::apply_accel_limits(std::vector<double> speeds, 
    std::vector<int> regions,
    double accel_limit,
    double decel_limit)
{
    return speeds; // NO-OP for now
}

autoware_msgs::LaneArray WaypointGenerator::generate_lane_array_message(std::vector<double> speeds, std::vector<lanelet::ConstLanelet> lanelets)
{
    autoware_msgs::LaneArray out;

    for (int i = 0; i < lanelets.size(); i++) {
        
    }
}
};