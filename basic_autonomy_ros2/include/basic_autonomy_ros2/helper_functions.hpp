#pragma once
/*
 * Copyright (C) 2021-2022 LEIDOS.
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
#include <basic_autonomy_ros2/basic_autonomy.hpp>

namespace basic_autonomy
{
namespace waypoint_generation
{
    //Small value corresponding to 0.1 mm in meters for comparing distance to a near zero.
    const double epsilon_ = 0.0000001;
    /**
     * \brief Returns the nearest point (in terms of cartesian 2d distance) to the provided vehicle pose in the provided l
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * 
     * \return index of nearest point in points
     */
    int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                                 const carma_planning_msgs::msg::VehicleState& state);

    /**
     * \brief Returns the nearest point (in terms of cartesian 2d distance) to the provided vehicle pose in the provided list
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * 
     * \return index of nearest point in points
     */
    int get_nearest_point_index(const std::vector<PointSpeedPair>& points,
                                                 const carma_planning_msgs::msg::VehicleState& state);

    /**
     * \brief Returns the nearest "less than" point to the provided vehicle pose in the provided list by utilizing the downtrack measured along the route
     * NOTE: This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given point's downtrack.
     * Therefore, it is rather costlier method than comparing cartesian distance between the points and getting the closest. This way, however, the function
     * correctly returns the end point's index if the given state, despite being valid, is farther than the given points and can technically be near any of them.
     * 
     * \param points BasicLineString2d points
     * \param target_downtrack target downtrack along the route to get index near to
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm, double target_downtrack);

    /**
     * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists 
     * \param points Point Speed pair to split
     * \param basic_points points vector to be filled
     * \param speeds speeds vector to be filled
     */ 
    void split_point_speed_pairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds);

    /**
     * \brief Overload: Returns the nearest point to the provided vehicle pose in the provided list by utilizing the downtrack measured along the route
     * NOTE: This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given point's downtrack.
     * Therefore, it is rather costlier method than comparing cartesian distance between the points and getting the closest. This way, however, the function
     * correctly returns the end point's index if the given state, despite being valid, is farther than the given points and can technically be near any of them.
     * 
     * \param points The points and speed pairs to evaluate
     * \param state The current vehicle state
     * \param wm The carma world model
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<PointSpeedPair>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const carma_planning_msgs::msg::VehicleState& state);

    /**
     * \brief Overload: Returns the nearest point to the provided vehicle pose  in the provided list by utilizing the downtrack measured along the route
     * NOTE: This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given point's downtrack.
     * Therefore, it is rather costlier method than comparing cartesian distance between the points and getting the closest. This way, however, the function
     * correctly returns the end point if the given state, despite being valid, is farther than the given points and can technically be near any of them.
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * \param wm The carma world model
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const carma_planning_msgs::msg::VehicleState& state);    

}
}
