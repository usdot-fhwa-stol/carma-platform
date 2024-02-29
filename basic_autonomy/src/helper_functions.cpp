/*
 * Copyright (C) 2021-2024 LEIDOS.
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

#include <algorithm>
#include <basic_autonomy/helper_functions.hpp>


namespace basic_autonomy
{
namespace waypoint_generation
{
    int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                                 const carma_planning_msgs::msg::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.x_pos_global, state.y_pos_global);
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p, veh_point);
            if (distance < min_distance)
            {
                best_index = i;
                min_distance = distance;
            }
            i++;
        }
        return best_index;
    }

    int get_nearest_point_index(const std::vector<PointSpeedPair>& points,
                                                 const carma_planning_msgs::msg::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.x_pos_global, state.y_pos_global);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "veh_point: " << veh_point.x() << ", " << veh_point.y());
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }
        return best_index;
    }

    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm, double target_downtrack)
    {
        if(points.empty()){
            RCLCPP_WARN_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "Empty points vector received, returning -1");
            return -1;
        }

        // Find first point with a downtrack greater than target_downtrack
        const auto itr = std::find_if(std::cbegin(points), std::cend(points), 
            [&wm = std::as_const(wm), target_downtrack](const auto & point) { return wm->routeTrackPos(point).downtrack > target_downtrack; });

        int best_index = points.size() - 1;

        // Set best_index to the last point with a downtrack less than target_downtrack
        if(itr != std::cbegin(points)){
            best_index = std::distance(std::cbegin(points), std::prev(itr));
        }
        else{
            best_index = 0;
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), "get_nearest_index_by_downtrack>> Found best_index: " << best_index<<", points[i].x(): " << points.at(best_index).x() << ", points[i].y(): " << points.at(best_index).y());
        
        return best_index;
    }

    void split_point_speed_pairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

    int get_nearest_index_by_downtrack(const std::vector<PointSpeedPair>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const carma_planning_msgs::msg::VehicleState& state)
    {
        lanelet::BasicPoint2d state_pos(state.x_pos_global, state.y_pos_global);
        double ending_downtrack = wm->routeTrackPos(state_pos).downtrack;
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        split_point_speed_pairs(points, &basic_points, &speeds);
        return get_nearest_index_by_downtrack(basic_points, wm, ending_downtrack);
    }

    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const carma_planning_msgs::msg::VehicleState& state)
    {
        lanelet::BasicPoint2d state_pos(state.x_pos_global, state.y_pos_global);
        double ending_downtrack = wm->routeTrackPos(state_pos).downtrack;
        return get_nearest_index_by_downtrack(points, wm, ending_downtrack);
    }

}   // namespace waypoint_generation
}   // namespace basic_autonomy
