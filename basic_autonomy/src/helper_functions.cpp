/*
 * Copyright (C) 2021 LEIDOS.
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

#include <basic_autonomy/helper_functions.h>


namespace basic_autonomy
{
namespace waypoint_generation
{
    int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                                 const cav_msgs::VehicleState& state)
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
                                                 const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.x_pos_global, state.y_pos_global);
        ROS_DEBUG_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
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
        size_t best_index = points.size() - 1;
        for(size_t i = 0;i < points.size(); i++){
            double downtrack = wm->routeTrackPos(points[i]).downtrack;
            if(downtrack > target_downtrack){
                //If value is negative, best index should be index 0
                best_index = std::max((size_t)0, i - 1);

                ROS_DEBUG_STREAM("get_nearest_index_by_downtrack>> Found best_idx: " << best_index<<", points[i].x(): " << points[best_index].x() << ", points[i].y(): " << points[best_index].y() << ", downtrack: "<< downtrack);
                break;
            }
        }
        ROS_DEBUG_STREAM("get_nearest_index_by_downtrack>> Found best_idx: " << best_index<<", points[i].x(): " << points[best_index].x() << ", points[i].y(): " << points[best_index].y());

        return static_cast<int>(best_index);
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
                                      const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d state_pos(state.x_pos_global, state.y_pos_global);
        double ending_downtrack = wm->routeTrackPos(state_pos).downtrack;
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        split_point_speed_pairs(points, &basic_points, &speeds);
        return get_nearest_index_by_downtrack(basic_points, wm, ending_downtrack);
    }

    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d state_pos(state.x_pos_global, state.y_pos_global);
        double ending_downtrack = wm->routeTrackPos(state_pos).downtrack;
        return get_nearest_index_by_downtrack(points, wm, ending_downtrack);
    }

}
}