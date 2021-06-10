#pragma once
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
#include <basic_autonomy/basic_autonomy.h>

namespace basic_autonomy
{
namespace waypoint_generation
{
    /**
   * \brief Returns the nearest point to the provided vehicle pose in the provided list
   * 
   * \param points The points to evaluate
   * \param state The current vehicle state
   * 
   * \return index of nearest point in points
   */
    int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
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

    /**
   * \brief Returns the nearest point to the provided vehicle pose in the provided list
   * 
   * \param points The points to evaluate
   * \param state The current vehicle state
   * 
   * \return index of nearest point in points
   */
    int get_nearest_point_index(const std::vector<PointSpeedPair>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
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

    /**
   * \brief Returns the nearest point to the provided vehicle pose in the provided list
   * 
   * \param points The points to evaluate
   * \param wm The carma world model object used to query x,y position against downtrack along route
   * \param target_downtrack The downtrack to check closest point against
   * 
   * \return index of nearest point in points
   */
    int get_nearest_point_index(std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm, double target_downtrack){
        int best_index = points.size()-1;
        for(int i=points.size()-1;i>=0;i--){
            double downtrack = wm->routeTrackPos(points[i]).downtrack;
            if(downtrack <= target_downtrack){
                best_index = i;
                break;
            }
        }
        return best_index;
    }

    /**
   * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists 
   */ 
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

}
}