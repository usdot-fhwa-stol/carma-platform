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
#include <basic_autonomy/log/log.hpp>


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
        RCLCPP_DEBUG_STREAM(basic_autonomy::get_logger(), "veh_point: " << veh_point.x() << ", " << veh_point.y());
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

    size_t get_nearest_point_index(
        const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory,
        const lanelet::BasicPoint2d& position)
    {
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            auto dist = sqrt(pow(position.x() - trajectory.at(i).x, 2) +
                pow(position.y() - trajectory.at(i).y, 2));

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm, double target_downtrack)
    {
        if(std::empty(points)){
            RCLCPP_WARN_STREAM(basic_autonomy::get_logger(), "Empty points vector received, returning -1");
            return -1;
        }

        // Find first point with a downtrack greater than target_downtrack
        const auto itr = std::find_if(std::cbegin(points), std::cend(points),
            [&wm = std::as_const(wm), target_downtrack](const auto & point) { return wm->routeTrackPos(point).downtrack > target_downtrack; });

        int best_index = std::size(points) - 1;

        // Set best_index to the last point with a downtrack less than target_downtrack
        if(itr != std::cbegin(points)){
            best_index = std::distance(std::cbegin(points), std::prev(itr));
        }
        else{
            best_index = 0;
        }

        RCLCPP_DEBUG_STREAM(basic_autonomy::get_logger(), "get_nearest_index_by_downtrack>> Found best_index: " << best_index<<", points[i].x(): " << points.at(best_index).x() << ", points[i].y(): " << points.at(best_index).y());

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

    std::vector<lanelet::BasicPoint2d> build_chain_centerline(const carma_wm::WorldModelConstPtr &wm,
                                                               lanelet::ConstLanelet pivot,
                                                               double backward_length,
                                                               double forward_length)
    {
        std::vector<lanelet::ConstLanelet> chain{pivot};
        std::unordered_set<lanelet::Id> visited{pivot.id()};

        double covered_back = carma_wm::geometry::get_lanelet_centerline_length(pivot);
        while (covered_back < backward_length)
        {
            auto previous = wm->getMapRoutingGraph()->previous(chain.front(), false);
            bool no_predecessor = previous.empty();
            bool loop_detected = !no_predecessor && visited.count(previous.front().id()) > 0;

            if (no_predecessor)
            {
                RCLCPP_WARN_STREAM(basic_autonomy::get_logger(),
                    "create_lanechange_geometry: No routable predecessor lanelet found before lanelet "
                    << chain.front().id() << " (possibly closed or missing from the map). Using the "
                    << covered_back << "m of centerline that was reachable going backward.");
            }

            if (loop_detected)
            {
                RCLCPP_WARN_STREAM(basic_autonomy::get_logger(),
                    "create_lanechange_geometry: Detected a loop in lanelet connectivity before lanelet "
                    << chain.front().id() << "; stopping centerline extension.");
            }

            if (no_predecessor || loop_detected)
            {
                break;
            }

            lanelet::ConstLanelet prev = previous.front();
            visited.insert(prev.id());
            covered_back += carma_wm::geometry::get_lanelet_centerline_length(prev);
            chain.insert(chain.begin(), prev);
        }

        double covered_fwd = 0.0;
        while (covered_fwd < forward_length)
        {
            auto following = wm->getMapRoutingGraph()->following(chain.back(), false);
            bool no_successor = following.empty();
            bool loop_detected = !no_successor && visited.count(following.front().id()) > 0;

            if (no_successor)
            {
                RCLCPP_WARN_STREAM(basic_autonomy::get_logger(),
                    "create_lanechange_geometry: No routable successor lanelet found after lanelet "
                    << chain.back().id() << " (possibly closed or missing from the map). Using the "
                    << covered_fwd << "m of centerline that was reachable going forward.");
            }

            if (loop_detected)
            {
                RCLCPP_WARN_STREAM(basic_autonomy::get_logger(),
                    "create_lanechange_geometry: Detected a loop in lanelet connectivity after lanelet "
                    << chain.back().id() << "; stopping centerline extension.");
            }

            if (no_successor || loop_detected)
            {
                break;
            }

            lanelet::ConstLanelet next = following.front();
            visited.insert(next.id());
            covered_fwd += carma_wm::geometry::get_lanelet_centerline_length(next);
            chain.push_back(next);
        }

        std::vector<lanelet::BasicPoint2d> centerline;
        centerline.reserve(400);
        for (size_t i = 0; i < chain.size(); ++i)
        {
            auto ls = chain[i].centerline2d().basicLineString();
            if (i == 0)
            {
                centerline.insert(centerline.end(), ls.begin(), ls.end());
            }
            else
            {
                // Concatenate linestring starting from + 1 to avoid duplicating the shared endpoint
                centerline.insert(centerline.end(), ls.begin() + 1, ls.end());
            }
        }
        return centerline;
    }

    void extrapolate_to_length(std::vector<lanelet::BasicPoint2d>& centerline, double target_length, const std::string& description)
    {
        if (centerline.size() < 2)
        {
            throw std::invalid_argument("create_lanechange_geometry: " + description +
                " has fewer than 2 centerline points; cannot build or extrapolate a lane change trajectory from this map data");
        }

        double current_length = carma_wm::geometry::compute_arc_lengths(centerline).back();
        if (current_length >= target_length)
        {
            return;
        }

        RCLCPP_WARN_STREAM(basic_autonomy::get_logger(),
            "create_lanechange_geometry: Only " << current_length << "m of connected lanelet centerline was "
            << "available for " << description << " (needed " << target_length << "m). Extrapolating a "
            << "straight line from the last known heading so a lane change trajectory can still be produced.");

        lanelet::BasicPoint2d last = centerline.back();
        lanelet::BasicPoint2d prev = centerline[centerline.size() - 2];
        lanelet::BasicPoint2d direction = last - prev;
        if (direction.norm() < 1e-6)
        {
            // Degenerate direction from the last segment; fall back to the vector from the first to last point
            direction = last - centerline.front();
        }
        direction.normalize();

        constexpr double step = 1.0; // meters between synthetic points, similar to typical map point spacing
        double remaining = target_length - current_length;
        for (int step_count = 1; step_count * step < remaining; ++step_count)
        {
            centerline.push_back(last + direction * (step_count * step));
        }
        centerline.push_back(last + direction * remaining); // ensure the full requested length is covered
    }

}   // namespace waypoint_generation
}   // namespace basic_autonomy
