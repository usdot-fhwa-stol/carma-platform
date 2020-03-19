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

std::vector<int> WaypointGenerator::compute_constant_curvature_regions(
    std::vector<double> curvatures, 
    double epsilon,
    int linearity_constraint) const
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
    std::vector<int> out; for (int i = 0; i < regions.size() - 1; i++) {
        if (regions[i + 1] - regions[i] < linearity_constraint) {
            continue;
        } else {
            out.push_back(regions[i]);
        }
    }

    out.push_back(curvatures.size() - 1);

    return out;
}

std::vector<double> WaypointGenerator::normalize_curvature_regions(
    std::vector<double> curvatures, 
    std::vector<int> regions) const
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

double WaypointGenerator::compute_speed_for_curvature(double curvature, double lateral_accel_limit) const
{
    // Solve a = v^2/r (k = 1/r) for v
    // a = v^2 * k
    // a * k = v^2
    // v = sqrt(a * k)
    return std::sqrt(lateral_accel_limit * curvature);
}
std::vector<double> WaypointGenerator::compute_ideal_speeds(std::vector<double> curvatures, 
    double lateral_accel_limit) const
{
    std::vector<double> out;
    for (double k : curvatures) {
        out.push_back(compute_speed_for_curvature(k, lateral_accel_limit));
    }

    return out;
}

std::vector<double> WaypointGenerator::apply_speed_limits(
    const std::vector<double> speeds, 
    const std::vector<double> speed_limits) const
{
    if (speeds.size() != speed_limits.size()){
        throw std::invalid_argument("Speeds and speed limit lists not same size");
    }
    std::vector<double> out;
    for (int i = 0; i < speeds.size(); i++) {
        out.push_back(std::min(speeds[i], speed_limits[i]));
    }

    return out;
}

std::vector<double> WaypointGenerator::apply_accel_limits(std::vector<double> speeds, 
    std::vector<int> regions,
    lanelet::BasicLineString2d centerline,
    double accel_limit,
    double decel_limit) const
{
    if (speeds.size() != centerline.size()){
        throw std::invalid_argument("Speeds and geometry do not match");
    }

    std::vector<double> out{speeds};

    for (int i = 0; i < regions.size() - 1; i++) {
        int idx = regions[i];
        double initial_v = speeds[idx];
        double final_v = speeds[idx + 1];
        if (final_v < initial_v) {
            // Slowing down

            double delta_v = speeds[idx] - speeds[idx + 1];
            double avg_v = delta_v/2.0;
            double delta_t = delta_v/decel_limit;
            double delta_d = avg_v * delta_t;

            double dist_accum = 0;
            for (int j = idx; j >= 0; j--) {
                // Iterate until we find a point that gives us enough delta_d to
                // slow down in time
                dist_accum += carma_wm::WorldModel::compute_euclidean_distance(
                    centerline[j], 
                    centerline[j + 1]);

                if (dist_accum >= delta_d) {
                    for (int k = j; k <= idx; k++) {
                        // Linearly interpolate between speeds starting at that
                        // point
                        double dist = 
                            carma_wm::WorldModel::compute_euclidean_distance(
                                centerline[k], 
                                centerline[idx + 1]);
                        out[k] = 
                            initial_v - 
                            (1.0 - std::min(dist/dist_accum, 1.0)) *  
                            delta_v;
                    }
                }

            }
        }

        if (final_v > initial_v) {
            // Speeding up
            double delta_v = speeds[idx + 1] - speeds[idx];
            double avg_v = delta_v/2.0;
            double delta_t = delta_v/accel_limit;
            double delta_d = avg_v * delta_t;

            double dist_accum = 0;
            for (int j = idx; j < speeds.size(); j++) {
                // Iterate until we find a point that gives us enough delta_d to
                // speed up acceptably
                if (dist_accum >= delta_d) {
                    for (int k = idx; k < j; k++) {
                        // Linearly interpolate between speeds starting at that
                        // point
                        double dist = 
                            carma_wm::WorldModel::compute_euclidean_distance(
                                centerline[k], 
                                centerline[j]);
                        out[k] = 
                            initial_v + 
                            (1.0 - std::min(dist/dist_accum, 1.0)) *  
                            delta_v;
                    }
                }

                auto a = centerline[j];
                auto b = centerline[j + 1];
                dist_accum += carma_wm::WorldModel::compute_euclidean_distance(a, b);
            }
        }
    }

    return out;
}

std::vector<geometry_msgs::Quaternion> WaypointGenerator::compute_orientations(
    const std::vector<lanelet::ConstLanelet> lanelets) const
{
    std::vector<geometry_msgs::Quaternion> out;

    lanelet::BasicLineString2d centerline = carma_wm::WorldModel::concatenate_lanelets(lanelets);

    std::vector<Eigen::Vector2d> tangents = carma_wm::WorldModel::compute_finite_differences(centerline);

    Eigen::Vector2d x_axis = {1, 0};
    for (int i = 0; i < tangents.size(); i++) {
        geometry_msgs::Quaternion q;

        // Derive angle by cos theta = (u . v)/(||u| * ||v||)
        // TODO: std::acos is in radians, verify if this needs to be in degrees
        double yaw = std::acos(tangents[i].dot(x_axis)/(tangents[i].norm() * x_axis.norm()));

        q = tf::createQuaternionMsgFromYaw(yaw);
    }

    return out;
}

autoware_msgs::LaneArray WaypointGenerator::generate_lane_array_message(
    std::vector<double> speeds, 
    std::vector<geometry_msgs::Quaternion> orientations, 
    std::vector<lanelet::ConstLanelet> lanelets) const
{
    autoware_msgs::LaneArray out;

    int centerline_point_idx = 0;
    for (int i = 0; i < lanelets.size(); i++) {
        autoware_msgs::Lane lane;
        lane.lane_id = i;

        std_msgs::Header header;
        header.frame_id = "/map";
        header.seq = 0;
        header.stamp = ros::Time::now();
        lane.header = header;

        std::vector<autoware_msgs::Waypoint> waypoints;
        for (int j = 0; j < lanelets[j].centerline3d().size(); j++) {
            autoware_msgs::Waypoint wp;
            wp.lane_id = i;
            
            geometry_msgs::Pose p;
            p.position.x = lanelets[i].centerline3d()[j].x();
            p.position.y = lanelets[i].centerline3d()[j].y();
            p.position.z = lanelets[i].centerline3d()[j].z();
            p.orientation = orientations[centerline_point_idx];

            geometry_msgs::Twist t;
            t.linear.x = speeds[centerline_point_idx]; // Vehicle's forward velocity corresponds to x

            wp.change_flag = 0; 
            wp.direction = 0;
            wp.cost = 0;
            wp.time_cost = 0;

            /* 
            // left undefined until we understand what we need
            wp.wpstate;
            wp.dtlane;
            wp.gid;
            wp.lid;
            wp.right_lane_id;
            wp.stop_line_id;
            wp.lid;
            */

            waypoints.push_back(wp);
            centerline_point_idx++;
        }

        lane.waypoints = waypoints;
    }

}
std::vector<double> WaypointGenerator::get_speed_limits(
    std::vector<lanelet::ConstLanelet> lanelets) const
{
    std::vector<double> out;
    for (int i = 0; i < lanelets.size(); i++) {
        lanelet::traffic_rules::SpeedLimitInformation sli = 
            _wm->getTrafficRules()->get()->speedLimit(lanelets[i]);

        for (int j = 0; j < lanelets[i].centerline2d().size(); j++) {
            out.push_back(sli.speedLimit.value());
        }
    }
}
};