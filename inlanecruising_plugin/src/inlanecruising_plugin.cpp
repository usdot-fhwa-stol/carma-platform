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

#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/optional/optional.hpp>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <lanelet2_core/geometry/Point.h>
#include "inlanecruising_plugin.h"
#include "calculation.cpp"


namespace inlanecruising_plugin
{
    using PointSpeedPair = std::pair<BasicPoint2d, double>;
    InLaneCruisingPlugin::InLaneCruisingPlugin() :
                    current_speed_(0.0),
                    trajectory_time_length_(6.0),
                    trajectory_point_spacing_(0.1),
                    smooth_accel_(0.5) {}

    void InLaneCruisingPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/InLaneCruisingPlugin/plan_trajectory", &InLaneCruisingPlugin::plan_trajectory_cb, this);
        base_waypoints_pub_ = nh_->advertise<autoware_msgs::Lane>("plugin_base_waypoints", 1);                

        inlanecruising_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "InLaneCruisingPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";

        waypoints_sub_ = nh_->subscribe("final_waypoints", 1, &InLaneCruisingPlugin::waypoints_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 1, &InLaneCruisingPlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &InLaneCruisingPlugin::twist_cd, this);
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<double>("trajectory_point_spacing", trajectory_point_spacing_, 0.1);

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            inlanecruising_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });
    }


    void InLaneCruisingPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }



    bool InLaneCruisingPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){


        auto points_and_target_speeds = maneuvers_to_points(req.maneuver_plan.maneuvers);
        auto downsampled_points = downsample_points(points_and_target_speeds, 8); // TODO make config param
        
        ROS_WARN_STREAM("PlanTrajectory");
        cav_msgs::TrajectoryPlan trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
        ROS_WARN_STREAM("1");
        trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points, req.vehicle_state);
        ROS_WARN_STREAM("2");
        trajectory_msg = trajectory;

        resp.trajectory_plan = trajectory_msg;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_FOLLOWING);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        ROS_WARN_STREAM("3");

        return true;
    }


    std::vector<lanelet::BasicPoint2d> waypointsToBasicPoints(const std::vector<autoware_msgs::Waypoint>& waypoints) {
        std::vector<lanelet::BasicPoint2d> basic_points;
        for (auto wp : waypoints) {
            lanelet::BasicPoint2d pt(wp.pose.pose.position.x, wp.pose.pose.position.y);
            basic_points.push_back(pt);
        }

        return basic_points;
    }

    double compute_speed_for_curvature(double curvature, double lateral_accel_limit)
    {
        // Check at compile time for infinity availability
        static_assert(std::numeric_limits<double>::has_infinity, "This code requires compilation using a system that supports IEEE 754 for access to positive infinity values");

        // Solve a = v^2/r (k = 1/r) for v
        // a = v^2 * k
        // a / k = v^2
        // v = sqrt(a / k)
        
        if (fabs(curvature) < 0.00000001) { // Check for curvature of 0.
            return std::numeric_limits<double>::infinity();
        }
        return std::sqrt(fabs(lateral_accel_limit / curvature));
    }
    std::vector<double> compute_ideal_speeds(std::vector<double> curvatures,
                                                                double lateral_accel_limit)
    {
        std::vector<double> out;
        for (double k : curvatures)
        {
            out.push_back(compute_speed_for_curvature(k, lateral_accel_limit));
        }

        return out;
    }

    std::vector<double> apply_speed_limits(const std::vector<double> speeds,
                                                            const std::vector<double> speed_limits)
    {
        ROS_ERROR_STREAM("Speeds list size: " << speeds.size());
        ROS_ERROR_STREAM("SpeedLimits list size: " << speed_limits.size());
        if (speeds.size() != speed_limits.size())
        {
            throw std::invalid_argument("Speeds and speed limit lists not same size");
        }
        std::vector<double> out;
        for (int i = 0; i < speeds.size(); i++)
        {
            out.push_back(std::min(speeds[i], speed_limits[i]));
        }

        return out;
    }

    std::vector<double> compute_downtracks(std::vector<lanelet::BasicPoint2d> basic_points) {
        std::vector<double> downtracks;
        downtracks.reserve(basic_points.size());
        double current_dt = 0;
        boost::optional<lanelet::BasicPoint2d> prev_p;
        for (const auto& p : basic_points) {
            if (!prev_p) {
                downtracks.push_back(0);
                continue;
            }
            double dx = p.x() - prev_p->x();
            double dy = p.y() - prev_p->y();
            double dist = sqrt(dx * dx + dy * dy);
            current_dt += dist;
            downtracks.push_back(current_dt);
            prev_p = p;
        }

        return downtracks;
    }

    class DiscreteCurve {
        public: 
            tf2::Transform frame;
            std::vector<PointSpeedPair> points;
    };

    tf2::Transform compute_heading_frame(const tf2::Vector3& p1, const tf2::Vector3& p2) {
        tf2::Matrix3x3 rot_mat = tf2::Matrix3x3::getIdentity();

        double yaw = atan2(p2.y() - p1.y(), p2.x() - p1.x());

        rot_mat.setRPY(0, 0, yaw);
        tf2::Vector3 position(p1.x(), p1.y(), 0);
        tf2::Transform frame(rot_mat, position);
        return frame;
    }

    tf2::Vector3 point2DToTF2Vec(const lanelet::BasicPoint2d& p) {
        return tf2::Vector3(p.x(), p.y(), 0);
    }

    lanelet::BasicPoint2d tf2VecToPoint2D(const tf2::Vector3& p) {
        return lanelet::BasicPoint2d(p.x(), p.y());
    }

    bool transformExactMatch(const tf2::Transform& t1, const tf2::Transform& t2) {
        return t1.getRotation().x() == t2.getRotation().x() && 
        t1.getRotation().y() == t2.getRotation().y() && 
        t1.getRotation().z() == t2.getRotation().z() && 
        t1.getRotation().w() == t2.getRotation().w() && 
        t1.getOrigin().x() == t2.getOrigin().x() && 
        t1.getOrigin().y() == t2.getOrigin().y() && 
        t1.getOrigin().z() == t2.getOrigin().z(); 
    }

    std::vector<DiscreteCurve> compute_sub_curves(const std::vector<PointSpeedPair>& basic_points) {
        if (basic_points.size() < 2) {
            throw std::invalid_argument("Not enough points");
        }

        bool x_going_positive = true; // Since we define the frame to be positive x along line this always starts as true

        std::vector<DiscreteCurve> curves;
        DiscreteCurve curve;
        curve.frame = compute_heading_frame(point2DToTF2Vec(std::get<0>(basic_points[0])), point2DToTF2Vec(std::get<0>(basic_points[1])));
        tf2::Transform map_in_curve = curve.frame.inverse();



        for (int i = 0; i < basic_points.size() - 1; i++) {
            tf2::Vector3 p1 = map_in_curve * point2DToTF2Vec(std::get<0>(basic_points[i]));
            tf2::Vector3 p2 = map_in_curve * point2DToTF2Vec(std::get<0>(basic_points[i+1])); // TODO Optimization to cache this value
            
            curve.points.push_back(tf2VecToPoint2D(p1));

            bool x_dir = (p2.x() - p1.x()) >= 0;
            if (x_going_positive != x_dir) { // TODO this check could be simplified to (!x_dir)
                // New Curve
                curves.push_back(curve);

                curve = DiscreteCurve();
                curve.frame = compute_heading_frame(p1, p2);
                map_in_curve = curve.frame.inverse();
                curve.points.push_back(tf2VecToPoint2D(p1)); // Include first point in curve
                x_going_positive = true; // Reset to true because we are using a new frame
            }
        }

        if (curves.size() == 0 || (!transformExactMatch(curves.back().frame, curve.frame))) {
            curves.push_back(curve);
        }

        return curves;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {
        std::vector<cav_msgs::TrajectoryPlanPoint> final_trajectory;
        int nearest_pt_index = getNearestPointIndex(points, state);


        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end());        

        ROS_WARN("Got basic points ");
        std::vector<DiscreteCurve> sub_curves = compute_sub_curves(future_points);

        ROS_WARN_STREAM("Got sub_curves " << sub_curves.size());

        std::vector<tf2::Quaternion> final_yaw_values;
        std::vector<double> final_actual_speeds;
        std::vector<lanelet::BasicPoint2d> all_sampling_points;

        for (const auto& discreet_curve : sub_curves) {
            ROS_WARN("SubCurve");
            boost::optional<tk::spline> fit_curve = compute_fit(discreet_curve.points); // Returned data type TBD

            if (!fit_curve) { // TODO how better to handle this case
                for (auto p : discreet_curve.points) {
                    final_yaw_values.push_back(final_yaw_values.back());
                    final_actual_speeds.push_back(final_actual_speeds.back());
                }
                continue;
            }

            ROS_WARN("Got fit");

            std::vector<lanelet::BasicPoint2d> sampling_points;
            sampling_points.reserve(1 + discreet_curve.points.size() * 2);

            double totalDist = 0;
            bool firstLoop = true;
            lanelet::BasicPoint2d prev_point(0.0, 0.0);
            for (auto p : discreet_curve.points) {
                if (firstLoop) {
                    prev_point = p; 
                    firstLoop = false;
                    continue;
                }
                
                totalDist += lanelet::geometry::distance2d(prev_point, p);
            }

            double current_dist = 0;
            double step_size = 1; // TODO make parameter
            tk::spline actual_fit_curve = fit_curve.get();
            while(current_dist < totalDist - step_size) {
                double x = current_dist;
                double y = actual_fit_curve(x);
                lanelet::BasicPoint2d p(x, y);
                sampling_points.push_back(p);
                current_dist += step_size;
            }

             ROS_WARN("Sampled points");
            std::vector<double> yaw_values = compute_orientation_from_fit(actual_fit_curve, sampling_points);

             ROS_WARN("Got yaw");
            std::vector<double> curvatures = compute_curvature_from_fit(actual_fit_curve, sampling_points);
            for (auto c : curvatures) {
                ROS_WARN_STREAM("curvatures[i]: " << c);
            }

            ROS_WARN("Got curvatures");
            std::vector<double> speed_limits;
            std::vector<lanelet::BasicPoint2d> basic_points;
            splitPointSpeedPairs(discreet_curve.points, &basic_points, &speed_limits);

            ROS_WARN("Got speeds limits");
            std::vector<double> ideal_speeds = compute_ideal_speeds(curvatures, 1.5);
            ROS_WARN("Got ideal limits");
            std::vector<double> actual_speeds = apply_speed_limits(ideal_speeds, speed_limits);
            ROS_WARN("Got actual");

            for (int i = 0; i < yaw_values.size() - 1; i++) { // Drop last point
                double yaw = yaw_values[i];
                ROS_WARN_STREAM("yaw_values[i]: " << yaw_values[i]);
                tf2::Matrix3x3 rot_mat = tf2::Matrix3x3::getIdentity();
                rot_mat.setRPY(0, 0, yaw);
                tf2::Transform c_to_yaw(rot_mat); // NOTE: I'm pretty certain the origin does not matter here but unit test to confirm
                tf2::Transform m_to_yaw = discreet_curve.frame * c_to_yaw;
                final_yaw_values.push_back(m_to_yaw.getRotation());

                tf2::Vector3 vec = point2DToTF2Vec(sampling_points[i]);
                tf2::Vector3 map_frame_vec = discreet_curve.frame * vec;
                all_sampling_points.push_back(tf2VecToPoint2D(map_frame_vec));

            }

            ROS_WARN("Converted yaw to quat");

            final_actual_speeds.insert(final_actual_speeds.end(), actual_speeds.begin(), actual_speeds.end() - 1);

            ROS_WARN("Appended to final");
        }

        ROS_WARN("Processed all curves");


        int i = 0;
        for (auto speed : final_actual_speeds) {
            if (i == final_actual_speeds.size() - 1) {
                break; // TODO rework loop at final yaw and speed arrays should be 1 less element than original waypoint set
            }
            ROS_WARN_STREAM("final_actual_speeds[i]: " << final_actual_speeds[i]);
            ROS_WARN_STREAM("final_yaw_values[i]: " << final_yaw_values[i]);
            i++;
        }


        
        std::vector<cav_msgs::TrajectoryPlanPoint> uneven_traj = create_uneven_trajectory_from_points(all_sampling_points, final_actual_speeds, final_yaw_values);
        final_trajectory = post_process_traj_points(uneven_traj);


        return final_trajectory;
    }

// TODO comments: Takes in a waypoint list that is from the next waypoint till the time boundary
    std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::create_uneven_trajectory_from_points(const std::vector<lanelet::BasicPoint2d>& points,
    const std::vector<double>& speeds, const std::vector<tf2::Quaternion>& orientations, const cav_msgs::VehicleState& state)
    {
        std::vector<cav_msgs::TrajectoryPlanPoint> uneven_traj;
        // TODO land id is not populated because we are not using it in Autoware
        // Adding current vehicle location as the first trajectory point if it is not on the first waypoint
        // TODO there is an equivalent loop to this in the platooning plugin that should also be updated to assign the orientation value
        // Add vehicle location as first point
        cav_msgs::TrajectoryPlanPoint starting_point;
        starting_point.target_time = ros::Time(0.0);
        starting_point.x = state.X_pos_global;
        starting_point.y = state.Y_pos_global;
        starting_point.yaw = state.orientation;
        uneven_traj.push_back(starting_point);

        if (points.size() == 0) {
            ROS_ERROR_STREAM("Trying to create uneven trajectory from 0 points");
            return uneven_traj;
        }
        // Update previous wp
        double previous_wp_v = current_speed_;
        double previous_wp_x = starting_point.x;
        double previous_wp_y = starting_point.y;
        double previous_wp_yaw = starting_point.yaw;
        ros::Time previous_wp_t = starting_point.target_time;

        ROS_WARN_STREAM("previous_wp_v" << previous_wp_v);

        for(int i = 0; i < points.size(); i++)
        {


            cav_msgs::TrajectoryPlanPoint traj_point;
            // assume the vehicle is starting from stationary state because it is the same assumption made by pure pursuit wrapper node
            double average_speed = std::max(previous_wp_v, 1.2352); // TODO need better solution for this
            double delta_d = sqrt(pow(points[i].x() - previous_wp_x, 2) + pow(points[i].y() - previous_wp_y, 2));
            if (speeds[i] > previous_wp_v){
                average_speed = sqrt(previous_wp_v*previous_wp_v + 2*smooth_accel_*delta_d);
            }
            if (speeds[i] < previous_wp_v){
                average_speed = sqrt(previous_wp_v*previous_wp_v - 2*smooth_accel_*delta_d);
            }
            ros::Duration delta_t(delta_d / average_speed);
            traj_point.target_time = previous_wp_t + delta_t;
            traj_point.x = points[i].x();
            traj_point.y = points[i].y();
            double roll,pitch,yaw;
            carma_wm::geometry::rpyFromQuaternion(orientations[i], roll, pitch, yaw);
            traj_point.yaw = yaw;
            uneven_traj.push_back(traj_point);

            previous_wp_v = std::min(average_speed, speeds[i]);
            previous_wp_x = uneven_traj.back().x;
            previous_wp_y = uneven_traj.back().y;
            previous_wp_y = uneven_traj.back().y;
            previous_wp_t = uneven_traj.back().target_time;
        }

        return uneven_traj;
    }

    // TODO comments: This method takes in all waypoints from the nearest waypoint + 1 and returns all waypoints in that set that fit within the time boundary
    std::vector<autoware_msgs::Waypoint> InLaneCruisingPlugin::get_waypoints_in_time_boundary(const std::vector<autoware_msgs::Waypoint>& waypoints, double time_span)
    {
        // Find nearest waypoint
        ROS_WARN_STREAM("15");
        std::vector<autoware_msgs::Waypoint> sublist;

        double total_time = 0.0;
        for(int i = 0; i < waypoints.size(); ++i) // 
        {
            sublist.push_back(waypoints[i]);
            if(i == 0)
            {
                ROS_WARN_STREAM("21");
                continue;
            }
            ROS_WARN_STREAM("20");
            double delta_x_square = pow(waypoints[i].pose.pose.position.x - waypoints[i - 1].pose.pose.position.x, 2);
            double delta_y_square = pow(waypoints[i].pose.pose.position.y - waypoints[i - 1].pose.pose.position.y, 2);
            //double delta_z_square = waypoints[i].pose.pose.position.z - waypoints[i - 1].pose.pose.position.z;
            // Here we ignore z attribute because it is not used by Autoware
            double delta_d = sqrt(delta_x_square + delta_y_square);
            double average_v = 0.5 * (waypoints[i].twist.twist.linear.x + waypoints[i - 1].twist.twist.linear.x);
            double delta_t = delta_d / average_v;
            total_time += delta_t;
            if(total_time >= time_span)
            {
                break;
            }
        }
        return sublist;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory)
    {
        ros::Time now = ros::Time::now();
        ros::Duration now_duration(now.sec, now.nsec);
        for(int i = 0; i < trajectory.size(); i++)
        {
            trajectory[i].controller_plugin_name = "default";
            trajectory[i].planner_plugin_name = "autoware";
            trajectory[i].target_time += now_duration;
        }

        return trajectory;
    }

    boost::optional<tk::spline> InLaneCruisingPlugin::compute_fit(std::vector<lanelet::BasicPoint2d> basic_points){
        if (basic_points.size()<3){
            ROS_WARN_STREAM("Insufficient Spline Points");
            return boost::none;
        }
        

        tk::spline spl;
        std::vector<double> points_x;
        std::vector<double> points_y;

        for (size_t i=0; i<basic_points.size(); i++){
            ROS_WARN_STREAM("basic_points[i]: " << basic_points[i].x() << ", " << basic_points[i].y());
            points_x.push_back(basic_points[i].x());
            points_y.push_back(basic_points[i].y());
        }

        spl.set_points(points_x, points_y);

        return spl;

    }

    std::vector<double> InLaneCruisingPlugin::compute_orientation_from_fit(tk::spline curve, std::vector<lanelet::BasicPoint2d> sampling_points){
        std::vector<double> orientations;
        std::vector<double> cur_point{0.0, 0.0};
        std::vector<double> next_point{0.0, 0.0};
        double lookahead = 0.3;
        for (size_t i=0; i<sampling_points.size() - 1; i++){
            cur_point[0] = sampling_points[i].x();
            cur_point[1] = sampling_points[i].y();
            next_point[0] = sampling_points[i+1].x();
            next_point[1] = sampling_points[i+1].y();
            double res = calculate_yaw(cur_point, next_point);
            orientations.push_back(res);

        }
        orientations.push_back(orientations.back());
        return orientations;
    }

    std::vector<double> InLaneCruisingPlugin::compute_curvature_from_fit(tk::spline curve, std::vector<lanelet::BasicPoint2d> sampling_points){
        std::vector<double> curvatures;
        std::vector<double> cur_point{0.0, 0.0};
        std::vector<double> next_point{0.0, 0.0};
        double lookahead = 0.3;
        ROS_WARN_STREAM("Computing Curvatures");
        for (size_t i=0; i<sampling_points.size() - 1; i++) {
            cur_point[0] = sampling_points[i].x();
            cur_point[1] = sampling_points[i].y();
            next_point[0] = sampling_points[i+1].x();
            next_point[1] = sampling_points[i+1].y();
            double cur = calculate_curvature(cur_point, next_point);
            curvatures.push_back(fabs(cur)); // TODO now using abs think in more detail if this will cause issues

        }
        curvatures.push_back(curvatures.back());
        return curvatures;
    }

    double InLaneCruisingPlugin::calculate_yaw(std::vector<double> cur_point, std::vector<double> next_point){
        double dx = next_point[0] - cur_point[0];
        double dy = next_point[1] - cur_point[1];
        double yaw = atan2 (dy, dx);
        return yaw;

    }

    double InLaneCruisingPlugin::calculate_curvature(std::vector<double> cur_point, std::vector<double> next_point){
        double dist = sqrt(pow(cur_point[0] - next_point[0], 2) + pow(cur_point[1] - next_point[0], 2));

        double angle = calculate_yaw(cur_point, next_point);

        double r = 0.5*(dist/std::sin(angle));

        double max_curvature = 100000;
        double curvature = std::min(1/r, max_curvature);

        return curvature;
    }



    // compute_fit(points);
    // compute_orientation_from_fit(curve, sampling_points)
}
