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

#include <waypoint_generator/waypoint_generator_node.hpp>
#include <waypoint_generator/waypoint_generator.hpp>
#include <carma_wm/Geometry.h>

namespace waypoint_generator
{
    void WaypointGeneratorNode::initialize()
    {
        ros::init(argc, argv, node_name);
        _nh.reset(new ros::CARMANodeHandle());
        _pnh.reset(new ros::CARMANodeHandle("~"));
        _wml.reset(new carma_wm::WMListener());
        _wm = _wml->getWorldModel();
        _wpg.setWorldModel(_wm);
        
        ROS_DEBUG("Initialized all node handles");

        _pnh->param<double>("curvature_epsilon", _curvature_epsilon, 3.0);
        _pnh->param<int>("linearity_constraint", _linearity_constraint, 2);
        _pnh->param<double>("lateral_accel_limit", _lateral_accel_limit, 1.5);
        _pnh->param<double>("longitudinal_accel_limit", _longitudinal_accel_limit, 1.5);
        _pnh->param<double>("longitudinal_decel_limit", _longitudinal_decel_limit, 1.5);
        ROS_DEBUG_STREAM("Parameters loaded!" << std::endl
            << "curvature_epsilon: " << _curvature_epsilon << std::endl
            << "linearity_constraint" << _linearity_constraint << std::endl
            << "lateral_accel_limit: " << _lateral_accel_limit << std::endl
            << "longitudinal_accel_limit: " << _longitudinal_accel_limit << std::endl
            << "longitudinal_decel_limit: " << _longitudinal_decel_limit << std::endl);
    }

    void WaypointGeneratorNode::run()
    {
        ROS_DEBUG("Beginning execution of waypoint generator node.");

        _waypoints_pub = _nh->advertise<autoware_msgs::LaneArray>("carma_waypoints", 5);

        std::function<void()> cb = std::bind(&WaypointGeneratorNode::new_route_callback, this);
        _wml->setRouteCallback(cb);

        ROS_DEBUG("Subscribers and publishers initialized.");

        ROS_DEBUG("Beginning spin for waypoint generator node.");
        _nh->spin();
        ROS_DEBUG("Waypoint generator node shutting down.");
    }

    void WaypointGeneratorNode::new_route_callback() {
        ROS_DEBUG_STREAM("Received new route message, processing...");
        
        auto shortest_path = _wm->getRoute()->shortestPath();
        ROS_DEBUG_STREAM("Processing " << shortest_path.size() << " lanelets.");
        std::vector<lanelet::ConstLanelet> tmp;
        for (lanelet::ConstLanelet l : shortest_path) {
            tmp.push_back(l);
        }

        lanelet::BasicLineString2d route_geometry = 
            carma_wm::geometry::concatenate_lanelets(tmp);

        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("concatenate_lanelets resulted in data: ");

        for (auto p : route_geometry) {
            ROS_DEBUG_STREAM(" Point: ( " << p.x() << ", " << p.y() << " )");
        }

        ROS_DEBUG("Processing curvatures...");
        std::vector<double> curvatures = 
            carma_wm::geometry::getLocalCurvatures(tmp);

        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Computed Curvatures");

        for (auto p : curvatures) {
            ROS_DEBUG_STREAM(" Curvature: " << p);
        }


        std::vector<int> constant_curvature_regions = 
            _wpg.compute_constant_curvature_regions(
                curvatures, 
                _curvature_epsilon, 
                _linearity_constraint);

        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Curvature Regions");

        for (auto p : constant_curvature_regions) {
            ROS_DEBUG_STREAM(" Region: " << p);
        }

        ROS_DEBUG("Normalizing curvatures...");

        std::vector<double> processed_curvatures = 
            _wpg.normalize_curvature_regions(
                curvatures, 
                constant_curvature_regions);

        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Normalized Curvatures");

        for (auto p : processed_curvatures) {
            ROS_DEBUG_STREAM(" Curvature: " << p);
        }

        ROS_DEBUG("Processing speeds...");
        std::vector<double> ideal_speeds = _wpg.compute_ideal_speeds(
            processed_curvatures, 
            _lateral_accel_limit);

        
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Ideal Speeds");

        for (auto p : ideal_speeds) {
            ROS_DEBUG_STREAM("IdealSpeed: " << p);
        }

        std::vector<double> accel_limited_speeds = _wpg.apply_accel_limits(
            ideal_speeds, 
            constant_curvature_regions, 
            route_geometry,
            _longitudinal_accel_limit,
            _longitudinal_decel_limit);
        
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Accel Limited Speeds");

        for (auto p : accel_limited_speeds) {
            ROS_DEBUG_STREAM(" Speed: " << p);
        }

        std::vector<double> speed_limits = _wpg.get_speed_limits(tmp);


        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Speed Limits");

        for (auto p : speed_limits) {
            ROS_DEBUG_STREAM(" SpeedLimits: " << p);
        }

        std::vector<double> final_speeds = _wpg.apply_speed_limits(
            accel_limited_speeds, 
            speed_limits);

        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Final Speeds");

        for (auto p : final_speeds) {
            ROS_DEBUG_STREAM(" Final Speed: " << p);
        }

        ROS_DEBUG("Processing orientations...");
        std::vector<geometry_msgs::Quaternion> orientations = 
            _wpg.compute_orientations(tmp);

        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM("Orientations");

        for (auto p : orientations) {
            ROS_DEBUG_STREAM(" Orientation: ( " << p.x << ", " << p.y << ", " << p.z << ", " << p.w  << " )");
        }


        // Update current waypoints
        ROS_DEBUG("Generating final waypoint message.");
        _cur_waypoints = _wpg.generate_lane_array_message(
            final_speeds,
            orientations,
            tmp);

        ROS_DEBUG_STREAM("Finished processing route.");

        _waypoints_pub.publish(_cur_waypoints);
        ROS_DEBUG_STREAM("Published waypoints list!");
    }
}