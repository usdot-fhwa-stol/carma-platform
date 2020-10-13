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
#include <waypoint_generator/waypoint_generator_config.hpp>
#include <carma_wm/Geometry.h>

namespace waypoint_generator
{

    void WaypointGeneratorNode::publishWaypoints(const autoware_msgs::LaneArray& msg) {
        _waypoints_pub.publish(msg);
    }

    void WaypointGeneratorNode::initialize()
    {
        ros::init(argc, argv, node_name);
        _nh.reset(new ros::CARMANodeHandle());
        _pnh.reset(new ros::CARMANodeHandle("~"));
        _wml.reset(new carma_wm::WMListener());
        _wm = _wml->getWorldModel();
        
        ROS_DEBUG("Initialized all node handles");

        WaypointGeneratorConfig config;
        _pnh->param<double>("curvature_epsilon", config._curvature_epsilon, config._curvature_epsilon);
        _pnh->param<int>("linearity_constraint", config._linearity_constraint, config._linearity_constraint);
        _pnh->param<double>("lateral_accel_limit", config._lateral_accel_limit, config._lateral_accel_limit);
        _pnh->param<double>("longitudinal_accel_limit", config._longitudinal_accel_limit, config._longitudinal_accel_limit);
        _pnh->param<double>("longitudinal_decel_limit", config._longitudinal_decel_limit, config._longitudinal_decel_limit);
        _pnh->param<int>("downsample_ratio", config._downsample_ratio, config._downsample_ratio);
        ROS_DEBUG_STREAM("Parameters loaded!" << std::endl
            << "curvature_epsilon: " << config._curvature_epsilon << std::endl
            << "linearity_constraint" << config._linearity_constraint << std::endl
            << "downsample_ratio" << config._downsample_ratio << std::endl
            << "lateral_accel_limit: " << config._lateral_accel_limit << std::endl
            << "longitudinal_accel_limit: " << config._longitudinal_accel_limit << std::endl
            << "longitudinal_decel_limit: " << config._longitudinal_decel_limit << std::endl);

        _wpg.reset(new WaypointGenerator(_wm, config, std::bind(&WaypointGeneratorNode::publishWaypoints, this, std::placeholders::_1)));
    }

    void WaypointGeneratorNode::run()
    {
        ROS_DEBUG("Beginning execution of waypoint generator node.");

        _waypoints_pub = _nh->advertise<autoware_msgs::LaneArray>("carma_waypoints", 5);

        std::function<void()> cb = std::bind(&WaypointGenerator::new_route_callback, _wpg);
        _wml->setRouteCallback(cb);

        ROS_DEBUG("Subscribers and publishers initialized.");

        ROS_DEBUG("Beginning spin for waypoint generator node.");
        _nh->spin();
        ROS_DEBUG("Waypoint generator node shutting down.");
    }
}