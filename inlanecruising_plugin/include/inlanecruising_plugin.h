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

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <autoware_msgs/Lane.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>


namespace inlanecruising_plugin
{
    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> Boost2DPoint;
    typedef std::pair<Boost2DPoint, size_t> PointIndexPair;
    typedef boost::geometry::index::rtree< PointIndexPair, boost::geometry::index::quadratic<16> > Point2DRTree;

    class InLaneCruisingPlugin
    {

    public:

        InLaneCruisingPlugin();

        // general starting point of this node
        void run();

        // create uneven trajectory from waypoints
        std::vector<cav_msgs::TrajectoryPlanPoint> create_uneven_trajectory_from_waypoints(const std::vector<autoware_msgs::Waypoint>& waypoints);

        // get a sublist of waypoints marked by desired time span
        std::vector<autoware_msgs::Waypoint> get_waypoints_in_time_boundary(const std::vector<autoware_msgs::Waypoint>& waypoints, double time_span);

        // postprocess traj to add plugin names and shift time origin to the current ROS time
        std::vector<cav_msgs::TrajectoryPlanPoint> post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory);

        // set waypoints and construct rtee
        // returns Point2DRTree for unit test purposes
        Point2DRTree set_waypoints(const std::vector<autoware_msgs::Waypoint>& waypoints);

        // local copy of pose
        boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg_;
        
    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;


        ros::Publisher inlanecruising_plugin_discovery_pub_;
        ros::Publisher base_waypoints_pub_;

        // subscriber for Autoware waypoints
        ros::Subscriber waypoints_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;

        // ros service servers
        ros::ServiceServer trajectory_srv_;
        ros::ServiceServer maneuver_srv_;

        // service callbacks for carma trajectory planning
        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

        // generated trajectory plan
        cav_msgs::TrajectoryPlan trajectory_msg;
        // Array of waypoints
        std::vector<autoware_msgs::Waypoint> waypoints_list;

        Point2DRTree rtree;

        // Length of maneuver
        double mvr_length = 16;

        size_t mpc_back_waypoints_num_ = 20; // Double the number of points expected by mpc converter to account for errors in our approach

        // Plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        // ROS params
        double trajectory_time_length_;
        double trajectory_point_spacing_;

        // current vehicle speed
        double current_speed_;

        // initialize this node
        void initialize();

        // callback for the subscribers
        void waypoints_cb(const autoware_msgs::LaneConstPtr& msg);
        void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);

        // convert waypoints to a trajectory
        std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_waypoints(const std::vector<autoware_msgs::Waypoint>& waypoints);

    };

}

