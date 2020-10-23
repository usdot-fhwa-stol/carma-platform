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
#include <tf2/LinearMath/Matrix3x3.h> // TODO it should be possible to use tf2_eigen instead which would be more efficient since lanelet2 uses eigen under the hood
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_wm/WMListener.h>

#include "third_party_library/spline.h"

namespace inlanecruising_plugin
{
    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> Boost2DPoint;
    typedef std::pair<Boost2DPoint, size_t> PointIndexPair;
    typedef boost::geometry::index::rtree< PointIndexPair, boost::geometry::index::quadratic<16> > Point2DRTree;
    using PointSpeedPair = std::pair<lanelet::BasicPoint2d, double>;

    class InLaneCruisingPlugin
    {

    public:

        InLaneCruisingPlugin();

        // general starting point of this node
        void run();

        // create uneven trajectory from waypoints
        std::vector<cav_msgs::TrajectoryPlanPoint> create_uneven_trajectory_from_points(const std::vector<lanelet::BasicPoint2d>& points,
            const std::vector<double>& speeds, const std::vector<tf2::Quaternion>& orientations, const cav_msgs::VehicleState& state);

        // postprocess traj to add plugin names and shift time origin to the current ROS time
        std::vector<cav_msgs::TrajectoryPlanPoint> post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory);

        // Fit a cubic spline to the points
        boost::optional<tk::spline> compute_fit(std::vector<lanelet::BasicPoint2d> basic_points);

        // calculate the orientations of given points on the curve
        std::vector<double> compute_orientation_from_fit(tk::spline curve, std::vector<lanelet::BasicPoint2d> sampling_points);

        std::vector<double> compute_curvature_from_fit(tk::spline curve, std::vector<lanelet::BasicPoint2d> sampling_points);

        double calculate_yaw(std::vector<double> cur_point, std::vector<double> prev_point);
        double calculate_curvature(std::vector<double> cur_point, std::vector<double> next_point);
        
    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;
        carma_wm::WorldModelConstPtr wm_;
        std::shared_ptr<carma_wm::WMListener> wml_;


        ros::Publisher inlanecruising_plugin_discovery_pub_;
        ros::Publisher base_waypoints_pub_;

        // ros service servers
        ros::ServiceServer trajectory_srv_;
        ros::ServiceServer maneuver_srv_;

        // service callbacks for carma trajectory planning
        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

        // generated trajectory plan
        cav_msgs::TrajectoryPlan trajectory_msg;

        // Plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        // ROS params
        double trajectory_time_length_;
        double trajectory_point_spacing_;
        double smooth_accel_;

        // initialize this node
        void initialize();

        // convert waypoints to a trajectory
        std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

    };

}

