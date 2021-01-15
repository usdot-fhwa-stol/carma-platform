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
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include "third_party_library/spline.h"
#include <carma_wm/Geometry.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/Geometry.h>
#include "smoothing/SplineI.h"
#include "smoothing/BSpline.h"




namespace unobstructed_lanechange
{
    struct PointSpeedPair
    {
    lanelet::BasicPoint2d point;
    double speed = 0;
    };
    /**
     * \brief Class representing a curve in space defined by a set of discrete points in the specified frame
     */ 
    struct DiscreteCurve
    {
    Eigen::Isometry2d frame; // Frame which points are in
    std::vector<PointSpeedPair> points;
    };

    class UnobstructedLaneChangePlugin
    {
        public:
            
            // Default constructor for UnobstructedLaneChangePlugin class
            UnobstructedLaneChangePlugin();

            // general starting point of this node
            void run();

             // service callbacks for carma trajectory planning
            bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

            std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                double max_starting_downtrack,
                                                const carma_wm::WorldModelConstPtr& wm);
            
            lanelet::BasicLineString2d create_route_geom(double starting_downtrack, int start_lane_id, double ending_downtrack, int end_lane_id, const carma_wm::WorldModelConstPtr& wm);

            int findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path);

            bool identifyLaneChange(lanelet::routing::LaneletRelations relations, int target_id);
            lanelet::BasicLineString2d create_lanechange_route(lanelet::BasicPoint2d start, lanelet::BasicPoint2d end);

            std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
            const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state);

            int getNearestPointIndex(const std::vector<PointSpeedPair>& points,
                                               const cav_msgs::VehicleState& state);

            std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,double time_span);
        
            void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
                                            std::vector<lanelet::BasicPoint2d>* basic_points,
                                            std::vector<double>* speeds);
            
            std::vector<DiscreteCurve> compute_sub_curves(const std::vector<PointSpeedPair>& map_points);

            Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2);
            
            std::unique_ptr<smoothing::SplineI>
            compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points);

            std::vector<double> apply_speed_limits(const std::vector<double> speeds,
                                                             const std::vector<double> speed_limits);

            double get_adaptive_lookahead(double velocity);
            std::vector<double> get_lookahead_speed(const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& speeds, const double& lookahead);
            
            std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
            const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
            ros::Time startTime);
            
            carma_wm::WorldModelConstPtr wm_;

            // start vehicle speed
            double start_speed_;


            private:

            
            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;


            ros::Publisher ubobstructed_lanechange_plugin_discovery_pub_;



            // ros service servers
            ros::ServiceServer trajectory_srv_;
            ros::ServiceServer maneuver_srv_;

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            // trajectory frequency
            double traj_freq = 10;

            // ROS params
            double trajectory_time_length_ = 6;
            std::string control_plugin_name_ = "mpc_follower";
            
            // target vehicle speed
            double target_speed_;


            int num_points = traj_freq * trajectory_time_length_;

            // initialize this node
            void initialize();

            // generated trajectory plan
            cav_msgs::TrajectoryPlan trajectory_msg;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_{nullptr};
            


    
    };
}