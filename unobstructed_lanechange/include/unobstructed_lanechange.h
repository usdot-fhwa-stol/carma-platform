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



namespace unobstructed_lanechange
{
    class UnobstructedLaneChangePlugin
    {
        public:
            
            // Default constructor for UnobstructedLaneChangePlugin class
            UnobstructedLaneChangePlugin();

            // general starting point of this node
            void run();

            // postprocess traj to add plugin names and shift time origin to the current ROS time
            std::vector<cav_msgs::TrajectoryPlanPoint> post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory);

            /**
                * \brief Given start and end points, create a continous lanechange trajectory
                * \param start start point of lanechange
                * \param end end point of lanechange
                * \return vector of trajectory points
            */
            std::vector<cav_msgs::TrajectoryPlanPoint> create_lanechange_trajectory(std::vector<double> start, std::vector<double> end);

            /**
                * \brief Compose lane change trajectory given its input parameters
                * \param start_id Lanechange starting point lanelet id
                * \param start_downtrack Lanechange starting point downtrack
                * \param end_id Lanechange ending point lanelet id
                * \param end_downtrack Lanechange ending point downtrack
                * \return vector of trajectory points
            */
            std::vector<cav_msgs::TrajectoryPlanPoint> compose_lanechange_trajectory(carma_wm::WorldModelConstPtr wm, const std::string& start_id, double start_downtrack, const std::string& end_id, double end_downtrack);

            /**
                * \brief Given Lanelet id and downtrack, find the corresponding point coordinates on the lanelet's centerline
                * \param lanelet_id Lanelet ID
                * \param downtrack downtrack value
                * \return Vector of x and y coordinates
            */
            std::vector<double> extract_point_from_lanelet(carma_wm::WorldModelConstPtr wm, const std::string& lanelet_id, double downtrack);
        
        private:

            
            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;


            ros::Publisher ubobstructed_lanechange_plugin_discovery_pub_;

            // service callbacks for carma trajectory planning
            bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

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
            

            // start vehicle speed
            double start_speed_;
            // target vehicle speed
            double target_speed_;


            int num_points = traj_freq * trajectory_time_length_;

            // initialize this node
            void initialize();

            // generated trajectory plan
            cav_msgs::TrajectoryPlan trajectory_msg;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> _wml{nullptr};
            carma_wm::WorldModelConstPtr _wm;


    
    };
}