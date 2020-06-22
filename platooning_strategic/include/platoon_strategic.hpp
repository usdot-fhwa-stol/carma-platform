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
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlanType.h>
#include <state_machine.hpp>
#include <leader_state.hpp>



namespace platoon_strategic
{
    class PlatoonStrategicPlugin
    {
        public:
            
            // Default constructor for PlatoonStrategicPlugin class
            PlatoonStrategicPlugin();

            // general starting point of this node
            void run();

            // local copy of pose
            boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg_;

            void states();

        protected:

            void run_standby();
            void run_leader();
            void run_leader_waiting();
            void run_candidate_follower();
            void run_follower();

        
        private:

            
            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;


            ros::Publisher platoon_strategic_plugin_discovery_pub_;
            ros::Subscriber pose_sub_;

            // service callbacks for carma trajectory planning
            bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

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

            PlatooningStateMachine *psm_;

    
    };
}