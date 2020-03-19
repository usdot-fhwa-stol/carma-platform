/*
 * Copyright (C) 2018-2020 LEIDOS.
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


// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// msgs
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/TrajectoryPlan.h>

// autoware
#include "autoware_msgs/Lane.h"
#include "autoware_config_msgs/ConfigWaypointFollower.h"
#include "autoware_msgs/ControlCommandStamped.h"

#include <carma_utils/CARMAUtils.h>

#include "mpc_follower_wrapper_worker.hpp"

namespace mpc_follower_wrapper {

/*!
 * Main class for the node to handle the ROS interfacing.
 */

class MPCFollowerWrapper {
    public:
        /*!
        * Constructor.
        * @param nodeHandle the ROS node handle.
        */
        MPCFollowerWrapper(ros::CARMANodeHandle& nodeHandle);

        /*!
        * Destructor.
        */
        virtual ~MPCFollowerWrapper();

        // @brief ROS initialize.
        void Initialize();
        


        // message filter subscribers
        message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
        message_filters::Subscriber<cav_msgs::TrajectoryPlan> trajectory_plan_sub;
        
        // SyncPolicy
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, cav_msgs::TrajectoryPlan> SyncPolicy;

        void TrajectoryPlanPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& pose, const cav_msgs::TrajectoryPlan::ConstPtr& tp);

    private:

        //@brief ROS node handle.
        ros::CARMANodeHandle& nh_;


        // @brief ROS publishers.
        ros::Publisher way_points_pub_;


        MPCFollowerWrapperWorker mpcww;


        // @brief ROS pusblishers.
        void PublisherForWayPoints(autoware_msgs::Lane& msg);


};

}  // namespace mpc_follower_wrapper
