#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <string>
#include <vector>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <bsm_helper/bsm_helper.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/MobilityPath.h>
#include <cav_msgs/BSM.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>

namespace mobilitypath_publisher
{

    class MobilityPathPublication
    {

    public:

        MobilityPathPublication();

        // general starting point of this node
        void run();

        cav_msgs::MobilityPath mobilityPathMessageGenerator(const cav_msgs::TrajectoryPlan& trajectory_plan);

        /**
         * \brief Callback for map projection string to define lat/lon -> map conversion
         * \brief msg The proj string defining the projection.
         */ 
        void georeference_cb(const std_msgs::StringConstPtr& msg);
        

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        double path_pub_rate_;

        bool spinCallback();

        // ROS publisher
        ros::Publisher  path_pub_;

        // ROS subscribers
        ros::Subscriber traj_sub_;
        ros::Subscriber accel_sub_;
        ros::Subscriber bsm_sub_;
        ros::Subscriber georeference_sub_;

        ros::Timer path_pub_timer_;

        // ROS publishers
        ros::Publisher mob_path_pub_;

        cav_msgs::TrajectoryPlan latest_trajectory_;
        cav_msgs::MobilityPath latest_mobility_path_;

        std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

        // BSM Message
        cav_msgs::BSMCoreData bsm_core_;

        
        // initialize this node
        void initialize();

        // callbacks for the subscribers
        void currentpose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void trajectory_cb(const cav_msgs::TrajectoryPlanConstPtr& msg);
        void bsm_cb(const cav_msgs::BSMConstPtr& msg);
        

        // Compose Mobility Header
        cav_msgs::MobilityHeader composeMobilityHeader(uint64_t time);

        // Convert Trajectory Plan to (Mobility) Trajectory
        cav_msgs::Trajectory TrajectoryPlantoTrajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points) const;

    
        // Convert Trajectory Point to ECEF Transform (accepts meters and outputs in cm)
        cav_msgs::LocationECEF TrajectoryPointtoECEF(const cav_msgs::TrajectoryPlanPoint& traj_point) const;

        // sender's static ID which is its license plate
        std::string sender_id = "USDOT-49096";

        // recipient's static ID
        // Empty string indicates a broadcast message
        std::string recipient_id = "";

        // sender's dynamic ID which is its BSM id in hex string
        std::string sender_bsm_id = "FFFF";

        

        
    };

}