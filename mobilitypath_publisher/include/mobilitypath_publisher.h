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
#include <string>
#include <vector>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/MobilityPath.h>
#include <geometry_msgs/PoseStamped.h>

namespace mobilitypath_publisher
{

    class MobilityPathPublication
    {

    public:

        MobilityPathPublication();

        // general starting point of this node
        void run();

        // local copy of pose
        boost::shared_ptr<geometry_msgs::PoseStamped const> current_pose_;

        cav_msgs::MobilityPath mobilityPathMessageGenerator(cav_msgs::TrajectoryPlan trajectory_plan);
        

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;


        // ROS publisher
        ros::Publisher  path_pub_;

        // ROS subscribers
        ros::Subscriber traj_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber accel_sub_;

        // ROS publishers
        ros::Publisher mob_path_pub_;

        cav_msgs::TrajectoryPlan latest_trajectory;

        // TF listenser
        tf2_ros::Buffer tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

        // size of the vehicle
        double vehicle_length_, vehicle_width_;

        // uncertainty threshould for updating the mobility path trajectory
        double x_threshold , y_threshold; //m (width of a lane)

        // initialize this node
        void initialize();

        // callbacks for the subscribers
        void currentpose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void trajectory_cb(const cav_msgs::TrajectoryPlanConstPtr& msg);

        // Compise Mobility Header
        cav_msgs::MobilityHeader composeMobilityHeader();

        // Convert Trajectory Plan to (Mobility) Trajectory
        cav_msgs::Trajectory TrajectoryPlantoTrajectory(std::vector<cav_msgs::TrajectoryPlanPoint> traj_points);

        // Check for distance between current position and trajectory point
        bool trajectoryPoseCheck(cav_msgs::TrajectoryPlanPoint point);
        
        // Convert Trajectory Point to ECEF Transform
        cav_msgs::LocationECEF TrajectoryPointtoECEF(cav_msgs::TrajectoryPlanPoint traj_point);

        // sender's static ID which is its license plate
        std::string sender_id = "USDOT-49096";

        // recipient's static ID
        // Empty string indicates a broadcast message
        std::string recipient_id = "";

        // sender's dynamic ID which is its BSM id in hex string
        std::string sender_bsm_id = "FFFFFFFF";

        
    };

}