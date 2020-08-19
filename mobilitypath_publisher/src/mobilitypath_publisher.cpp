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


#include "mobilitypath_publisher.h"

namespace mobilitypath_publisher
{
    
    MobilityPathPublication::MobilityPathPublication():
                            x_threshold(20.0),
                            y_threshold(3.0) {}

    void MobilityPathPublication::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("trajectory_threshold_x", x_threshold, 20.0);
        pnh_->param<double>("trajectory_threshold_y", y_threshold, 3.0);
        mob_path_pub_ = nh_->advertise<cav_msgs::MobilityPath>("mobility_path_msg", 5);
        traj_sub_ = nh_->subscribe("plan_trajectory", 5, &MobilityPathPublication::trajectory_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 5, &MobilityPathPublication::currentpose_cb, this);
        ros::CARMANodeHandle::setSpinRate(10.0);
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
    }

    void MobilityPathPublication::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void MobilityPathPublication::trajectory_cb(const cav_msgs::TrajectoryPlanConstPtr& msg)
    {
        latest_trajectory = *msg;
        cav_msgs::MobilityPath mobility_path = mobilityPathMessageGenerator(latest_trajectory);
        mob_path_pub_.publish(mobility_path);
    }

    void MobilityPathPublication::currentpose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        current_pose_ = msg;
    }

    cav_msgs::MobilityPath MobilityPathPublication::mobilityPathMessageGenerator(cav_msgs::TrajectoryPlan trajectory_plan)
    {
        cav_msgs::MobilityPath mobility_path_msg;
        mobility_path_msg.header = composeMobilityHeader();
        

        std::vector<cav_msgs::TrajectoryPlanPoint> traj_points;
        traj_points = trajectory_plan.trajectory_points;
        cav_msgs::Trajectory mob_path_traj = TrajectoryPlantoTrajectory(traj_points);
        mobility_path_msg.trajectory = mob_path_traj;

        return mobility_path_msg;
    }

    cav_msgs::MobilityHeader MobilityPathPublication::composeMobilityHeader(){
        cav_msgs::MobilityHeader header;
        header.sender_id = sender_id;
        header.recipient_id = recipient_id;
        header.sender_bsm_id = sender_bsm_id;
        // random GUID that identifies this particular plan for future reference
        header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
        header.timestamp = ros::Time::now().toNSec()/1000; //time in millisecond
        
        return header;
    }

    cav_msgs::Trajectory MobilityPathPublication::TrajectoryPlantoTrajectory(std::vector<cav_msgs::TrajectoryPlanPoint> traj_points){
        cav_msgs::Trajectory traj;
        cav_msgs::LocationECEF ecef_location = TrajectoryPointtoECEF(traj_points[0]);

        if (traj_points.size()<2){
            ROS_WARN("Received Trajectory Plan is too small");
            traj.offsets = {};
        }
        else{
            for (int i=1; i<traj_points.size(); i++){
                if (trajectoryPoseCheck(traj_points[i])){
                    cav_msgs::LocationOffsetECEF offset;
                    cav_msgs::LocationECEF new_point = TrajectoryPointtoECEF(traj_points[i]);
                    offset.offset_x = new_point.ecef_x - ecef_location.ecef_x;
                    offset.offset_y = new_point.ecef_y - ecef_location.ecef_y;
                    offset.offset_z = new_point.ecef_z - ecef_location.ecef_z;
                    traj.offsets.push_back(offset);
                }
                else{
                    ecef_location = TrajectoryPointtoECEF(traj_points[i]);
                    traj.offsets = {};
                }
            }
        }
        
        
        traj.location = ecef_location;
        return traj;
    }

    bool MobilityPathPublication::trajectoryPoseCheck(cav_msgs::TrajectoryPlanPoint point){
        if (fabs(current_pose_->pose.position.x - point.x) > x_threshold || fabs(current_pose_->pose.position.y - point.y) > y_threshold)
        {
            return false;
        }
        return true;
    }

    cav_msgs::LocationECEF MobilityPathPublication::TrajectoryPointtoECEF(cav_msgs::TrajectoryPlanPoint traj_point){
        cav_msgs::LocationECEF ecef_point;    
        try
        {
            geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("earth", "map", ros::Time(0));
            ecef_point.ecef_x = traj_point.x * tf.transform.translation.x;
            ecef_point.ecef_y = traj_point.x * tf.transform.translation.y;
            ecef_point.ecef_z = traj_point.x * tf.transform.translation.z;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
        return ecef_point;
    } 
    
}
