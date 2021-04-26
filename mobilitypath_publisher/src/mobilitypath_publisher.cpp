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
// @SONAR_STOP@
    MobilityPathPublication::MobilityPathPublication():
                            path_pub_rate_(10.0) {}

    void MobilityPathPublication::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("path_pub_rate", path_pub_rate_, 10.0);
        pnh_->getParam("vehicle_id", sender_id);
        mob_path_pub_ = nh_->advertise<cav_msgs::MobilityPath>("mobility_path_msg", 5);
        traj_sub_ = nh_->subscribe("plan_trajectory", 5, &MobilityPathPublication::trajectory_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 5, &MobilityPathPublication::currentpose_cb, this);
        bsm_sub_ = nh_->subscribe("bsm_outbound", 1, &MobilityPathPublication::bsm_cb, this);
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));

        path_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(path_pub_rate_)),
            [this](const auto&) { this->spinCallback(); });

    }

    void MobilityPathPublication::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    bool MobilityPathPublication::spinCallback()
    {
        

        mob_path_pub_.publish(latest_mobility_path_); 
        return true;
    }

    void MobilityPathPublication::trajectory_cb(const cav_msgs::TrajectoryPlanConstPtr& msg)
    {
        latest_trajectory_ = *msg;

        try
        {
            geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("earth", "map", ros::Time(0));
            latest_mobility_path_ = mobilityPathMessageGenerator(latest_trajectory_, tf);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }

    void MobilityPathPublication::currentpose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        current_pose_ = msg;
    }

    void MobilityPathPublication::bsm_cb(const cav_msgs::BSMConstPtr& msg)
    {
        bsm_core_ = msg->core_data;
        
    }
    
// @SONAR_START@
    cav_msgs::MobilityPath MobilityPathPublication::mobilityPathMessageGenerator(const cav_msgs::TrajectoryPlan& trajectory_plan, const geometry_msgs::TransformStamped& tf)
    {
        cav_msgs::MobilityPath mobility_path_msg;
        uint64_t millisecs =trajectory_plan.header.stamp.toNSec()/1000000;
        mobility_path_msg.header = composeMobilityHeader(millisecs);
        
        cav_msgs::Trajectory mob_path_traj = TrajectoryPlantoTrajectory(trajectory_plan.trajectory_points, tf);
        mobility_path_msg.trajectory = mob_path_traj;

        return mobility_path_msg;
    }

    cav_msgs::MobilityHeader MobilityPathPublication::composeMobilityHeader(uint64_t time){
        cav_msgs::MobilityHeader header;
        header.sender_id = sender_id;
        header.recipient_id = recipient_id;
        header.sender_bsm_id = bsmIDtoString(bsm_core_);
        // random GUID that identifies this particular plan for future reference
        header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
        header.timestamp = time; //time in millisecond
        
        return header;
    }

    cav_msgs::Trajectory MobilityPathPublication::TrajectoryPlantoTrajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points, const geometry_msgs::TransformStamped& tf) const{
        cav_msgs::Trajectory traj;

        tf2::Stamped<tf2::Transform> transform;
        tf2::fromMsg(tf, transform);

        cav_msgs::LocationECEF ecef_location = TrajectoryPointtoECEF(traj_points[0], transform); //m to cm to fit the msg standard

        if (traj_points.size()<2){
            ROS_WARN("Received Trajectory Plan is too small");
            traj.offsets = {};
        }
        else{
            cav_msgs::LocationECEF prev_point = ecef_location;
            for (size_t i=1; i<traj_points.size(); i++){
                
                cav_msgs::LocationOffsetECEF offset;
                cav_msgs::LocationECEF new_point = TrajectoryPointtoECEF(traj_points[i], transform); //m to cm to fit the msg standard
                offset.offset_x = (int16_t)(new_point.ecef_x - prev_point.ecef_x);  
                offset.offset_y = (int16_t)(new_point.ecef_y - prev_point.ecef_y);
                offset.offset_z = (int16_t)(new_point.ecef_z - prev_point.ecef_z);
                prev_point = new_point;
                traj.offsets.push_back(offset);
            }
        }
        //according to msg standard, ecef_location is in meters
        ecef_location.ecef_x = ecef_location.ecef_x /100;
        ecef_location.ecef_y = ecef_location.ecef_y /100;
        ecef_location.ecef_z = ecef_location.ecef_z /100;

        traj.location = ecef_location; 

        return traj;
    }



    cav_msgs::LocationECEF MobilityPathPublication::TrajectoryPointtoECEF(const cav_msgs::TrajectoryPlanPoint& traj_point, const tf2::Transform& transform) const{
        cav_msgs::LocationECEF ecef_point;    
        
        auto traj_point_vec = tf2::Vector3(traj_point.x, traj_point.y, 0.0);
        tf2::Vector3 ecef_point_vec = transform * traj_point_vec;
        ecef_point.ecef_x = (int32_t)(ecef_point_vec.x() * 100.0); // m to cm
        ecef_point.ecef_y = (int32_t)(ecef_point_vec.y() * 100.0);
        ecef_point.ecef_z = (int32_t)(ecef_point_vec.z() * 100.0); 

        return ecef_point;
    } 
    
}