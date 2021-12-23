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
        bsm_sub_ = nh_->subscribe("bsm_outbound", 1, &MobilityPathPublication::bsm_cb, this);
        georeference_sub_ = nh_->subscribe("georeference", 1, &MobilityPathPublication::georeference_cb, this);

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

    void MobilityPathPublication::georeference_cb(const std_msgs::StringConstPtr& msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  // Build projector from proj string
    }

    void MobilityPathPublication::trajectory_cb(const cav_msgs::TrajectoryPlanConstPtr& msg)
    {
        latest_trajectory_ = *msg;

        latest_mobility_path_ = mobilityPathMessageGenerator(latest_trajectory_);
    }

    void MobilityPathPublication::bsm_cb(const cav_msgs::BSMConstPtr& msg)
    {
        bsm_core_ = msg->core_data;
        
    }
    
// @SONAR_START@
    cav_msgs::MobilityPath MobilityPathPublication::mobilityPathMessageGenerator(const cav_msgs::TrajectoryPlan& trajectory_plan)
    {
        cav_msgs::MobilityPath mobility_path_msg;
        uint64_t millisecs = ros::Time::now().toNSec()/1000000;//trajectory_plan.header.stamp.toNSec()/1000000;
        mobility_path_msg.header = composeMobilityHeader(millisecs);
        
        if (!map_projector_) {
            ROS_ERROR_STREAM("MobilityPath cannot be populated as map projection is not available");
            return mobility_path_msg;
        }

        cav_msgs::Trajectory mob_path_traj = TrajectoryPlantoTrajectory(trajectory_plan.trajectory_points);
        mobility_path_msg.trajectory = mob_path_traj;

        return mobility_path_msg;
    }

    cav_msgs::MobilityHeader MobilityPathPublication::composeMobilityHeader(uint64_t time){
        cav_msgs::MobilityHeader header;
        header.sender_id = sender_id;
        header.recipient_id = recipient_id;
        header.sender_bsm_id = BSMHelper::BSMHelper::bsmIDtoString(bsm_core_.id);
        // random GUID that identifies this particular plan for future reference
        header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
        header.timestamp = time; //time in millisecond
        
        return header;
    }

    cav_msgs::Trajectory MobilityPathPublication::TrajectoryPlantoTrajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points) const{
        cav_msgs::Trajectory traj;

        cav_msgs::LocationECEF ecef_location = TrajectoryPointtoECEF(traj_points[0]); //m to cm to fit the msg standard 

        if (traj_points.size()<2){
            ROS_WARN("Received Trajectory Plan is too small");
            traj.offsets = {};
        }
        else{
            cav_msgs::LocationECEF prev_point = ecef_location;
            for (size_t i=1; i<traj_points.size(); i++){
                
                cav_msgs::LocationOffsetECEF offset;
                cav_msgs::LocationECEF new_point = TrajectoryPointtoECEF(traj_points[i]); //m to cm to fit the msg standard
                offset.offset_x = (int16_t)(new_point.ecef_x - prev_point.ecef_x);  
                offset.offset_y = (int16_t)(new_point.ecef_y - prev_point.ecef_y);
                offset.offset_z = (int16_t)(new_point.ecef_z - prev_point.ecef_z);
                prev_point = new_point;
                traj.offsets.push_back(offset);
                if( i >= 60 ){ break;}; 
            }
        }

        traj.location = ecef_location; 

        return traj;
    }



    cav_msgs::LocationECEF MobilityPathPublication::TrajectoryPointtoECEF(const cav_msgs::TrajectoryPlanPoint& traj_point) const{
        if (!map_projector_) {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
        cav_msgs::LocationECEF location;    
        
        lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({traj_point.x, traj_point.y, 0.0}, 1);
        location.ecef_x = ecef_point.x() * 100.0;
        location.ecef_y = ecef_point.y() * 100.0;
        location.ecef_z = ecef_point.z() * 100.0;

        return location;
    } 
    
}