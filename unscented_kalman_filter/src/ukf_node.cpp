/*
 * Copyright (C) 2019 LEIDOS.
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

#include "unscented_kalman_filter/ukf_node.h"

UKFNode::UKFNode(ros::NodeHandle *nh_)
{
    //Publisher object for ukf output pose with covariance
    pose_pub_ = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose", 10);
    //Subscriber object for imu driver
    imu_sub_ = nh_->subscribe("imu",10,&imuCallback,this);
    //Subscriber object for gps driver
    gps_sub_ = nh_->subscribe("gnss_pose",10,&gpsCallback,this);
    //Subscriber object for speeds of individual wheels
    wheel_sub_ = nh_->subscribe("parsed_tx/wheel_speed_rpt",10,&wheelrtCallback,this);
    //Subscriber object for steering wheel angle
    steering_sub_ = nh_->subscribe("can/steering_wheel_angle",10,&wheelangCallback,this);
    //Subscriber object for ndt_matching
    ndt_sub_= nh_->subscribe("current_pose",10,&ndtCallback,this);
}

void UKFNode::imuCallback(const sensor_msgs::Imu &imu_msg)
{
    //converts a quartention coordinate to a roll, pitch, and yaw
    tf::Quaternion q(imu_msg.orientation.x,
                     imu_msg.orientation.y,
                     imu_msg.orientation.z,
                     imu_msg.orientation.w
                     );

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //End of convertion and we have roll,pitch,yaw

    z_imu_raw_(0)=yaw; //Yaw
    //take the speed from the most recent known state vector and add the measured acceleration
    // to it to determine what the speed should be we will then compare it to the predicted
    // speed inside the filter
    z_imu_raw_(1)=x_previous(3)+(imu_msg.linear_acceleration.x * delta_t_); //Linear acceleration along x-direction
    z_imu_raw_(2)=x_previous(4)+(imu_msg.linear_acceleration.y * delta_t_); //Linear acceleration along y-direction

    z_imu_raw_(3)=imu_msg.angular_velocity.z; //Angular velocity
    z_imu_raw_(4)=1;//Note the number one is augmented for vector translation

    //IMU measurement function H matrix
    H_imu_helper_.fill(0);
    H_imu_helper_(0,2)=1;
    H_imu_helper_(1,3)=1;
    H_imu_helper_(2,4)=1;
    H_imu_helper_(3,5)=1;

    //cg_distance is the distance from the back axle to the center of gravity of the vehicle
    Translation_(0,9)= -(cg_distance*cos(yaw));
    Translation_(1,9)= -(cg_distance*sin(yaw));

    //Measurement function translation from back axle to the center of gravity of the vehicle
    H_imu_=H_imu_helper_*Translation_;

    //Covariance R
    R_imu_.fill(0);
    R_imu_(0,0)=imu_msg.orientation_covariance[8];//yaw
    //this is the covairance of linear velocity x and linear velocity y
    R_imu_(1,1)=P_previous(3,3)+(delta_t_*delta_t_*imu_msg.linear_acceleration_covariance[0]);//linear accelertion x
    R_imu_(2,2)=P_previous(4,4)+(delta_t_*delta_t_*imu_msg.linear_acceleration_covariance[4]);//linear acceleration y
    R_imu_(3,3)=imu_msg.angular_velocity_covariance[8];//angular velocity z
    R_imu_(4,4)=0.00000000000000000000000000001;//Adjust the sensor covariance for the addition of number one, we can't assume zero because the matrix will be invertible
    //P Matrix need extra zero
    //Same with vehicle state

    ukf_node.time_current_=imu_msg.header.stamp;
}

void UKFNode::gpsCallback(const geometry_msgs::PoseWithCovarianceStamped &gps_msg)
{
    z_gps_raw_(0)=gps_msg.pose.pose.position.x;
    z_gps_raw_(1)=gps_msg.pose.pose.position.y;
    z_gps_raw_(2)=gps_msg.pose.pose.orientation.z;
    z_gps_raw_(3)=1; //Note the number one is augmented for vector translation

    //Measurement function
    H_gps_helper_.fill(0);
    H_gps_helper_(0,0)=1;
    H_gps_helper_(1,1)=1;
    H_gps_helper_(2,2)=1;
    H_gps_helper_(3,9)=1;

    //cg_distance is the distance from the back axle to the center of gravity of the vehicle
    Translation_(0,9)= -(cg_distance*cos(gps_msg.pose.pose.orientation.z));
    Translation_(1,9)= -(cg_distance*sin(gps_msg.pose.pose.orientation.z));

    //Measurement function translation from back axle to the center of gravity of the vehicle
    H_gps_=H_gps_helper_*Translation_;

    //Covariance R of the gps
    covariance_gps_=gps_msg.pose.covariance;
    R_gps_.fill(0);
    R_gps_(0,0)=covariance_gps_[0];
    R_gps_(1,1)=covariance_gps_[7];
    R_gps_(2,2)=covariance_gps_[35];
    R_gps_(3,3)=0.00000000000000000000000000001;//Adjust the sensor covariance for the addition of number one, we can't assume zero because the matrix will be invertible
    //P Matrix need extra zero
    //Same with vehicle state
    ukf_node.time_current_=gps_msg.header.stamp;
}

void UKFNode::wheelrtCallback(const pacmod_msgs::WheelSpeedRpt &wheelrt_msg)
{
    // Need to average as per state space requirement
    //Front wheel average
    w_f_=(((wheelrt_msg.front_left_wheel_speed)+(wheelrt_msg.front_right_wheel_speed))/2);
    //Rear wheel average
    w_r_=(((wheelrt_msg.rear_left_wheel_speed)+(wheelrt_msg.rear_right_wheel_speed))/2);
    //Raw wheel speed value
    z_wheelrt_raw_(0)=w_f_;
    z_wheelrt_raw_(1)=w_r_;
    //Measurement function H for wheel speed
    H_wheelrt_.fill(0);
    H_wheelrt_(0,6)=1;
    H_wheelrt_(1,7)=1;

    //Covariance R of wheel rotation front and back
    R_wheel_.fill(0);
    R_wheel_(0,0)=1000;
    R_wheel_(1,1)=1000;

    time_current_=wheelrt_msg.header.stamp;
}

void UKFNode::wheelangCallback(const std_msgs::Float64 &wheelang_msg)
{
    z_wheelang_raw_(0)=wheelang_msg.data;

    H_wheelang_.fill(0);
    H_wheelang_(0,8)=1;

    //Covariance R of steering angle
    R_steering_(0,0)=1000;

    time_current_=ros::Time::now();
}

void ndtCallback(const geometry_msgs::PoseStamped &ndt_msg)
{
    z_ndt_raw_(0)=ndt_msg.pose.position.x;
    z_ndt_raw_(1)=ndt_msg.pose.position.y;
    z_ndt_raw_(2)=ndt_msg.pose.orientation.z;

    H_ndt_.fill(0);
    H_ndt_(0,0)=1;
    H_ndt_(1,1)=1;
    H_ndt_(2,2)=1;

    //Covariance R of ndt taken as identity as the accuracy is close to 10cm
    R_ndt_=MatrixXd::Identity(3,3);

    time_current_=ndt_msg.header.stamp;
}