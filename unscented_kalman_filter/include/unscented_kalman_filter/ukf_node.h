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
#ifndef UNSCENTED_KALMAN_FILTER_UKF_NODE_H
#define UNSCENTED_KALMAN_FILTER_UKF_NODE_H

#include "ros/ros.h" //ros header
#include "ukf_filter.h" //ukf library
#include "sensor_msgs/Imu.h" // IMU message type
#include "pacmod_msgs/WheelSpeedRpt.h" //Speeds of individual wheels [m/s] message type
#include "std_msgs/Float64.h" //Steering wheel angle message type
#include "geometry_msgs/PoseWithCovarianceStamped.h" //GPS message and UKF output with pose and covariance message type
#include "geometry_msgs/Pose.msg" //Output message type of ndt_matching
#include <vector>
#include <tf/tf.h>

class UKFNode{
private:
    VectorXd z_wheelrt_raw_(2);
    VectorXd z_wheelang_raw_(1);
    VectorXd z_ndt_raw_(3);
    VectorXd z_gps_raw_(4);
    VectorXd z_imu_raw_(5);
    double covariance_; //Covariance from gps
    MatrixXd R_gps_(4,4);// Measurement GPS covariance
    MatrixXd R_ndt_;// Measurement ndt covariance
    MatrixXd R_wheel_(2,2);
    MatrixXd R_steering_(1,1);
    MatrixXd R_imu_(5,5);
    MatrixXd H_gps_helper_(4,10);
    MatrixXd H_gps_;
    MatrixXd H_imu_;
    MatrixXd Translation_=MatrixXd::Identity(10, 10);
    MatrixXd H_imu_helper_(4,9);
    MatrixXd H_wheelrt_(2,9);
    MatrixXd H_wheelang_(1,9);
    MatrixXd H_ndt_(3,9);

    long long time_current_;
    long long time_previous_;

    //Publisher object for ukf output pose with covariance
    ros::Publisher pose_pub_ ;
    //Subscriber object for imu driver
    ros::Subscriber imu_sub_ ;
    //Subscriber object for gps driver
    ros::Subscriber gps_sub_ ;
    //Subscriber object for speeds of individual wheels
    ros::Subscriber wheel_sub_ ;
    //Subscriber object for steering wheel angle
    ros::Subscriber steering_sub_ ;
    //Subscriber object for ndt_matching
    ros::Subscriber ndt_sub_ ;
    //Wheel rotation front and back
    double w_f_,w_r_;

public:
    UKFNode();
    ~UKFNode();
    //IMU call back function
    void imuCallback(const sensor_msgs::Imu &imu_msg);
    //GPS call back function
    void gpsCallback(const geometry_msgs::PoseWithCovarianceStamped &gps_msg);
    //Wheel rotations front and rear call back function
    void wheelrtCallback(const pacmod_msgs::WheelSpeedRpt &wheelrt_msg);
    //Steering angle call back function
    void wheelangCallback(const std_msgs::Float64 &wheelang_msg);
    //Ndt_matching call back function
    void ndtCallback(const geometry_msgs::PoseStamped &ndt_msg);

};

#endif //UNSCENTED_KALMAN_FILTER_UKF_NODE_H
