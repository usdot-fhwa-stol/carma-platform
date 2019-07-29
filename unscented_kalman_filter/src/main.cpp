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

#include "ros/ros.h" //ros header
#include "ukf_filter.h" //ukf library
#include "geometry_msgs/PoseStamped.h" // GPS message type
#include "sensor_msgs/Imu.h" // IMU message type
#include "pacmod_msgs/WheelSpeedRpt.h" //Speeds of individual wheels [m/s] message type
#include "std_msgs/Float64.h" //Steering wheel angle message type
#include "geometry_msgs/PoseWithCovarianceStamped.h" //UKF output with pose and covariance message type

VectorXd z_can_raw(3);
VectorXd z_gps_raw(3);
VectorXd z_imu_raw(4);

MatrixXd H_gps(3,9);
MatrixXd H_imu_yaw(1,9);
MatrixXd H_imu_angv(1,9);
MatrixXd H_imu_lina(2,9);
MatrixXd H_wheelrt(2,9);
MatrixXd H_wheelang(1,9);

void imuCallback(const sensor_msgs::Imu &imu_msg);
void gpsCallback(const geometry_msgs::PoseStamped &gps_msg);
void wheelrtCallback(const pacmod_msgs::WheelSpeedRpt &wheelrt_msg);
void wheelangCallback(const std_msgs::Float64 &wheelang_msg);


int main(int argc, char** argv)
{
    //ukf_fusion node initialization
    ros::init(argc, argv,"ukf_fusion");
    //ros NodeHandle
    ros::NodeHandle nh_;
    //Vehicle library initialization
    lib_vehicle_model::init(nh_);

    //Publisher object for ukf output pose with covariance
    ros::Publisher pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose", 10);

    //Subscriber object for imu driver
    ros::Subscriber imu_sub_ = nh_.subscribe("imu",10,&imuCallback);
    //Subscriber object for gps driver
    ros::Subscriber gps_sub_ = nh_.subscribe("gnss_map_fix",10,&gpsCallback); //confirm
    //Subscriber object for speeds of individual wheels
    ros::Subscriber can_sub_ = nh_.subscribe("parsed_tx/wheel_speed_rpt",10,&wheelrtCallback);
    //Subscriber object for steering wheel angle
    ros::Subscriber can_sub_ = nh_.subscribe("can/steering_wheel_angle",10,&wheelangCallback);

    ros::Rate loop_rate(10);

    //UKFilter class instance
    UKFilter ukf;

    VectorXd X(9);  //State Space Vector
    X.fill(0);      //Initialize with zero
    MatrixXd P(9,9); // State Covariance Matrix
    // Generate Identitiy matrix for the initial covariance
    P = MatrixXd::Identity(X.size(), X.size());

    int initialize=0; // Variable to check if initialized for the 1st time before entering into loop

    while(ros::ok())
    {
        if (initialize==0)
        {
            X(0)=0;  //X
            X(1)=0;  //Y
            X(2)=0;  //Yaw

            ukf.Prediction(X,P,0); //Need to work on timestamp
            //Need to update 6 times to incoperate all the sensors
            ukf.Update(X,P,H_,R_,z_raw_); // GPS
            ukf.Update(X,P,H_,R_,z_raw_); // IMU Yaw
            ukf.Update(X,P,H_,R_,z_raw_); // IMU angular velocity
            ukf.Update(X,P,H_,R_,z_raw_); // IMU linear acceleration
            ukf.Update(X,P,H_,R_,z_raw_); // CAN wheel rotation
            ukf.Update(X,P,H_,R_,z_raw_); // CAN wheel angle

            initialize=1;
        }
        else if(initialize==1)
        {
            ukf.Prediction(X,P,0); //Need to work on timestamp
            //Need to update 6 times to incoperate all the sensors
            ukf.Update(X,P,H_,R_,z_raw_); // GPS
            ukf.Update(X,P,H_,R_,z_raw_); // IMU Yaw
            ukf.Update(X,P,H_,R_,z_raw_); // IMU angular velocity
            ukf.Update(X,P,H_,R_,z_raw_); // IMU linear acceleration
            ukf.Update(X,P,H_,R_,z_raw_); // CAN wheel rotation
            ukf.Update(X,P,H_,R_,z_raw_); // CAN wheel angle
        }

        geometry_msgs::PoseWithCovarianceStamped ukf_pose_msg;

        ukf_pose_msg.pose.pose.position.x=X(0);
        ukf_pose_msg.pose.pose.position.y=X(1);
        ukf_pose_msg.pose.pose.orientation.z=X(2);

        pose_pub_.publish(ukf_pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void imuCallback(const sensor_msgs::Imu &imu_msg)
 {
     z_imu_raw(0)=imu_msg.orientation.z; //Yaw
     z_imu_raw(1)=imu_msg.linear_acceleration.x; //Linear acceleration along x-direction
     z_imu_raw(2)=imu_msg.linear_acceleration.y; //Linear acceleration along y-direction
     z_imu_raw(3)=imu_msg.angular_velocity.z; //Angular velocity

     //IMU measurement function H matrix
     H_imu_yaw.fill(0);
     H_imu_angv.fill(0);
     H_imu_lina.fill(0);

     H_imu_yaw(0,2)=1;
     H_imu_angv(0,5)=1;
     H_imu_lina(0,3)=1;
     H_imu_lina(1,4)=1;

     //Covariance R
 }

void gpsCallback(const geometry_msgs::PoseStamped &gps_msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    z_gps_raw(0)=gps_msg.pose.position.x;
    z_gps_raw(1)=gps_msg.pose.position.y;
    z_gps_raw(2)=0; //Heading

    //GPS measurement function H matrix
    H_gps.fill(0);
    H_gps(0,0)=1;
    H_gps(1,1)=1;
    H_gps(2,2)=1;

    // Need to multiply with a linear transformation to get according to cg

    //Covariance R

}

void wheelrtCallback(const pacmod_msgs::WheelSpeedRpt &wheelrt_msg)
{
    double w_f_,w_r_; //Wheel rotation front and back
    // Need to average as per state space requirement
    w_f_=(((wheelrt_msg.front_left_wheel_speed) + (wheelrt_msg.front_right_wheel_speed))/2);
    w_r_=(((wheelrt_msg.rear_left_wheel_speed) + (wheelrt_msg.rear_right_wheel_speed))/2);

    z_can_raw(0)=w_f_;
    z_can_raw(1)=w_r_;

    H_wheelrt.fill(0);
    H_wheelrt(0,6)=1;
    H_wheelrt(1,7)=1;

    //Covariance R
}

void wheelangCallback(const std_msgs::Float64 &wheelang_msg)
{
    z_can_raw(2)=wheelang_msg.data;

    H_wheelang.fill(0);
    H_wheelang(0,8)=1;

    //Covariance R
}