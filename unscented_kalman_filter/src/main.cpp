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
#include "ukf_node.h" //ros components

int main(int argc, char** argv)
{
    //ukf_fusion node initialization
    ros::init(argc, argv,"ukf_filter");
    //ros NodeHandle
    ros::NodeHandle nh_;
    //Vehicle library initialization
    lib_vehicle_model::init(nh_);
    //UKFNode class object instance
    UKFNode ukf_node(&nh_);

    //UKFilter class object instance
    UKFilter ukf_lib;

    bool initialize=0; // Variable to check if initialized for the 1st time before entering into loop

    while(ros::ok())
    {
        //Initialization for the very first time either using ndt_matching(LASER)/GPS
        if (initialize==0)
        {

            if(ukf_lib.sensortype==UKFilter::LASER)
            {
                ukf_lib.X(0)=ukf_node.z_raw_(0);
                ukf_lib.X(1)=ukf_node.z_raw_(1);
                ukf_lib.X(2)=ukf_node.z_raw_(2);
            }
            else if(ukf_lib.sensortype==UKFilter::GPS)
            {
                ukf_lib.X(0)=ukf_node.z_raw_(0);
                ukf_lib.X(1)=ukf_node.z_raw_(1);
                ukf_lib.X(2)=ukf_node.z_raw_(2);
            }
            //Very first timestamp in sec
            time_previous_=ukf_node.time_current; //Get from ROS header
            initialize=1;
        }
        else if(initialize==1)
        {
            //time difference delta t between two instance
            ukf_lib.delta_t_=ukf_node.time_current_-ukf_node.time_previous_;
            ukf_node.time_previous_=ukf_node.time_current_;

            //Filter loop
            ukf_lib.Prediction(ukf_lib.X,ukf_lib.P,ukf_lib.delta_t_);
            //Need to update 6 times to incoperate all the sensors
            ukf_lib.Update(X,P,H_,R_,z_raw_); // GPS
            ukf_lib.Update(X,P,H_,R_,z_raw_); // IMU
            ukf_lib.Update(X,P,H_,R_,z_raw_); // CAN wheel rotation
            ukf_lib.Update(X,P,H_,R_,z_raw_); // CAN wheel angle
        }

        //Output pose of the ukf node with pose, heading and covariance
        geometry_msgs::PoseWithCovarianceStamped ukf_pose_msg;

        ukf_pose_msg.pose.pose.position.x=X(0);
        ukf_pose_msg.pose.pose.position.y=X(1);
        ukf_pose_msg.pose.pose.orientation.z=X(2);
        //Convert matrix to vector
        VectorXd ukf_cov(90);
        ukf_cov.fill(0);
        ukf_cov(0)=P(0,0);
        ukf_cov(10)=P(1,1);
        ukf_cov(20)=P(2,2);
        ukf_cov(30)=P(3,3);
        ukf_cov(40)=P(4,4);
        ukf_cov(50)=P(5,5);
        ukf_cov(60)=P(6,6);
        ukf_cov(70)=P(7,7);
        ukf_cov(80)=P(8,8);

        ukf_pose_msg.pose.covariance=ukf_cov;

        uk.pose_pub_.publish(ukf_pose_msg);
        ros::spinOnce();

    }
    return 0;
}

