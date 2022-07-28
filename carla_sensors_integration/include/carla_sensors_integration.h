/*
 * Copyright (C) 2022 LEIDOS.
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

#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>



namespace carla_sensors
{
    class CarlaSensors
    {
        public:

            CarlaSensors();
        
        //TODO: add @brief for each callback function
             
            void publish();
            void initialize();
            void spin();

            void point_cloud_cb(sensor_msgs::PointCloud2 point_cloud);
            void image_raw_cb(sensor_msgs::Image image_raw);
            void image_color_cb(sensor_msgs::Image image_color);
            void image_rect_cb(sensor_msgs::Image image_rect);
            void gnss_fixed_fused_cb(sensor_msgs::NavSatFix gnss_fixed);



        private:

            //CARMA ROS node handles
            ros::CARMANodeHandle nh_;
            
            /*Publishers and Subscribers*/

            ros::Subscriber point_cloud_sub_;
            ros::Subscriber image_raw_sub_;
            ros::Subscriber image_color_sub_;
            ros::Subscriber image_rect_sub_;
            ros::Subscriber gnss_fixed_fused_sub_;
            ros::Subscriber camera_info_sub_;

            ros::Publisher points_raw_pub_;
            ros::Publisher image_raw_pub_;
            ros::Publisher image_color_pub_;
            ros::Publisher image_rect_pub_;
            ros::Publisher gnss_fixed_fused_pub_;
            ros::Publisher camera_info_pub_;

            sensor_msgs::PointCloud2 point_cloud_msg;
            sensor_msgs::Image image_raw_msg;
            sensor_msgs::Image image_color_msg;
            sensor_msgs::Image image_rect_msg;
            sensor_msgs::NavSatFix gnss_fixed_msg;
            




    }

}
