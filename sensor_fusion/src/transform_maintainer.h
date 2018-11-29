#pragma once

/*
 * Copyright (C) 2018 LEIDOS.
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

#include "object_tracker.h"
#include "twist_history_buffer.h"
#include <sensor_fusion/SensorFusionConfig.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_msgs/HeadingStamped.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/BSM.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/bind.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bondcpp/bond.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cav_msgs/HeadingStamped.h>
#include "wgs84_utils.h"

namespace tf2
{
    inline // inline is required to make this compile. Not sure why
    geometry_msgs::TransformStamped toMsg(const tf2::Transform& tf, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
    {
        geometry_msgs::TransformStamped msg;
        msg.transform = tf2::toMsg(tf);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.child_frame_id = child_frame_id;
        
        return msg;
    }
}

/**
 * TODO
 * @brief A ROS node that monitors multiple sources to produce a filtered version
 *
 * This class monitors all drivers that provide position and tracked objects api*
 *
 */
class TransformMaintainer
{
public:

    void init(tf2_ros::Buffer* tf2_buffer, tf2_ros::TransformBroadcaster* tf2_broadcaster,
        std::unordered_map<std::string, nav_msgs::OdometryConstPtr>* odom_map,
        std::unordered_map<std::string, sensor_msgs::NavSatFixConstPtr>* navsatfix_map,
        std::unordered_map<std::string, cav_msgs::HeadingStampedConstPtr>* heading_map,
        std::string earth_frame, std::string map_frame, std::string odom_frame, 
        std::string base_link_frame, std::string global_pos_sensor_frame, std::string local_pos_sensor_frame)
    {
        tf2_buffer_ = tf2_buffer;
        tf2_broadcaster_ = tf2_broadcaster;
        
        odom_map_ = odom_map;
        navsatfix_map_ = navsatfix_map;
        heading_map_ = heading_map;
        
        earth_frame_ = earth_frame;
        map_frame_ = map_frame;
        odom_frame_ = odom_frame;
        base_link_frame_ = base_link_frame;
        global_pos_sensor_frame_ = global_pos_sensor_frame;
        local_pos_sensor_frame_ = local_pos_sensor_frame;
    }
    //void heading_update_cb();

    void nav_sat_fix_update_cb(const sensor_msgs::NavSatFixConstPtr host_veh_loc, const cav_msgs::HeadingStampedConstPtr heading_msg);

    void odometry_update_cb(const nav_msgs::OdometryConstPtr odometry);

    tf2::Stamped<tf2::Transform> get_transform(
        std::string parent_frame, std::string child_frame, ros::Time stamp,
        bool can_use_most_recent_tf);

    static tf2::Transform calculate_map_to_odom_tf(
        const wgs84_utils::wgs84_coordinate& host_veh_coord,
        const tf2::Transform&  base_to_global_pos_sensor, const tf2::Transform& earth_to_map,
        const tf2::Transform& odom_to_base_link);

    // void heading_update_cb(const ros::MessageEvent<cav_msgs::HeadingStamped>& event);

    // void nav_sat_fix_update_cb(const ros::MessageEvent<sensor_msgs::NavSatFix>& event);

    // void odometry_update_cb(const ros::MessageEvent<nav_msgs::Odometry>& event);

private:

    tf2_ros::Buffer* tf2_buffer_;
    tf2_ros::TransformBroadcaster* tf2_broadcaster_;

    std::unordered_map<std::string, nav_msgs::OdometryConstPtr>* odom_map_;
    std::unordered_map<std::string, sensor_msgs::NavSatFixConstPtr>* navsatfix_map_;
    std::unordered_map<std::string, cav_msgs::HeadingStampedConstPtr>* heading_map_;
    
    // The heading of the vehicle in degrees east of north in an NED frame.
    // Frame ids
    std::string earth_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    std::string global_pos_sensor_frame_;
    std::string local_pos_sensor_frame_;

    // Transforms
    tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();
    tf2::Transform earth_to_map_;
    tf2::Transform base_to_local_pos_sensor_;
    tf2::Transform base_to_global_pos_sensor_;
    tf2::Transform odom_to_base_link_ = tf2::Transform::getIdentity();// The odom frame will start in the same orientation as the base_link frame on startup

    bool no_earth_to_map_ = true;
    bool no_base_to_local_pos_sensor_ = true;
    bool no_base_to_global_pos_sensor_ = true;

    ros::Time last_nav_sat_stamp;
    ros::Time last_odom_stamp;
    ros::Time last_heading_stamp;
};

