/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include "transform_maintainer.h"

namespace tf2
{
    // Functions based off tf2/buffer_core.cpp
    void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg)
    {
        msg.translation.x = tf2.getOrigin().x();
        msg.translation.y = tf2.getOrigin().y();
        msg.translation.z = tf2.getOrigin().z();
        msg.rotation.x = tf2.getRotation().x();
        msg.rotation.y = tf2.getRotation().y();
        msg.rotation.z = tf2.getRotation().z();
        msg.rotation.w = tf2.getRotation().w();
    }

    void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::TransformStamped& msg, ros::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
    {
        transformTF2ToMsg(tf2, msg.transform);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.child_frame_id = child_frame_id;
    }
}

void TransformMaintainer::nav_sat_fix_update_cb(const sensor_msgs::NavSatFixConstPtr host_veh_loc, const cav_msgs::HeadingStampedConstPtr heading_msg)
{
    
    // These values must be time synched
    if (host_veh_loc->header.stamp != heading_msg->header.stamp) {
      ROS_WARN_STREAM("TRANSFORM | Tried to update transforms without synched heading and nav_sat_fix");
      return;
    }

    if (last_nav_sat_stamp >= host_veh_loc->header.stamp) {
      ROS_WARN_STREAM("TRANSFORM | Old nav sat received prev: " << last_nav_sat_stamp << " update: " << host_veh_loc->header.stamp);
      return;
    }

    if (last_heading_stamp >= heading_msg->header.stamp) {
      ROS_WARN_STREAM("TRANSFORM | Old heading received prev: " << last_heading_stamp << " update: " << heading_msg->header.stamp);
      return;
    }

    last_nav_sat_stamp = host_veh_loc->header.stamp;
    ROS_DEBUG_STREAM("TRANSFORM | New nav sat received: " << last_nav_sat_stamp);

    last_heading_stamp = heading_msg->header.stamp;
    ROS_DEBUG_STREAM("TRANSFORM | New heading received: " << last_heading_stamp);

    std::string frame_id = host_veh_loc->header.frame_id;
    if (frame_id != global_pos_sensor_frame_) {
      std::string msg = "NavSatFix message with unsupported frame received. Frame: " + frame_id;
      ROS_ERROR_STREAM("TRANSFORM | " << msg);
      return;
    }
    
    // Check if base_link->position_sensor tf is available. If not look it up
    if (no_base_to_global_pos_sensor_) {
      // This transform should be static. No need to look up more than once
      try {
        base_to_global_pos_sensor_ = get_transform(base_link_frame_, global_pos_sensor_frame_, ros::Time(0), true);
      } catch (tf2::TransformException e) {
        std::string msg = "TransformMaintainer nav_sat_fix_update_cb failed to get transform for global position sensor in base link";
        ROS_WARN_STREAM("TRANSFORM | " << msg);
        return;
      }

      no_base_to_global_pos_sensor_ = false;
    }

    std::vector<geometry_msgs::TransformStamped> tf_stamped_msgs;

    // Extract geodesic data and convert to radians
    wgs84_utils::wgs84_coordinate host_veh_coord;
    host_veh_coord.lat = host_veh_loc->latitude * wgs84_utils::DEG2RAD;
    host_veh_coord.lon = host_veh_loc->longitude * wgs84_utils::DEG2RAD;
    host_veh_coord.elevation = host_veh_loc->altitude;
    host_veh_coord.heading = heading_msg->heading * wgs84_utils::DEG2RAD;

    // Update map location on start
    if (no_earth_to_map_) { 
      // Map will be an NED frame on the current vehicle location
      earth_to_map_ = wgs84_utils::ecef_to_ned_from_loc(host_veh_coord);
      no_earth_to_map_ = false;
    }
    // Keep publishing transform to maintain timestamp
    geometry_msgs::TransformStamped earth_to_map_msg;
    tf2::transformTF2ToMsg(earth_to_map_, earth_to_map_msg, host_veh_loc->header.stamp, earth_frame_, map_frame_);
    
    tf_stamped_msgs.push_back(earth_to_map_msg);

    // Get odom->base_link transform with matching timestamp
    tf2::Transform odom_to_base_link;
    try {
      odom_to_base_link =  get_transform(odom_frame_, base_link_frame_, host_veh_loc->header.stamp, true);
    } catch (tf2::TransformException e) {
      std::string msg = "TransformMaintainer nav_sat_fix_update_cb failed to get transform for odom to base_link";
      ROS_WARN_STREAM("TRANSFORM | " << msg);
      return;
    }

    // Calculate updated tf
    map_to_odom_ = calculate_map_to_odom_tf(
      host_veh_coord, base_to_global_pos_sensor_,
      earth_to_map_, odom_to_base_link);
    //TODO do we need to sync the heading too? Currently pinpoint's heading and nav_sat_fix are always synched. But this maynot always be the case
    // Publish newly calculated transforms
    geometry_msgs::TransformStamped map_to_odom_msg;
    tf2::transformTF2ToMsg(map_to_odom_, map_to_odom_msg, host_veh_loc->header.stamp, map_frame_, odom_frame_);
    
    tf_stamped_msgs.push_back(map_to_odom_msg);

    // Publish transform
    for (int i =0; i < tf_stamped_msgs.size(); i++) {
      tf2_buffer_->setTransform(tf_stamped_msgs.at(i), "/carma/sensor_fusion");
    }
    tf2_broadcaster_->sendTransform(tf_stamped_msgs);
}

// Broken out for unit testing
// Heading is assumed to be in rad
// All geodesic angles assumed to be in rad
  tf2::Transform TransformMaintainer::calculate_map_to_odom_tf(
    const wgs84_utils::wgs84_coordinate& host_veh_coord,
    const tf2::Transform&  base_to_global_pos_sensor, const tf2::Transform& earth_to_map,
    const tf2::Transform& odom_to_base_link)
  {
    // Calculate map->global_position_sensor transform

    tf2::Vector3 global_sensor_in_map = wgs84_utils::geodesic_to_ecef(host_veh_coord, earth_to_map.inverse());

    // T_x_y = transform describing location of y with respect to x
    // m = map frame
    // b = baselink frame (from odometry)
    // B = baselink frame (from nav sat fix)
    // o = odom frame
    // p = global position sensor frame
    // We want to find T_m_o. This is the new transform from map to odom.
    // T_m_o = T_m_B * inv(T_o_b)  since b and B are both odom.
    tf2::Vector3 sensor_trans_in_map = global_sensor_in_map;
    // The vehicle heading is relative to NED so over short distances heading in NED = heading in map
    tf2::Vector3 zAxis = tf2::Vector3(0, 0, 1);
    tf2::Quaternion sensor_rot_in_map(zAxis, host_veh_coord.heading);
    sensor_rot_in_map = sensor_rot_in_map.normalize();

    tf2::Transform T_m_p = tf2::Transform(sensor_rot_in_map, sensor_trans_in_map);
    tf2::Transform T_B_p = base_to_global_pos_sensor;
    tf2::Transform T_m_B = T_m_p * T_B_p.inverse();
    tf2::Transform T_o_b = odom_to_base_link;

    // Modify map to odom with the difference from the expected and real sensor positions
    return T_m_B * T_o_b.inverse();
  }

void TransformMaintainer::odometry_update_cb(const nav_msgs::OdometryConstPtr odometry) 
{
      // Check if base_link->position_sensor tf is available. If not look it up
    if (no_base_to_local_pos_sensor_) {
      // This transform should be static. No need to look up more than once
      try {
        base_to_local_pos_sensor_ = get_transform(base_link_frame_, local_pos_sensor_frame_, ros::Time(0), true);
      } catch (tf2::TransformException e) {
        std::string msg = "TransformMaintainer odometry_update_cb failed to get transform for local position sensor in base link";
        ROS_WARN_STREAM("TRANSFORM | " << msg);
        return;
      }

      no_base_to_local_pos_sensor_ = false;
    }

    //nav_msgs::OdometryConstPtr odometry = odom_map_->begin()->second;
    std::string parent_frame_id = odometry->header.frame_id;
    std::string child_frame_id = odometry->child_frame_id;
    // If the odometry is already in the base_link frame
    if (parent_frame_id == odom_frame_ && child_frame_id == base_link_frame_) {
      tf2::fromMsg(odometry->pose.pose, odom_to_base_link_);
      // Publish updated transform
      geometry_msgs::TransformStamped odom_to_base_link_msg; 
      tf2::transformTF2ToMsg(odom_to_base_link_, odom_to_base_link_msg, odometry->header.stamp, odom_frame_, base_link_frame_);
      tf2_buffer_->setTransform(odom_to_base_link_msg, "/carma/sensor_fusion");
      tf2_broadcaster_->sendTransform(odom_to_base_link_msg);

    } else if (parent_frame_id == odom_frame_ && child_frame_id == local_pos_sensor_frame_) {
      // Extract the location of the position sensor relative to the odom frame
      // Covariance is ignored as filtering was already done by sensor fusion
      // Calculate odom->base_link
      // T_x_y = transform describing location of y with respect to x
      // p = position sensor frame (from odometry)
      // o = odom frame
      // b = baselink frame (as has been calculated by odometry up to this point)
      // T_o_b = T_o_p * inv(T_b_p)
      tf2::Transform T_o_p;
      tf2::fromMsg(odometry->pose.pose, T_o_p);

      tf2::Transform T_b_p = base_to_local_pos_sensor_;
      tf2::Transform T_o_b = T_o_p * T_b_p.inverse();
      odom_to_base_link_ = T_o_b;
      // Publish updated transform
      geometry_msgs::TransformStamped odom_to_base_link_msg;
      tf2::transformTF2ToMsg(odom_to_base_link_, odom_to_base_link_msg, odometry->header.stamp, odom_frame_, base_link_frame_);
      tf2_buffer_->setTransform(odom_to_base_link_msg, "/carma/sensor_fusion");
      tf2_broadcaster_->sendTransform(odom_to_base_link_msg);

    } else {
      std::string msg =
        "Odometry message with unsupported frames received. ParentFrame: " + parent_frame_id
          + " ChildFrame: " + child_frame_id;
      ROS_ERROR_STREAM("TRANSFORM | " << msg);//TODO should we throw an exception here?
    }
}

// Helper function
tf2::Stamped<tf2::Transform> TransformMaintainer::get_transform(
  std::string parent_frame, std::string child_frame, ros::Time stamp,
  bool can_use_most_recent_tf) 
{
  geometry_msgs::TransformStamped transform_stamped;

  if(tf2_buffer_->canTransform(parent_frame, child_frame, stamp))
  {
    transform_stamped = tf2_buffer_->lookupTransform(parent_frame, child_frame, stamp);
  }
  else if(tf2_buffer_->canTransform(parent_frame, child_frame, ros::Time(0)) && can_use_most_recent_tf)
  {
    ROS_DEBUG_STREAM("TRANSFORM | Using latest transform available"); // This expected on the nav_sat update
    transform_stamped = tf2_buffer_->lookupTransform(parent_frame, child_frame, ros::Time(0));
  }
  else
  {
    ROS_WARN_STREAM("No transform available from " << parent_frame << " to " << child_frame);

    throw tf2::TransformException("Failed to get transform");
  }
  
  tf2::Transform result;
  tf2::fromMsg(transform_stamped.transform, result);
  tf2::Stamped<tf2::Transform> stamped_result(result, transform_stamped.header.stamp, transform_stamped.header.frame_id);
  return stamped_result;
}



