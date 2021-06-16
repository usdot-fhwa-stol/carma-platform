/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <gnss_to_map_convertor/GNSSToMapConvertor.h>

namespace gnss_to_map_convertor {

  geometry_msgs::PoseWithCovarianceStamped poseFromGnss(
    const tf2::Transform& baselink_in_sensor,
    const tf2::Transform& sensor_in_ned_heading,
    const gps_common::GPSFixConstPtr& fix_msg
  ) {

    //// Convert the position information into the map frame using the proj library
    const double lat = fix_msg->latitude * wgs84_utils::DEG2RAD;
    const double lon = fix_msg->longitude * wgs84_utils::DEG2RAD;
    const double alt = fix_msg->altitude;

    lanelet::projection::LocalFrameProjector wgs84_to_map("TODO");
    lanelet2::BasicPoint3d map_point = wgs84_to_map.forward({ lat, lon, alt});

    
    //// Convert the orientation information into the map frame
    // TODO: A Key question here is how to handle the heading. The heading is described as degrees east of north by GPSFix
    // Is it more accurate to use an ned tie point or extract the lat/lon from the projection string? 
    // Or should you get a tie point and the origin from the projection string then get the transform between the two and rectify the orientation via rotation matrix?



    // Using this approach we can remove the ned_heading in sensor frame transform which is not very meaningful anyway since the heading is defined via message spec
    // Sensor position in map frame from proj
    // Sensor orientation in map frame is R_m_n * R_n_s = R_m_s
    // This gives
    // T_m_s (With assumption reguarding heading)
    // T_m_s * T_s_b = T_m_b


    T_n_h;
    T_h_s;
    T_n_s = T_n_h * T_h_s;
    T_m_n(/*axis orientation*/, /*position*/)


    const tf2::Vector3 identity_trans(0,0,0);
    tf2::Quaternion heading_in_ned_quat;
    
    heading_in_ned_quat.setRPY(0, 0, fix_msg->track * wgs84_utils::DEG2RAD);


    tf2::Quaternion ned_in_map_quat = ned_in_map_quat_; // TODO pass in?

    tf2::Quaternion R_m_n = ned_in_map_quat_;
    tf2::Quaternion R_n_h = heading_in_ned_quat;
    tf2::Quaternion R_m_h = R_m_n * R_n_h; // Heading report orientation in map frame
    tf2::Vector3 sensor_in_map_translation(map_point.x(), map_point.y(), map_point.z());

    tf2::Transform sensor_in_map(, sensor_in_map_translation)


    tf2::Transform T_m_n(ned_in_map_quat);
    tf2::Transform T_n_h(heading_in_ned_quat);
    tf2::Transform T_h_s(sensor_in_ned_heading);
    tf2::Transform T_s_b(baselink_in_sensor);

    tf2::Transform T_m_b_rot_only = T_m_n * T_n_h * T_h_s * T_s_b




    // const tf2::Transform T_n_h(heading_in_ned_quat);

    // const tf2::Transform T_h_s(sensor_in_ned_heading);

    // const tf2::Transform T_n_s = T_n_h * T_h_s; // Transform defining sensor orientation in NED frame

    // const tf2::Transform T_s_b(baselink_in_sensor);

    // const tf2::Transform T_n_b = T_n_s * T_s_b; // Transform defining vehicle orientation in NED frame

    // TODO handle covariance

    const tf2::Transform T_e_b = T_e_n * T_n_b;

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = fix_msg->header;

    pose.pose.pose.position.x = T_e_b.getOrigin().getX();
    pose.pose.pose.position.y = T_e_b.getOrigin().getY();
    pose.pose.pose.position.z = T_e_b.getOrigin().getZ();

    pose.pose.pose.orientation.x = T_e_b.getRotation().getX();
    pose.pose.pose.orientation.y = T_e_b.getRotation().getY();
    pose.pose.pose.orientation.z = T_e_b.getRotation().getZ();
    pose.pose.pose.orientation.w = T_e_b.getRotation().getW();

    return pose;
  }
}
