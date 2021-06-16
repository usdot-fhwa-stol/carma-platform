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

namespace gnss_to_map_convertor
{
GNSSToMapConvertor::GNSSToMapConvertor(PosePubCallback pose_pub, TransformLookupCallback tf_lookup,
                                       std::string map_frame_id, std::string base_link_frame_id)
  : pose_pub_(pose_pub), tf_lookup_(tf_lookup), map_frame_id_(map_frame_id), base_link_frame_id_(base_link_frame_id)
{
}

void GNSSToMapConvertor::gnssFixCb(const gps_common::GPSFixConstPtr& fix_msg)
{
  if (!baselink_in_sensor_)  // Extract the assumed static transform between baselink and the sensor if it was not
                             // already optained
  {
    baselink_in_sensor_ = tf_lookup_(fix_msg->header.frame_id, base_link_frame_);

    if (!baselink_in_sensor_)  // Failed to get transform
    {
      ROS_WARN_STREAM("Ignoring fix message: Could not locate static transform between "
                      << fix_msg->header.frame_id << " and " << base_link_frame_);
      return;
    }
  }

  if (!map_projector_ || !ned_in_map_rotation_)  // Check if map projection is available
  {
    ROS_WARN_STREAM("Ignoring fix message as no map projection has been recieved.");
    return;
  }

  geometry_msgs::PoseWithCovarianceStamped pose_msg = poseFromGnss(
      baselink_in_sensor_.get(), map_projection_.get(), ned_in_map_rotation_.get(), fix_msg);  // Convert to pose

  geometry_msgs::PoseStamped msg; // TODO until covariance is added drop it from the output
  msg.header = pose_msg.header;
  msg.header.frame_id = map_frame_id_;
  msg.pose = pose_msg.pose.pose;

  pose_pub_(msg);
}

void GNSSToMapConvertor::geoReferenceCallback(const std_msgs::String& geo_ref)
{
  map_projector_(geo_ref.data);  // Build projector from proj string

  ROS_INFO_STREAM("Recieved map georeference: " << geo_ref.data.get());

  std::string axis =
      wgs84_utils::proj_tools::getAxisFromProjString(geo_ref.data.get());  // Extract axis for orientation calc

  ROS_INFO_STREAM("Extracted Axis: " << axis);

  ned_in_map_rotation_ =
      wgs84_utils::proj_tools::getRotationFromNEDFromProjAxis(axis);  // Extract map rotation from axis

  ROS_DEBUG_STREAM("Extracted NED in Map Rotation (x,y,z,w) : ( "
                   << ned_in_map_rotation_.x() << ", " << ned_in_map_rotation_.y() << ", " << ned_in_map_rotation_.z()
                   << ", " << ned_in_map_rotation_.w());
}

boost::optional<tf2::Quaternion> GNSSToMapConvertor::getNedInMapRotation()
{
  return ned_in_map_rotation_;
}

geometry_msgs::PoseWithCovarianceStamped poseFromGnss(const tf2::Transform& baselink_in_sensor,
                                                      const lanelet::projection::LocalFrameProjector& projector,
                                                      const tf2::Quaternion& ned_in_map_rotation,
                                                      const gps_common::GPSFixConstPtr& fix_msg)
{
  //// Convert the position information into the map frame using the proj library
  const double lat = fix_msg->latitude * wgs84_utils::DEG2RAD;
  const double lon = fix_msg->longitude * wgs84_utils::DEG2RAD;
  const double alt = fix_msg->altitude;

  lanelet2::BasicPoint3d map_point = projector.forward({ lat, lon, alt });

  if (fabs(map_point).x() > 10000.0 || fabs(map_point).y() > 10000.0)
  {  // Above 10km from map origin earth curvature will start to have a negative impact on system performance

    ROS_WARN_STREAM("Distance from map origin is larger than supported by system assumptions. Strongly advise "
                    "alternative map origin be used. ")
  }

  //// Convert the orientation information into the map frame
  // This logic assumes that the orientation difference between an NED frame located at the map origin and an NED frame
  // located at the GNSS point are sufficiently small that they can be ignored. Therefore it is assumed the heading
  // report of the GNSS system reguardless of its poition in the map without change in its orientation will give the
  // same result (as far as we are concered).

  tf2::Quaternion R_m_n(ned_in_map_rotation);  // Rotation of NED frame in map frame
  tf2::Quaternion R_n_s;                       // Rotation of sensor in NED frame
  R_n_s.setRPY(0, 0, fix_msg->track * wgs84_utils::DEG2RAD);

  tf2::Quaternion R_m_s =
      R_m_n * R_n_s;  // Rotation of sensor in map frame under assumption that distance from map origin is sufficiently
                      // small so as to ignore local changes in NED orientation

  tf2::Transform T_m_s(R_m_s,
                       tf2::Vector3(map_point.x(), map_point.y(), map_point.z()));  // Reported position and orientation
                                                                                    // of sensor frame in map frame
  tf2::Transform T_s_b(baselink_in_sensor);  // Transform between sensor and baselink frame
  tf2::Transform T_m_b = T_m_s * T_s_b;      // Transform between map and baselink frame

  // TODO handle covariance

  // Populate message
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = fix_msg->header;

  pose.pose.pose.position.x = T_m_b.getOrigin().getX();
  pose.pose.pose.position.y = T_m_b.getOrigin().getY();
  pose.pose.pose.position.z = T_m_b.getOrigin().getZ();

  pose.pose.pose.orientation.x = T_m_b.getRotation().getX();
  pose.pose.pose.orientation.y = T_m_b.getRotation().getY();
  pose.pose.pose.orientation.z = T_m_b.getRotation().getZ();
  pose.pose.pose.orientation.w = T_m_b.getRotation().getW();

  return pose;
}
}  // namespace gnss_to_map_convertor
