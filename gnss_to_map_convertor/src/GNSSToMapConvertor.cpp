/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include "gnss_to_map_convertor/GNSSToMapConvertor.hpp"
#include <wgs84_utils/proj_tools.h>
#include <lanelet2_core/geometry/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gnss_to_map_convertor
{
GNSSToMapConvertor::GNSSToMapConvertor(PosePubCallback pose_pub, TransformLookupCallback tf_lookup,
                                       std::string map_frame_id, std::string base_link_frame_id,
                                       std::string heading_frame_id,rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger)
  : pose_pub_(pose_pub)
  , tf_lookup_(tf_lookup)
  , map_frame_id_(map_frame_id)
  , base_link_frame_id_(base_link_frame_id)
  , heading_frame_id_(heading_frame_id) 
  , logger_(logger)
{
}

void GNSSToMapConvertor::gnssFixCb(gps_msgs::msg::GPSFix::UniquePtr fix_msg)
{
  if (!baselink_in_sensor_)  // Extract the assumed static transform between baselink and the sensor if it was not
                             // already optained
  {
    auto tf_msg = tf_lookup_(fix_msg->header.frame_id, base_link_frame_id_);
    if (!tf_msg)  // Failed to get transform
    {
      RCLCPP_WARN_STREAM(logger_->get_logger(),"Ignoring fix message: Could not locate static transform between "
                     << fix_msg->header.frame_id << " and " << base_link_frame_id_);
      return;
    }

    tf2::Transform tf;
    tf2::convert(tf_msg.get().transform, tf);

    baselink_in_sensor_ = tf;
  }

  if (!sensor_in_ned_heading_rotation_)  // Extract the assumed static transform between heading frame and the position
                                         // sensor frame if it was not already optained
  {
    auto tf_msg = tf_lookup_(fix_msg->header.frame_id, heading_frame_id_);
    if (!tf_msg)  // Failed to get transform
    {
      RCLCPP_WARN_STREAM(logger_->get_logger(), "Ignoring fix message: Could not locate static transform between " << heading_frame_id_ << " and "
                                                                                        << fix_msg->header.frame_id);
      return;
    }

    tf2::Transform tf;
    tf2::convert(tf_msg.get().transform, tf);

    if (tf.getOrigin().x() != 0.0 || tf.getOrigin().y() != 0.0 || tf.getOrigin().z() != 0.0)
    {
      RCLCPP_WARN_STREAM(logger_->get_logger(), "Heading frame does not have rotation only transform with sensor frame. The translation will not "
                     "be handled by the GNSS convertor");
    }

    sensor_in_ned_heading_rotation_ = tf.getRotation();
  }

  if (!map_projector_ || !ned_in_map_rotation_)  // Check if map projection is available
  {
    RCLCPP_WARN_STREAM(logger_->get_logger(), "Ignoring fix message as no map projection has been recieved.");
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      poseFromGnss(baselink_in_sensor_.get(), sensor_in_ned_heading_rotation_.get(), *map_projector_, ned_in_map_rotation_.get(), *fix_msg);  // Convert to pose

  geometry_msgs::msg::PoseStamped msg;  // TODO until covariance is added drop it from the output
  msg.header = pose_msg.header;
  msg.header.frame_id = map_frame_id_;
  msg.pose = pose_msg.pose.pose;

  pose_pub_(msg);
}

void GNSSToMapConvertor::geoReferenceCallback(std_msgs::msg::String::UniquePtr geo_ref)
{

  map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(
      geo_ref->data.c_str());  // Build projector from proj string

  RCLCPP_INFO_STREAM(logger_->get_logger(), "Recieved map georeference: " << geo_ref->data);

  std::string axis = wgs84_utils::proj_tools::getAxisFromProjString(geo_ref->data);  // Extract axis for orientation calc

  RCLCPP_INFO_STREAM(logger_->get_logger(), "Extracted Axis: " << axis);

  ned_in_map_rotation_ = wgs84_utils::proj_tools::getRotationOfNEDFromProjAxis(axis);  // Extract map rotation from axis

  RCLCPP_INFO_STREAM(logger_->get_logger(), "Extracted NED in Map Rotation (x,y,z,w) : ( "
                  << ned_in_map_rotation_.get().x() << ", " << ned_in_map_rotation_.get().y() << ", "
                  << ned_in_map_rotation_.get().z() << ", " << ned_in_map_rotation_.get().w());
}

boost::optional<tf2::Quaternion> GNSSToMapConvertor::getNedInMapRotation()
{
  return ned_in_map_rotation_;
}

std::shared_ptr<lanelet::projection::LocalFrameProjector> GNSSToMapConvertor::getMapProjector()
{
  return map_projector_;
}


geometry_msgs::msg::PoseWithCovarianceStamped GNSSToMapConvertor::poseFromGnss(
    const tf2::Transform& baselink_in_sensor, 
    const tf2::Quaternion& sensor_in_ned_heading_rotation,
    const lanelet::projection::LocalFrameProjector& projector, 
    const tf2::Quaternion& ned_in_map_rotation,
    gps_msgs::msg::GPSFix fix_msg)
{
  //// Convert the position information into the map frame using the proj library
  const double lat = fix_msg.latitude;
  const double lon = fix_msg.longitude;
  const double alt = fix_msg.altitude;

  lanelet::BasicPoint3d map_point = projector.forward({ lat, lon, alt });
  RCLCPP_DEBUG_STREAM(logger_->get_logger(), "map_point: " << map_point.x() << ", " << map_point.y() << ", " << map_point.z());

  if (fabs(map_point.x()) > 10000.0 || fabs(map_point.y()) > 10000.0)
  {  // Above 10km from map origin earth curvature will start to have a negative impact on system performance

    RCLCPP_WARN_STREAM(logger_->get_logger(), "Distance from map origin is larger than supported by system assumptions. Strongly advise "
                   "alternative map origin be used. ");
  }

  //// Convert the orientation information into the map frame
  // This logic assumes that the orientation difference between an NED frame located at the map origin and an NED frame
  // located at the GNSS point are sufficiently small that they can be ignored. Therefore it is assumed the heading
  // report of the GNSS system reguardless of its poition in the map without change in its orientation will give the
  // same result (as far as we are concered).

  tf2::Quaternion R_m_n(ned_in_map_rotation);  // Rotation of NED frame in map frame
  tf2::Quaternion R_n_h;                       // Rotation of sensor heading report in NED frame
  R_n_h.setRPY(0, 0, fix_msg.track * wgs84_utils::DEG2RAD);

  tf2::Quaternion R_h_s = sensor_in_ned_heading_rotation;  // Rotation of heading report in sensor frame

  tf2::Quaternion R_m_s =
      R_m_n * R_n_h * R_h_s;  // Rotation of sensor in map frame under assumption that distance from map origin is
                              // sufficiently small so as to ignore local changes in NED orientation

    RCLCPP_DEBUG_STREAM(logger_->get_logger(), "R_m_n (x,y,z,w) : ( "
                  << R_m_n.x() << ", " << R_m_n.y() << ", "
                  << R_m_n.z() << ", " << R_m_n.w());
    
    RCLCPP_DEBUG_STREAM(logger_->get_logger(), "R_n_h (x,y,z,w) : ( "
                  << R_n_h.x() << ", " << R_n_h.y() << ", "
                  << R_n_h.z() << ", " << R_n_h.w());
    
    RCLCPP_DEBUG_STREAM(logger_->get_logger(), "R_h_s (x,y,z,w) : ( "
                  << R_h_s.x() << ", " << R_h_s.y() << ", "
                  << R_h_s.z() << ", " << R_h_s.w());

    RCLCPP_DEBUG_STREAM(logger_->get_logger(), "R_m_s (x,y,z,w) : ( "
                  << R_m_s.x() << ", " << R_m_s.y() << ", "
                  << R_m_s.z() << ", " << R_m_s.w());

  tf2::Transform T_m_s(R_m_s,
                       tf2::Vector3(map_point.x(), map_point.y(), map_point.z()));  // Reported position and orientation
                                                                                    // of sensor frame in map frame
  tf2::Transform T_s_b(baselink_in_sensor);  // Transform between sensor and baselink frame
  tf2::Transform T_m_b = T_m_s * T_s_b;      // Transform between map and baselink frame

  RCLCPP_DEBUG_STREAM(logger_->get_logger(), "T_m_s (x,y,z,w) : ( "
               << T_m_s.getRotation().x() << ", " << T_m_s.getRotation().y() << ", "
               << T_m_s.getRotation().z() << ", " << T_m_s.getRotation().w());
  RCLCPP_DEBUG_STREAM(logger_->get_logger(), "T_s_b (x,y,z,w) : ( "
               << T_s_b.getRotation().x() << ", " << T_s_b.getRotation().y() << ", "
               << T_s_b.getRotation().z() << ", " << T_s_b.getRotation().w());
  RCLCPP_DEBUG_STREAM(logger_->get_logger(), "T_m_b (x,y,z,w) : ( "
               << T_m_b.getRotation().x() << ", " << T_m_b.getRotation().y() << ", "
               << T_m_b.getRotation().z() << ", " << T_m_b.getRotation().w());

  // TODO handle covariance

  // Populate message
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = fix_msg.header;

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
