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

#include <gnss_to_map_convertor/GNSSToMapNode.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <algorithm>

namespace gnss_to_map_convertor
{
GNSSToMapNode::GNSSToMapNode()
{
}

void GNSSToMapNode::fixCb(const gps_common::GPSFixConstPtr& fix_msg)
{
  // Get sensor and base_link transforms if not yet loaded
  if (!baselink_in_sensor_set_)
  {
    try
    {
      tf2::convert(tfBuffer_.lookupTransform(fix_msg->header.frame_id, base_link_frame_, ros::Time(0)).transform,
                   baselink_in_sensor_);
      tf2::convert(tfBuffer_.lookupTransform(ned_heading_frame_, fix_msg->header.frame_id, ros::Time(0)).transform,
                   sensor_in_ned_);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Ignoring fix message: Could not locate static transforms with exception " << ex.what());
      return;
    }
    baselink_in_sensor_set_ = true;
  }

  geometry_msgs::PoseWithCovarianceStamped ecef_pose =
      gnss_to_map_convertor::poseFromGnss(baselink_in_sensor_, sensor_in_ned_, fix_msg);
  ecef_pose.header.frame_id = earth_frame_;  // Set correct frame id

  tf2::Transform map_in_earth;
  try
  {
    // The map_in_earth transform should only change occasionally so no need to lookup a specific time
    tf2::convert(tfBuffer_.lookupTransform(earth_frame_, map_frame_, ros::Time(0)).transform, map_in_earth);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Ignoring re-initialization request: Could not lookup transform with exception " << ex.what());
    ecef_pose_pub_.publish(ecef_pose);  // Still publish valid ecef_pose even if the map lookup fails
    return;
  }

  // Store pose as transform
  tf2::Transform base_link_in_earth;
  tf2::convert(ecef_pose.pose.pose, base_link_in_earth);

  // Compute pose in map frame
  geometry_msgs::PoseStamped map_pose;
  map_pose.pose = gnss_to_map_convertor::ecefTFToMapPose(base_link_in_earth, map_in_earth);
  map_pose.header = fix_msg->header;
  map_pose.header.frame_id = map_frame_;

  // Publish ECEF and Map poses
  map_pose_pub_.publish(map_pose);
  ecef_pose_pub_.publish(ecef_pose);
}

std::string getAxisFromProjString(std::string proj_string)
{
  boost::erase_all(proj_string, " ");  // Remove white space from string

  ROS_DEBUG_STREAM("No white space georeference: " << map_georef);

  vector<string> strs;
  boost::split(strs, line, boost::is_any_of("+"));  // Split on + sign used to denote proj parameters

  std::string axis = "enu";  // Default axis alignment is ENU in proj

  for (const auto& param : strs)
  {
    ROS_DEBUG_STREAM("Extracted parameter: " << param);

    size_t axis_pos = strs[i].find("axis=");
    if (axis_pos != 0)
    {
      continue;
    }

    size_t equal_pos = strs[i].find("=");
    if (equal_pos >= strs[i].size() - 1)
    {
      throw std::invalid_argument("Cannot compute gps->map transform as provided georeference has an +axis tag which "
                                  "is empty: " +
                                  param);
    }

    axis = strs[i].substr(strs[i].find("=") + 1);
    ROS_DEBUG_STREAM("Identified axis alignment value: " << axis);

    break;  // If axis is found no reason to keep iterating.
  }

  return axis;
}

tf2::Quaternion getRotationOfNEDFromProjAxis(const std::string& axis)
{
  /*

    Proj Axis notation
    “e” - Easting
    “w” - Westing
    “n” - Northing
    “s” - Southing
    “u” - Up
    “d” - Down

    */

  tf2::Quaternion axis_in_ned;

  // Convert axis into transform with NED frame
  // Only support right handed coordinate systems at this time
  if (axis.compare("enu") == 0)
  {  // East North Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0,
                       90.0 * wgs84_utils::DEG2RAD);  // Convert from NED frame using fixed axis roll pitch yaw
  }
  else if (axis.compare("nwu") == 0)
  {  // North West Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, 0);
  }
  else if (axis.compare("wsu") == 0)
  {  // West South Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, -90.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("seu") == 0)
  {  // South East Up

    axis_in_ned.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, -180.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("ned") == 0)
  {  // North East Down

    axis_in_ned = tf2::Quaternion::getIdentity();
  }
  else if (axis.compare("wnd") == 0)
  {  // West North Down

    axis_in_ned.setRPY(0, 0, -90.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("swd") == 0)
  {  // South West Down

    axis_in_ned.setRPY(0, 0, 180.0 * wgs84_utils::DEG2RAD);
  }
  else if (axis.compare("esd") == 0)
  {  // East South Down

    axis_in_ned.setRPY(0, 0, 90.0 * wgs84_utils::DEG2RAD);
  }
  else
  {
    throw std::invalid_argument("GPS->map conversion only supports projections using right handed coordinate frames")
  }

  return axis_in_ned.inverse();
}

void WMBroadcaster::geoReferenceCallback(const std_msgs::String& geo_ref)
{
  // TODO axis extraction can be split into seperate function for unit testing
  std::string map_georef = geo_ref.data;

  ROS_INFO_STREAM("Recieved map georeference: " << map_georef);

  std::string axis = getAxisFromProjString(map_georef);

  tf2::Quaternion axis_in_ned_rot = getRotationFromNEDFromProjAxis(axis);

  // TODO Function probably can end here. Next junk of logic is thinking through the map  


  const tf2::Vector3 identity_trans(0, 0, 0);
  tf2::Quaternion heading_in_ned_quat;

  heading_in_ned_quat.setRPY(0, 0, fix_msg->track * wgs84_utils::DEG2RAD);

  const tf2::Transform T_n_h(heading_in_ned_quat, identity_trans);
}

int GNSSToMapNode::run()
{
  // Load parameters
  base_link_frame_ = p_cnh_.param("base_link_frame_id", base_link_frame_);
  earth_frame_ = p_cnh_.param("earth_frame_id", earth_frame_);
  map_frame_ = p_cnh_.param("map_frame_id", map_frame_);
  ned_heading_frame_ = p_cnh_.param("ned_heading_frame_id", ned_heading_frame_);
  // ECEF Pose publisher
  ecef_pose_pub_ = cnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ecef_pose_with_cov", 10);
  // Map pose publisher
  map_pose_pub_ = cnh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10, true);
  // Fix Subscriber
  fix_sub_ = cnh_.subscribe("gnss_fix_fused", 2, &GNSSToMapNode::fixCb, this);

  // Spin
  cnh_.spin();
  return 0;
}
}  // namespace gnss_to_map_convertor
