#pragma once

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

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <wgs84_utils/wgs84_utils.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_gps_msgs/NovatelDualAntennaHeading.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

/**
 * \class GNSSToMapNode
 * \brief ROS Node which maintains a tf2 transform tree which can be accessed by other nodes.
 *
 * The get_transform service can be used to obtain coordinate transformations between two frames.
 * Only transforms published on the /tf or /tf_static topics are recorded by this node.
 */
namespace gnss_to_map_convertor
{
  geometry_msgs::PoseWithCovarianceStamped poseFromGnss(
    const tf2::Transform& baselink_in_sensor,
    const tf2::Transform& sensor_in_ned_heading,
    const sensor_msgs::NavSatFixConstPtr& fix_msg, 
    const novatel_gps_msgs::NovatelDualAntennaHeadingConstPtr& heading_msg
  );

  geometry_msgs::Pose ecefTFToMapPose(const tf2::Transform& baselink_in_earth, const tf2::Transform& map_in_earth);

};