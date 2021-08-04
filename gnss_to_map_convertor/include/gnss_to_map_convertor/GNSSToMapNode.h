#pragma once
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

#include <gnss_to_map_convertor/GNSSToMapConvertor.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <carma_utils/CARMANodeHandle.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <algorithm>

namespace gnss_to_map_convertor
{
class GNSSToMapNode
{
public:
  /**
   * \brief Run this node. This method is responsable for setting up the ros network wiring and starting callback execution.
   * \return Linux error code if needed. 0 on no error 
   */ 
  int run()
  {
    // Buffer which holds the tree of transforms
    tf2_ros::Buffer tfBuffer;
    // tf2 listeners. Subscribes to the /tf and /tf_static topics
    tf2_ros::TransformListener tfListener{ tfBuffer };

    ros::CARMANodeHandle cnh;
    ros::CARMANodeHandle pnh("~");

    // Load parameters
    std::string base_link_frame;
    pnh.param<std::string>("base_link_frame_id", base_link_frame, "base_link");
    
    std::string map_frame;
    pnh.param<std::string>("map_frame_id", map_frame, "map");

    std::string heading_frame;
    pnh.param<std::string>("heading_frame_id", heading_frame, "ned_heading");

    // Map pose publisher
    ros::Publisher map_pose_pub = cnh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10, true);

    // Initialize primary worker object 
    GNSSToMapConvertor convertor(
        [&map_pose_pub](const auto& msg) { map_pose_pub.publish(msg); },  // Lambda representing publication

        [&tfBuffer](const auto& target, const auto& source) -> boost::optional<geometry_msgs::TransformStamped> {  // Lambda representing transform lookup
          geometry_msgs::TransformStamped tf;
          try
          {
            tf = tfBuffer.lookupTransform(target, source, ros::Time(0));
          }
          catch (tf2::TransformException& ex)
          {
            ROS_ERROR_STREAM("Could not lookup transform with exception " << ex.what());
            return boost::none;
          }
          return tf;
        },

        map_frame, base_link_frame, heading_frame);

    // Fix Subscriber
    ros::Subscriber fix_sub_ = cnh.subscribe("gnss_fix_fused", 2, &GNSSToMapConvertor::gnssFixCb, &convertor);

    // Georeference subsciber
    ros::Subscriber geo_sub = cnh.subscribe("georeference", 1, &GNSSToMapConvertor::geoReferenceCallback, &convertor);

    // Spin
    cnh.spin();
    return 0;
  }
};
}  // namespace gnss_to_map_convertor
