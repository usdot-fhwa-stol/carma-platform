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
class GNSSToMapNode
{
public:
  int run()
  {
    // Buffer which holds the tree of transforms
    tf2_ros::Buffer tfBuffer;
    // tf2 listeners. Subscribes to the /tf and /tf_static topics
    tf2_ros::TransformListener tfListener{ tfBuffer_ };

    // Load parameters
    std::string base_link_frame = p_cnh_.param("base_link_frame_id", base_link_frame_);
    std::string map_frame = p_cnh_.param("map_frame_id", map_frame_);
    // Map pose publisher
    ros::Publisher map_pose_pub = cnh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10, true);

    GNSSToMapConvertor convertor([&map_pose_pub](const auto& msg) { map_pose_pub_.publish(msg); },

                                 [&tf_buffer](const auto& target, const auto& source) {
                                   tf2::Transform tf;
                                   try
                                   {
                                     tf2::convert(tf_buffer.lookupTransform(target, source, ros::Time(0)).transform,
                                                  tf);
                                   }
                                   catch (tf2::TransformException& ex)
                                   {
                                     ROS_ERROR_STREAM("Could not lookup transform with exception " << ex.what());
                                     return boost::none;
                                   }
                                   return tf;
                                 },

                                 map_frame, base_link_frame);

    // Fix Subscriber
    ros::Subscriber fix_sub_ = cnh_.subscribe("gnss_fix_fused", 2, &GNSSToMapConvertor::gnssFixCb, &convertor);

    // Spin
    cnh_.spin();
    return 0;
  }
}
}  // namespace gnss_to_map_convertor
