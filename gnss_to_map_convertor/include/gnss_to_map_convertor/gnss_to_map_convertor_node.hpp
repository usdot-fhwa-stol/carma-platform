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

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <string>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>

#include "gnss_to_map_convertor/gnss_to_map_convertor_config.hpp"
#include "gnss_to_map_convertor/GNSSToMapConvertor.hpp"

namespace gnss_to_map_convertor
{

  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {
  private:

    // Node configuration
    Config config_;

    // Buffer which holds the tree of transforms

    tf2_ros::Buffer tfBuffer_;
    
    // tf2 listeners. Subscribes to the /tf and /tf_static topics
    tf2_ros::TransformListener tfListener_ {tfBuffer_};

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseStamped> map_pose_pub;

    carma_ros2_utils::SubPtr<gps_msgs::msg::GPSFix> fix_sub_;

    carma_ros2_utils::SubPtr<std_msgs::msg::String> geo_sub;

  };

} // gnss_to_map_convertor
