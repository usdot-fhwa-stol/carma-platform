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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <approximate_intersection/lookup_grid.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "points_map_filter/points_map_filter_config.hpp"

namespace points_map_filter
{

  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;
  /**
   * \brief TODO for USER: Add class description
   * 
   */
  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<sensor_msgs::msg::PointCloud2> points_sub_;
    carma_ros2_utils::SubPtr<autoware_lanelet2_msgs::msg::MapBin> map_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<sensor_msgs::msg::PointCloud2> filtered_points_pub_;

    // Node configuration
    Config config_;

    // The lanelet2 map to be checked against
    lanelet::LaneletMapPtr map_;

    approximate_intersection::LookupGrid<PointT> lookup_grid_;

    void recompute_lookup_grid();

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    /**
     * \brief Example callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Example subscription callback
     */
    void points_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg);

    void map_callback(autoware_lanelet2_msgs::msg::MapBin::UniquePtr msg);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

  };

} // points_map_filter
