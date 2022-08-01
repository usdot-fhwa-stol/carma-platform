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
#include "points_map_filter/points_map_filter_node.hpp"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/regulatory_elements/RegionAccessRule.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_extension/regulatory_elements/DirectionOfTravel.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/regulatory_elements/DigitalMinimumGap.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

// NOTE: The following include can be changed to <pcl/filters/experimental/functor_filter.h>
//       once the underlying PCL version is updated to 1.12.1 which should occur with a ROS version update
#include "pcl_expirimental-1.12.1/functor_filter.h"

namespace points_map_filter
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.cell_side_length = declare_parameter<double>("cell_side_length", config_.cell_side_length);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({{"cell_side_length", config_.cell_side_length}}, parameters);

    if (!error)
    {
      auto lookup_config = lookup_grid_.get_config();
      lookup_config.cell_side_length = config_.cell_side_length;

      lookup_grid_ = approximate_intersection::LookupGrid<PointT>(lookup_config);
      recompute_lookup_grid();
    }

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("cell_side_length", config_.cell_side_length);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    RCLCPP_INFO_STREAM(get_logger(), "Loaded " << config_);

    // Setup subscribers
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("points_raw", 10,
                                                                     std::bind(&Node::points_callback, this, std_ph::_1));

    // NOTE: Currently, intra-process comms must be disabled for subcribers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
    rclcpp::SubscriptionOptions map_sub_options; 
    map_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this SubscriptionOptions object

    auto map_sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)); // Set the queue size for a subscriber with this QoS
    map_sub_qos.transient_local();  // If it is possible that this node is a late-joiner to its topic, it must be set to transient_local to receive earlier messages that were missed.
                                    // NOTE: The publisher's QoS must be set to transisent_local() as well for earlier messages to be resent to this later-joiner.

    map_sub_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>("lanelet2_map", map_sub_qos,
                                                                        std::bind(&Node::map_callback, this, std_ph::_1), map_sub_options);

    // Setup publishers
    filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void Node::points_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {

    const auto input_cloud = pcl::make_shared<CloudT>();
    const auto filtered_cloud = pcl::make_shared<CloudT>();

    pcl::moveFromROSMsg(*msg, *input_cloud);

    pcl::experimental::FilterFunction<PointT> filter =
        [this](const CloudT &cloud, pcl::index_t idx)
    {
      return this->lookup_grid_.intersects(cloud[idx]);
    };

    // build the filter
    pcl::experimental::FunctionFilter<PointT> func_filter(filter);
    func_filter.setInputCloud(input_cloud);

    // apply filter
    func_filter.filter(*filtered_cloud);

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*filtered_cloud, out_msg);

    filtered_points_pub_->publish(out_msg);
  }

  namespace
  {
    /*
    * NOTE: The following function is taken from the Autoware.ai system's 
    *       lanelet2_extension library and is ported here as a ROS2 version
    *       Once this functionality is fully ported to ROS2 this function 
    *       can be replaced with a library call
    *
    * Copyright 2015-2019 Autoware Foundation. All rights reserved.
    *
    * Licensed under the Apache License, Version 2.0 (the "License");
    * you may not use this file except in compliance with the License.
    * You may obtain a copy of the License at
    *
    *     http://www.apache.org/licenses/LICENSE-2.0
    *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS,
    * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    * See the License for the specific language governing permissions and
    * limitations under the License.
    *
    * Authors: Simon Thompson, Ryohsuke Mitsudome
    */
    void fromBinMsg(const autoware_lanelet2_msgs::msg::MapBin &msg, lanelet::LaneletMapPtr map)
    {
      if (!map)
      {
        return;
      }

      std::string data_str;
      data_str.assign(msg.data.begin(), msg.data.end());

      std::stringstream ss;
      ss << data_str;
      boost::archive::binary_iarchive oa(ss);
      oa >> *map;
      lanelet::Id id_counter;
      oa >> id_counter;
      lanelet::utils::registerId(id_counter);
    }
  }

  void Node::recompute_lookup_grid()
  {

    if (!map_)
    {
      RCLCPP_ERROR(get_logger(), "No map received yet. Cannot recompute lookup grid");
      return;
    }

    for (const auto &point : map_->pointLayer)
    {
      PointT p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      lookup_grid_.insert(p);
    }
  }

  void Node::map_callback(autoware_lanelet2_msgs::msg::MapBin::UniquePtr msg)
  {

    lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

    fromBinMsg(*msg, new_map);

    map_ = new_map;

    approximate_intersection::Config intersection_config;
    intersection_config.cell_side_length = config_.cell_side_length;

    // TODO it would be great if lanelet2 already knew the map bounds and this iteration didn't need to happen twice
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    for (const auto &point : map_->pointLayer)
    {
      if (point.x() < min_x)
        min_x = point.x();

      if (point.x() > max_x)
        max_x = point.x();

      if (point.y() < min_y)
        min_y = point.y();

      if (point.y() > max_y)
        max_y = point.y();
    }

    double three_cells = config_.cell_side_length * 3;
    intersection_config.min_x = min_x - three_cells;
    intersection_config.max_x = max_x + three_cells;
    intersection_config.min_y = min_y - three_cells;
    intersection_config.max_y = max_y + three_cells;

    RCLCPP_INFO_STREAM(get_logger(), "Found base map bounds of (min_x, max_x, min_y, max_y): "
                                         << "( " << min_x << ", " << max_x << ", " << min_y << ", " << max_y << ")");
    RCLCPP_INFO_STREAM(get_logger(), "Expanded map bounds to: "
                                         << "( " << intersection_config.min_x << ", " << intersection_config.max_x << ", " << intersection_config.min_y << ", " << intersection_config.max_y << ")");

    lookup_grid_ = approximate_intersection::LookupGrid<PointT>(intersection_config);

    recompute_lookup_grid();
  }

} // points_map_filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(points_map_filter::Node)
