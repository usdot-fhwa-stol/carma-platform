/*
 * Copyright (C) 2018-2022 LEIDOS.
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
#include "gnss_to_map_convertor/gnss_to_map_convertor_node.hpp"

namespace gnss_to_map_convertor
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options): carma_ros2_utils::CarmaLifecycleNode(options), tfBuffer_(get_clock()), tfListener_(tfBuffer_)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.base_link_frame = declare_parameter<std::string>("base_link_frame", config_.base_link_frame);
    config_.map_frame = declare_parameter<std::string>("map_frame", config_.map_frame);
    config_.heading_frame = declare_parameter<std::string>("heading_frame", config_.heading_frame);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<std::string>({{"base_link_frame", config_.base_link_frame},{"map_frame", config_.map_frame},{"heading_frame", config_.heading_frame}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<std::string>("base_link_frame", config_.base_link_frame);
    get_parameter<std::string>("map_frame", config_.map_frame);
    get_parameter<std::string>("heading_frame", config_.heading_frame);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    // Map pose publisher
    map_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", 10);

    // Initialize primary worker object 
    convertor_worker_ = std::make_shared<GNSSToMapConvertor>(
        [this](const auto& msg) { map_pose_pub->publish(msg); },  // Lambda representing publication

        [this](const auto& target, const auto& source) -> boost::optional<geometry_msgs::msg::TransformStamped> {  // Lambda representing transform lookup
          geometry_msgs::msg::TransformStamped tf;
          try
          {
            tf = tfBuffer_.lookupTransform(target, source, rclcpp::Time(1, 0));
          }
          catch (tf2::TransformException& ex)
          {
            RCLCPP_ERROR_STREAM(get_logger(),"Could not lookup transform with exception " << ex.what());
            return boost::none;
          }
          return tf;
        },

        config_.map_frame, config_.base_link_frame, config_.heading_frame, this->get_node_logging_interface());

    // Fix Subscriber

    fix_sub_ = create_subscription<gps_msgs::msg::GPSFix>("gnss_fix_fused", 2,
                                                          std::bind(&GNSSToMapConvertor::gnssFixCb, convertor_worker_.get(), std_ph::_1));

    // Georeference subsciber

    geo_sub = create_subscription<std_msgs::msg::String>("georeference", 1,
              std::bind(&GNSSToMapConvertor::geoReferenceCallback, convertor_worker_.get(), std_ph::_1));
    
                            
    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

} // gnss_to_map_convertor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(gnss_to_map_convertor::Node)
