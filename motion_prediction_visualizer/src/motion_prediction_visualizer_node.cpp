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

#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "motion_prediction_visualizer/motion_prediction_visualizer.hpp"

namespace motion_prediction_visualizer
{
    namespace std_ph = std::placeholders;

    Node::Node(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
    {
        
    }

    void Node::external_object_callback(const carma_perception_msgs::msg::ExternalObjectList::UniquePtr msg)
    {
        geometry_msgs::msg::PoseArray posearray;
        posearray.header.stamp = this->now();
        posearray.header.frame_id = "map";

        for (auto& obj : msg->objects){
            for (auto& p : obj.predictions) {
                posearray.poses.push_back(p.predicted_position);
            }
        }

        pose_array_pub_->publish(posearray);
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        // Setup subscribers
        external_object_sub_ = create_subscription<carma_perception_msgs::msg::ExternalObjectList>("external_objects", 1000,
                                                              std::bind(&Node::external_object_callback, this, std_ph::_1));

        // Setup publishers
        pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("motion_computation_visualize", 1);

        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;

    }

}// motion_prediction_visualizer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(motion_prediction_visualizer::Node)


