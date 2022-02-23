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
#include <geometry_msgs/msg/pose_array.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>


#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace motion_prediction_visualizer
{
    
class Node : public carma_ros2_utils::CarmaLifecycleNode
  {
    private:
    
    carma_ros2_utils::SubPtr<carma_perception_msgs::msg::ExternalObjectList> external_object_sub_;

    carma_ros2_utils::PubPtr<geometry_msgs::msg::PoseArray> pose_array_pub_;


    public:

    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);

    void external_object_callback(const carma_perception_msgs::msg::ExternalObjectList::UniquePtr msg);
    
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);


  };
} //motion_prediction_visualizer