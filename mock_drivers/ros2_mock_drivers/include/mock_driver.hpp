// Copyright 2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_MOCK_DRIVER_HPP_
#define ROS2_MOCK_DRIVER_HPP_


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <carma_driver_msgs/msg/driver_status.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "mock_driver.hpp"

namespace mock_driver{

class DriverStatusPublisher : public rclcpp::Node {
public:
    DriverStatusPublisher(const rclcpp::NodeOptions &options) : rclcpp::Node("carla_carma_driver_status", options) {
        init();
    }

private:
    void init();
    void publish_controller_driver_status();
    void publish_camera_driver_status();
    void publish_lidar_driver_status();
    void publish_gnss_driver_status();

    rclcpp::Publisher<carma_driver_msgs::msg::DriverStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


}

#endif  // ROS2_MOCK_DRIVER_HPP_