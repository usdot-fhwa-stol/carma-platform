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

namespace mock_driver{

class DriverStatusPublisher : public rclcpp::Node {
public:
    DriverStatusPublisher() : Node("carma_carla_driver_status") {
        // Initialize publisher
        publisher_ = this->create_publisher<carma_driver_msgs::msg::DriverStatus>("/hardware_interface/driver_discovery", 10);

        // Initialize parameters
        this->declare_parameter("driver_status_pub_rate", 10);
        this->declare_parameter("lidar_enabled", true);
        this->declare_parameter("controller_enabled", true);
        this->declare_parameter("camera_enabled", true);
        this->declare_parameter("gnss_enabled", true);

        // Create timer
        auto timer_callback = [this]() -> void {
            this->publish_driver_status();
        };
        int pub_rate = this->get_parameter("driver_status_pub_rate").as_int();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / pub_rate), timer_callback);
    }

private:
    void publish_driver_status() {
        auto message = carma_driver_msgs::msg::DriverStatus();

        // Retrieve parameters
        message.lidar = this->get_parameter("lidar_enabled").as_bool();
        message.controller = this->get_parameter("controller_enabled").as_bool();
        message.camera = this->get_parameter("camera_enabled").as_bool();
        message.gnss = this->get_parameter("gnss_enabled").as_bool();

        // Set other message fields
        message.name = "/hardware_interface/carla_driver";
        message.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;

        // Publish message
        publisher_->publish(message);
    }

    rclcpp::Publisher<carma_driver_msgs::msg::DriverStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


}

#endif  // ROS2_MOCK_DRIVER_HPP_