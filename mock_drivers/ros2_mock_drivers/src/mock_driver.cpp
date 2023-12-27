
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

#include "mock_driver.hpp"

namespace mock_driver
{

    void DriverStatusPublisher::init()
    {
        // Initialize publisher
        RCLCPP_ERROR_STREAM(this->get_logger(), "Print1");
        publisher_ = this->create_publisher<carma_driver_msgs::msg::DriverStatus>("/hardware_interface/driver_discovery", 10);

        while (!this->get_clock()->ros_time_is_active())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Waiting for ROS time to be available");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        // Initialize parameters
        this->declare_parameter("driver_status_pub_rate", 10);

        RCLCPP_ERROR_STREAM(this->get_logger(), "Print2");

        // Create timer
        RCLCPP_ERROR_STREAM(this->get_logger(), "Print3");

        int pub_rate = this->get_parameter("driver_status_pub_rate").as_int();
        timer_ = rclcpp::create_timer(this,
            get_clock(),
            rclcpp::Duration(1000 / pub_rate * 1e6), [this]() -> void {
            this->publish_controller_driver_status();
            this->publish_lidar_driver_status();
            this->publish_camera_driver_status();
            this->publish_gnss_driver_status();
        });
        RCLCPP_ERROR_STREAM(this->get_logger(), "Print4");
    }
     void DriverStatusPublisher::publish_controller_driver_status() {
        carma_driver_msgs::msg::DriverStatus message;
        RCLCPP_ERROR_STREAM(this->get_logger(), "Print5");

        // Set other message fields
        message.name = "/hardware_interface/carla_driver";
        message.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        message.controller = true;

        // Publish message
        publisher_->publish(message);
        RCLCPP_ERROR_STREAM(this->get_logger(), "Print6");

    }

    void DriverStatusPublisher::publish_camera_driver_status() {
        carma_driver_msgs::msg::DriverStatus message;

        // Set other message fields
        message.name = "/hardware_interface/carla_camera_driver";
        message.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        message.camera = true;

        // Publish message
        publisher_->publish(message);
    }

    void DriverStatusPublisher::publish_lidar_driver_status() {
        carma_driver_msgs::msg::DriverStatus message;

        // Set other message fields
        message.name = "/hardware_interface/carla_lidar_driver";
        message.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        message.lidar = true;
        // Publish message
        publisher_->publish(message);
    }

    void DriverStatusPublisher::publish_gnss_driver_status() {
        carma_driver_msgs::msg::DriverStatus message;

        // Set other message fields
        message.name = "/hardware_interface/carla_gnss_driver";
        message.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        message.gnss = true;

        // Publish message
        publisher_->publish(message);
    }
}