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

#include "carma_guidance_plugins/strategic_plugin.hpp"
#include "carma_guidance_plugins/tactical_plugin.hpp"
#include "carma_guidance_plugins/control_plugin.hpp"

namespace carma_guidance_plugins
{
  class TestStrategicPlugin : public StrategicPlugin
  {
  public:

    using StrategicPlugin::StrategicPlugin;

    ~TestStrategicPlugin() override = default;


    void plan_maneuvers_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr, 
      carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr) override
    {
       // Intentionally unimplemented...
    }

    bool get_availability() override {
      return true;
    }

    std::string get_plugin_name() override {
      return "TestStrategicPlugin";
    }

    std::string get_version_id() override {
      return "1.0";
    }
    
    carma_ros2_utils::CallbackReturn on_configure_plugin() override
    {
      RCLCPP_INFO(get_logger(), "Configuring TestStrategicPlugin");
      return carma_ros2_utils::CallbackReturn::SUCCESS;
    }

    std::string get_capability() override {
      return StrategicPlugin::get_capability() + "/test_capability";
    };

  };

  class TestTacticalPlugin : public TacticalPlugin
  {
  public:

    using TacticalPlugin::TacticalPlugin;

    ~TestTacticalPlugin() override = default;


    void plan_trajectory_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr, 
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr) override
    {
       // Intentionally unimplemented...
    }

    bool get_availability() override {
      return true;
    }

    std::string get_plugin_name() override {
      return "TestTacticalPlugin";
    }

    std::string get_version_id() override {
      return "1.1";
    }
    
    carma_ros2_utils::CallbackReturn on_configure_plugin() override
    {
      RCLCPP_INFO(get_logger(), "Configuring TestTacticalPlugin");
      return carma_ros2_utils::CallbackReturn::SUCCESS;
    }

    std::string get_capability() override {
      return TacticalPlugin::get_capability() + "/test_capability";
    };

  };

  class TestControlPlugin : public ControlPlugin
  {
  public:

    using ControlPlugin::ControlPlugin;

    ~TestControlPlugin() override = default;


    autoware_msgs::msg::ControlCommandStamped generate_command() override
    {
      autoware_msgs::msg::ControlCommandStamped msg;
      return msg;
    }

    bool get_availability() override {
      return true;
    }

    std::string get_plugin_name() override {
      return "TestControlPlugin";
    }

    std::string get_version_id() override {
      return "1.2";
    }
    
    carma_ros2_utils::CallbackReturn on_configure_plugin() override
    {
      RCLCPP_INFO(get_logger(), "Configuring TestControlPlugin");

      return carma_ros2_utils::CallbackReturn::SUCCESS;
    }

    std::string get_capability() override {
      return ControlPlugin::get_capability() + "/test_capability";
    };

  };
}