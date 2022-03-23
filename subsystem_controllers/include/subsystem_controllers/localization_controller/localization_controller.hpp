#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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

#include <memory>

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller.hpp"
#include "subsystem_controllers/localization_controller/localization_controller_config.hpp"

namespace subsystem_controllers
{

  class LocalizationControllerNode : public BaseSubsystemController
  {
  public:
    LocalizationControllerNode() = delete;

    ~LocalizationControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit LocalizationControllerNode(const rclcpp::NodeOptions &options);

    ////
    // Overrides
    ////
    void on_system_alert(const carma_msgs::msg::SystemAlert::UniquePtr msg);
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);

  protected:
    /**
     * \brief Helper function to convert a json string into an unordered map of sensor status's to required alerts
     * TODO add example
     */ 
    std::unordered_map<std::vector<SensorBooleanStatus>, SensorAlertStatus, VectorHash> sensor_fault_map_from_json(std::string json_string);

    // A map of sensor node names to their status and index in the config_.sensor_nodes list
    std::unordered_map<std::string, SensorBooleanStatus> sensor_status_;
    LocalizationControllerConfig config_;

    // TODO The ROS1 localization manager functionality should be updated to properly interact with or exist in this node
    // https://github.com/usdot-fhwa-stol/carma-platform/issues/1498 

  };

} // namespace subsystem_controllers
