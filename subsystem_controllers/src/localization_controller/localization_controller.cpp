
/*
 * Copyright (C) 2021-2022 LEIDOS.
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

#include "subsystem_controllers/localization_controller/localization_controller.hpp"

namespace subsystem_controllers
{

  
  LocalizationControllerNode::LocalizationControllerNode(const rclcpp::NodeOptions &options)
      : BaseSubsystemController(options)
  {
    config_.sensor_nodes = declare_parameter<std::vector<std::string>>("sensor_nodes", config_.sensor_nodes);
    config_.sensor_fault_map = sensor_fault_map_from_json(declare_parameter<std::string>("sensor_fault_map", ""));
  
    // Handle fact that parameter vectors cannot be empty
    if (config_.sensor_nodes.size() == 1 && config_.sensor_nodes[0].empty()) {
      config_.sensor_nodes.clear();
    }
  
  }

  carma_ros2_utils::CallbackReturn LocalizationControllerNode::handle_on_configure(const rclcpp_lifecycle::State &prev_state) {
    auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

    if (base_return != carma_ros2_utils::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Localization Controller could not configure");
      return base_return;
    }

    // Reset and reload config
    config_ = LocalizationControllerConfig();
    get_parameter<std::vector<std::string>>("sensor_nodes", config_.sensor_nodes);
    
    // Handle fact that parameter vectors cannot be empty
    if (config_.sensor_nodes.size() == 1 && config_.sensor_nodes[0].empty()) {
      config_.sensor_nodes.clear();
    }
    
    std::string json_string;
    get_parameter<std::string>("sensor_fault_map", json_string);
    
    RCLCPP_INFO_STREAM(get_logger(), "Loaded sensor_fault_map json string: " << json_string);
    
    config_.sensor_fault_map = sensor_fault_map_from_json(json_string);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded localization config: " << config_);

    for (auto node : config_.sensor_nodes) {
      sensor_status_[node] = SensorBooleanStatus::OPERATIONAL; // Start tracking the sensor status's
    }

    return carma_ros2_utils::CallbackReturn::SUCCESS;
  }

  void LocalizationControllerNode::on_system_alert(carma_msgs::msg::SystemAlert::UniquePtr msg)
  {
    carma_msgs::msg::SystemAlert alert = *msg; // Keep local copy so parent method can be called with std::move
    carma_msgs::msg::SystemAlert output_alert;
    BaseSubsystemController::on_system_alert(std::move(msg)); // Call parent method

    // Check if our list of coordinated sensors contains the node providing the alert
    // No need to check required nodes as that has already been handled in the parent call
    if (sensor_status_.find(alert.source_node) == sensor_status_.end()) {
      RCLCPP_INFO_STREAM(get_logger(), "Alert not from a sensor node. Alerting node: " << alert.source_node);
      return;
    }

    // Update our tracking of sensor statuses if this node failed
    if (alert.type == carma_msgs::msg::SystemAlert::FATAL) {
      sensor_status_[alert.source_node] = SensorBooleanStatus::FAILED;
    }

    // Combine all sensor statuses into a single set to index into the alert map
    std::vector<SensorBooleanStatus> current_sensor_status;
    for (const auto& node : config_.sensor_nodes) {
      current_sensor_status.emplace_back(sensor_status_[node]);
    }

    // If the controller has not been configured to handle this fault configuration then do nothing
    if (config_.sensor_fault_map.find(current_sensor_status) == config_.sensor_fault_map.end()) {
      RCLCPP_INFO_STREAM(get_logger(), "Failure does not match with fault configuration. Assuming system remains operational ");
      return;
    }

    // Extract the alert required by this type of sensor statuses
    SensorAlertStatus alert_status = config_.sensor_fault_map[current_sensor_status];
    output_alert.description = "Localization optional sensor failure from: " + alert.source_node;

    switch(alert_status) {
      case SensorAlertStatus::FATAL:
        output_alert.type = carma_msgs::msg::SystemAlert::FATAL;
        output_alert.description = "Localization subsystem failed due to sensor failure from: " + alert.source_node;
        break;
      case SensorAlertStatus::CAUTION:
        output_alert.type = carma_msgs::msg::SystemAlert::CAUTION;
        break;
      case SensorAlertStatus::WARNING:
        output_alert.type = carma_msgs::msg::SystemAlert::WARNING;
        break;
      default:
      return; // OPERATIONAL case no-op as other subsystems do not need notification of operational status
    }

    // Publish the alert
    output_alert.source_node =  get_node_base_interface()->get_fully_qualified_name();
    publish_system_alert(output_alert);
  }

  std::unordered_map<std::vector<SensorBooleanStatus>, SensorAlertStatus, VectorHash> LocalizationControllerNode::sensor_fault_map_from_json(std::string json_string) {
      
    std::unordered_map<std::vector<SensorBooleanStatus>, SensorAlertStatus, VectorHash> map;
    
    rapidjson::Document d;

    if(d.Parse(json_string.c_str()).HasParseError())
    {
        RCLCPP_WARN(get_logger(), "Failed to parse sensor_fault map. Invalid json structure");
        return map;
    }

    if (!d.HasMember("sensor_fault_map")) {
      RCLCPP_WARN(get_logger(), "No sensor_fault_map found in localization config");
      return map;
    }

    rapidjson::Value& map_value = d["sensor_fault_map"];

    if (!map_value.IsArray()) {
      RCLCPP_WARN(get_logger(), "sensor_fault_map cannot be read as array");
      return map;
    }

    if (map_value.Size() == 0) {
      RCLCPP_WARN(get_logger(), "sensor_fault_map has zero size");
      return map;
    }

    for (rapidjson::SizeType i = 0; i < map_value.Size(); i++) {
      if (!map_value[i].IsArray()) {
        RCLCPP_WARN(get_logger(), "sensor_fault_map rows are not arrays");
        continue;
      }

      if (map_value[i].Size() < 2) {
        RCLCPP_WARN(get_logger(), "sensor_fault_map rows do not contain more than 1 element");
        continue;
      }

      std::vector<SensorBooleanStatus> statuses; 
      statuses.reserve(map_value[i].Size()-1);
      for (rapidjson::SizeType j = 0; j < map_value[i].Size() - 1; j++) {
        if (!map_value[i][j].IsInt()) {
          RCLCPP_WARN_STREAM(get_logger(), "sensor_fault_map row " << i << " element " << j << " is not an int");
          continue;
        }

        statuses.emplace_back(static_cast<SensorBooleanStatus>(map_value[i][j].GetInt()));
      }

      rapidjson::Value& alert = map_value[i][map_value[i].Size() - 1];

      if (!alert.IsInt()) {
        RCLCPP_WARN_STREAM(get_logger(), "sensor_fault_map alert type is not an int in row " << i);
        continue;
      }

      map[statuses] = static_cast<SensorAlertStatus>(alert.GetInt());
    }

    return map;
  }

} // namespace subsystem_controllers

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(subsystem_controllers::LocalizationControllerNode)
