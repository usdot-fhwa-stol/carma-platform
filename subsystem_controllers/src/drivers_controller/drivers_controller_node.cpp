
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

#include "subsystem_controllers/drivers_controller/drivers_controller_node.hpp"

namespace subsystem_controllers
{
  DriversControllerNode::DriversControllerNode(const rclcpp::NodeOptions &options)
      : BaseSubsystemController(options)
  {
    // Don't automatically trigger state transitions from base class on configure
      // In this class the managed nodes list first needs to be modified then the transition will be triggered manually
    trigger_managed_nodes_configure_from_base_class_ = false;

    config_.startup_duration_ = declare_parameter<double>("startup_duration", config_.startup_duration_);
    config_.driver_timeout_ = declare_parameter<double>("required_driver_timeout", config_.driver_timeout_);
    
    // carma-config parameters
    config_.required_drivers_ = declare_parameter<std::vector<std::string>>("required_drivers", config_.required_drivers_); 
    config_.lidar_gps_drivers_ = declare_parameter<std::vector<std::string>>("lidar_gps_drivers", config_.lidar_gps_drivers_); 
    config_.camera_drivers_ = declare_parameter<std::vector<std::string>>("camera_drivers", config_.camera_drivers_);
    config_.truck_ = declare_parameter<bool>("truck", config_.truck_);
    config_.car_ = declare_parameter<bool>("car", config_.car_);

  }

  carma_ros2_utils::CallbackReturn DriversControllerNode::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
  {
    auto base_return = BaseSubsystemController::handle_on_configure(prev_state);

    if (base_return != carma_ros2_utils::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Drivers Controller could not configure");
      return base_return;
    }

    config_ = DriversControllerConfig();

    // Load required plugins and default enabled plugins
    get_parameter<std::vector<std::string>>("required_drivers", config_.required_drivers_); 
    get_parameter<std::vector<std::string>>("lidar_gps_drivers", config_.lidar_gps_drivers_); 
    get_parameter<std::vector<std::string>>("camera_drivers", config_.camera_drivers_); 
    get_parameter<double>("startup_duration", config_.startup_duration_);
    get_parameter<bool>("truck", config_.truck_);
    get_parameter<bool>("car", config_.car_);
    get_parameter<double>("required_driver_timeout", config_.driver_timeout_);

    RCLCPP_INFO_STREAM(get_logger(), "Config: " << config_);


    auto base_managed_nodes = lifecycle_mgr_.get_managed_nodes();

    std::string drivers_namespace = base_config_.subsystem_namespace;

    //Add listed drivers to managed nodes if they are ros2 lifecycle nodes and not already managed by lifecycle_mgr
    std::vector<std::string> additional_nodes = config_.required_drivers_;
    additional_nodes.insert(additional_nodes.end(), config_.lidar_gps_drivers_.begin(), config_.lidar_gps_drivers_.end());
    additional_nodes.insert(additional_nodes.end(), config_.camera_drivers_.begin(), config_.camera_drivers_.end());

    // Updated list of required ros2 nodes 
    std::vector<std::string> updated_required_nodes;

    for (auto node : additional_nodes)
    {
      if(is_ros2_lifecycle_node(node)){
        updated_required_nodes.push_back(node);
        //Add nodes that are not already managed and can be managed to lifecycle_mgr_
        if(std::find(base_managed_nodes.begin(), base_managed_nodes.end(), node) != base_managed_nodes.end())
        {
          base_managed_nodes.push_back(node);
        }
      }
      
    }

    // Update required_subsystem_nodes to be this list of ros2 required_drivers obtained from carma-config. TODO: Check if this is correct
    base_config_.required_subsystem_nodes = updated_required_nodes;
    
    // Set lifecycle managed nodes
    lifecycle_mgr_.set_managed_nodes(base_managed_nodes);


    driver_manager_ = std::make_shared<DriverManager>(
      config_.required_drivers_, 
      config_.lidar_gps_drivers_,
      config_.camera_drivers_, 
      base_managed_nodes,
      std::make_shared<ros2_lifecycle_manager::Ros2LifecycleManager>(lifecycle_mgr_),
      [this](){ return get_current_state().id(); },
      [this](auto node, auto ns){ return get_service_names_and_types_by_node(node, ns); },
      config_.driver_timeout_
    );

    // record starup time
    start_up_timestamp_ = this->now().nanoseconds() / 1e6;

    driver_status_sub_ = create_subscription<carma_driver_msgs::msg::DriverStatus>("driver_discovery", 1, std::bind(&DriversControllerNode::driver_discovery_cb, this, std::placeholders::_1));
    

    timer_ = create_timer(get_clock(), std::chrono::milliseconds(100), std::bind(&DriversControllerNode::timer_callback,this));

    // Configure our drivers
    bool success = lifecycle_mgr_.configure(std::chrono::milliseconds(base_config_.service_timeout_ms), std::chrono::milliseconds(base_config_.call_timeout_ms)).empty();
    
    if (success)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to configure");
      return CallbackReturn::SUCCESS;
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to configure");
      return CallbackReturn::FAILURE;
    }

  }

  carma_ros2_utils::CallbackReturn DriversControllerNode::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {

    // Reset to automatically trigger state transitions from base class 
    trigger_managed_nodes_configure_from_base_class_ = true;

    auto base_return = BaseSubsystemController::handle_on_activate(prev_state); // This will activate all base_managed_nodes

    if (base_return != carma_ros2_utils::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Driver Controller could not activate");
      return base_return;
    }

  }

  void DriversControllerNode::timer_callback()
  {
    
    long time_now = this->now().nanoseconds() / 1e6;
    rclcpp::Duration sd(config_.startup_duration_);
    long start_duration = sd.nanoseconds()/1e6;

    auto dm = driver_manager_->handle_spin(config_.truck_, config_.car_, time_now, start_up_timestamp_, config_.startup_duration_);
    if (!prev_alert) {
      prev_alert = dm;
      publish_system_alert(dm);
    } 
    else if ( prev_alert->type == dm.type && prev_alert->description.compare(dm.description) == 0) { // Do not publish duplicate alerts
      RCLCPP_DEBUG_STREAM(get_logger(), "No change to alert status");
    }
    else{
      prev_alert = dm;
      publish_system_alert(dm);
    }

  }


  bool DriversControllerNode::is_ros2_lifecycle_node(const std::string& node)
  {
      // Determine if this driver is a ROS1 or ROS2 driver
      std::vector<std::string> name_parts;
      boost::split(name_parts, node, boost::is_any_of("/"));

      if (name_parts.empty()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("subsystem_controllers"), "Invalid name for driver: " << node << " Driver may not function in system.");
        return false;
      }

      std::string base_name = name_parts.back();
      name_parts.pop_back();
      std::string namespace_joined = boost::algorithm::join(name_parts, "/");

      std::map<std::string, std::vector<std::string>> services_and_types;
      try {
          services_and_types = get_service_names_and_types_by_node(base_name, namespace_joined);
      } 
      catch (const std::runtime_error& e) {
          return false; // Seems this method can throw an exception if not a ros2 node
      }
      

      // Next we check if both services are available with the correct type
      // Short variable names used here to make conditional more readable
      const std::string cs_srv = node + "/change_state";
      const std::string gs_srv = node + "/get_state";

      if (services_and_types.find(cs_srv) != services_and_types.end() 
        && services_and_types.find(gs_srv) != services_and_types.end()
        && std::find(services_and_types.at(cs_srv).begin(), services_and_types.at(cs_srv).end(), "lifecycle_msgs/srv/ChangeState") != services_and_types.at(cs_srv).end()
        && std::find(services_and_types.at(gs_srv).begin(), services_and_types.at(gs_srv).end(), "lifecycle_msgs/srv/GetState") != services_and_types.at(gs_srv).end())
      {

        return true;

      }

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("subsystem_controllers"), "Detected non-ros2 lifecycle driver " << node);
      return false;
  }

  void DriversControllerNode::driver_discovery_cb(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg)
  {
    // Driver discovery only published by ros1 drivers
    driver_manager_->update_driver_status(msg, this->now().nanoseconds()/1e6);
  }

  void DriversControllerNode::setDriverManager(DriverManager dm)
    {
        driver_manager_ = std::make_shared<DriverManager>(dm);
    }

  void DriversControllerNode::setCarTrue()
  {
      car_ = true;
      if(truck_ == true)
          throw std::invalid_argument("truck_ = true");
  }

  void DriversControllerNode::setTruckTrue()
  {
      truck_ = true;
      if(car_ == true)
          throw std::invalid_argument("car_ = true");
      
  }

} // namespace subsystem_controllers

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(subsystem_controllers::DriversControllerNode)
