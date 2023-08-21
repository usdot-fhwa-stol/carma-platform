#pragma once

/*
 * Copyright (C) 2023 LEIDOS.
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

#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/get_plugin_api.hpp>
#include <carma_planning_msgs/srv/plugin_list.hpp>
#include <carma_planning_msgs/srv/plugin_activation.hpp>
#include <ros2_lifecycle_manager/lifecycle_manager_interface.hpp>
#include <unordered_set>
#include <functional>
#include <vector>
#include <memory>
#include <chrono>
#include <rmw/types.h>
#include <map>
#include "entry_manager.hpp"
#include "entry.hpp"
#include <carma_driver_msgs/msg/driver_status.hpp>
#include <carma_msgs/msg/system_alert.hpp>


namespace subsystem_controllers
{
    using GetParentNodeStateFunc = std::function<uint8_t()>;
    using SrvHeader = const std::shared_ptr<rmw_request_id_t>;
    /**
     * \brief Function which will return a map of service names and their message types based on the provided base node name and namespace
     */ 
    using ServiceNamesAndTypesFunc = std::function<std::map<std::string, std::vector<std::string, std::allocator<std::string>>>(const std::string &,const std::string &)>;

    /**
     * \brief The DriverManager serves as a component to manage CARMA required Drivers via their ros2 lifecycle interfaces
     */ 
    class DriverManager
    {
        public:
        /**
         * \brief Constructor for DriverManager
         * 
         * \param critical_driver_names The set of drivers which will be treated as required. A failure in these plugins will result in an exception
         * \param lidar_gps_entries The set of lidar and gps drivers .
         * \param camera_entries The set of camera drivers.
         * \param driver_lifecycle_mgr A fully initialized lifecycle manager which will be used trigger driver transitions
         * \param get_parent_state_func A callback which will allow this object to access the parent process lifecycle state
         * \param get_service_names_and_types_func A callback which returns a map of service names to service types based on the provided base node name and namespace
         * \param driver_timeout The timeout threshold for essential drivers
         */
        DriverManager(const std::vector<std::string>& critical_driver_names,
                        const std::vector<std::string>& lidar_gps_entries,
                        const std::vector<std::string>& camera_entries,
                        const std::vector<std::string>& base_managed_ros2_nodes,
                        std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> driver_lifecycle_mgr, 
                        GetParentNodeStateFunc get_parent_state_func,
                        ServiceNamesAndTypesFunc get_service_names_and_types_func,
                        const long driver_timeout);
 

        /*!
         * \brief Update driver status
         */
        void update_driver_status(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg, long current_time);

        /*!
         * \brief Check if all critical drivers are operational for truck
         */
        std::string are_critical_drivers_operational_truck(long current_time);

        /*!
         * \brief Check if all critical drivers are operational for car
         */
        std::string are_critical_drivers_operational_car(long current_time);

        /*!
         * \brief Evaluate if the sensor is available
         */
        void evaluate_sensor(int &sensor_input,bool available,long current_time,long timestamp,long driver_timeout, std::string source_node, bool is_ros1);

        /*!
         * \brief Handle the spin and publisher
         */
        carma_msgs::msg::SystemAlert handle_spin(bool truck,bool car,long time_now,long start_up_timestamp,long startup_duration);

        protected:

        /**
         * \brief Returns true if the specified fully qualified node name is a ROS2 lifecycle node
         * 
         * \param node The fully specified name of the node to evaluate
         * 
         * \return True if ros2 lifecycle node. False otherwise
         */ 
        bool is_ros2_lifecycle_node(const std::string& node);

        //list of critical drivers
        std::vector<std::string> critical_drivers_;

        //list of lidar and gps entries
        std::vector<std::string> lidar_gps_entries_;

        //list of camera entries
        std::vector<std::string> camera_entries_;

        std::unordered_set<std::string> ros2_drivers_;

        //! Callback to retrieve the lifecycle state of the parent process 
        GetParentNodeStateFunc get_parent_state_func_;

        //! Callback to get service names and types for the given node
        ServiceNamesAndTypesFunc get_service_names_and_types_func_;

        std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> driver_lifecycle_mgr_;

        //! Entry manager to keep track of detected plugins
        std::shared_ptr<EntryManager> em_;  

        // timeout for critical driver timeout
        long driver_timeout_; 

        bool starting_up_ = true;

    };
}