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
#include <gtest/gtest_prod.h>


namespace subsystem_controllers
{

    /**
     * \brief The DriverManager serves as a component to manage CARMA required ROS1 Drivers 
     */ 
    class DriverManager
    {
        public:

        /*!
         * \brief Default constructor for DriverManager with driver_timeout_ = 1000ms
         */
        DriverManager();

        /**
         * \brief Constructor for DriverManager
         * 
         * \param critical_driver_names The set of drivers which will be treated as required. A failure in these plugins will result in an exception
         * \param camera_entries The set of camera drivers.
         * \param driver_timeout The timeout threshold for essential drivers
         */
        DriverManager(const std::vector<std::string>& critical_driver_names,
                        const std::vector<std::string>& camera_entries,
                        const long driver_timeout);
 

        /*!
         * \brief Update driver status
         */
        void update_driver_status(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg, long current_time);

        /*!
         * \brief Check if all critical drivers are operational
         */
        std::string are_critical_drivers_operational(long current_time);

        /*!
         * \brief Evaluate if the sensor is available
         */
        void evaluate_sensor(int &sensor_input,bool available,long current_time,long timestamp,long driver_timeout);

        /*!
         * \brief Handle the spin and publisher
         */
        carma_msgs::msg::SystemAlert handle_spin(long time_now,long start_up_timestamp,long startup_duration);

        protected:

        //list of critical drivers
        std::vector<std::string> critical_drivers_;

        //list of camera entries
        std::vector<std::string> camera_entries_;

        //! Entry manager to keep track of detected plugins
        std::shared_ptr<EntryManager> em_;  

        // timeout for critical driver timeout in milliseconds
        long driver_timeout_ = 1000; 

        bool starting_up_ = true;

        FRIEND_TEST(DriverManagerTest, testCarTruckHandleSpinFatalUnknown);

    };
}