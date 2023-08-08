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


#include <boost/algorithm/string.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "subsystem_controllers/drivers_controller/driver_manager.hpp"


using std_msec = std::chrono::milliseconds;

namespace subsystem_controllers
{
    DriverManager::DriverManager(const std::vector<std::string>& critical_driver_names,
                        const std::vector<std::string>& lidar_gps_entries,
                        const std::vector<std::string>& camera_entries,
                        const std::vector<std::string>& ros2_drivers,
                        const std::vector<std::string>& unmanaged_required_nodes, 
                        std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> drivers_lifecycle_mgr,
                        GetParentNodeStateFunc get_parent_state_func,
                        ServiceNamesAndTypesFunc get_service_names_and_types_func,
                        std::chrono::nanoseconds driver_timeout)
            : critical_driver_names_(critical_driver_names.begin(), critical_driver_names.end()),
            lidar_gps_entries_(lidar_gps_entries.begin(), lidar_gps_entries.end()),
            camera_entries_(camera_entries.begin(), camera_entries.end()),
            ros2_drivers_(ros2_drivers.begin(), ros2_drivers.end()),
            plugin_lifecycle_mgr_(plugin_lifecycle_mgr), get_parent_state_func_(get_parent_state_func),
            get_service_names_and_types_func_(get_service_names_and_types_func),
            service_timeout_(service_timeout), call_timeout_(call_timeout)
    {
        if (!drivers_lifecycle_mgr)
            throw std::invalid_argument("Input plugin_lifecycle_mgr to DriverManager constructor cannot be null");

        for (const auto& p : critical_driver_names_) {
            bool is_ros1 = ros2_drivers_.find(p) == ros2_drivers_.end();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("subsystem_controllers"), "Added: "<< p <<", as is_ros1:"<< is_ros1);
            // Initialize the entry with available and active status as false
            Entry e(false, false, p, 0, "", is_ros1);
            em_update_entry(e);

            if(!is_ros1)
                driver_lifecycle_mgr_->add_managed_node(p);
        }

    }
    
    bool DriverManager::configure()
    {
        bool full_success = true;
        //Bring all known drivers to inactive state
        for(auto driver : em_.get_entries())
        {
            if(driver.is_ros1) // we do not manage lifecycle of ros1 nodes
                continue;

            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name_, service_timeout_, call_timeout_);

            if(results_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
            {
                // If this driver was required then trigger exception
                if(critical_drivers_.find(driver.name_) != critical_drivers_.end())
                {
                    throw std::runtime_error("Required driver " + driver.name_ + " could not be configured");
                }

                // If this driver was not required log an error and mark it is unavailable and deactivated
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to configure newly discovered non-required driver: " 
                    << driver.name_ << " Marking as deactivated and unavailable!");

                Entry deactivated_entry = driver;
                deactivated_entry.active = false;
                deactivated_entry.available = false;
                em_.update_entry(deactivated_entry);

                full_success = false;
            }
        }

        return full_success;
    }

    bool DriverManager::activate()
    {
        
    }

    bool DriverManager::is_ros2_lifecycle_node(const std::string& node)
    {
        // Determine if this driver is a ROS1 or ROS2 driver
        std::vector<std::string> name_parts;
        boost::split(name_parts, node, boost::is_any_of("/"));

        if (name_parts.empty()) {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("subsystem_controllers"), "Invalid name for plugin: " << node << " Plugin may not function in system.");
          return false;
        }

        std::string base_name = name_parts.back();
        name_parts.pop_back();
        std::string namespace_joined = boost::algorithm::join(name_parts, "/");

        std::map<std::string, std::vector<std::string>> services_and_types;
        try {
            services_and_types = get_service_names_and_types_func_(base_name, namespace_joined);
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

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("subsystem_controllers"), "Detected non-ros2 lifecycle plugin " << node);
        return false;
    }

    void DriverManager::update_driver_status(const carma_driver_msgs::msg::DriverStatus::UnquePtr msg, long current_time)
    {
        // update driver status is only called in response to a message received on driver_discovery. This topic is only being published in ros1
        Entry driver_status(msg->status == carma_driver_msgs::msg::DriverStatus::OPERATIONAL || msg->status == carma_driver_msgs::msg::DriverStatus::DEGRADED,
                            true, msg->name, 0, "", true);

        em_.update_entry(e);                            
    }


    void DriverManager::evaluate_sensor(int &sensor_input,bool available,long current_time,long timestamp,long driver_timeout)
    {
        if((!available) || (current_time-timestamp > driver_timeout))
        {
            sensor_input=0;
        }
        else
        {
            sensor_input=1;
        }
    }

    std::string DriverManager::are_critical_drivers_operational_truck(long current_time)
    {
        int ssc=0;
        int lidar1=0;
        int lidar2=0;
        int gps=0;
        int camera=0; //Add Camera Driver

        std::vector<Entry> driver_list = em_.get_entries(); //Real time driver list from driver status
        for(auto i = driver_list.begin(); i < driver_list.end(); ++i)
        {
            if(em_.is_entry_required(i->name_))
            {
              evaluate_sensor(ssc,i->available_,current_time,i->timestamp_,driver_timeout_);
            }

            if(em_.is_lidar_gps_entry_required(i->name_)==0) //Lidar1
            {
               evaluate_sensor(lidar1,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_lidar_gps_entry_required(i->name_)==1) //Lidar2
            {
              evaluate_sensor(lidar2,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_lidar_gps_entry_required(i->name_)==2) //GPS
            {
              evaluate_sensor(gps,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_camera_entry_required(i->name_)==0)
            {
                evaluate_sensor(camera,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            
        }

        //////////////////////
        // NOTE: THIS IS A MANUAL DISABLE OF ALL LIDAR AND GPS FAILURE DETECTION FOLLOWING THE ROS2 PORT
        /////////////////////
        lidar1=1;
        lidar2=1;
        gps=1;
        /////////////////////

        //Decision making 
        if (ssc == 0)
        {
            return "s_0";
        }
        // if ssc= 1
        if((lidar1==0) && (lidar2==0) && (gps==0))
        {
            return "s_1_l1_0_l2_0_g_0";
        }
        else if((lidar1==0) && (lidar2==0) && (gps==1))
        {
            return "s_1_l1_0_l2_0_g_1";
        }
        else if((lidar1==0) && (lidar2==1) && (gps==0))
        {
            return "s_1_l1_0_l2_1_g_0";
        }
        else if((lidar1==0) && (lidar2==1) && (gps==1))
        {
            return "s_1_l1_0_l2_1_g_1";
        }
        else if((lidar1==1) && (lidar2==0) && (gps==0))
        {
            return "s_1_l1_1_l2_0_g_0";
        }
        else if((lidar1==1) && (lidar2==0) && (gps==1))
        {
            return "s_1_l1_1_l2_0_g_1";
        }
        else if((lidar1==1) && (lidar2==1) && (gps==0))
        {
            return "s_1_l1_1_l2_1_g_0";
        }
        else if ((camera==0) && (lidar1 ==1) && (lidar2 == 1) && (gps == 1) )
        {
            return "s_1_l1_1_l2_1_g_1_c_0";
        }
        else if((lidar1==1) && (lidar2==1) && (gps==1) && (camera == 1))
        {
            return "s_1_l1_1_l2_1_g_1_c_1";
        }
        
    }


    std::string DriverManager::are_critical_drivers_operational_car(long current_time)
    {
        int ssc=0;
        int lidar=0;
        int gps=0;
        int camera=0;
        std::vector<Entry> driver_list = em_.get_entries(); //Real time driver list from driver status
        for(auto i = driver_list.begin(); i < driver_list.end(); ++i)
        {
            if(em_.is_entry_required(i->name_))
            {
                evaluate_sensor(ssc,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            if(em_.is_lidar_gps_entry_required(i->name_)==0) //Lidar
            {   
                evaluate_sensor(lidar,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_lidar_gps_entry_required(i->name_)==1) //GPS
            {
                evaluate_sensor(gps,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_camera_entry_required(i->name_)==0)
            {
                evaluate_sensor(camera,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
        }

        //////////////////////
        // NOTE: THIS IS A MANUAL DISABLE OF ALL LIDAR FAILURE DETECTION FOLLOWING THE ROS2 PORT
        /////////////////////
        lidar=1;
        gps=1;
        /////////////////////

        //Decision making 
        if(ssc==1)
        {
            if((lidar==0) && (gps==0))
            {
                return "s_1_l_0_g_0";
            }
            
            else if((lidar==0) && (gps==1) && (camera == 1))
            {
                return "s_1_l_0_g_1";
            }
            else if((lidar==1) && (gps==0) && (camera == 1))
            {
                return "s_1_l_1_g_0";
            }
            else if ((camera==0) && (lidar == 1) && (gps ==1))
            {
                return "s_1_l_1_g_1_c_0";
            }
            else if ((camera==0) && (lidar == 0) && (gps == 1))
            {
                return "s_1_l_0_g_1_c_0";
            }
            else if ((camera==0) && (lidar == 1) && (gps == 0))
            {
                return "s_1_l_1_g_0_c_0";
            }
            else if((lidar==1) && (gps==1) && (camera == 1))
            {
                return "s_1_l_1_g_1_c_1";
            }

        }
        else
        {
            return "s_0";
        }

    }


}