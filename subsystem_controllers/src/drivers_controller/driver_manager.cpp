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
    DriverManager::DriverManager() {}

    DriverManager::DriverManager(const std::vector<std::string>& critical_driver_names,
                        const std::vector<std::string>& camera_entries,
                        const long driver_timeout)
            : critical_drivers_(critical_driver_names.begin(), critical_driver_names.end()),
            camera_entries_(camera_entries.begin(), camera_entries.end()),
            driver_timeout_(driver_timeout)
    { 
        // Intialize entry manager
        em_ = std::make_shared<EntryManager>(critical_driver_names, camera_entries);

    }
    

    carma_msgs::msg::SystemAlert DriverManager::handle_spin(long time_now,long start_up_timestamp,long startup_duration)
    {
        carma_msgs::msg::SystemAlert alert;

        std::string status = are_critical_drivers_operational(time_now);
        if(status.compare("s_1_c_1") == 0)
        {
            starting_up_ = false;
            alert.description = "All essential drivers are ready";
            alert.type = carma_msgs::msg::SystemAlert::DRIVERS_READY;
            return alert;
        } 
        else if(starting_up_ && (time_now - start_up_timestamp <= startup_duration))
        {
            alert.description = "System is starting up...";
            alert.type = carma_msgs::msg::SystemAlert::NOT_READY;
            return alert;
        }
        else if(status.compare("s_1_c_0")==0)
        {
            alert.description = "Camera Failed";
            alert.type = carma_msgs::msg::SystemAlert::SHUTDOWN;
            return alert;
        }
        else if(status.compare("s_0") == 0)
        {
            alert.description = "SSC Failed";
            alert.type = carma_msgs::msg::SystemAlert::SHUTDOWN;
            return alert;
        }
        else
        {
            alert.description = "Unknown problem assessing essential driver availability";
            alert.type = carma_msgs::msg::SystemAlert::FATAL;
            return alert;  
        }
    
    }


    void DriverManager::update_driver_status(const carma_driver_msgs::msg::DriverStatus::SharedPtr msg, long current_time)
    {
        // update driver status is only called in response to a message received on driver_discovery. This topic is only being published in ros1
        Entry driver_status(msg->status == carma_driver_msgs::msg::DriverStatus::OPERATIONAL || msg->status == carma_driver_msgs::msg::DriverStatus::DEGRADED,
                            msg->name,current_time);

        em_->update_entry(driver_status);                            
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

    std::string DriverManager::are_critical_drivers_operational(long current_time)
    {
        int ssc=0;
        int camera=0;

        std::vector<Entry> driver_list = em_->get_entries(); //Real time driver list from driver status
        for(auto i = driver_list.begin(); i < driver_list.end(); ++i)
        {
            if(em_->is_entry_required(i->name_))
            {
                evaluate_sensor(ssc,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_->is_camera_entry_required(i->name_)==0)
            {
                evaluate_sensor(camera,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
        }

        // Manual disable of ssc entry in case ssc wrapper is in ros2
        if (critical_drivers_.empty())
        {
            ssc = 1;
        }

        /////////////////////
        //Decision making 
        if (ssc == 1 && camera == 1)
        {
            return "s_1_c_1";
        }
        else if (ssc == 1 && camera == 0)
        {
            return "s_1_c_0";
        }
        else{
            return "s_0";
        }


    }


}