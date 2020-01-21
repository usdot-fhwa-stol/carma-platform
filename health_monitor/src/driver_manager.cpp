/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "driver_manager.h"

namespace health_monitor
{
    
    DriverManager::DriverManager() : driver_timeout_(1000) {}
    
    DriverManager::DriverManager(std::vector<std::string> critical_driver_names, const long driver_timeout)
    {
        em_ = EntryManager(critical_driver_names);
        driver_timeout_ = driver_timeout;
        critical_driver_number_ = critical_driver_names.size();
    }


    void DriverManager::update_driver_status(const cav_msgs::DriverStatusConstPtr& msg, long current_time)
    {
        // Params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry driver_status(msg->status == cav_msgs::DriverStatus::OPERATIONAL || msg->status == cav_msgs::DriverStatus::DEGRADED,
                            true, msg->name, current_time, 0, "");
        em_.update_entry(driver_status);
    }

    bool DriverManager::are_critical_drivers_operational(long current_time)
    {
        int critical_driver_counter = 0;
        std::vector<Entry> driver_list = em_.get_entries();
        for(auto i = driver_list.begin(); i < driver_list.end(); ++i)
        {
            if(em_.is_entry_required(i->name_))
            {
                // if a required driver is not optional or has been timeout
                if((!i->available_) || (current_time - i->timestamp_ > driver_timeout_))
                {
                    // TODO: return the name of the non-functional driver for easy debugging
                    return false;
                } else
                {
                    ++critical_driver_counter;
                }
                
            }
        }
        // all criticial driver in the driver_list are operational, but the number of critical driver does not match the desired number
        if(critical_driver_counter != critical_driver_number_)
        {
            return false;
        }
        return true;
    }

}