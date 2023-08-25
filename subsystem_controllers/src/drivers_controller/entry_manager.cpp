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

#include "subsystem_controllers/drivers_controller/entry_manager.hpp"
#include <iostream>

namespace subsystem_controllers
{
    EntryManager::EntryManager() {}

    EntryManager::EntryManager(std::vector<std::string> required_entries):required_entries_(required_entries) {}

    EntryManager::EntryManager(std::vector<std::string> required_entries, std::vector<std::string> camera_entries) 
                :required_entries_(required_entries), camera_entries_(camera_entries){}

    void EntryManager::update_entry(const Entry& entry)
    {
        for(auto i = entry_list_.begin(); i < entry_list_.end(); ++i)
        {
            if(i->name_.compare(entry.name_) == 0)
            {
                // name entry wont change
                i->available_ = entry.available_;
                i->timestamp_ = entry.timestamp_;
                return;
            }
        }
        entry_list_.push_back(entry);
    }

    std::vector<Entry> EntryManager::get_entries() const
    {
        // returns the copy of the original list
        return std::vector<Entry>(entry_list_);
    }

    void EntryManager::delete_entry(const std::string& name)
    {
        for(auto i = entry_list_.begin(); i < entry_list_.end(); ++i)
        {
            if(i->name_.compare(name) == 0)
            {
                entry_list_.erase(i);
                return;
            }
        }
    }

    boost::optional<Entry> EntryManager::get_entry_by_name(const std::string&  name) const
    {
        for(auto i = entry_list_.begin(); i < entry_list_.end(); ++i)
        {
            if(i->name_.compare(name) == 0)
            {
                return *i;
            }
        }
        // use boost::optional because requested entry might not exist
        return boost::none;
    }

    bool EntryManager::is_entry_required(const std::string& name) const
    {
        for(auto i = required_entries_.begin(); i < required_entries_.end(); ++i)
        {
            if(i->compare(name) == 0)
            {
                return true;
            }
        }
        return false;
    }

    int EntryManager::is_camera_entry_required(const std::string& name) const
    {
        for(int i = 0;i < camera_entries_.size(); i++)
        {
            if(camera_entries_[i].compare(name) == 0)
            {
                return i;
            }
        }
        
        // default like above?
        return -1;
    }


}