/*
 * Copyright (C) 2019 LEIDOS.
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

#include "entry_manager.h"

namespace health_monitor
{

    void EntryManager::update_entry(Entry entry)
    {
        for(size_t i = 0; i < entry_list_.size(); ++i)
        {
            if(entry_list_[i].name_.compare(entry.name_) == 0)
            {
                entry_list_[i] = entry;
                return;
            }
        }
        entry_list_.push_back(entry);
    }

    std::vector<Entry> EntryManager::get_entries()
    {
        return std::vector<Entry>(entry_list_);
    }

    void EntryManager::delete_entry(std::string name)
    {
        for(size_t i = 0; i < entry_list_.size(); ++i)
        {
            if(entry_list_[i].name_.compare(name) == 0)
            {
                entry_list_.erase(entry_list_.begin() + i);
                return;
            }
        }
    }

    Entry* EntryManager::get_entry_by_name(std::string name)
    {
        for(size_t i = 0; i < entry_list_.size(); ++i)
        {
            if(entry_list_[i].name_.compare(name) == 0)
            {
                return &(entry_list_[i]);
            }
        }
        return nullptr;
    }
}