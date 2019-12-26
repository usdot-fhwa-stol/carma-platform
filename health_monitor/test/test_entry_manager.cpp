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
#include <gtest/gtest.h>
#include <iostream>

namespace health_monitor
{
    
    TEST(EntryManagerTest, testAddNewEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "autoware_plugin", 1000, 1, "");
        em.update_entry(entry);
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(true, res.begin()->available_);
        EXPECT_EQ(false, res.begin()->active_);
        EXPECT_EQ("autoware_plugin", res.begin()->name_);
        EXPECT_EQ(1000, res.begin()->timestamp_);
        EXPECT_EQ(1, res.begin()->type_);
    }

    TEST(EntryManagerTest, testUpdateEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "autoware_plugin", 1000, 1, "");
        em.update_entry(entry);
        Entry new_entry(false, true, "autoware_plugin", 2000, 3, "");
        em.update_entry(new_entry);
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(false, res.begin()->available_);
        EXPECT_EQ(true, res.begin()->active_);
        EXPECT_EQ(2000, res.begin()->timestamp_);
        // the following two attributes should not change once set
        EXPECT_EQ("autoware_plugin", res.begin()->name_);
        EXPECT_EQ(1, res.begin()->type_);
    }

    TEST(EntryManagerTest, testDeleteEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "autoware_plugin", 1000, 1, "");
        em.update_entry(entry);
        Entry new_entry(false, true, "autoware_plugin_2", 2000, 3, "");
        em.update_entry(new_entry);
        em.delete_entry("autoware_plugin");
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(false, res.begin()->available_);
        EXPECT_EQ(true, res.begin()->active_);
        EXPECT_EQ(2000, res.begin()->timestamp_);
        // the following two attributes should not change once set
        EXPECT_EQ("autoware_plugin_2", res.begin()->name_);
        EXPECT_EQ(3, res.begin()->type_);
    }

    TEST(EntryManagerTest, testGetEntryByValidName)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "autoware_plugin", 1000, 1, "");
        em.update_entry(entry);
        boost::optional<Entry> res = em.get_entry_by_name("autoware_plugin");
        EXPECT_EQ(true, res->available_);
        EXPECT_EQ(false, res->active_);
        EXPECT_EQ(1000, res->timestamp_);
        EXPECT_EQ(1, res->type_);
        EXPECT_EQ("autoware_plugin", res->name_);
    }

    TEST(EntryManagerTest, testGetEntryByInvalidName)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "autoware_plugin", 1000, 1, "");
        em.update_entry(entry);
        boost::optional<Entry> res = em.get_entry_by_name("plugin");
        if(!res)
        {
            ASSERT_TRUE(1 == 1);
        } else
        {
            ASSERT_TRUE(1 == 2);
        }
    }

    TEST(EntryManagerTest, testRequiredEntryCheck)
    {
        std::vector<std::string> required_names;
        required_names.push_back("autoware");
        required_names.push_back("autoware_plugin");
        EntryManager em(required_names);
        Entry entry(true, false, "autoware_plugin", 1000, 1, "");
        em.update_entry(entry);
        Entry new_entry(false, true, "autoware_plugin_2", 2000, 3, "");
        em.update_entry(new_entry);
        EXPECT_EQ(true, em.is_entry_required("autoware_plugin"));
        EXPECT_EQ(true, em.is_entry_required("autoware"));
        EXPECT_EQ(false, em.is_entry_required("autoware_plugin_2"));
    }

}
