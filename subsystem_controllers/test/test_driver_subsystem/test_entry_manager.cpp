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

#include <subsystem_controllers/drivers_controller/entry_manager.hpp>
#include <gtest/gtest.h>
#include <iostream>

namespace subsystem_controllers
{
    TEST(EntryManagerTest, testNewEntry)
    {
        // Entry(bool available, bool active, const std::string& name, uint8_t type, const std::string& capability, bool is_ros1)
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true,"cruising_plugin", 1000);
        em.update_entry(entry);

        
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(true, res.begin()->available_);
        EXPECT_EQ("cruising_plugin", res.begin()->name_);
        EXPECT_EQ(1000, res.begin()->timestamp_);
        
    }

    TEST(EntryManagerTest, testUpdatedEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, "cruising_plugin", 1000);
        em.update_entry(entry);
        Entry new_entry(false, "cruising_plugin", 2000);
        em.update_entry(new_entry);
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(false, res.begin()->available_);
        EXPECT_EQ(2000, res.begin()->timestamp_);
        // the following two attributes should not change once set
        EXPECT_EQ("cruising_plugin", res.begin()->name_);
    }

    TEST(EntryManagerTest, testDeletedEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, "cruising_plugin",1000);
        em.update_entry(entry);
        Entry new_entry(false, "cruising_plugin_2", 2000);
        em.update_entry(new_entry);
        em.delete_entry("cruising_plugin");
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(false, res.begin()->available_);
        EXPECT_EQ(2000, res.begin()->timestamp_);
        // the following two attributes should not change once set
        EXPECT_EQ("cruising_plugin_2", res.begin()->name_);
    }

    TEST(EntryManagerTest, testGetEntryWithValidName)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, "cruising_plugin", 1000);
        em.update_entry(entry);
        boost::optional<Entry> res = em.get_entry_by_name("cruising_plugin");
        EXPECT_EQ(true, res->available_);
        EXPECT_EQ(1000, res->timestamp_);
        EXPECT_EQ("cruising_plugin", res->name_);
    }

    TEST(EntryManagerTest, testGetEntryWithInvalidName)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, "cruising_plugin", true);
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

    TEST(EntryManagerTest, testRequiredEntry)
    {
        std::vector<std::string> required_names;
        required_names.push_back("cruising");
        required_names.push_back("cruising_plugin");
        EntryManager em(required_names);
        Entry entry(true, "cruising_plugin", 1000);
        em.update_entry(entry);
        Entry new_entry(false, "cruising_plugin_2", 2000);
        em.update_entry(new_entry);
        EXPECT_EQ(true, em.is_entry_required("cruising_plugin"));
        EXPECT_EQ(true, em.is_entry_required("cruising"));
        EXPECT_EQ(false, em.is_entry_required("cruising_plugin_2"));
    }

}