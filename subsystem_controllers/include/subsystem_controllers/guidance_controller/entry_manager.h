#pragma once

/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <vector>
#include <boost/optional.hpp>
#include <unordered_map>
#include "entry.h"

namespace subsystem_controllers
{
    /**
     * \brief An entry manager keeps track of the set of entries and makes it easy to add or remove entries
     */ 
    class EntryManager
    {
        public:
            
            /*!
             * \brief Default constructor for EntryManager.
             */
            EntryManager() = default;

            /*!
             * \brief Add a new entry if the given name does not exist.
            *         Update an existing entry if the given name exists.
            * 
            * \param entry The entry to update or add
             */
            void update_entry(const Entry& entry);

            /*!
             * \brief Get all entries as a list.
             */
            std::vector<Entry> get_entries() const;

            /*!
             * \brief Get all entry names as a list
             */
            std::vector<std::string> get_entry_names() const;

            /*!
             * \brief Get a entry using name as the key.
             */
            boost::optional<Entry> get_entry_by_name(const std::string& name) const;

            /*!
             * \brief Delete an entry using the given name as the key.
             */
            void delete_entry(const std::string& name);

        private:

            //! private map by entry name to keep track of all entries
            std::unordered_map<std::string, Entry> entry_map_;

    };
}