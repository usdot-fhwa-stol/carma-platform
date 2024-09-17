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

#include <unordered_map>
#include <vector>

#include "entry.hpp"
#include <boost/optional.hpp>

namespace subsystem_controllers
{
/**
 * \brief The EntryManager serves as a component to track the status of each required CARMA ROS 1
 * driver
 */
class EntryManager
{
public:
  /*!
   * \brief Default constructor for EntryManager.
   */
  EntryManager();
  /*!
   * \brief Constructor for EntryManager to set required entries.
   */
  EntryManager(std::vector<std::string> required_entries);

  /*!
   * \brief Add a new entry if the given name does not exist.
   * Update an existing entry if the given name exists.
   */
  void update_entry(const Entry & entry);

  /*!
   * \brief Get all registed entries as a list.
   */
  std::vector<Entry> get_entries() const;

  /*!
   * \brief Get a entry using name as the key.
   */
  boost::optional<Entry> get_entry_by_name(const std::string & name) const;

  /*!
   * \brief Delete an entry using the given name as the key.
   */
  void delete_entry(const std::string & name);

  /*!
   * \brief Check if the entry is required
   */
  bool is_entry_required(const std::string & name) const;

private:
  //! private list to keep track of all entries
  std::vector<Entry> entry_list_;

  // list of required entries
  std::vector<std::string> required_entries_;
};

}  // namespace subsystem_controllers
