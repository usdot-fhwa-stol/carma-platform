#pragma once
/*
 * Copyright (C) 2020 LEIDOS.
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
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <unordered_set>

/**
 * @brief This header file contains useful functions for regulatory elements
 *
 */
namespace lanelet
{
/**
 * @brief Returns true if the requested participant is in the provided set accounting for the lanelet2 heirarchy
 */
bool inline setContainsParticipant(const std::unordered_set<std::string>& set, const std::string& participant)
{
  std::vector<std::string> strs;
  boost::split(strs, participant, boost::is_any_of(":"));
  std::string current_string;
  bool first = true;
  for (auto participant_class : strs)
  {
    if (first)
    {
      current_string = participant_class;
      first = false;
    }
    else
    {
      current_string += ":" + participant_class;
    }
    if (set.find(current_string) != set.end())
    {
      return true;
    }
  }

  return false;
}

void inline addParticipantsToSetFromMap(std::unordered_set<std::string>& participant_set,
                                        const AttributeMap& attributes, const std::string& required_val = "yes")
{
  static const std::string participant_tag = std::string(AttributeNamesString::Participant) + ":";
  for (const auto pair : attributes)
  {
    size_t part_index = pair.first.find(participant_tag);

    if (part_index != std::string::npos && pair.second.value().compare(required_val) == 0)
    {  // If there is a participant tag in the attribute map add it to the set
      std::string participant = pair.first.substr(part_index + participant_tag.size());
      participant_set.insert(participant);
    }
  }
}
}  // namespace lanelet