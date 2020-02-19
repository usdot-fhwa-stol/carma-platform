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
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <boost/algorithm/string.hpp>
#include <lanelet2_core/Forward.h>
#include <unordered_set>

namespace lanelet
{
/**
 * @brief Represents an access restriction for a lanelet or area. An example would be that only high occupancy vehicles
 * are allowed in a given lane or a bike only lane.
 *
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
class RegionAccessRule : public RegulatoryElement
{
public:
  static constexpr char RuleName[] = "region_access_rule";
  std::unordered_set<std::string> participants_;

  /**
   * @brief Returns the lanelets this rule applies to
   *
   * @return Lanelets affected by this access rule
   */
  ConstLanelets getLanelets() const;

  /**
   * @brief Returns the areas this rule applies to
   *
   * @return The areas affected by this rule
   */
  ConstAreas getAreas() const;

  /**
   * @brief Returns true if the provided participant can access lanelets or areas affected by this regulation
   *
   * @param participant The string description of the participant to evaluate
   *
   * @return True if the participant can access the lanelets or areas affected by this regulation
   */
  bool accessable(const std::string& participant) const;

  /**
   * @brief Constructor defined to support loading from lanelet files
   */
  explicit RegionAccessRule(const lanelet::RegulatoryElementDataPtr& data);

  /**
   * @brief Static helper function that creates a region access rule data object based on the provided inputs
   *
   * @param id The lanelet::Id to give this regulation
   * @param lanelets The lanelets impacted by this regulation
   * @param areas The areas impacted by this regulation
   * @param participants The participants which can access the provided lanelets and areas
   *
   * @return RegulatoryElementData containing all the necessary information to construct a region access rule object
   */
  static lanelet::RegulatoryElementDataPtr buildData(Id id, Lanelets lanelets, Areas areas,
                                                     std::vector<std::string> participants);

protected:
  // the following lines are required so that lanelet2 can create this object when loading a map with this regulatory
  // element
  friend class RegisterRegulatoryElement<RegionAccessRule>;
};

// Convienace Ptr Declarations
using RegionAccessRulePtr = std::shared_ptr<RegionAccessRule>;
using RegionAccessRuleConstPtr = std::shared_ptr<const RegionAccessRule>;

}  // namespace lanelet