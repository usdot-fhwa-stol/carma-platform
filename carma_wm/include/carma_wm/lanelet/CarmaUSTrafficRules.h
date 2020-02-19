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
#include <vector>
#include <memory>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/Exceptions.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <carma_wm/lanelet/RegionAccessRule.h>
#include <carma_wm/lanelet/DigitalSpeedLimit.h>
#include <carma_wm/lanelet/PassingControlLine.h>
#include <carma_wm/lanelet/DirectionOfTravel.h>

namespace lanelet
{
namespace traffic_rules
{
//! Class for inferring traffic rules for lanelets and areas
class CarmaUSTrafficRules : public TrafficRules
{
public:
  // Declare new US location. Prefix with carma to prevent future collisions if lanelet adds US support
  static constexpr char Location[] = "carma_us";

  CarmaUSTrafficRules(Configuration config = Configuration()) : TrafficRules(config){};

  virtual ~CarmaUSTrafficRules() = default;

  bool canPass(const ConstLanelet& lanelet) const override;

  bool canPass(const ConstArea& area) const override;

  /**
   * NOTE: Based on the implementation found in the GenericTrafficRules class this function is for non-lanechange
   * passing. (Proceeding into another lanelet)
   */
  bool canPass(const ConstLanelet& from, const ConstLanelet& to) const override;

  bool canPass(const ConstLanelet& from, const ConstArea& to) const override;

  bool canPass(const ConstArea& from, const ConstLanelet& to) const override;

  bool canPass(const ConstArea& from, const ConstArea& to) const override;

  bool canChangeLane(const ConstLanelet& from, const ConstLanelet& to) const override;

  SpeedLimitInformation speedLimit(const ConstLanelet& lanelet) const override;

  SpeedLimitInformation speedLimit(const ConstArea& area) const override;

  bool isOneWay(const ConstLanelet& lanelet) const override;

  /**
   * @brief NOTE: This function always returns true as all elements in CARMA can contain dynamic rules there is never a
   * reason to assume otherwise
   */
  bool hasDynamicRules(const ConstLanelet& lanelet) const override;

private:
  /**
   * @brief Same usage as PassingControlLine::boundPassable, but the participant is provided by this class
   *
   */
  bool boundPassable(const ConstLineString3d& bound,
                     const std::vector<std::shared_ptr<const PassingControlLine>>& controlLines, bool fromLeft) const;

  /**
   * @brief Returns true if the lanelet or area can be accessed
   *
   * @param region A lanelet or area to evaluate
   *
   * @return True if region can be accessed.
   */
  bool canAccessRegion(const ConstLaneletOrArea& region) const;

  /**
   * @brief Returns a velocity based on a traffic sign type specified by the provided string
   *
   * @param typeString The traffic sign identification string, expects the MUTCD code followed by a "-" with the limit
   * such as R2-1-10mph
   *
   * @return The velocity specified by the traffic sign
   */
  Velocity trafficSignToVelocity(const std::string& typeString) const;

  /**
   * @brief Determines the speed limit for a lanelet or area
   *
   * @param lanelet_or_area The lanelet or area to evaluate
   *
   * @return The speed limit for the provided region
   */
  SpeedLimitInformation speedLimit(const ConstLaneletOrArea& lanelet_or_area) const;
};

}  // namespace traffic_rules
}  // namespace lanelet
