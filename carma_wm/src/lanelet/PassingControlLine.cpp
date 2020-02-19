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
#include <carma_wm/lanelet/PassingControlLine.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 parameter export
#if __cplusplus < 201703L
constexpr char PassingControlLine::RuleName[];  // instantiate string in cpp file
constexpr char PassingControlLine::FromLeft[];
constexpr char PassingControlLine::FromRight[];
constexpr char PassingControlLine::FromBoth[];
#endif

ConstLineStrings3d PassingControlLine::controlLine() const
{
  return getParameters<ConstLineString3d>(RoleName::RefLine);
}

LineStrings3d PassingControlLine::controlLine()
{
  return getParameters<LineString3d>(RoleName::RefLine);
}

bool PassingControlLine::passableFromLeft(const std::string& participant) const
{
  return setContainsParticipant(left_participants_, participant);
}

bool PassingControlLine::passableFromRight(const std::string& participant) const
{
  return setContainsParticipant(right_participants_, participant);
}

bool PassingControlLine::boundPassable(const ConstLineString3d& bound,
                                       const std::vector<std::shared_ptr<PassingControlLine>>& controlLines,
                                       bool fromLeft, const std::string& participant)
{
  return boundPassable(bound, utils::transformSharedPtr<const PassingControlLine>(controlLines), fromLeft, participant);
}

bool PassingControlLine::boundPassable(const ConstLineString3d& bound,
                                       const std::vector<std::shared_ptr<const PassingControlLine>>& controlLines,
                                       bool fromLeft, const std::string& participant)
{
  for (auto control_line : controlLines)
  {
    for (auto sub_line : control_line->controlLine())
    {
      if (bound.id() == sub_line.id())
      {
        if ((fromLeft && !bound.inverted()) || (!fromLeft && bound.inverted()))
        {  // If from the left or coming from the right and the bound is inverted
          return control_line->passableFromLeft(participant);
        }
        else
        {
          return control_line->passableFromRight(participant);
        }
      }
    }
  }
  return true;
}

PassingControlLine::PassingControlLine(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{
  // Read participants
  addParticipantsToSetFromMap(left_participants_, attributes(), FromLeft);
  addParticipantsToSetFromMap(right_participants_, attributes(), FromRight);
  addParticipantsToSetFromMap(left_participants_, attributes(), FromBoth);
  addParticipantsToSetFromMap(right_participants_, attributes(), FromBoth);
}

lanelet::RegulatoryElementDataPtr PassingControlLine::buildData(Id id, LineStrings3d controlLine,
                                                                std::vector<std::string> left_participants,
                                                                std::vector<std::string> right_participants)
{
  // Add parameters
  RuleParameterMap rules;

  rules[lanelet::RoleNameString::RefLine].insert(rules[lanelet::RoleNameString::RefLine].end(), controlLine.begin(),
                                                 controlLine.end());

  // Add attributes
  AttributeMap attribute_map({
      { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
      { AttributeNamesString::Subtype, RuleName },
  });

  for (auto participant : left_participants)
  {
    const std::string key = std::string(AttributeNamesString::Participant) + ":" + participant;
    attribute_map[key] = FromLeft;
  }

  for (auto participant : right_participants)
  {
    const std::string key = std::string(AttributeNamesString::Participant) + ":" + participant;
    // If this participant is also allowed from the left then add to both
    if (attribute_map[key].value().compare(FromLeft) == 0)
    {
      attribute_map[key] = FromBoth;
    }
    else
    {
      attribute_map[key] = FromRight;
    }
  }

  return std::make_shared<RegulatoryElementData>(id, rules, attribute_map);
}

namespace
{
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<lanelet::PassingControlLine> reg;
}  // namespace

}  // namespace lanelet
