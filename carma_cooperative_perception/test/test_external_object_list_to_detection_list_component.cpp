// Copyright 2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <carma_cooperative_perception/external_object_list_to_detection_list_component.hpp>
#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <string>
#include <utility>

TEST(TransformFromMapToUtm, Simple)
{
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list;
  carma_cooperative_perception_interfaces::msg::Detection detection;
  detection_list.detections.push_back(std::move(detection));

  // This PROJ string places the projection's origin over Washington, DC.
  const std::string map_origin{
    "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 "
    "+datum=WGS84 +units=m +vunits=m +no_defs"};

  const auto result{
    carma_cooperative_perception::transform_from_map_to_utm(detection_list, map_origin)};

  EXPECT_EQ(result.detections.at(0).header.frame_id, "18N");
  EXPECT_NEAR(result.detections.at(0).pose.pose.position.x, 313835.60, 1e-1);
  EXPECT_NEAR(result.detections.at(0).pose.pose.position.y, 4313642.41, 1e-1);
}
