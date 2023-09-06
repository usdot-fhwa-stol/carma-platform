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

#include "carma_cooperative_perception/external_object_list_to_detection_list_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <utility>
#include <vector>

#include "carma_cooperative_perception/geodetic.hpp"
#include "carma_cooperative_perception/units_extensions.hpp"

namespace carma_cooperative_perception
{

auto transform_from_map_to_utm(
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list,
  const std::string & map_origin) -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  gsl::owner<PJ_CONTEXT *> context = proj_context_create();
  proj_log_level(context, PJ_LOG_NONE);

  if (context == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ context: " + error_string + '.');
  }

  gsl::owner<PJ *> map_transformation = proj_create(context, map_origin.c_str());

  if (map_transformation == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ transform: " + error_string + '.');
  }

  std::vector<carma_cooperative_perception_interfaces::msg::Detection> new_detections;
  for (auto detection : detection_list.detections) {
    // Coordinate order is easting (meters), northing (meters)
    const auto position_planar{
      proj_coord(detection.pose.pose.position.x, detection.pose.pose.position.y, 0, 0)};
    const auto proj_inverse{proj_trans(map_transformation, PJ_DIRECTION::PJ_INV, position_planar)};
    const Wgs84Coordinate position_wgs84{
      units::angle::radian_t{proj_inverse.lp.phi}, units::angle::radian_t{proj_inverse.lp.lam},
      units::length::meter_t{detection.pose.pose.position.z}};

    const auto utm_zone{calculate_utm_zone(position_wgs84)};
    const auto position_utm{project_to_utm(position_wgs84)};

    detection.header.frame_id = to_string(utm_zone);
    detection.pose.pose.position.x = remove_units(position_utm.easting);
    detection.pose.pose.position.y = remove_units(position_utm.northing);

    new_detections.push_back(std::move(detection));
  }

  std::swap(detection_list.detections, new_detections);

  proj_destroy(map_transformation);
  proj_context_destroy(context);

  return detection_list;
}

}  // namespace carma_cooperative_perception

RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::ExternalObjectListToDetectionListNode)
