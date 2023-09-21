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

#include "carma_cooperative_perception/msg_conversion.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <j2735_v2x_msgs/msg/d_date_time.hpp>
#include <j3224_v2x_msgs/msg/detected_object_data.hpp>
#include <j3224_v2x_msgs/msg/measurement_time_offset.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

#include "carma_cooperative_perception/geodetic.hpp"
#include "carma_cooperative_perception/j2735_types.hpp"
#include "carma_cooperative_perception/j3224_types.hpp"
#include "carma_cooperative_perception/units_extensions.hpp"

namespace carma_cooperative_perception
{
auto to_time_msg(const DDateTime & d_date_time) noexcept -> builtin_interfaces::msg::Time
{
  double seconds;
  const auto fractional_secs{
    std::modf(remove_units(d_date_time.second.value_or(units::time::second_t{0.0})), &seconds)};

  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(seconds);
  msg.nanosec = static_cast<std::int32_t>(fractional_secs * 1e9);

  return msg;
}

auto calc_detection_time_stamp(DDateTime sdsm_time, const MeasurementTimeOffset & offset) noexcept
  -> DDateTime
{
  sdsm_time.second.value() += offset.measurement_time_offset;

  return sdsm_time;
}

auto to_position_msg(const UtmCoordinate & position_utm) noexcept -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point msg;

  msg.x = remove_units(position_utm.easting);
  msg.y = remove_units(position_utm.northing);
  msg.z = remove_units(position_utm.elevation);

  return msg;
}

auto heading_to_enu_yaw(const units::angle::degree_t & heading) noexcept -> units::angle::degree_t
{
  return units::angle::degree_t{std::fmod(-(remove_units(heading) - 90.0) + 360.0, 360.0)};
}

auto to_detection_list_msg(const carma_v2x_msgs::msg::SensorDataSharingMessage & sdsm) noexcept
  -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list;

  const auto ref_pos_3d{Position3D::from_msg(sdsm.ref_pos)};
  const Wgs84Coordinate ref_pos_wgs84{
    ref_pos_3d.latitude, ref_pos_3d.longitude, ref_pos_3d.elevation.value()};
  const auto ref_pos_utm{project_to_utm(ref_pos_wgs84)};

  for (const auto & object_data : sdsm.objects.detected_object_data) {
    const auto common_data{object_data.detected_object_common_data};

    carma_cooperative_perception_interfaces::msg::Detection detection;
    detection.header.frame_id = to_string(ref_pos_utm.utm_zone);

    const auto detection_time{calc_detection_time_stamp(
      DDateTime::from_msg(sdsm.sdsm_time_stamp),
      MeasurementTimeOffset::from_msg(common_data.measurement_time))};

    detection.header.stamp = to_time_msg(detection_time);

    detection.id = std::to_string(common_data.detected_id.object_id);

    const auto pos_offset{PositionOffsetXYZ::from_msg(common_data.pos)};
    const auto utm_displacement{
      UtmDisplacement{pos_offset.offset_x, pos_offset.offset_y, pos_offset.offset_z.value()}};

    const auto detection_pos_utm{ref_pos_utm + utm_displacement};
    detection.pose.pose.position = to_position_msg(detection_pos_utm);

    const auto true_heading{units::angle::degree_t{Heading::from_msg(common_data.heading).heading}};

    // Note: This should really use the detection's WGS-84 position, so the
    // convergence will be off slightly. TODO
    const auto grid_convergence{calculate_grid_convergence(ref_pos_wgs84, ref_pos_utm.utm_zone)};

    const auto grid_heading{true_heading - grid_convergence};
    const auto enu_yaw{heading_to_enu_yaw(grid_heading)};

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, remove_units(units::angle::radian_t{enu_yaw}));
    detection.pose.pose.orientation = tf2::toMsg(quat_tf);

    const auto speed{Speed::from_msg(common_data.speed)};
    detection.twist.twist.linear.x =
      remove_units(units::velocity::meters_per_second_t{speed.speed});

    const auto speed_z{Speed::from_msg(common_data.speed_z)};
    detection.twist.twist.linear.z =
      remove_units(units::velocity::meters_per_second_t{speed_z.speed});

    const auto accel_set{AccelerationSet4Way::from_msg(common_data.accel_4_way)};
    detection.accel.accel.linear.x =
      remove_units(units::acceleration::meters_per_second_squared_t{accel_set.longitudinal});
    detection.accel.accel.linear.y =
      remove_units(units::acceleration::meters_per_second_squared_t{accel_set.lateral});
    detection.accel.accel.linear.z =
      remove_units(units::acceleration::meters_per_second_squared_t{accel_set.vert});

    detection.twist.twist.angular.z =
      remove_units(units::angular_velocity::degrees_per_second_t{accel_set.yaw_rate});

    switch (common_data.obj_type.object_type) {
      case common_data.obj_type.ANIMAL:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
        break;
      case common_data.obj_type.VRU:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
        break;
      case common_data.obj_type.VEHICLE:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
        break;
      default:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
    }

    detection_list.detections.push_back(std::move(detection));
  }

  return detection_list;
}

auto to_detection_msg(
  const carma_perception_msgs::msg::ExternalObject & object,
  const MotionModelMapping & motion_model_mapping) noexcept
  -> carma_cooperative_perception_interfaces::msg::Detection
{
  carma_cooperative_perception_interfaces::msg::Detection detection;

  detection.header = object.header;

  if (object.presence_vector & object.BSM_ID_PRESENCE_VECTOR) {
    detection.id = "";
    std::transform(
      std::cbegin(object.bsm_id), std::cend(object.bsm_id), std::back_inserter(detection.id),
      [](const auto & i) { return i + '0'; });
  }

  if (object.presence_vector & object.ID_PRESENCE_VECTOR) {
    detection.id += '-' + std::to_string(object.id);
  }

  if (object.presence_vector & object.POSE_PRESENCE_VECTOR) {
    detection.pose = object.pose;
  }

  if (object.presence_vector & object.VELOCITY_INST_PRESENCE_VECTOR) {
    detection.twist = object.velocity_inst;
  }

  if (object.presence_vector & object.OBJECT_TYPE_PRESENCE_VECTOR) {
    switch (object.object_type) {
      case object.SMALL_VEHICLE:
        detection.motion_model = motion_model_mapping.small_vehicle_model;
        break;
      case object.LARGE_VEHICLE:
        detection.motion_model = motion_model_mapping.large_vehicle_model;
        break;
      case object.MOTORCYCLE:
        detection.motion_model = motion_model_mapping.motorcycle_model;
        break;
      case object.PEDESTRIAN:
        detection.motion_model = motion_model_mapping.pedestrian_model;
        break;
      case object.UNKNOWN:
      default:
        detection.motion_model = motion_model_mapping.unknown_model;
    }
  }

  return detection;
}

auto to_detection_list_msg(
  const carma_perception_msgs::msg::ExternalObjectList & object_list,
  const MotionModelMapping & motion_model_mapping) noexcept
  -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list;

  std::transform(
    std::cbegin(object_list.objects), std::cend(object_list.objects),
    std::back_inserter(detection_list.detections),
    [&motion_model_mapping = std::as_const(motion_model_mapping)](const auto & object) {
      return to_detection_msg(object, motion_model_mapping);
    });

  return detection_list;
}

}  // namespace carma_cooperative_perception
