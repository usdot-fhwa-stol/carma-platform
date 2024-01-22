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
#include <carma_cooperative_perception_interfaces/msg/track.hpp>
#include <carma_cooperative_perception_interfaces/msg/track_list.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <j2735_v2x_msgs/msg/d_date_time.hpp>
#include <j2735_v2x_msgs/msg/personal_device_user_type.hpp>
#include <j3224_v2x_msgs/msg/detected_object_data.hpp>
#include <j3224_v2x_msgs/msg/measurement_time_offset.hpp>
#include <j3224_v2x_msgs/msg/object_type.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <proj.h>
#include <gsl/pointers>
#include <memory>

#include "carma_cooperative_perception/geodetic.hpp"
#include "carma_cooperative_perception/j2735_types.hpp"
#include "carma_cooperative_perception/j3224_types.hpp"
#include "carma_cooperative_perception/units_extensions.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <boost/date_time/posix_time/conversion.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace carma_cooperative_perception
{
auto to_time_msg(const DDateTime & d_date_time) -> builtin_interfaces::msg::Time
{
  double seconds;
  const auto fractional_secs{
    std::modf(remove_units(d_date_time.second.value_or(units::time::second_t{0.0})), &seconds)};

  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(seconds);
  msg.nanosec = static_cast<std::int32_t>(fractional_secs * 1e9);

  return msg;
}

auto calc_detection_time_stamp(DDateTime sdsm_time, const MeasurementTimeOffset & offset)
  -> DDateTime
{
  sdsm_time.second.value() += offset.measurement_time_offset;

  return sdsm_time;
}

auto to_ddate_time_msg(const builtin_interfaces::msg::Time & builtin_time)
  -> j2735_v2x_msgs::msg::DDateTime
{
  j2735_v2x_msgs::msg::DDateTime ddate_time_output;

  // Add the time components from epoch seconds
  boost::posix_time::ptime posix_time = boost::posix_time::from_time_t(builtin_time.sec) +
                                        boost::posix_time::nanosec(builtin_time.nanosec);

  const auto time_stamp_year = posix_time.date().year();
  const auto time_stamp_month = posix_time.date().month();
  const auto time_stamp_day = posix_time.date().day();

  const auto hours_of_day = posix_time.time_of_day().hours();
  const auto minutes_of_hour = posix_time.time_of_day().minutes();
  const auto seconds_of_minute = posix_time.time_of_day().seconds();

  ddate_time_output.presence_vector = 0;

  ddate_time_output.presence_vector |= j2735_v2x_msgs::msg::DDateTime::YEAR;
  ddate_time_output.year.year = time_stamp_year;
  ddate_time_output.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MONTH;
  ddate_time_output.month.month = time_stamp_month;
  ddate_time_output.presence_vector |= j2735_v2x_msgs::msg::DDateTime::DAY;
  ddate_time_output.day.day = time_stamp_day;
  ddate_time_output.presence_vector |= j2735_v2x_msgs::msg::DDateTime::HOUR;
  ddate_time_output.hour.hour = hours_of_day;
  ddate_time_output.presence_vector |= j2735_v2x_msgs::msg::DDateTime::MINUTE;
  ddate_time_output.minute.minute = minutes_of_hour;
  ddate_time_output.presence_vector |= j2735_v2x_msgs::msg::DDateTime::SECOND;
  ddate_time_output.second.millisecond = seconds_of_minute;

  return ddate_time_output;
}

auto calc_sdsm_time_offset(
  const builtin_interfaces::msg::Time & external_object_list_stamp,
  const builtin_interfaces::msg::Time & external_object_stamp)
  -> carma_v2x_msgs::msg::MeasurementTimeOffset
{
  carma_v2x_msgs::msg::MeasurementTimeOffset time_offset;

  boost::posix_time::ptime external_object_list_time =
    boost::posix_time::from_time_t(external_object_list_stamp.sec) +
    boost::posix_time::nanosec(external_object_list_stamp.nanosec);

  boost::posix_time::ptime external_object_time =
    boost::posix_time::from_time_t(external_object_stamp.sec) +
    boost::posix_time::nanosec(external_object_stamp.nanosec);

  boost::posix_time::time_duration offset_duration =
    (external_object_list_time - external_object_time);

  time_offset.measurement_time_offset = offset_duration.total_seconds();

  return time_offset;
}

auto to_position_msg(const UtmCoordinate & position_utm) -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point msg;

  msg.x = remove_units(position_utm.easting);
  msg.y = remove_units(position_utm.northing);
  msg.z = remove_units(position_utm.elevation);

  return msg;
}

auto to_position_msg(const MapCoordinate & position_map) -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point msg;

  msg.x = remove_units(position_map.easting);
  msg.y = remove_units(position_map.northing);
  msg.z = remove_units(position_map.elevation);

  return msg;
}

auto heading_to_enu_yaw(const units::angle::degree_t & heading) -> units::angle::degree_t
{
  return units::angle::degree_t{std::fmod(-(remove_units(heading) - 90.0) + 360.0, 360.0)};
}

auto enu_orientation_to_true_heading(
  double yaw, const lanelet::BasicPoint3d & obj_pose,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection)
  -> units::angle::degree_t
{
  // Get object geodetic position
  lanelet::GPSPoint wgs_obj_pose = map_projection->reverse(obj_pose);

  // Get WGS84 Heading
  gsl::owner<PJ_CONTEXT *> context = proj_context_create();
  gsl::owner<PJ *> transform = proj_create(context, map_projection->ECEF_PROJ_STR);
  units::angle::degree_t grid_heading{std::fmod(90 - yaw + 360, 360)};

  const auto factors = proj_factors(
    transform, proj_coord(proj_torad(wgs_obj_pose.lon), proj_torad(wgs_obj_pose.lat), 0, 0));
  units::angle::degree_t grid_convergence{proj_todeg(factors.meridian_convergence)};

  auto wgs_heading = grid_convergence + grid_heading;

  proj_destroy(transform);
  proj_context_destroy(context);

  return wgs_heading;
}

// determine the object position offset in m from the current reference pose
// in map frame and external object pose
auto calc_relative_position(
  const geometry_msgs::msg::PoseStamped & source_pose,
  const carma_v2x_msgs::msg::PositionOffsetXYZ & position_offset)
  -> carma_v2x_msgs::msg::PositionOffsetXYZ
{
  carma_v2x_msgs::msg::PositionOffsetXYZ adjusted_offset;

  adjusted_offset.offset_x.object_distance =
    position_offset.offset_x.object_distance - source_pose.pose.position.x;
  adjusted_offset.offset_y.object_distance =
    position_offset.offset_y.object_distance - source_pose.pose.position.y;
  adjusted_offset.offset_z.object_distance =
    position_offset.offset_z.object_distance - source_pose.pose.position.z;
  adjusted_offset.presence_vector = carma_v2x_msgs::msg::PositionOffsetXYZ::HAS_OFFSET_Z;

  return adjusted_offset;
}

auto transform_pose_from_map_to_wgs84(
  const geometry_msgs::msg::PoseStamped & source_pose,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection)
  -> carma_v2x_msgs::msg::Position3D
{
  carma_v2x_msgs::msg::Position3D ref_pos;
  lanelet::BasicPoint3d source_pose_basicpoint{
    source_pose.pose.position.x, source_pose.pose.position.y, 0.0};

  lanelet::GPSPoint wgs84_ref_pose = map_projection->reverse(source_pose_basicpoint);

  ref_pos.longitude = wgs84_ref_pose.lon;
  ref_pos.latitude = wgs84_ref_pose.lat;
  ref_pos.elevation = wgs84_ref_pose.ele;
  ref_pos.elevation_exists = true;

  return ref_pos;
}

auto to_detection_list_msg(
  const carma_v2x_msgs::msg::SensorDataSharingMessage & sdsm, std::string_view georeference)
  -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list;

  const auto ref_pos_3d{Position3D::from_msg(sdsm.ref_pos)};
  const Wgs84Coordinate ref_pos_wgs84{
    ref_pos_3d.latitude, ref_pos_3d.longitude, ref_pos_3d.elevation.value()};
  const auto ref_pos_map{project_to_carma_map(ref_pos_wgs84, georeference)};

  for (const auto & object_data : sdsm.objects.detected_object_data) {
    const auto common_data{object_data.detected_object_common_data};

    carma_cooperative_perception_interfaces::msg::Detection detection;
    detection.header.frame_id = "map";

    const auto detection_time{calc_detection_time_stamp(
      DDateTime::from_msg(sdsm.sdsm_time_stamp),
      MeasurementTimeOffset::from_msg(common_data.measurement_time))};

    detection.header.stamp = to_time_msg(detection_time);

    // TemporaryID and octet string terms come from the SAE J2735 message definitions
    static constexpr auto to_string = [](const std::vector<std::uint8_t> & temporary_id) {
      std::string str;
      str.reserve(2 * std::size(temporary_id));  // Two hex characters per octet string

      std::array<char, 2> buffer;
      for (const auto & octet_string : temporary_id) {
        std::to_chars(std::begin(buffer), std::end(buffer), octet_string, 16);
        str.push_back(std::toupper(std::get<0>(buffer)));
        str.push_back(std::toupper(std::get<1>(buffer)));
      }

      return str;
    };

    detection.id =
      to_string(sdsm.source_id.id) + "-" + std::to_string(common_data.detected_id.object_id);

    const auto pos_offset{PositionOffsetXYZ::from_msg(common_data.pos)};
    detection.pose.pose.position = to_position_msg(MapCoordinate{
      ref_pos_map.easting + pos_offset.offset_x, ref_pos_map.northing + pos_offset.offset_y,
      ref_pos_map.elevation + pos_offset.offset_z.value_or(units::length::meter_t{0.0})});

    const auto true_heading{units::angle::degree_t{Heading::from_msg(common_data.heading).heading}};

    // Note: This should really use the detection's WGS-84 position, so the
    // convergence will be off slightly. TODO
    const units::angle::degree_t grid_convergence{
      calculate_grid_convergence(ref_pos_wgs84, georeference)};

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
        // We don't have a good semantic class mapping for animals
        detection.semantic_class = detection.SEMANTIC_CLASS_UNKNOWN;
        break;
      case common_data.obj_type.VRU:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
        detection.semantic_class = detection.SEMANTIC_CLASS_PEDESTRIAN;
        break;
      case common_data.obj_type.VEHICLE:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
        detection.semantic_class = detection.SEMANTIC_CLASS_SMALL_VEHICLE;
        break;
      default:
        detection.motion_model = detection.MOTION_MODEL_CTRV;
        detection.semantic_class = detection.SEMANTIC_CLASS_UNKNOWN;
    }

    detection_list.detections.push_back(std::move(detection));
  }

  return detection_list;
}

auto to_detection_msg(
  const carma_perception_msgs::msg::ExternalObject & object,
  const MotionModelMapping & motion_model_mapping)
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

  if (object.presence_vector & object.VELOCITY_PRESENCE_VECTOR) {
    detection.twist = object.velocity;
  }

  if (object.presence_vector & object.OBJECT_TYPE_PRESENCE_VECTOR) {
    switch (object.object_type) {
      case object.SMALL_VEHICLE:
        detection.motion_model = motion_model_mapping.small_vehicle_model;
        detection.semantic_class = detection.SEMANTIC_CLASS_SMALL_VEHICLE;
        break;
      case object.LARGE_VEHICLE:
        detection.motion_model = motion_model_mapping.large_vehicle_model;
        detection.semantic_class = detection.SEMANTIC_CLASS_LARGE_VEHICLE;
        break;
      case object.MOTORCYCLE:
        detection.motion_model = motion_model_mapping.motorcycle_model;
        detection.semantic_class = detection.SEMANTIC_CLASS_MOTORCYCLE;
        break;
      case object.PEDESTRIAN:
        detection.motion_model = motion_model_mapping.pedestrian_model;
        detection.semantic_class = detection.SEMANTIC_CLASS_PEDESTRIAN;
        break;
      case object.UNKNOWN:
      default:
        detection.motion_model = motion_model_mapping.unknown_model;
        detection.semantic_class = detection.SEMANTIC_CLASS_UNKNOWN;
    }
  }

  return detection;
}

auto to_detection_list_msg(
  const carma_perception_msgs::msg::ExternalObjectList & object_list,
  const MotionModelMapping & motion_model_mapping)
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

auto to_external_object_msg(const carma_cooperative_perception_interfaces::msg::Track & track)
  -> carma_perception_msgs::msg::ExternalObject
{
  carma_perception_msgs::msg::ExternalObject external_object;
  external_object.header = track.header;
  external_object.presence_vector = 0;

  const auto to_numeric_id = [](std::string string_id) -> std::optional<uint32_t> {
    auto non_digit_start = std::remove_if(
      std::begin(string_id), std::end(string_id),
      [](const auto & ch) { return !std::isdigit(ch); });

    std::uint32_t numeric_id;
    const auto digit_substr_size{std::distance(std::begin(string_id), non_digit_start)};
    if (
      std::from_chars(string_id.c_str(), string_id.c_str() + digit_substr_size, numeric_id).ec ==
      std::errc{}) {
      return numeric_id;
    }

    return std::nullopt;
  };

  if (const auto numeric_id{to_numeric_id(track.id)}) {
    external_object.presence_vector |= external_object.ID_PRESENCE_VECTOR;
    external_object.id = numeric_id.value();
  } else {
    external_object.presence_vector &= ~external_object.ID_PRESENCE_VECTOR;
  }

  external_object.presence_vector |= external_object.POSE_PRESENCE_VECTOR;
  external_object.pose = track.pose;

  external_object.presence_vector |= external_object.VELOCITY_PRESENCE_VECTOR;

  const auto track_longitudinal_velocity{track.twist.twist.linear.x};
  const auto track_orientation = track.pose.pose.orientation;

  tf2::Quaternion q(
    track_orientation.x, track_orientation.y, track_orientation.z, track_orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  external_object.velocity.twist.linear.x = track_longitudinal_velocity * std::cos(yaw);
  external_object.velocity.twist.linear.y = track_longitudinal_velocity * std::sin(yaw);

  external_object.object_type = track.semantic_class;

  external_object.presence_vector |= external_object.OBJECT_TYPE_PRESENCE_VECTOR;
  switch (track.semantic_class) {
    case track.SEMANTIC_CLASS_SMALL_VEHICLE:
      external_object.object_type = external_object.SMALL_VEHICLE;
      break;
    case track.SEMANTIC_CLASS_LARGE_VEHICLE:
      external_object.object_type = external_object.LARGE_VEHICLE;
      break;
    case track.SEMANTIC_CLASS_MOTORCYCLE:
      external_object.object_type = external_object.MOTORCYCLE;
      break;
    case track.SEMANTIC_CLASS_PEDESTRIAN:
      external_object.object_type = external_object.PEDESTRIAN;
      break;
    case track.SEMANTIC_CLASS_UNKNOWN:
    default:
      external_object.object_type = external_object.UNKNOWN;
  }

  return external_object;
}

auto to_external_object_list_msg(
  const carma_cooperative_perception_interfaces::msg::TrackList & track_list)
  -> carma_perception_msgs::msg::ExternalObjectList
{
  carma_perception_msgs::msg::ExternalObjectList external_object_list;

  for (const auto & track : track_list.tracks) {
    external_object_list.objects.push_back(to_external_object_msg(track));
  }

  return external_object_list;
}

auto to_sdsm_msg(
  const carma_perception_msgs::msg::ExternalObjectList & external_object_list,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection)
  -> carma_v2x_msgs::msg::SensorDataSharingMessage
{
  carma_v2x_msgs::msg::SensorDataSharingMessage sdsm;
  carma_v2x_msgs::msg::DetectedObjectList detected_object_list;

  sdsm.sdsm_time_stamp = to_ddate_time_msg(external_object_list.header.stamp);

  sdsm.ref_pos = transform_pose_from_map_to_wgs84(current_pose, map_projection);

  // Convert external objects within the list to detected_object_data
  for (const auto & external_object : external_object_list.objects) {
    auto sdsm_detected_object = to_detected_object_data_msg(external_object, map_projection);

    // Calculate the time offset between individual objects and the respective SDSM container msg
    sdsm_detected_object.detected_object_common_data.measurement_time =
      calc_sdsm_time_offset(external_object.header.stamp, external_object.header.stamp);

    // Calculate the position offset from the current reference pose (in m)
    sdsm_detected_object.detected_object_common_data.pos =
      calc_relative_position(current_pose, sdsm_detected_object.detected_object_common_data.pos);

    detected_object_list.detected_object_data.push_back(sdsm_detected_object);
  }
  sdsm.objects = detected_object_list;

  return sdsm;
}

auto to_detected_object_data_msg(
  const carma_perception_msgs::msg::ExternalObject & external_object,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection)
  -> carma_v2x_msgs::msg::DetectedObjectData
{
  carma_v2x_msgs::msg::DetectedObjectData detected_object_data;
  detected_object_data.presence_vector = 0;

  carma_v2x_msgs::msg::DetectedObjectCommonData detected_object_common_data;
  detected_object_common_data.presence_vector = 0;

  // common data //////////

  // obj_type_conf - convert from percentile, cast to proper uint type
  if (external_object.presence_vector & external_object.OBJECT_TYPE_PRESENCE_VECTOR) {
    detected_object_common_data.obj_type_cfd.classification_confidence =
      static_cast<std::uint8_t>(external_object.confidence * 100);
  }

  // detected_id - cast proper type
  if (external_object.presence_vector & external_object.ID_PRESENCE_VECTOR) {
    detected_object_common_data.detected_id.object_id =
      static_cast<std::uint16_t>(external_object.id);
  }

  // pos - Add offset to ref_pos to get object position
  // in map frame -> convert to WGS84 coordinates for sdsm

  // To get offset: Subtract the external object pose from
  // the current vehicle location given by the current_pose topic
  if (external_object.presence_vector & external_object.POSE_PRESENCE_VECTOR) {
    detected_object_common_data.pos.offset_x.object_distance =
      static_cast<float>(external_object.pose.pose.position.x);
    detected_object_common_data.pos.offset_y.object_distance =
      static_cast<float>(external_object.pose.pose.position.y);
    detected_object_common_data.pos.offset_z.object_distance =
      static_cast<float>(external_object.pose.pose.position.z);
  }

  // speed/speed_z - convert vector velocity to scalar speed val given x/y components
  if (external_object.presence_vector & external_object.VELOCITY_PRESENCE_VECTOR) {
    detected_object_common_data.speed.speed =
      std::hypot(external_object.velocity.twist.linear.x, external_object.velocity.twist.linear.y);

    detected_object_common_data.presence_vector |=
      carma_v2x_msgs::msg::DetectedObjectCommonData::HAS_SPEED_Z;
    detected_object_common_data.speed_z.speed = external_object.velocity.twist.linear.z;

    // heading - convert ang vel to scale heading
    lanelet::BasicPoint3d external_object_position{
      external_object.pose.pose.position.x, external_object.pose.pose.position.y,
      external_object.pose.pose.position.z};
    // Get yaw from orientation
    auto obj_orientation = external_object.pose.pose.orientation;
    tf2::Quaternion q(obj_orientation.x, obj_orientation.y, obj_orientation.z, obj_orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    detected_object_common_data.heading.heading =
      remove_units(enu_orientation_to_true_heading(yaw, external_object_position, map_projection));
  }

  // optional data (determine based on object type)
  // use object type struct for better control
  carma_v2x_msgs::msg::DetectedObjectOptionalData detected_object_optional_data;

  switch (external_object.object_type) {
    case external_object.SMALL_VEHICLE:
      detected_object_common_data.obj_type.object_type = j3224_v2x_msgs::msg::ObjectType::VEHICLE;

      if (external_object.presence_vector & external_object.SIZE_PRESENCE_VECTOR) {
        detected_object_optional_data.det_veh.presence_vector =
          carma_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
        detected_object_optional_data.det_veh.presence_vector |=
          carma_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;

        detected_object_optional_data.det_veh.size.vehicle_width = external_object.size.y;
        detected_object_optional_data.det_veh.size.vehicle_length = external_object.size.x;
        detected_object_optional_data.det_veh.height.vehicle_height = external_object.size.z;
      }
      break;
    case external_object.LARGE_VEHICLE:
      detected_object_common_data.obj_type.object_type = j3224_v2x_msgs::msg::ObjectType::VEHICLE;

      if (external_object.presence_vector & external_object.SIZE_PRESENCE_VECTOR) {
        detected_object_optional_data.det_veh.presence_vector =
          carma_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
        detected_object_optional_data.det_veh.presence_vector |=
          carma_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;

        detected_object_optional_data.det_veh.size.vehicle_width = external_object.size.y;
        detected_object_optional_data.det_veh.size.vehicle_length = external_object.size.x;
        detected_object_optional_data.det_veh.height.vehicle_height = external_object.size.z;
      }
      break;
    case external_object.MOTORCYCLE:
      detected_object_common_data.obj_type.object_type = j3224_v2x_msgs::msg::ObjectType::VEHICLE;

      if (external_object.presence_vector & external_object.SIZE_PRESENCE_VECTOR) {
        detected_object_optional_data.det_veh.presence_vector =
          carma_v2x_msgs::msg::DetectedVehicleData::HAS_SIZE;
        detected_object_optional_data.det_veh.presence_vector |=
          carma_v2x_msgs::msg::DetectedVehicleData::HAS_HEIGHT;

        detected_object_optional_data.det_veh.size.vehicle_width = external_object.size.y;
        detected_object_optional_data.det_veh.size.vehicle_length = external_object.size.x;
        detected_object_optional_data.det_veh.height.vehicle_height = external_object.size.z;
      }
      break;
    case external_object.PEDESTRIAN:
      detected_object_common_data.obj_type.object_type = j3224_v2x_msgs::msg::ObjectType::VRU;

      detected_object_optional_data.det_vru.presence_vector =
        carma_v2x_msgs::msg::DetectedVRUData::HAS_BASIC_TYPE;
      detected_object_optional_data.det_vru.basic_type.type |=
        j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;

      break;
    case external_object.UNKNOWN:
    default:
      detected_object_common_data.obj_type.object_type = j3224_v2x_msgs::msg::ObjectType::UNKNOWN;

      if (external_object.presence_vector & external_object.SIZE_PRESENCE_VECTOR) {
        detected_object_optional_data.det_obst.obst_size.width.size_value = external_object.size.y;
        detected_object_optional_data.det_obst.obst_size.length.size_value = external_object.size.x;

        detected_object_optional_data.det_obst.obst_size.presence_vector =
          carma_v2x_msgs::msg::ObstacleSize::HAS_HEIGHT;
        detected_object_optional_data.det_obst.obst_size.height.size_value = external_object.size.z;
      }
  }

  detected_object_data.detected_object_common_data = std::move(detected_object_common_data);
  detected_object_data.detected_object_optional_data = std::move(detected_object_optional_data);

  return detected_object_data;
}

}  // namespace carma_cooperative_perception
