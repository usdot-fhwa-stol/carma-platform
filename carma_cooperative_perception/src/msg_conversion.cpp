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

#include <carma_cooperative_perception_interfaces/msg/track.hpp>
#include <carma_cooperative_perception_interfaces/msg/track_list.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <j2735_v2x_msgs/msg/d_date_time.hpp>
#include <j2735_v2x_msgs/msg/personal_device_user_type.hpp>
#include <j2735_v2x_msgs/msg/positional_accuracy.hpp>
#include <j2735_v2x_msgs/to_floating_point.hpp>
#include <j3224_v2x_msgs/msg/detected_object_data.hpp>
#include <j3224_v2x_msgs/msg/equipment_type.hpp>
#include <j3224_v2x_msgs/msg/measurement_time_offset.hpp>
#include <j3224_v2x_msgs/msg/object_type.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <cstdlib>  // for setenv, unsetenv
#include <ctime>    // for tzset

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
auto to_time_msg(const DDateTime & d_date_time, bool is_simulation) -> builtin_interfaces::msg::Time
{
  // Convert DDateTime to builtin_interfaces::msg::Time
  builtin_interfaces::msg::Time msg;
  if (!is_simulation) {
    // Create a tm structure to hold the date and time components
    std::tm timeinfo = {};

    // Year
    if (d_date_time.year){
      if(remove_units(d_date_time.year.value()) >= 1970){
        // std::tm is counted since 1900
        timeinfo.tm_year = remove_units(d_date_time.year.value()) - 1900;
      }
      else{
        throw std::invalid_argument(
          "Year must be greater than 1970 for live date/time conversion");
      }
    }

    // Month
    if (d_date_time.month && static_cast<int>(d_date_time.month.value().get_value()) != 0)
    {
      // std::tm is counted from 0 to 11, J2735 is counted from 1 to 12
      timeinfo.tm_mon = static_cast<int>(d_date_time.month.value().get_value()) - 1;
    }

    // Day
    if (d_date_time.day && static_cast<int>(d_date_time.day.value()) != 0)
    {
      // Day is counted from 1 to 31 in both std::tm and J2735
      timeinfo.tm_mday = static_cast<int>(d_date_time.day.value());
    }
    else{
      timeinfo.tm_mday = 1; // Default to 1 if day is not provided as C++ initializes to 0
    }

    // Hour
    if (d_date_time.hour && static_cast<int>(d_date_time.hour.value()) != 31)
    {
      // Hour is counted from 0 to 23 in both std::tm and J2735
      timeinfo.tm_hour = static_cast<int>(d_date_time.hour.value());
    }

    // Minute
    if (d_date_time.minute && static_cast<int>(d_date_time.minute.value()) != 60)
    {
      // Minute is counted from 0 to 59 in both std::tm and J2735
      timeinfo.tm_min = static_cast<int>(d_date_time.minute.value());
    }
    // Set seconds field (which actually uses ms in j2735) to 0
    // for now and add milliseconds later
    timeinfo.tm_sec = 0;

    std::time_t timeT;

    if (d_date_time.time_zone_offset)
    {
      timeinfo.tm_gmtoff = static_cast<int>(d_date_time.time_zone_offset.value());
      timeT = std::mktime(&timeinfo);
    }
    else
    {
      // Get the current timezone from the system
      // Use tzset() to initialize timezone data from system
      tzset();

      // Get current timestamp to determine DST status
      // NOTE: If the system is running in a docker container (which it mostly is),
      // the timezone is by default GMT unless otherwise set. Just a caution.
      std::time_t currentTime = std::time(nullptr);
      std::tm* localTimeInfo = std::localtime(&currentTime);

      long timezone_offset = localTimeInfo->tm_gmtoff;

      timeinfo.tm_gmtoff = timezone_offset;
      timeinfo.tm_isdst = localTimeInfo->tm_isdst;

      // Convert to time_t
      timeT = std::mktime(&timeinfo);
    }

    // Convert time_t to system_clock::time_point
    auto timePoint = std::chrono::system_clock::from_time_t(timeT);

    // Add milliseconds
    int milliseconds = 0;
    if (d_date_time.second)
    {
      milliseconds = static_cast<int>(d_date_time.second.value());
    }
    timePoint += std::chrono::milliseconds(milliseconds);

    // Extract seconds and nanoseconds since epoch
    auto duration = timePoint.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    msg.sec = static_cast<int32_t>(seconds.count());
    msg.nanosec = static_cast<uint32_t>(nanoseconds.count());
  }
  else
  {
    // if simulation, we ignore the date, month, year etc because the simulation won't be that long
    double seconds;
    const auto fractional_secs{std::modf(
      remove_units(units::time::second_t{d_date_time.hour.value_or(units::time::second_t{0.0})}) +
        remove_units(units::time::second_t{d_date_time.minute.value_or(units::time::second_t{0.0})}) +
        remove_units(units::time::second_t{d_date_time.second.value_or(units::time::second_t{0.0})}),
      &seconds)};

    msg.sec = static_cast<std::int32_t>(seconds);
    msg.nanosec = static_cast<std::int32_t>(fractional_secs * 1e9);
  }

  return msg;
}

auto calc_detection_time_stamp(DDateTime sdsm_time, const MeasurementTimeOffset & offset)
  -> DDateTime
{
  sdsm_time.second.value() += offset.measurement_time_offset;

  return sdsm_time;
}

auto ned_to_enu(const PositionOffsetXYZ & offset_ned) noexcept
{
  auto offset_enu{offset_ned};

  // NED to ENU: swap x and y axis and negate z axis
  offset_enu.offset_x = offset_ned.offset_y;
  offset_enu.offset_y = offset_ned.offset_x;

  if (offset_enu.offset_z) {
    offset_enu.offset_z.value() *= -1;
  }

  return offset_enu;
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


/**
 * @brief Converts a carma_v2x_msgs::msg::SensorDataSharingMessage (SDSM)
 *  to carma_cooperative_perception_interfaces::msg::DetectionList format
 *
 * This function transforms data from the V2X SDSM format into the CARMA cooperative perception
 * DetectionList format, handling the necessary coordinate transformations.
 *
 * @details Important coordinate system transformations:
 * - SDSM uses NED (North-East-Down) coordinate system for position offsets
 * - SDSM heading is measured clockwise from true north (0° at north, 90° at east)
 * - Output DetectionList uses ENU (East-North-Up) coordinate system
 * - Output heading is converted to ENU yaw (0° at east, 90° at north)
 *
 * The function performs the following key operations:
 * 1. Projects reference position from WGS84 to the local map frame
 * 2. Converts NED position offsets to ENU
 * 3. Handles heading conversion from true north to map grid
 * 4. Transforms detection confidence values to covariance values
 * 5. Maps object types to appropriate semantic classes
 *
 * @param sdsm The input J3224 SDSM message containing detected objects
 * @param georeference String containing the georeference information for coordinate projection
 * @param is_simulation Boolean flag indicating if running in simulation mode (affects timestamps)
 * @param conversion_adjustment Optional configuration for position and covariance adjustments
 *
 * @return carma_cooperative_perception_interfaces::msg::DetectionList
 *          message containing the transformed detections in CARMA Platform format
 */
auto to_detection_list_msg(
  const carma_v2x_msgs::msg::SensorDataSharingMessage & sdsm, std::string_view georeference,
  bool is_simulation, const std::optional<SdsmToDetectionListConfig>& conversion_adjustment)
  -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list;
  const auto ref_pos_3d{Position3D::from_msg(sdsm.ref_pos)};

  units::length::meter_t elevation(0.0);
  if(ref_pos_3d.elevation){
    elevation = ref_pos_3d.elevation.value();
  }
  const Wgs84Coordinate ref_pos_wgs84{
    ref_pos_3d.latitude, ref_pos_3d.longitude, elevation};

  const auto ref_pos_map{project_to_carma_map(ref_pos_wgs84, georeference)};

  for (const auto & object_data : sdsm.objects.detected_object_data) {
    const auto common_data{object_data.detected_object_common_data};

    carma_cooperative_perception_interfaces::msg::Detection detection;
    detection.header.frame_id = "map";

    const auto detection_time{calc_detection_time_stamp(
      DDateTime::from_msg(sdsm.sdsm_time_stamp),
      MeasurementTimeOffset::from_msg(common_data.measurement_time))};

    detection.header.stamp = to_time_msg(detection_time, is_simulation);
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

    const auto pos_offset_enu{ned_to_enu(PositionOffsetXYZ::from_msg(common_data.pos))};
    detection.pose.pose.position = to_position_msg(MapCoordinate{
      ref_pos_map.easting + pos_offset_enu.offset_x, ref_pos_map.northing + pos_offset_enu.offset_y,
      ref_pos_map.elevation + pos_offset_enu.offset_z.value_or(units::length::meter_t{0.0})});

    // Adjust object's position to match vector map coordinates as sensor calibrations are not
    // always reliable
    if (conversion_adjustment && conversion_adjustment.value().adjust_pose)
    {
      detection.pose.pose.position.x += conversion_adjustment.value().x_offset;
      detection.pose.pose.position.y += conversion_adjustment.value().y_offset;
    }

    // Variables to store original covariance values for debugging
    double original_pose_covariance_x = 0.0;
    double original_pose_covariance_y = 0.0;
    double original_pose_covariance_z = 0.0;
    double original_pose_covariance_yaw = 0.0;
    double original_twist_covariance_x = 0.0;
    double original_twist_covariance_z = 0.0;
    double original_twist_covariance_yaw = 0.0;

    // Calculate and log original pose covariance values
    try {
      original_pose_covariance_x =
        0.5 * std::pow(j2735_v2x_msgs::to_double(common_data.pos_confidence.pos).value(), 2);
      original_pose_covariance_y = original_pose_covariance_x;

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Original pose covariance X/Y: " << original_pose_covariance_x);
    } catch (const std::bad_optional_access &) {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Missing position confidence");
    }

    try {
      original_pose_covariance_z =
        0.5 * std::pow(j2735_v2x_msgs::to_double(common_data.pos_confidence.elevation).value(), 2);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Original pose covariance Z: " << original_pose_covariance_z);
    } catch (const std::bad_optional_access &) {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Missing elevation confidence");
    }

    const auto true_heading{units::angle::degree_t{Heading::from_msg(common_data.heading).heading}};

    // Note: This should really use the detection's WGS-84 position, so the
    // convergence will be off slightly. TODO
    const units::angle::degree_t grid_convergence{
      calculate_grid_convergence(ref_pos_wgs84, georeference)};

    const auto grid_heading{true_heading - grid_convergence};
    const auto enu_yaw{heading_to_enu_yaw(grid_heading)};

    tf2::Quaternion quat_tf;

    if (conversion_adjustment && conversion_adjustment.value().adjust_pose)
    {
      // Adjust object's heading to match vector map coordinates as sensor calibrations are not
      // always reliable
      auto yaw_with_offset = units::angle::radian_t{enu_yaw} +
        units::angle::radian_t{units::angle::degree_t{conversion_adjustment.value().yaw_offset}};
      auto new_yaw = std::fmod(remove_units(yaw_with_offset) + 2 * M_PI, 2 * M_PI);
      quat_tf.setRPY(0, 0, new_yaw);
    }
    else
    {
      // No adjustment needed
      quat_tf.setRPY(0, 0, remove_units(units::angle::radian_t{enu_yaw}));
    }

    detection.pose.pose.orientation = tf2::toMsg(quat_tf);

    try {
      // Get original heading/yaw covariance
      original_pose_covariance_yaw =
        0.5 * std::pow(j2735_v2x_msgs::to_double(common_data.heading_conf).value(), 2);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Original pose covariance yaw: " << original_pose_covariance_yaw);
    } catch (const std::bad_optional_access &) {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Missing heading confidence");
    }

    const auto speed{Speed::from_msg(common_data.speed)};
    detection.twist.twist.linear.x =
      remove_units(units::velocity::meters_per_second_t{speed.speed});

    try {
      // Get original linear x velocity covariance
      original_twist_covariance_x =
        0.5 * std::pow(j2735_v2x_msgs::to_double(common_data.speed_confidence).value(), 2);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Original twist covariance X: " << original_twist_covariance_x);
    } catch (const std::bad_optional_access &) {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Missing speed confidence");
    }

    if (common_data.speed_z.speed){
      const auto speed_z{Speed::from_msg(common_data.speed_z)};
      detection.twist.twist.linear.z =
        remove_units(units::velocity::meters_per_second_t{speed_z.speed});

      try {
        // Get original linear z velocity covariance
        original_twist_covariance_z =
          0.5 * std::pow(j2735_v2x_msgs::to_double(common_data.speed_confidence_z).value(), 2);

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
          "Original twist covariance Z: " << original_twist_covariance_z);
      } catch (const std::bad_optional_access &) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
          "Missing z-speed confidence");
      }
    }
    else{
      detection.twist.twist.linear.z = remove_units(units::velocity::meters_per_second_t{0.0});
      original_twist_covariance_z = 0.0;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Original twist covariance Z: 0.0 (speed_z not provided)");
    }

    // NOTE: common_data.accel_4_way.longitudinal, lateral, vert not supported
    // and not needed at the moment for multiple object tracking algorithm
    if(common_data.accel_4_way.yaw_rate){
      const auto accel_set{AccelerationSet4Way::from_msg(common_data.accel_4_way)};
      detection.twist.twist.angular.z =
        remove_units(units::angular_velocity::degrees_per_second_t{accel_set.yaw_rate});

      try {
        // Get original angular z velocity (yaw rate) covariance
        original_twist_covariance_yaw =
          0.5 * std::pow(j2735_v2x_msgs::to_double(common_data.acc_cfd_yaw).value(), 2);

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
          "Original twist covariance yaw: " << original_twist_covariance_yaw);
      } catch (const std::bad_optional_access &) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
          "Missing yaw-rate confidence");
      }
    }
    else{
      detection.twist.twist.angular.z = 0.0;
      original_twist_covariance_yaw = 0.0;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "Original twist covariance yaw: 0.0 (yaw_rate not provided)");
    }

    if (conversion_adjustment && conversion_adjustment.value().overwrite_covariance)
    {
      // Hardcoded pose covariance
      detection.pose.covariance[0] = conversion_adjustment.value().pose_covariance_x;
      detection.pose.covariance[7] = conversion_adjustment.value().pose_covariance_y;
      detection.pose.covariance[14] = conversion_adjustment.value().pose_covariance_z;
      detection.pose.covariance[35] = conversion_adjustment.value().pose_covariance_yaw;

      // Hardcoded twist covariance
      detection.twist.covariance[0] = conversion_adjustment.value().twist_covariance_x;
      detection.twist.covariance[14] = conversion_adjustment.value().twist_covariance_z;
      detection.twist.covariance[35] = conversion_adjustment.value().twist_covariance_yaw;

      // Print comparison between original and hardcoded values
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "POSE COVARIANCE COMPARISON - Original vs Hardcoded: " <<
        "X: " << original_pose_covariance_x << " -> " << detection.pose.covariance[0] << ", " <<
        "Y: " << original_pose_covariance_y << " -> " << detection.pose.covariance[7] << ", " <<
        "Z: " << original_pose_covariance_z << " -> " << detection.pose.covariance[14] << ", " <<
        "Yaw: " << original_pose_covariance_yaw << " -> " << detection.pose.covariance[35]);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sdsm_to_detection_list_node"),
        "TWIST COVARIANCE COMPARISON - Original vs Hardcoded: " <<
        "X: " << original_twist_covariance_x << " -> , " << detection.twist.covariance[0] << ", " <<
        "Z: " << original_twist_covariance_z << " -> , " << detection.twist.covariance[14] << ", " <<
        "Yaw: " << original_twist_covariance_yaw << " -> " << detection.twist.covariance[35]);
    }
    else
    {
      // Original pose covariance
      detection.pose.covariance[0] = original_pose_covariance_x;
      detection.pose.covariance[7] = original_pose_covariance_y;
      detection.pose.covariance[14] = original_pose_covariance_z;
      detection.pose.covariance[35] = original_pose_covariance_yaw;

      // Original twist covariance
      detection.twist.covariance[0] = original_twist_covariance_x;
      detection.twist.covariance[14] = original_twist_covariance_z;
      detection.twist.covariance[35] = original_twist_covariance_yaw;
    }

    // Fill zeros for all other twist covariance values
    for (size_t i = 0; i < 36; ++i) {
      if (i != 0 && i != 14 && i != 35) {
        detection.twist.covariance[i] = 0.0;
      }
    }

    // Fill zeros for all other pose covariance values
    for (size_t i = 0; i < 36; ++i) {
      if (i != 0 && i != 7 && i != 14 && i != 35) {
        detection.pose.covariance[i] = 0.0;
      }
    }

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

  std::vector<uint8_t> id = {0x00, 0x00, 0x00, 0x01};
  sdsm.source_id.id = id;
  sdsm.equipment_type.equipment_type = j3224_v2x_msgs::msg::EquipmentType::OBU;
  sdsm.ref_pos_xy_conf.semi_major = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
  sdsm.ref_pos_xy_conf.semi_minor = j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_UNAVAILABLE;
  sdsm.ref_pos_xy_conf.orientation =
    j2735_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_UNAVAILABLE;
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
