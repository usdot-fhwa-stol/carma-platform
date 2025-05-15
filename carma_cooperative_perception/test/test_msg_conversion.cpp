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

#include <carma_cooperative_perception/j2735_types.hpp>
#include <carma_cooperative_perception/msg_conversion.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>

#include <proj.h>
#include <gsl/pointers>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <numeric>

TEST(ToTimeMsg, LiveDateTime)
{
  // Force tests to be in EDT timezone specifically (not switching between EST/EDT)
  setenv("TZ", "EDT4", 1);  // EDT is UTC-4, no DST transition
  tzset();
  // Test with full date/time
  {
    carma_cooperative_perception::DDateTime d_date_time;
    d_date_time.year = units::time::year_t{2023};
    d_date_time.month = carma_cooperative_perception::Month{7};
    d_date_time.day = units::time::day_t{15};
    d_date_time.hour = units::time::hour_t{14};
    d_date_time.minute = units::time::minute_t{30};
    d_date_time.second = units::time::second_t{25.975};

    const auto msg = carma_cooperative_perception::to_time_msg(d_date_time, false);
    EXPECT_EQ(msg.sec, 1689445825);
    EXPECT_EQ(msg.nanosec, 975'000'000);
  }

  // Test year adjustment logic (year > 1900)
  {
    carma_cooperative_perception::DDateTime d_date_time;
    d_date_time.year = units::time::year_t{1950};
    d_date_time.month = carma_cooperative_perception::Month{1};
    d_date_time.day = units::time::day_t{1};
    d_date_time.hour = units::time::hour_t{0};
    d_date_time.minute = units::time::minute_t{0};
    d_date_time.second = units::time::second_t{0};

    EXPECT_THROW(
      carma_cooperative_perception::to_time_msg(d_date_time, false),
      std::invalid_argument);
  }

  // Test optional fields
  {
    carma_cooperative_perception::DDateTime d_date_time;
    d_date_time.year = units::time::year_t{1970}; //1970 Jan 1st 0:0:0 ET is epoch 18000

    const auto msg = carma_cooperative_perception::to_time_msg(d_date_time, false);
    // should be 18000 because this date was not Daylight time, but
    // since the test is in EDT timezone, it will be 14400 seconds
    EXPECT_EQ(msg.sec, 14400);
    EXPECT_EQ(msg.nanosec, 0);
  }

  // Force tests to be in arbitrary timezone
  setenv("TZ", "EDT15", 1);  // EDT is UTC-4, no DST transition
  tzset();
  // Test optional fields
  {
    carma_cooperative_perception::DDateTime d_date_time;
    d_date_time.year = units::time::year_t{1970}; //1970 Jan 1st 0:0:0 ET is epoch 18000

    const auto msg = carma_cooperative_perception::to_time_msg(d_date_time, false);
    EXPECT_EQ(msg.sec, 54000);
    EXPECT_EQ(msg.nanosec, 0);
  }
}

TEST(ToTimeMsg, SimulationModeHasSeconds)
{
  carma_cooperative_perception::DDateTime d_date_time;
  d_date_time.second = units::time::second_t{42.13};

  builtin_interfaces::msg::Time expected_msg;
  expected_msg.sec = 42.0;
  expected_msg.nanosec = 130'000'000;

  const auto actual_msg{carma_cooperative_perception::to_time_msg(d_date_time, true)};

  EXPECT_DOUBLE_EQ(actual_msg.sec, expected_msg.sec);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, expected_msg.nanosec);
}

TEST(ToTimeMsg, SimulationModeNulloptSeconds)
{
  const carma_cooperative_perception::DDateTime d_date_time;

  builtin_interfaces::msg::Time expected_msg;
  expected_msg.sec = 0.0;
  expected_msg.nanosec = 0;

  const auto actual_msg{carma_cooperative_perception::to_time_msg(d_date_time, true)};

  EXPECT_DOUBLE_EQ(actual_msg.sec, expected_msg.sec);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, expected_msg.nanosec);
}

TEST(ToTimeMsg, SimulationModeGeneralConversions)
{
  carma_cooperative_perception::DDateTime d_date_time;

  d_date_time.hour = units::time::hour_t{0};
  d_date_time.minute = units::time::minute_t{0};
  d_date_time.second = units::time::second_t{1};
  auto actual_msg{carma_cooperative_perception::to_time_msg(d_date_time, true)};

  EXPECT_DOUBLE_EQ(actual_msg.sec, 1);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, 0);

  d_date_time.hour = units::time::hour_t{0};
  d_date_time.minute = units::time::minute_t{3};
  d_date_time.second = units::time::second_t{5};
  actual_msg = carma_cooperative_perception::to_time_msg(d_date_time, true);

  EXPECT_DOUBLE_EQ(actual_msg.sec, 185);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, 0);

  d_date_time.hour = units::time::hour_t{2};
  d_date_time.minute = units::time::minute_t{0};
  d_date_time.second = units::time::second_t{0};
  actual_msg = carma_cooperative_perception::to_time_msg(d_date_time, true);

  EXPECT_DOUBLE_EQ(actual_msg.sec, 7200);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, 0);

  d_date_time.hour = units::time::hour_t{2};
  d_date_time.minute = units::time::minute_t{10};
  d_date_time.second = units::time::second_t{30};
  actual_msg = carma_cooperative_perception::to_time_msg(d_date_time, true);

  EXPECT_DOUBLE_EQ(actual_msg.sec, 7830);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, 0);

  d_date_time.hour = units::time::hour_t{3};
  d_date_time.minute = units::time::minute_t{0};
  d_date_time.second = units::time::second_t{50.25};
  actual_msg = carma_cooperative_perception::to_time_msg(d_date_time, true);

  EXPECT_DOUBLE_EQ(actual_msg.sec, 10850);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, 250000000);
}

TEST(ToDetectionMsg, Simple)
{
  carma_v2x_msgs::msg::SensorDataSharingMessage sdsm_msg;
  sdsm_msg.source_id.id = {0xBA, 0xDD, 0xCA, 0xFE};
  sdsm_msg.sdsm_time_stamp.second.millisecond = 1000;
  sdsm_msg.sdsm_time_stamp.presence_vector |= sdsm_msg.sdsm_time_stamp.SECOND;
  sdsm_msg.ref_pos.longitude = -90.703125;  // degrees
  sdsm_msg.ref_pos.latitude = 32.801128;    // degrees
  sdsm_msg.ref_pos.elevation_exists = true;
  sdsm_msg.ref_pos.elevation = 300.0;  // m
  carma_v2x_msgs::msg::DetectedObjectData object_data;
  object_data.detected_object_common_data.detected_id.object_id = 1;
  object_data.detected_object_common_data.measurement_time.measurement_time_offset = -0.1;  // s

  object_data.detected_object_common_data.heading.heading = 34;  // true heading; degrees
  object_data.detected_object_common_data.obj_type.object_type =
    object_data.detected_object_common_data.obj_type.VEHICLE;

  object_data.detected_object_common_data.pos.offset_x.object_distance = 50.0;  // m
  object_data.detected_object_common_data.pos.offset_y.object_distance = 100.0;  // m

  object_data.detected_object_common_data.pos.presence_vector |=
    object_data.detected_object_common_data.pos.HAS_OFFSET_Z;
  object_data.detected_object_common_data.pos.offset_z.object_distance = 100.0;  // m

  object_data.detected_object_common_data.speed.speed = 10;    // m/s
  object_data.detected_object_common_data.speed_z.speed = 20;  // m/s

  object_data.detected_object_common_data.accel_4_way.longitudinal = 0.5;  // m/s^2
  object_data.detected_object_common_data.accel_4_way.lateral = 1.00;      // m/s^2
  object_data.detected_object_common_data.accel_4_way.vert = 23.536;       // m/s^2
  object_data.detected_object_common_data.accel_4_way.yaw_rate = 5.0;      // degrees/s

  object_data.detected_object_common_data.heading_conf.confidence = 3;  // 1 deg
  object_data.detected_object_common_data.pos_confidence.pos.confidence = 9;  // 1 m
  object_data.detected_object_common_data.pos_confidence.elevation.confidence = 9;  // 1 m
  object_data.detected_object_common_data.speed_confidence_z.speed_confidence = 4;  // 1m/s
  object_data.detected_object_common_data.speed_confidence.speed_confidence = 4;  // 1m/s
  object_data.detected_object_common_data.acc_cfd_yaw.yaw_rate_confidence = 4;  // 1 deg/sec

  sdsm_msg.objects.detected_object_data.push_back(object_data);
  constexpr std::string_view georeference{"+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs"};

  auto detection_list{
    carma_cooperative_perception::to_detection_list_msg(sdsm_msg, georeference, true, std::nullopt)};
  ASSERT_EQ(std::size(detection_list.detections), 1U);

  auto detection{detection_list.detections.at(0)};
  EXPECT_EQ(detection.header.stamp.sec, 0);
  EXPECT_NEAR(detection.header.stamp.nanosec, 900'000'000U, 2);  // +/- 2 ns is probably good enough
  EXPECT_EQ(detection.header.frame_id, "map");

  EXPECT_NEAR(detection.pose.pose.position.x, 715068.54 + 100.0, 1e-2);   // m (NED->ENU x, y swaps)
  EXPECT_NEAR(detection.pose.pose.position.y, 3631576.38 + 50.0, 1e-2); // m (NED->ENU x, y swaps)
  EXPECT_NEAR(detection.pose.pose.position.z, 300.0 - 100.0, 1e-3); // m (NED->ENU y sign flips)
  EXPECT_DOUBLE_EQ(detection.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(detection.pose.pose.orientation.y, 0.0);
  EXPECT_NEAR(detection.pose.pose.orientation.z, 0.479035, 1e-5);
  EXPECT_NEAR(detection.pose.pose.orientation.w, 0.877796, 1e-5);

  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.x, 10.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.z, 20.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.angular.z, 5.0);

  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.x, 0.0); //not supported
  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.y, 0.0); //not supported
  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.z, 0.0); //not supported

  EXPECT_EQ(detection.id, "BADDCAFE-1");
  EXPECT_EQ(detection.motion_model, detection.MOTION_MODEL_CTRV);

  EXPECT_DOUBLE_EQ(detection.pose.covariance[0], 0.5);
  EXPECT_DOUBLE_EQ(detection.pose.covariance[7], 0.5);
  EXPECT_DOUBLE_EQ(detection.pose.covariance[14], 0.5);
  EXPECT_DOUBLE_EQ(detection.pose.covariance[35], 0.5);

  EXPECT_DOUBLE_EQ(detection.twist.covariance[0], 0.5);
  EXPECT_DOUBLE_EQ(detection.twist.covariance[14], 0.5);
  EXPECT_DOUBLE_EQ(detection.twist.covariance[35], 0.5);

  // Test with offsets
  carma_cooperative_perception::SdsmToDetectionListConfig config;
  config.overwrite_covariance = true;
  config.adjust_pose = true;
  config.x_offset = 10.0;
  config.y_offset = 50.0;
  config.yaw_offset = 25.0;
  const auto detection_list_with_offsets{
    carma_cooperative_perception::to_detection_list_msg(sdsm_msg, georeference, true, config)};
  ASSERT_EQ(std::size(detection_list_with_offsets.detections), 1U);

  detection = detection_list_with_offsets.detections.at(0);
  EXPECT_EQ(detection.header.stamp.sec, 0);
  EXPECT_NEAR(detection.header.stamp.nanosec, 900'000'000U, 2);  // +/- 2 ns is probably good enough
  EXPECT_EQ(detection.header.frame_id, "map");

  EXPECT_NEAR(detection.pose.pose.position.x, 715068.54 + 100.0 + 10.0, 1e-2);   // m (NED->ENU x, y swaps)
  EXPECT_NEAR(detection.pose.pose.position.y, 3631576.38 + 50.0 + 50.0, 1e-2); // m (NED->ENU x, y swaps)
  EXPECT_NEAR(detection.pose.pose.position.z, 300.0 - 100.0, 1e-3); // m (NED->ENU y sign flips)
  EXPECT_DOUBLE_EQ(detection.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(detection.pose.pose.orientation.y, 0.0);
  EXPECT_NEAR(detection.pose.pose.orientation.z, 0.657670, 1e-5); // added 25 degrees
  EXPECT_NEAR(detection.pose.pose.orientation.w, 0.753307, 1e-5); // added 25 degrees

  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.x, 10.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.z, 20.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.angular.z, 5.0);

  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.x, 0.0); //not supported
  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.y, 0.0); //not supported
  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.z, 0.0); //not supported

  for (size_t i = 0; i < 36; ++i) {
    if (i != 0 && i != 7 && i != 14 && i != 35) {
      EXPECT_DOUBLE_EQ(detection.pose.covariance[i], 0.0);
    }
  }
  for (size_t i = 0; i < 36; ++i) {
    if (i != 0 && i != 14 && i != 35) {
      EXPECT_DOUBLE_EQ(detection.twist.covariance[i], 0.0);
    }
  }
  EXPECT_DOUBLE_EQ(detection.pose.covariance[0], 0.125);
  EXPECT_DOUBLE_EQ(detection.pose.covariance[7], 0.125);
  EXPECT_DOUBLE_EQ(detection.pose.covariance[14], 0.125);
  EXPECT_DOUBLE_EQ(detection.pose.covariance[35], 0.005);

  EXPECT_DOUBLE_EQ(detection.twist.covariance[0], 0.005);
  EXPECT_DOUBLE_EQ(detection.twist.covariance[14], 0.005);
  EXPECT_DOUBLE_EQ(detection.twist.covariance[35], 0.005);


}

TEST(CalcDetectionTimeStamp, Simple)
{
  carma_cooperative_perception::DDateTime d_date_time;
  d_date_time.second = units::time::second_t{5.0};

  carma_cooperative_perception::MeasurementTimeOffset offset{units::time::millisecond_t{2}};

  const auto stamp{carma_cooperative_perception::calc_detection_time_stamp(d_date_time, offset)};

  ASSERT_TRUE(stamp.second.has_value());
  // This reports back as milliseconds because stamp is
  // J2735 DDateTime object with second field represented as milliseconds
  EXPECT_NEAR(carma_cooperative_perception::remove_units(stamp.second.value()), 5002, 0.0001);
}

TEST(ToPositionMsg, Simple)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_m;

  constexpr carma_cooperative_perception::UtmZone zone{
    32, carma_cooperative_perception::Hemisphere::kNorth};
  constexpr carma_cooperative_perception::UtmCoordinate position_utm{zone, 12.0_m, 13.5_m, -0.5_m};
  const auto position_msg{carma_cooperative_perception::to_position_msg(position_utm)};

  EXPECT_DOUBLE_EQ(
    carma_cooperative_perception::remove_units(position_utm.easting), position_msg.x);
  EXPECT_DOUBLE_EQ(
    carma_cooperative_perception::remove_units(position_utm.northing), position_msg.y);
  EXPECT_DOUBLE_EQ(
    carma_cooperative_perception::remove_units(position_utm.elevation), position_msg.z);
}

// No ToDetectionListMsg test because a DetectionList.msg contains only a list of Detection.msg
// elements, the test for which is covered by ToDetectionMsg.
// TEST(ToDetectionListMsg, Simple) {}

TEST(ToDetectionMsg, FromExternalObject)
{
  carma_perception_msgs::msg::ExternalObject object;
  object.header.stamp.sec = 1;
  object.header.stamp.nanosec = 2;
  object.header.frame_id = "test_frame";
  object.bsm_id = {3, 4, 5, 6};
  object.id = 7;
  object.pose.pose.position.x = 8;
  object.pose.pose.position.y = 9;
  object.pose.pose.position.z = 10;
  object.pose.pose.orientation.x = 11;
  object.pose.pose.orientation.y = 12;
  object.pose.pose.orientation.z = 13;
  object.pose.pose.orientation.w = 14;
  object.velocity.twist.linear.x = 15;
  object.velocity.twist.linear.y = 16;
  object.velocity.twist.linear.z = 17;
  object.velocity.twist.angular.x = 18;
  object.velocity.twist.angular.y = 19;
  object.velocity.twist.angular.z = 20;
  object.object_type = object.SMALL_VEHICLE;

  object.presence_vector |= object.BSM_ID_PRESENCE_VECTOR | object.ID_PRESENCE_VECTOR |
                            object.POSE_PRESENCE_VECTOR | object.VELOCITY_PRESENCE_VECTOR |
                            object.OBJECT_TYPE_PRESENCE_VECTOR;

  constexpr carma_cooperative_perception::MotionModelMapping motion_model_mapping{
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRV,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRV,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRA,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CV,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CV};

  const auto detection{
    carma_cooperative_perception::to_detection_msg(object, motion_model_mapping)};

  EXPECT_EQ(detection.header, object.header);
  EXPECT_EQ(detection.id, "3456-7");
  EXPECT_EQ(detection.pose, object.pose);
  EXPECT_EQ(detection.twist, object.velocity);
  EXPECT_EQ(detection.motion_model, detection.MOTION_MODEL_CTRV);
}

TEST(ToDetectionListMsg, FromExternalObjectList)
{
  carma_perception_msgs::msg::ExternalObjectList object_list;
  object_list.objects.emplace_back();
  object_list.objects.emplace_back();

  constexpr carma_cooperative_perception::MotionModelMapping motion_model_mapping{
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRV,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRV,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CTRA,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CV,
    carma_cooperative_perception_interfaces::msg::Detection::MOTION_MODEL_CV};

  const auto detection_list{
    carma_cooperative_perception::to_detection_list_msg(object_list, motion_model_mapping)};

  EXPECT_EQ(std::size(detection_list.detections), 2U);
}

TEST(ToExternalObject, FromTrack)
{
  carma_cooperative_perception_interfaces::msg::Track track;
  track.header.stamp.sec = 1;
  track.header.stamp.nanosec = 2;
  track.header.frame_id = "test_frame";

  track.id = "1234";

  track.pose.pose.position.x = 1;
  track.pose.pose.position.y = 2;
  track.pose.pose.position.z = 3;

  track.pose.pose.orientation.x = 4;
  track.pose.pose.orientation.y = 5;
  track.pose.pose.orientation.z = 6;
  track.pose.pose.orientation.w = 7;

  std::iota(std::begin(track.pose.covariance), std::end(track.pose.covariance), 1U);

  track.twist.twist.linear.x = 1;
  track.twist.twist.linear.y = 2; //ignored as they are in base_link frame, signifying "drift"
  track.twist.twist.linear.z = 3; //ignored as they are in base_link frame, signifying "lift up"

  track.twist.twist.angular.x = 4;
  track.twist.twist.angular.y = 5;
  track.twist.twist.angular.z = 6;

  std::iota(std::begin(track.twist.covariance), std::end(track.twist.covariance), 1U);

  const auto external_object{carma_cooperative_perception::to_external_object_msg(track)};

  EXPECT_TRUE(external_object.presence_vector & external_object.ID_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.POSE_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.VELOCITY_PRESENCE_VECTOR);

  EXPECT_EQ(external_object.id, 1234U);

  EXPECT_EQ(external_object.header, track.header);
  EXPECT_EQ(external_object.pose, track.pose);
  // From base_link frame to map_frame twist accounting for orientation
  EXPECT_NEAR(external_object.velocity.twist.linear.x, 0.0322413, 0.001);
  EXPECT_NEAR(external_object.velocity.twist.linear.y, 0.99948, 0.001);
}


TEST(ToExternalObject, FromTrackNonNumericId)
{
  carma_cooperative_perception_interfaces::msg::Track track;
  track.header.stamp.sec = 1;
  track.header.stamp.nanosec = 2;
  track.header.frame_id = "test_frame";

  track.id = "abcd";

  track.pose.pose.position.x = 1;
  track.pose.pose.position.y = 2;
  track.pose.pose.position.z = 3;

  track.pose.pose.orientation.x = 4;
  track.pose.pose.orientation.y = 5;
  track.pose.pose.orientation.z = 6;
  track.pose.pose.orientation.w = 7;

  std::iota(std::begin(track.pose.covariance), std::end(track.pose.covariance), 1U);

  track.twist.twist.linear.x = 1;
  track.twist.twist.linear.y = 2;
  track.twist.twist.linear.z = 3;

  track.twist.twist.angular.x = 4;
  track.twist.twist.angular.y = 5;
  track.twist.twist.angular.z = 6;

  std::iota(std::begin(track.twist.covariance), std::end(track.twist.covariance), 1U);

  const auto external_object{carma_cooperative_perception::to_external_object_msg(track)};

  EXPECT_FALSE(external_object.presence_vector & external_object.ID_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.POSE_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.VELOCITY_PRESENCE_VECTOR);

  EXPECT_EQ(external_object.header, track.header);
  EXPECT_EQ(external_object.pose, track.pose);
  // From base_link frame to map_frame twist accounting for orientation
  EXPECT_NEAR(external_object.velocity.twist.linear.x, 0.0322413, 0.001);
  EXPECT_NEAR(external_object.velocity.twist.linear.y, 0.99948, 0.001);
}

TEST(ToExternalObject, FromTrackNegativeId)
{
  carma_cooperative_perception_interfaces::msg::Track track;
  track.header.stamp.sec = 1;
  track.header.stamp.nanosec = 2;
  track.header.frame_id = "test_frame";

  track.id = "-1234";

  track.pose.pose.position.x = 1;
  track.pose.pose.position.y = 2;
  track.pose.pose.position.z = 3;

  track.pose.pose.orientation.x = 4;
  track.pose.pose.orientation.y = 5;
  track.pose.pose.orientation.z = 6;
  track.pose.pose.orientation.w = 7;

  std::iota(std::begin(track.pose.covariance), std::end(track.pose.covariance), 1U);

  track.twist.twist.linear.x = 1;
  track.twist.twist.linear.y = 2;
  track.twist.twist.linear.z = 3;

  track.twist.twist.angular.x = 4;
  track.twist.twist.angular.y = 5;
  track.twist.twist.angular.z = 6;

  std::iota(std::begin(track.twist.covariance), std::end(track.twist.covariance), 1U);

  const auto external_object{carma_cooperative_perception::to_external_object_msg(track)};

  // Negative id is expected and converted into positive id (-1234 -> 1234)
  EXPECT_TRUE(external_object.presence_vector & external_object.ID_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.POSE_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.VELOCITY_PRESENCE_VECTOR);

  EXPECT_EQ(external_object.header, track.header);
  EXPECT_EQ(external_object.pose, track.pose);
  // From base_link frame to map_frame twist accounting for orientation
  EXPECT_NEAR(external_object.velocity.twist.linear.x, 0.0322413, 0.001);
  EXPECT_NEAR(external_object.velocity.twist.linear.y, 0.99948, 0.001);
}

TEST(ToExternalObject, FromTrackIdTooLarge)
{
  carma_cooperative_perception_interfaces::msg::Track track;
  track.header.stamp.sec = 1;
  track.header.stamp.nanosec = 2;
  track.header.frame_id = "test_frame";

  track.id = "5294967295";

  track.pose.pose.position.x = 1;
  track.pose.pose.position.y = 2;
  track.pose.pose.position.z = 3;

  track.pose.pose.orientation.x = 4;
  track.pose.pose.orientation.y = 5;
  track.pose.pose.orientation.z = 6;
  track.pose.pose.orientation.w = 7;

  std::iota(std::begin(track.pose.covariance), std::end(track.pose.covariance), 1U);

  track.twist.twist.linear.x = 1;
  track.twist.twist.linear.y = 2;
  track.twist.twist.linear.z = 3;

  track.twist.twist.angular.x = 4;
  track.twist.twist.angular.y = 5;
  track.twist.twist.angular.z = 6;

  std::iota(std::begin(track.twist.covariance), std::end(track.twist.covariance), 1U);

  const auto external_object{carma_cooperative_perception::to_external_object_msg(track)};

  EXPECT_FALSE(external_object.presence_vector & external_object.ID_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.POSE_PRESENCE_VECTOR);
  EXPECT_TRUE(external_object.presence_vector & external_object.VELOCITY_PRESENCE_VECTOR);

  EXPECT_EQ(external_object.header, track.header);
  EXPECT_EQ(external_object.pose, track.pose);
  // From base_link frame to map_frame twist accounting for orientation
  EXPECT_NEAR(external_object.velocity.twist.linear.x, 0.0322413, 0.001);
  EXPECT_NEAR(external_object.velocity.twist.linear.y, 0.99948, 0.001);
}

TEST(ToExternalObjectList, FromTrackList)
{
  carma_cooperative_perception_interfaces::msg::TrackList track_list;
  track_list.tracks.push_back(carma_cooperative_perception_interfaces::msg::Track{});
  track_list.tracks.push_back(carma_cooperative_perception_interfaces::msg::Track{});
  track_list.tracks.push_back(carma_cooperative_perception_interfaces::msg::Track{});

  const auto external_object_list{
    carma_cooperative_perception::to_external_object_list_msg(track_list)};

  ASSERT_EQ(std::size(external_object_list.objects), 3U);
}

TEST(ToDetectedObjectDataMsg, FromExternalObject)
{
  carma_perception_msgs::msg::ExternalObject object;
  object.header.stamp.sec = 1;
  object.header.stamp.nanosec = 2;
  object.header.frame_id = "test_frame";
  object.bsm_id = {3, 4, 5, 6};
  object.id = 7;
  object.pose.pose.position.x = 8;
  object.pose.pose.position.y = 9;
  object.pose.pose.position.z = 10;
  object.pose.pose.orientation.x = 11;
  object.pose.pose.orientation.y = 12;
  object.pose.pose.orientation.z = 13;
  object.pose.pose.orientation.w = 14;
  object.velocity.twist.linear.x = 15;
  object.velocity.twist.linear.y = 16;
  object.velocity.twist.linear.z = 17;
  object.velocity.twist.angular.x = 18;
  object.velocity.twist.angular.y = 19;
  object.velocity.twist.angular.z = 20;
  object.size.x = 21;
  object.size.y = 22;
  object.size.z = 23;
  object.confidence = 0.9;
  object.object_type = object.SMALL_VEHICLE;

  object.presence_vector |= object.BSM_ID_PRESENCE_VECTOR | object.ID_PRESENCE_VECTOR |
                            object.POSE_PRESENCE_VECTOR | object.VELOCITY_PRESENCE_VECTOR |
                            object.CONFIDENCE_PRESENCE_VECTOR | object.OBJECT_TYPE_PRESENCE_VECTOR |
                            object.SIZE_PRESENCE_VECTOR;

  std::string proj_string{
    "+proj=tmerc +lat_0=42.24375605014171 +lon_0=-83.55739733422793 +k=1 +x_0=0 +y_0=0 "
    "+datum=WGS84 +units=m +vunits=m +no_defs"};
  auto shared_transform =
    std::make_shared<lanelet::projection::LocalFrameProjector>(proj_string.c_str());
  const auto detected_object{
    carma_cooperative_perception::to_detected_object_data_msg(object, shared_transform)};

  EXPECT_EQ(detected_object.detected_object_common_data.obj_type.object_type, 1);
  EXPECT_EQ(detected_object.detected_object_common_data.obj_type_cfd.classification_confidence, 90);
  EXPECT_EQ(detected_object.detected_object_common_data.detected_id.object_id, 7);
  EXPECT_NEAR(detected_object.detected_object_common_data.speed.speed, std::sqrt(481), 1e-2);
  EXPECT_EQ(
    detected_object.detected_object_common_data.speed_z.speed, object.velocity.twist.linear.z);

  EXPECT_EQ(detected_object.detected_object_optional_data.det_veh.size.vehicle_width, 22);
  EXPECT_EQ(detected_object.detected_object_optional_data.det_veh.size.vehicle_length, 21);
  EXPECT_EQ(detected_object.detected_object_optional_data.det_veh.height.vehicle_height, 23);
}

TEST(ToSdsmMsg, FromExternalObjectList)
{
  carma_perception_msgs::msg::ExternalObjectList object_list;
  object_list.objects.emplace_back();
  object_list.objects.emplace_back();

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose.position.x = 1;
  current_pose.pose.position.y = 2;
  current_pose.pose.position.z = 3;

  lanelet::projection::LocalFrameProjector local_projector(
    "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 "
    "+datum=WGS84 +units=m +vunits=m +no_defs");
  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projection;
  map_projection = std::make_shared<lanelet::projection::LocalFrameProjector>(local_projector);

  const auto sdsm{
    carma_cooperative_perception::to_sdsm_msg(object_list, current_pose, map_projection)};

  EXPECT_EQ(std::size(sdsm.objects.detected_object_data), 2U);
}

TEST(ToWgsHeading, FromMapYaw)
{
  std::string proj_string{
    "+proj=tmerc +lat_0=42.24375605014171 +lon_0=-83.55739733422793 +k=1 +x_0=0 +y_0=0 "
    "+datum=WGS84 +units=m +vunits=m +no_defs"};

  // Test conversion from map to lat/lon
  // Define map coordinates
  lanelet::BasicPoint3d obj_map_coordinates;
  obj_map_coordinates.x() = -69.7311856222;
  obj_map_coordinates.y() = 331.419278969;
  obj_map_coordinates.z() = 37.8485517148;

  double yaw = 10.0;
  auto shared_transform =
    std::make_shared<lanelet::projection::LocalFrameProjector>(proj_string.c_str());
  double heading = carma_cooperative_perception::remove_units(
    carma_cooperative_perception::enu_orientation_to_true_heading(
      yaw, obj_map_coordinates, shared_transform));

  EXPECT_EQ(heading, 80);
}

TEST(toSdsmMsg, getSDSMOffset)
{
  builtin_interfaces::msg::Time external_object_list_stamp;
  external_object_list_stamp.sec = 100;
  builtin_interfaces::msg::Time external_object_stamp;
  external_object_stamp.sec = 50;
  auto time_offset = (carma_cooperative_perception::calc_sdsm_time_offset(
                        external_object_list_stamp, external_object_stamp))
                       .measurement_time_offset;

  EXPECT_EQ(time_offset, 50);
}

TEST(ToSdsmMsg, getRelativePosition)
{
  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.pose.position.x = 100;
  source_pose.pose.position.y = 100;
  carma_v2x_msgs::msg::PositionOffsetXYZ position_offset;
  position_offset.offset_x.object_distance = 110;
  position_offset.offset_y.object_distance = 110;

  carma_v2x_msgs::msg::PositionOffsetXYZ adjusted_pose =
    carma_cooperative_perception::calc_relative_position(source_pose, position_offset);
  EXPECT_EQ(adjusted_pose.offset_x.object_distance, 10);
}
