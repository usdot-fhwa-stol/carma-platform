// Copyright 2020-2023 Leidos
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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "motion_computation/impl/mobility_path_to_external_object_helpers.hpp"
#include "motion_computation/message_conversions.hpp"
#include "motion_computation/motion_computation_worker.hpp"

namespace motion_computation
{
TEST(MotionComputationWorker, Constructor)
{
  // Create logger object
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger =
    node->get_node_logging_interface();
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();

  // Create MotionComputationWorker object
  MotionComputationWorker worker(
    [](const carma_perception_msgs::msg::ExternalObjectList &) {}, logger, clock);
}

TEST(MotionComputationWorker, MotionPredictionCallback)
{
  bool published_data = false;
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger =
    node->get_node_logging_interface();
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
  MotionComputationWorker mcw_sensor_only(
    [&](const carma_perception_msgs::msg::ExternalObjectList & obj_pub) {
      published_data = true;
      ASSERT_EQ(obj_pub.objects.size(), 1ul);

      bool isFilled = false;
      for (auto item : obj_pub.objects) {
        if (item.predictions.size() > 0) isFilled = true;
      }
      ASSERT_EQ(isFilled, true);
    },
    logger, clock);

  MotionComputationWorker mcw_mobility_only(
    [&](const carma_perception_msgs::msg::ExternalObjectList & obj_pub) {
      published_data = true;
      ASSERT_EQ(obj_pub.objects.size(), 0ul);
    },
    logger, clock);

  MotionComputationWorker mcw_mixed_operation(
    [&](const carma_perception_msgs::msg::ExternalObjectList & obj_pub) {
      published_data = true;
      ASSERT_EQ(obj_pub.objects.size(), 2ul);
    },
    logger, clock);

  mcw_sensor_only.setDetectionInputFlags(true, false, false, false);     // SENSORS_ONLY
  mcw_mobility_only.setDetectionInputFlags(false, false, false, true);   // MOBILITY_PATH_ONLY
  mcw_mixed_operation.setDetectionInputFlags(true, false, false, true);  // PATH_AND_SENSORS

  // 1 to 1 transform
  std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
  std::unique_ptr<std_msgs::msg::String> georeference_ptr1 =
    std::make_unique<std_msgs::msg::String>();
  std::unique_ptr<std_msgs::msg::String> georeference_ptr2 =
    std::make_unique<std_msgs::msg::String>();
  std::unique_ptr<std_msgs::msg::String> georeference_ptr3 =
    std::make_unique<std_msgs::msg::String>();
  georeference_ptr1->data = base_proj;
  georeference_ptr2->data = base_proj;
  georeference_ptr3->data = base_proj;
  mcw_sensor_only.georeferenceCallback(move(georeference_ptr1));      // Set projection
  mcw_mobility_only.georeferenceCallback(move(georeference_ptr2));    // Set projection
  mcw_mixed_operation.georeferenceCallback(move(georeference_ptr3));  // Set projection

  carma_perception_msgs::msg::ExternalObject msg;

  /* Create test message */
  msg.presence_vector = 16;
  msg.object_type = 3;
  /* Test ExternalObject Presence Vector Values */
  ASSERT_GT(msg.presence_vector, 0);
  bool pvValid = false;
  for (auto i = 0; i < 10; i++)  // Test whether presence vector values in ExternalObject are valid
  {
    if (
      msg.presence_vector ==
      pow(2, i))  // presence vector is valid if it matches binary value between 1-512
      pvValid = true;
  }
  ASSERT_EQ(pvValid, true);
  /* Test ExternalObject Object Type Values */
  bool otValid = false;
  for (int i = 0; i <= 4; i++) {
    if (msg.object_type == i) otValid = true;
  }
  ASSERT_EQ(otValid, true);

  /* Test ExternalObjectList */
  carma_perception_msgs::msg::ExternalObjectList obj;

  obj.objects.push_back(msg);
  ASSERT_GT(obj.objects.size(), 0);

  // add mobilitypath data
  carma_v2x_msgs::msg::MobilityPath input_path;

  // INPUT PATH
  input_path.m_header.sender_bsm_id = "FFFFFFFF";
  input_path.m_header.timestamp = 1000;
  input_path.trajectory.location.ecef_x = 0;  // local map 0,0,0
  input_path.trajectory.location.ecef_y = 0;
  input_path.trajectory.location.ecef_z = 0;

  carma_v2x_msgs::msg::LocationOffsetECEF location;
  location.offset_x = 0;
  location.offset_y = 0;
  location.offset_z = 0;

  input_path.trajectory.offsets.push_back(location);

  std::unique_ptr<carma_v2x_msgs::msg::MobilityPath> input_path_ptr1 =
    std::make_unique<carma_v2x_msgs::msg::MobilityPath>(input_path);
  std::unique_ptr<carma_v2x_msgs::msg::MobilityPath> input_path_ptr2 =
    std::make_unique<carma_v2x_msgs::msg::MobilityPath>(input_path);

  std::unique_ptr<carma_perception_msgs::msg::ExternalObjectList> obj_list_ptr1 =
    std::make_unique<carma_perception_msgs::msg::ExternalObjectList>(obj);
  std::unique_ptr<carma_perception_msgs::msg::ExternalObjectList> obj_list_ptr2 =
    std::make_unique<carma_perception_msgs::msg::ExternalObjectList>(obj);

  // Check sensor only mode
  mcw_sensor_only.mobilityPathCallback(
    move(input_path_ptr1));  // added a mobilitypath which won't be processed
  mcw_sensor_only.predictionLogic(move(obj_list_ptr1));
  // assert published_data is true
  ASSERT_EQ(published_data, true);

  // check mixed operation mode
  published_data = false;
  mcw_mixed_operation.mobilityPathCallback(move(input_path_ptr2));
  mcw_mixed_operation.predictionLogic(move(obj_list_ptr2));
  // assert published_data is true
  ASSERT_EQ(published_data, true);
}

TEST(MotionComputationWorker, ComposePredictedState)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger =
    node->get_node_logging_interface();
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
  MotionComputationWorker mcw(
    [&](const carma_perception_msgs::msg::ExternalObjectList &) {}, logger, clock);

  // 1 to 1 transform
  std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
  std::unique_ptr<std_msgs::msg::String> georeference_ptr =
    std::make_unique<std_msgs::msg::String>();
  georeference_ptr->data = base_proj;
  mcw.georeferenceCallback(move(georeference_ptr));  // Set projection

  rclcpp::Time time_stamp = rclcpp::Time(5, 0);
  tf2::Vector3 curr = {5, 0, 0};
  tf2::Vector3 prev = {4, 0, 0};

  auto res = conversion::impl::composePredictedState(
    curr, prev, time_stamp, time_stamp + rclcpp::Duration(100000000), 0.0);
  auto test_result = std::get<0>(res);

  ASSERT_NEAR(test_result.predicted_position.position.x, 4.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_position.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_position.orientation.x, 0.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_position.orientation.y, 0.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_position.orientation.z, 0.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_velocity.linear.x, 10.0, 0.0001);
  ASSERT_EQ(test_result.header.stamp, time_stamp);

  curr = {5, 5, 0};
  prev = {0, 0, 0};

  res = conversion::impl::composePredictedState(
    curr, prev, time_stamp, time_stamp + rclcpp::Duration(100000000), std::get<1>(res));
  test_result = std::get<0>(res);

  ASSERT_NEAR(test_result.predicted_position.position.x, 0.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_position.position.y, 0.0, 0.0001);
  ASSERT_NEAR(test_result.predicted_position.orientation.w, 0.9238, 0.0001);
  ASSERT_NEAR(test_result.predicted_velocity.linear.x, 5.0 * sqrt(2) / 0.1, 0.0001);
  ASSERT_EQ(test_result.header.stamp, time_stamp);
}

TEST(MotionComputationWorker, PsmToExternalObject)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  MotionComputationWorker mcw(
    [&](const carma_perception_msgs::msg::ExternalObjectList &) {},
    node->get_node_logging_interface(), node->get_node_clock_interface());

  // 1 to 1 transform
  std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
  std::unique_ptr<std_msgs::msg::String> georeference_ptr =
    std::make_unique<std_msgs::msg::String>();
  georeference_ptr->data = base_proj;
  mcw.georeferenceCallback(move(georeference_ptr));  // Set projection

  std::string map_frame = "map";

  // Test no georef
  std::unique_ptr<carma_v2x_msgs::msg::PSM> input_ptr1 =
    std::make_unique<carma_v2x_msgs::msg::PSM>();
  carma_perception_msgs::msg::ExternalObject output, expected;

  RCLCPP_INFO_STREAM(node->get_logger(), "Running conversion test with empty message");
  conversion::convert(
    *input_ptr1, output, map_frame, mcw.prediction_period_, mcw.prediction_time_step_,
    *(mcw.map_projector_), mcw.ned_in_map_rotation_, mcw.node_clock_);

  boost::posix_time::ptime output1_timestamp =
    boost::posix_time::from_time_t(output.header.stamp.sec) +
    boost::posix_time::nanosec(output.header.stamp.nanosec);
  auto input1_time = builtin_interfaces::msg::Time(node->now());
  auto input1_timestamp = boost::posix_time::from_time_t(input1_time.sec) +
                          boost::posix_time::nanosec(input1_time.nanosec);
  boost::posix_time::time_duration duration = output1_timestamp - input1_timestamp;

  // Expect that time difference between when test start and ended is not more than 60 seconds
  EXPECT_TRUE(duration.seconds() < 60 && duration.seconds() > -60);

  // Test with values assigned
  // INPUT
  carma_v2x_msgs::msg::PSM input;
  input.basic_type.type |= j2735_v2x_msgs::msg::PersonalDeviceUserType::A_PEDESTRIAN;
  input.sec_mark.millisecond = 40998;
  input.id.id = {1, 2, 3, 4};

  input.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PATH_HISTORY;
  input.path_history.presence_vector |= carma_v2x_msgs::msg::PathHistory::HAS_INITIAL_POSITION;
  input.path_history.initial_position.presence_vector |=
    carma_v2x_msgs::msg::FullPositionVector::HAS_UTC_TIME;
  input.path_history.initial_position.utc_time.presence_vector |=
    j2735_v2x_msgs::msg::DDateTime::YEAR;
  input.path_history.initial_position.utc_time.year.year = 2022;
  input.path_history.initial_position.utc_time.presence_vector |=
    j2735_v2x_msgs::msg::DDateTime::MONTH;
  input.path_history.initial_position.utc_time.month.month = 4;
  input.path_history.initial_position.utc_time.presence_vector |=
    j2735_v2x_msgs::msg::DDateTime::DAY;
  input.path_history.initial_position.utc_time.day.day = 21;
  input.path_history.initial_position.utc_time.presence_vector |=
    j2735_v2x_msgs::msg::DDateTime::HOUR;
  input.path_history.initial_position.utc_time.hour.hour = 9;
  input.path_history.initial_position.utc_time.presence_vector |=
    j2735_v2x_msgs::msg::DDateTime::MINUTE;
  input.path_history.initial_position.utc_time.minute.minute = 21;
  input.path_history.initial_position.utc_time.presence_vector |=
    j2735_v2x_msgs::msg::DDateTime::SECOND;
  input.path_history.initial_position.utc_time.second.millisecond = 40998;

  // Input time as string for posix time
  std::string input_time_str = "2022-04-21 9:21:40.998";
  boost::posix_time::ptime input_timestamp = boost::posix_time::time_from_string(input_time_str);

  input.speed.velocity = 8.98;
  // Set Confidence
  input.accuracy.presence_vector |= carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_AVAILABLE;
  input.accuracy.presence_vector |=
    carma_v2x_msgs::msg::PositionalAccuracy::ACCURACY_ORIENTATION_AVAILABLE;
  input.accuracy.semi_major = 217;
  input.accuracy.semi_minor = 217;
  input.accuracy.orientation = 60050;

  // Define position
  input.position.latitude = 0.00001;
  input.position.longitude = 0.00002;
  input.position.elevation = 2;

  input.heading.heading = 60.0;

  input.presence_vector |= carma_v2x_msgs::msg::PSM::HAS_PATH_PREDICTION;
  input.path_prediction.radius_of_curvature = 50.0;

  RCLCPP_INFO_STREAM(node->get_logger(), "Running conversion test with defined values");

  std::string projection =
    "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
    "+no_defs";
  std_msgs::msg::String msg;
  msg.data = projection;
  lanelet::projection::LocalFrameProjector local_projector(msg.data.c_str());

  georeference_ptr = std::make_unique<std_msgs::msg::String>();
  georeference_ptr->data = projection;
  mcw.georeferenceCallback(move(georeference_ptr));

  carma_perception_msgs::msg::ExternalObject output2;
  std::unique_ptr<carma_v2x_msgs::msg::PSM> input_ptr2 =
    std::make_unique<carma_v2x_msgs::msg::PSM>(input);
  conversion::convert(
    *input_ptr2, output2, map_frame, mcw.prediction_period_, mcw.prediction_time_step_,
    local_projector, mcw.ned_in_map_rotation_, mcw.node_clock_);

  // compare id
  int id = 0;
  for (int i = 0; i < input.id.id.size(); i++) {
    id |= input.id.id[i] << (8 * i);
  }
  ASSERT_EQ(id, output2.id);
  ASSERT_EQ(output2.header.frame_id, map_frame);
  // check if object size matches pedestrian
  ASSERT_EQ(output2.size.x, 0.5);
  ASSERT_EQ(output2.size.y, 0.5);
  ASSERT_EQ(output2.size.z, 1.0);

  // check timestamp
  boost::posix_time::ptime output_timestamp =
    boost::posix_time::from_time_t(output2.header.stamp.sec) +
    boost::posix_time::nanosec(output2.header.stamp.nanosec);
  EXPECT_EQ(
    boost::posix_time::to_iso_extended_string(output_timestamp),
    boost::posix_time::to_iso_extended_string(input_timestamp));

  // check position
  float lat_deg_to_meter_conversion_const =
    1.11 / 0.00001;  // unit values from successful conversion check
  float Long_deg_to_meter_conversion_const =
    1.105 / 0.00001;  // unit values from successful conversion check

  EXPECT_NEAR(
    output2.pose.pose.position.x, input.position.longitude * Long_deg_to_meter_conversion_const,
    0.05);
  EXPECT_NEAR(
    output2.pose.pose.position.y, input.position.latitude * lat_deg_to_meter_conversion_const,
    0.05);

  // check heading
  // convert input deg to rad
  float heading_in_rad = input.heading.heading * 0.0174533;
  tf2::Quaternion R_n_h;  // Rotation of sensor heading report in NED frame
  R_n_h.setRPY(0, 0, heading_in_rad);
  tf2::Quaternion R_m_n(mcw.ned_in_map_rotation_);

  tf2::Quaternion R_m_s = R_m_n * R_n_h;

  EXPECT_NEAR(output2.pose.pose.orientation.x, std::fabs(R_m_s.getX()), 0.000001);
  EXPECT_NEAR(output2.pose.pose.orientation.y, std::fabs(R_m_s.getY()), 0.000001);
  EXPECT_NEAR(output2.pose.pose.orientation.z, std::fabs(R_m_s.getZ()), 0.000001);
  EXPECT_NEAR(output2.pose.pose.orientation.w, std::fabs(R_m_s.getW()), 0.000001);
}

TEST(MotionComputationWorker, MobilityPathToExternalObject)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
  MotionComputationWorker mcw(
    [&](const carma_perception_msgs::msg::ExternalObjectList &) {},
    node->get_node_logging_interface(), clock);

  // 1 to 1 transform
  std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
  std::unique_ptr<std_msgs::msg::String> georeference_ptr =
    std::make_unique<std_msgs::msg::String>();
  georeference_ptr->data = base_proj;
  mcw.georeferenceCallback(move(georeference_ptr));  // Set projection

  // Test no georef
  std::unique_ptr<carma_v2x_msgs::msg::MobilityPath> input_ptr1 =
    std::make_unique<carma_v2x_msgs::msg::MobilityPath>();
  carma_perception_msgs::msg::ExternalObject output, expected;

  conversion::convert(*input_ptr1, output, *(mcw.map_projector_));

  ASSERT_EQ(output.header.stamp, expected.header.stamp);  // empty object returned

  // INPUT
  carma_v2x_msgs::msg::MobilityPath input;
  input.m_header.sender_bsm_id = "FFFFFFFF";
  input.m_header.timestamp = 1000;
  input.trajectory.location.ecef_x = 0;  // local map 0,0,0
  input.trajectory.location.ecef_y = 0;
  input.trajectory.location.ecef_z = 0;

  carma_v2x_msgs::msg::LocationOffsetECEF location;
  location.offset_x = 0;
  location.offset_y = 0;
  location.offset_z = 0;

  input.trajectory.offsets.push_back(location);  // First point has no movement

  // Test static/dynamic
  std::unique_ptr<carma_v2x_msgs::msg::MobilityPath> input_ptr2 =
    std::make_unique<carma_v2x_msgs::msg::MobilityPath>(input);

  conversion::convert(*input_ptr2, output, *(mcw.map_projector_));

  ASSERT_FALSE(output.dynamic_obj);

  location.offset_x = 500.00;
  location.offset_y = 0;
  location.offset_z = 0;
  input.trajectory.offsets.push_back(location);  // Second point moves 5m forward

  // Test 0th, 1st point predicted state
  std::unique_ptr<carma_v2x_msgs::msg::MobilityPath> input_ptr3 =
    std::make_unique<carma_v2x_msgs::msg::MobilityPath>(input);

  conversion::convert(*input_ptr3, output, *(mcw.map_projector_));

  ASSERT_NEAR(output.pose.pose.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(output.pose.pose.position.x, 0.0, 0.005);
  ASSERT_NEAR(output.velocity.twist.linear.x, 0.0, 0.1);

  ASSERT_NEAR(output.predictions[0].predicted_position.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(output.predictions[0].predicted_position.position.x, 0.0, 0.03);

  // TODO(carma) VELOCITY ISSUE it really doesn't make sense that we can not move but
  // suddenly have 50m/s velocity. Discuss with reviewer
  ASSERT_NEAR(output.predictions[0].predicted_velocity.linear.x, 50.0, 0.1);

  ASSERT_NEAR(output.predictions[1].predicted_position.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(output.predictions[1].predicted_position.position.x, 5.0, 0.03);
  ASSERT_NEAR(output.predictions[1].predicted_velocity.linear.x, 50.0, 0.1);

  ASSERT_EQ(output.header.stamp, rclcpp::Time(1, 0));
  ASSERT_EQ(output.predictions[0].header.stamp, rclcpp::Time(1, 0.1 * 1e9));
}

TEST(MotionComputationWorker, SynchronizeAndAppend)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
  MotionComputationWorker mcw_mixed_operation(
    [&](const carma_perception_msgs::msg::ExternalObjectList &) {},
    node->get_node_logging_interface(), clock);

  mcw_mixed_operation.setDetectionInputFlags(true, false, false, true);  // enable sensors and paths

  // 1 to 1 transform
  std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
  std::unique_ptr<std_msgs::msg::String> georeference_ptr =
    std::make_unique<std_msgs::msg::String>();
  georeference_ptr->data = base_proj;
  mcw_mixed_operation.georeferenceCallback(move(georeference_ptr));  // Set projection

  // Test no georef
  carma_perception_msgs::msg::ExternalObject sensor_obj, mobility_path_obj;
  sensor_obj.header.stamp = rclcpp::Time(1.6 * 1e9);         // 1.6 Seconds
  mobility_path_obj.header.stamp = rclcpp::Time(1.1 * 1e9);  // 1.1 Seconds
  mobility_path_obj.pose.pose.orientation.w = 1;
  mobility_path_obj.pose.pose.orientation.x = 0;
  mobility_path_obj.pose.pose.orientation.y = 0;
  mobility_path_obj.pose.pose.orientation.z = 0;
  mobility_path_obj.pose.pose.position.x = 200;
  mobility_path_obj.pose.pose.position.y = 0;
  mobility_path_obj.pose.pose.position.z = 0;
  mobility_path_obj.velocity.twist.linear.x = 2000;

  carma_perception_msgs::msg::PredictedState next_state;  // 0
  next_state.header.stamp = rclcpp::Time(1.3 * 1e9);      // 1.3 Seconds
  next_state.predicted_position.orientation.w = 1;
  next_state.predicted_position.orientation.x = 0;
  next_state.predicted_position.orientation.y = 0;
  next_state.predicted_position.orientation.z = 0;
  next_state.predicted_position.position.x = 200;
  next_state.predicted_position.position.y = 0;
  next_state.predicted_position.position.z = 0;
  next_state.predicted_velocity.linear.x = 2000;
  mobility_path_obj.predictions.push_back(next_state);  // 1
  next_state.header.stamp = rclcpp::Time(1.5 * 1e9);    // 1.5 Seconds
  next_state.predicted_position.position.x = 400;
  mobility_path_obj.predictions.push_back(next_state);  // 2
  next_state.header.stamp = rclcpp::Time(1.7 * 1e9);    // 1.7 Seconds
  next_state.predicted_position.position.x = 600;
  mobility_path_obj.predictions.push_back(next_state);  // 3
  next_state.header.stamp = rclcpp::Time(1.9 * 1e9);    // 1.9 Seconds
  next_state.predicted_position.position.x = 800;
  mobility_path_obj.predictions.push_back(next_state);  // 4
  next_state.header.stamp = rclcpp::Time(2.1 * 1e9);    // 2.1 Seconds
  next_state.predicted_position.position.x = 1000;
  mobility_path_obj.predictions.push_back(next_state);  //  5

  carma_perception_msgs::msg::ExternalObjectList sensor_list, mobility_path_list;
  sensor_list.header.stamp = sensor_obj.header.stamp;
  sensor_list.objects.push_back(sensor_obj);
  mobility_path_list.objects.push_back(mobility_path_obj);
  auto result = mcw_mixed_operation.synchronizeAndAppend(sensor_list, mobility_path_list);

  ASSERT_EQ(result.objects.size(), 2ul);

  ASSERT_EQ(result.objects[1].predictions.size(), 2ul);                // we dropped 2 points
  ASSERT_EQ(result.objects[1].header.stamp, rclcpp::Time(1.6 * 1e9));  // 1.7 Seconds
  ASSERT_EQ(result.objects[1].pose.pose.position.x, 500);
  ASSERT_EQ(result.objects[1].predictions[0].header.stamp, rclcpp::Time(1.9 * 1e9));  // 1.9 Seconds
  ASSERT_EQ(result.objects[1].predictions[0].predicted_position.position.x, 800);
  ASSERT_EQ(result.objects[1].predictions[1].header.stamp, rclcpp::Time(2.1 * 1e9));  // 2.1 Seconds
  ASSERT_EQ(result.objects[1].predictions[1].predicted_position.position.x, 1000);
}

TEST(MotionComputationWorker, BSMtoExternalObject)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock = node->get_node_clock_interface();
  MotionComputationWorker mcw(
    [&](const carma_perception_msgs::msg::ExternalObjectList &) {},
    node->get_node_logging_interface(), clock);

  // 1 to 1 transform
  std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
  std::unique_ptr<std_msgs::msg::String> georeference_ptr =
    std::make_unique<std_msgs::msg::String>();
  georeference_ptr->data = base_proj;
  mcw.georeferenceCallback(move(georeference_ptr));  // Set projection

  // Test no georef
  std::unique_ptr<carma_v2x_msgs::msg::MobilityPath> input_ptr1 =
    std::make_unique<carma_v2x_msgs::msg::MobilityPath>();
  carma_perception_msgs::msg::ExternalObject output, expected;

  conversion::convert(*input_ptr1, output, *(mcw.map_projector_));

  ASSERT_EQ(output.header.stamp, expected.header.stamp);  // empty object returned

  // INPUT
  carma_v2x_msgs::msg::BSM input;

  // input.header.stamp = 1000;
  input.core_data.id = {0, 0, 0, 0, 0, 0, 0, 0};
  input.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE;
  input.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE;
  input.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::ELEVATION_AVAILABLE;

  input.core_data.latitude = 100000000;
  input.core_data.longitude = 1000000000;
  input.core_data.elev = 51000;

  input.core_data.presence_vector |= carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE;
  input.core_data.speed = 1000;

  input.core_data.accuracy.semi_major = 100;
  input.core_data.accuracy.semi_minor = 0;
  input.core_data.accuracy.orientation = 0;

  input.core_data.size.vehicle_width = 3;
  input.core_data.size.vehicle_length = 2;

  input.core_data.size.presence_vector |= carma_v2x_msgs::msg::VehicleSize::VEHICLE_WIDTH_AVAILABLE;
  input.core_data.size.presence_vector |=
    carma_v2x_msgs::msg::VehicleSize::VEHICLE_LENGTH_AVAILABLE;

  // Test static/dynamic
  std::unique_ptr<carma_v2x_msgs::msg::BSM> input_ptr2 =
    std::make_unique<carma_v2x_msgs::msg::BSM>(input);

  // TODO(carma) temporary info
  std::string map_frame_id = "map";
  double pred_period = 10.0;
  double pred_step_size = 0.1;
  tf2::Quaternion ned_in_map_rotation;

  conversion::convert(
    *input_ptr2, output, map_frame_id, pred_period, pred_step_size, *(mcw.map_projector_),
    ned_in_map_rotation);

  ASSERT_TRUE(output.dynamic_obj);
  ASSERT_NEAR(output.pose.pose.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(output.velocity.twist.linear.x, 163.8, 0.1);

  ASSERT_NEAR(output.predictions[0].predicted_position.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(output.predictions[0].predicted_position.position.x, 16.38, 0.03);
  ASSERT_NEAR(output.predictions[0].predicted_velocity.linear.x, 163.8, 0.1);

  ASSERT_NEAR(output.predictions[1].predicted_position.orientation.w, 1.0, 0.0001);
  ASSERT_NEAR(output.predictions[1].predicted_position.position.x, 32.76, 0.03);
  ASSERT_NEAR(output.predictions[1].predicted_velocity.linear.x, 163.8, 0.1);
}

}  // namespace motion_computation
