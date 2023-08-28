#include <gtest/gtest.h>

#include <carma_cooperative_perception/j2735_types.hpp>
#include <carma_cooperative_perception/msg_conversion.hpp>

TEST(ToTimeMsg, HasSeconds)
{
  carma_cooperative_perception::DDateTime d_date_time;
  d_date_time.second = units::time::second_t{42.13};

  builtin_interfaces::msg::Time expected_msg;
  expected_msg.sec = 42.0;
  expected_msg.nanosec = 130'000'000;

  const auto actual_msg{carma_cooperative_perception::to_time_msg(d_date_time)};

  EXPECT_DOUBLE_EQ(actual_msg.sec, expected_msg.sec);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, expected_msg.nanosec);
}

TEST(ToTimeMsg, NulloptSeconds)
{
  const carma_cooperative_perception::DDateTime d_date_time;

  builtin_interfaces::msg::Time expected_msg;
  expected_msg.sec = 0.0;
  expected_msg.nanosec = 0;

  const auto actual_msg{carma_cooperative_perception::to_time_msg(d_date_time)};

  EXPECT_DOUBLE_EQ(actual_msg.sec, expected_msg.sec);
  EXPECT_DOUBLE_EQ(actual_msg.nanosec, expected_msg.nanosec);
}

TEST(ToDetectionMsg, Simple)
{
  j3224_v2x_msgs::msg::SensorDataSharingMessage sdsm_msg;
  sdsm_msg.sdsm_time_stamp.second.millisecond = 1000;
  sdsm_msg.ref_pos.longitude = -90.703125 * 1e7;  // 1/10 micro degrees
  sdsm_msg.ref_pos.latitude = 32.801128 * 1e7;    // 1/10 micro degrees
  sdsm_msg.ref_pos.elevation_exists = true;
  sdsm_msg.ref_pos.elevation = 3000;  // 10 cm

  j3224_v2x_msgs::msg::DetectedObjectData object_data;
  object_data.detected_object_common_data.detected_id.object_id = 0xBEEF;
  object_data.detected_object_common_data.measurement_time.measurement_time_offset = -100;  // ms

  object_data.detected_object_common_data.heading.heading = 2720;  // true heading; 0.0125 degrees
  object_data.detected_object_common_data.obj_type.object_type =
    object_data.detected_object_common_data.obj_type.VEHICLE;

  object_data.detected_object_common_data.pos.offset_x.object_distance = 1000.0;  // 0.1 m
  object_data.detected_object_common_data.pos.offset_y.object_distance = 1000.0;  // 0.1 m

  object_data.detected_object_common_data.pos.presence_vector |=
    object_data.detected_object_common_data.pos.HAS_OFFSET_Z;
  object_data.detected_object_common_data.pos.offset_z.object_distance = 1000.0;  // 0.1 m

  object_data.detected_object_common_data.speed.speed = 500;     // 0.02 m/s
  object_data.detected_object_common_data.speed_z.speed = 1000;  // 0.02 m/s

  object_data.detected_object_common_data.accel_4_way.longitudinal = 50.0;  // 0.01 m/s^2
  object_data.detected_object_common_data.accel_4_way.lateral = 100.0;      // 0.01 m/s^2
  object_data.detected_object_common_data.accel_4_way.vert = 120.0;         // 0.02 G
  object_data.detected_object_common_data.accel_4_way.yaw_rate = 500.0;     // 0.01 degrees/s

  sdsm_msg.objects.detected_object_data.push_back(object_data);

  const auto detection_list{carma_cooperative_perception::to_detection_list_msg(sdsm_msg)};
  ASSERT_EQ(std::size(detection_list.detections), 1);

  const auto detection{detection_list.detections.at(0)};

  EXPECT_EQ(detection.header.stamp.sec, 0);
  EXPECT_EQ(detection.header.stamp.nanosec, 900'000'000);
  EXPECT_EQ(detection.header.frame_id, "15N");

  EXPECT_NEAR(detection.pose.pose.position.x, 715068.54 + 100.0, 1e-2);   // m (ref pos + offset)
  EXPECT_NEAR(detection.pose.pose.position.y, 3631576.38 + 100.0, 1e-2);  // m (ref pos + offset)
  EXPECT_NEAR(detection.pose.pose.position.z, 300 + 100.0, 1e-3);         // m (ref pos + offset)
  EXPECT_DOUBLE_EQ(detection.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(detection.pose.pose.orientation.y, 0.0);
  EXPECT_NEAR(detection.pose.pose.orientation.z, 0.479035, 1e-5);
  EXPECT_NEAR(detection.pose.pose.orientation.w, 0.877796, 1e-5);

  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.x, 10.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.linear.z, 20.0);
  EXPECT_DOUBLE_EQ(detection.twist.twist.angular.z, 5.0);

  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.y, 1.0);
  EXPECT_DOUBLE_EQ(detection.accel.accel.linear.z, 2.4 * 9.80665);

  EXPECT_EQ(detection.id, std::to_string(0xBEEF));
  EXPECT_EQ(detection.motion_model, detection.MOTION_MODEL_CTRV);
}

TEST(CalcDetectionTimeStamp, Simple) {}

TEST(ToPositionMsg, Simple) {}

TEST(ToDetectionListMsg, Simple) {}
