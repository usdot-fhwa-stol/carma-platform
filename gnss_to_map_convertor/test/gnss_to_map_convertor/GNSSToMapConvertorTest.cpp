/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gnss_to_map_convertor/GNSSToMapConvertor.h"

void assertNear(const tf2::Transform& tf, const geometry_msgs::Pose& pose, double error_bound_dist,
                double error_bound_rad)
{

  ASSERT_NEAR(tf.getOrigin().getX(), pose.position.x, error_bound_dist);
  ASSERT_NEAR(tf.getOrigin().getY(), pose.position.y, error_bound_dist);
  ASSERT_NEAR(tf.getOrigin().getZ(), pose.position.z, error_bound_dist);

  ASSERT_NEAR(tf.getRotation().getX(), pose.orientation.x, error_bound_rad);
  ASSERT_NEAR(tf.getRotation().getY(), pose.orientation.y, error_bound_rad);
  ASSERT_NEAR(tf.getRotation().getZ(), pose.orientation.z, error_bound_rad);
  ASSERT_NEAR(tf.getRotation().getW(), pose.orientation.w, error_bound_rad);
}

TEST(GNSSToMapConvertor, geoReferenceCallback)
{
  gnss_to_map_convertor::GNSSToMapConvertor convertor(
      [](auto msg) {}, [](auto a, auto b) -> boost::optional<geometry_msgs::TransformStamped> { return boost::none; },
      "map", "base_link", "ned_heading");
  std::string base_proj = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 "
                          "+datum=WGS84 +units=m +vunits=m +no_defs";

  // Test no axis in proj
  std_msgs::String msg;
  msg.data = base_proj;
  convertor.geoReferenceCallback(msg);
  auto rotation = convertor.getNedInMapRotation();
  auto projector = convertor.getMapProjector();
  ASSERT_TRUE(!!rotation);
  ASSERT_TRUE(!!projector);

  // Verify projector
  lanelet::GPSPoint gp;
  gp.lat = 38.95197911150576;
  gp.lon = -77.14835128349988;
  gp.ele = 0.0;
  lanelet::BasicPoint3d map_point = projector->forward(gp);  // Origin point of projection
  ASSERT_NEAR(map_point.x(), 0.0, 0.000001);
  ASSERT_NEAR(map_point.y(), 0.0, 0.000001);
  ASSERT_NEAR(map_point.z(), 0.0, 0.000001);

  // Verify rotation
  ASSERT_NEAR(rotation.get().x(), -0.7071068, 0.00001);
  ASSERT_NEAR(rotation.get().y(), -0.7071068, 0.00001);
  ASSERT_NEAR(rotation.get().z(), 0.0, 0.00001);
  ASSERT_NEAR(rotation.get().w(), 0.0, 0.00001);
}

TEST(GNSSToMapConvertor, poseFromGnss)
{
  gnss_to_map_convertor::GNSSToMapConvertor convertor(
      [](auto msg) {}, [](auto a, auto b) -> boost::optional<geometry_msgs::TransformStamped> { return boost::none; },
      "map", "base_link", "ned_heading");
  std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                          "+no_defs";

  lanelet::GPSPoint gp;
  gp.lat = 0.0;
  gp.lon = 0.0;
  gp.ele = 0.0;
  lanelet::projection::LocalFrameProjector projector(base_proj.c_str());
  lanelet::BasicPoint3d map_point = projector.forward(gp);  // Origin point of projection
  ASSERT_NEAR(map_point.x(), 0.0, 0.000001);
  ASSERT_NEAR(map_point.y(), 0.0, 0.000001);
  ASSERT_NEAR(map_point.z(), 0.0, 0.000001);

  tf2::Transform baselink_in_sensor = tf2::Transform::getIdentity();

  tf2::Quaternion ned_in_map_rot(-0.7071068, -0.7071068, 0, 0);  // ENU to NED
  tf2::Quaternion sensor_in_ned(1.0, 0, 0, 0);
  gps_common::GPSFix fix_msg;

  // Test point at prime meridian and equator with 0 heading
  fix_msg.latitude = 0;
  fix_msg.longitude = 0;
  fix_msg.altitude = 0;

  fix_msg.track = 0;

  gps_common::GPSFixConstPtr fix_ptr(new gps_common::GPSFix(fix_msg));

  geometry_msgs::PoseWithCovarianceStamped result;
  result = convertor.poseFromGnss(baselink_in_sensor, sensor_in_ned, projector, ned_in_map_rot, fix_ptr);

  tf2::Vector3 solTrans(0, 0, 0);  // At origin
  tf2::Quaternion solRot;
  solRot.setRPY(0, 0,
                90.0 * wgs84_utils::DEG2RAD);  // Facing north. setRPY uses extrinsic fixed frame not intrinsic frame
  tf2::Transform solution(solRot, solTrans);

  double error_bound_dist = 0.0001;
  double error_bound_rad = 0.000001;
  assertNear(solution, result.pose.pose, error_bound_dist, error_bound_rad);

  // TODO need to test different location and different heading

  // // Test point at prime meridian and equator with offset heading
  // fix_msg.latitude = 0;
  // fix_msg.longitude = 0;
  // fix_msg.altitude = 0;

  // fix_msg.track = 90.0;

  // fix_ptr = gps_common::GPSFixConstPtr(new gps_common::GPSFix(fix_msg));

  // result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr);

  // tf2::Vector3 solTrans2(6378137.0, 0, 0);
  // tf2::Quaternion solRot2;
  // solRot2.setRPY(90.0 * wgs84_utils::DEG2RAD, 0, 90.0 * wgs84_utils::DEG2RAD);
  // tf2::Transform solution2(solRot2, solTrans2);

  // assertNear(solution2, result.pose.pose, error_bound_dist, error_bound_rad);

  //////// TODO pitch is not currently accounted for in the function under test. This section can be used when it is.
  // Test point at prime meridian and equator with offset pitch
  // fix_msg.latitude = 0;
  // fix_msg.longitude = 0;
  // fix_msg.altitude = 0;

  // heading_msg.heading = 0;
  // heading_msg.pitch = 90.0 * wgs84_utils::DEG2RAD;

  // fix_ptr = sensor_msgs::NavSatFixConstPtr(new sensor_msgs::NavSatFix(fix_msg));
  // heading_ptr = novatel_gps_msgs::NovatelDualAntennaHeadingConstPtr(new
  // novatel_gps_msgs::NovatelDualAntennaHeading(heading_msg));

  // result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr, heading_ptr);

  // tf2::Vector3 solTrans3(6378137.0, 0, 0);
  // tf2::Quaternion solRot3;
  // solRot3.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, 0);
  // tf2::Transform solution3(solRot3, solTrans3);

  // assertNear(solution3, result.pose.pose, error_bound_dist, error_bound_rad);

  // Test point at prime meridian and equator with offset pitch and heading
  // fix_msg.latitude = 0;
  // fix_msg.longitude = 0;
  // fix_msg.altitude = 0;

  // heading_msg.heading = 90.0 * wgs84_utils::DEG2RAD;
  // heading_msg.pitch = 90.0 * wgs84_utils::DEG2RAD;

  // fix_ptr = sensor_msgs::NavSatFixConstPtr(new sensor_msgs::NavSatFix(fix_msg));
  // heading_ptr = novatel_gps_msgs::NovatelDualAntennaHeadingConstPtr(new
  // novatel_gps_msgs::NovatelDualAntennaHeading(heading_msg));

  // result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr, heading_ptr);

  // tf2::Vector3 solTrans4(6378137.0, 0, 0);
  // tf2::Quaternion solRot4;
  // solRot4.setRPY(90.0 * wgs84_utils::DEG2RAD, 0, 0);
  // tf2::Transform solution4(solRot4, solTrans4);

  // assertNear(solution4, result.pose.pose, error_bound_dist, error_bound_rad);
  /////////
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
