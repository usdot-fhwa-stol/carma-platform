/*
 * Copyright (C) 2022 LEIDOS.
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
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "gnss_to_map_convertor/GNSSToMapConvertor.hpp"
#include "gnss_to_map_convertor/gnss_to_map_convertor_node.hpp"


void assertNear(const tf2::Transform& tf, const geometry_msgs::msg::Pose& pose, double error_bound_dist,
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

    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<gnss_to_map_convertor::Node>(options);

    gnss_to_map_convertor::GNSSToMapConvertor convertor(
        [](auto msg) {}, [](auto a, auto b) -> boost::optional<geometry_msgs::msg::TransformStamped> { return boost::none; },
        "map", "base_link", "ned_heading", worker_node->get_node_logging_interface());

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::string base_proj = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 "
                            "+datum=WGS84 +units=m +vunits=m +no_defs";

    // Test no axis in proj
    std_msgs::msg::String msg;
    msg.data = base_proj;

    std_msgs::msg::String::UniquePtr msg_ptr(new std_msgs::msg::String(msg));
    convertor.geoReferenceCallback(std::move(msg_ptr));
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

    std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.arguments(remaps);

    auto worker_node = std::make_shared<gnss_to_map_convertor::Node>(options);

    gnss_to_map_convertor::GNSSToMapConvertor convertor(
        [](auto msg) {}, [](auto a, auto b) -> boost::optional<geometry_msgs::msg::TransformStamped> { return boost::none; },
        "map", "base_link", "ned_heading", worker_node->get_node_logging_interface());
    
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                            "+no_defs";

    //// Test origin facing north
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
    gps_msgs::msg::GPSFix fix_msg;

    // Test point at prime meridian and equator with 0 heading
    fix_msg.latitude = 0;
    fix_msg.longitude = 0;
    fix_msg.altitude = 0;

    fix_msg.track = 0;

    gps_msgs::msg::GPSFix::UniquePtr fix_ptr(new gps_msgs::msg::GPSFix(fix_msg));

    geometry_msgs::msg::PoseWithCovarianceStamped result;
    result = convertor.poseFromGnss(baselink_in_sensor, sensor_in_ned, projector, ned_in_map_rot, fix_msg);

    tf2::Vector3 solTrans(0, 0, 0);  // At origin
    tf2::Quaternion solRot;
    solRot.setRPY(0, 0,
                    90.0 * wgs84_utils::DEG2RAD);  // Facing north. setRPY uses extrinsic fixed frame not intrinsic frame
    tf2::Transform solution(solRot, solTrans);

    double error_bound_dist = 0.0001;
    double error_bound_rad = 0.000001;
    assertNear(solution, result.pose.pose, error_bound_dist, error_bound_rad);

    ///// Test origin facing east
    gp.lat = 0.0;
    gp.lon = 0.0;
    gp.ele = 0.0;
    map_point = projector.forward(gp);  // Origin point of projection
    ASSERT_NEAR(map_point.x(), 0.0, 0.000001);
    ASSERT_NEAR(map_point.y(), 0.0, 0.000001);
    ASSERT_NEAR(map_point.z(), 0.0, 0.000001);

    baselink_in_sensor = tf2::Transform::getIdentity();

    ned_in_map_rot = tf2::Quaternion(-0.7071068, -0.7071068, 0, 0);  // ENU to NED
    sensor_in_ned = tf2::Quaternion(1.0, 0, 0, 0);

    // Test point at prime meridian and equator with 0 heading
    fix_msg.latitude = 0;
    fix_msg.longitude = 0;
    fix_msg.altitude = 0;

    fix_msg.track = 90.0;

    fix_ptr = gps_msgs::msg::GPSFix::UniquePtr(new gps_msgs::msg::GPSFix(fix_msg));

    result = convertor.poseFromGnss(baselink_in_sensor, sensor_in_ned, projector, ned_in_map_rot, fix_msg);

    solTrans = tf2::Vector3(0, 0, 0);  // At origin
    solRot.setRPY(0, 0, 0.0);          // Facing east. setRPY uses extrinsic fixed frame not intrinsic frame
    solution = tf2::Transform(solRot, solTrans);

    assertNear(solution, result.pose.pose, error_bound_dist, error_bound_rad);

    ///// Test offset from origin facing north with baselink transform
    gp.lat = 0.00001;
    gp.lon = 0.00002;
    gp.ele = 0.0;
    map_point = projector.forward(gp);  // Origin point of projection
    ASSERT_NEAR(map_point.x(), 2.22638982, 0.000001);
    ASSERT_NEAR(map_point.y(), 1.10574276, 0.000001);
    ASSERT_NEAR(map_point.z(), 0.0, 0.000001);

    baselink_in_sensor = tf2::Transform::getIdentity();
    baselink_in_sensor.setOrigin(tf2::Vector3(1.0, 0, 0));  // With baselink transform

    ned_in_map_rot = tf2::Quaternion(-0.7071068, -0.7071068, 0, 0);  // ENU to NED
    sensor_in_ned = tf2::Quaternion(1.0, 0, 0, 0);

    // Test point at prime meridian and equator with 0 heading
    fix_msg.latitude = 0.00001;
    fix_msg.longitude = 0.00002;
    fix_msg.altitude = 0;

    fix_msg.track = 0.0;

    fix_ptr = gps_msgs::msg::GPSFix::UniquePtr(new gps_msgs::msg::GPSFix(fix_msg));

    result = convertor.poseFromGnss(baselink_in_sensor, sensor_in_ned, projector, ned_in_map_rot, fix_msg);

    solTrans = tf2::Vector3(2.22638982, 2.10574276, 0);  // At offset location with baselink transformation
    solRot.setRPY(0, 0,
                    90.0 * wgs84_utils::DEG2RAD);  // Facing north. setRPY uses extrinsic fixed frame not intrinsic frame
    solution = tf2::Transform(solRot, solTrans);

    assertNear(solution, result.pose.pose, error_bound_dist, error_bound_rad);
}


// This test depends on the poseFromGNSS test passing
TEST(GNSSToMapConvertor, gnssFixCb)
{
  std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.arguments(remaps);

  auto worker_node = std::make_shared<gnss_to_map_convertor::Node>(options);

  worker_node->configure(); //Call configure state transition
  worker_node->activate();  //Call activate state transition to get not read for runtime

  boost::optional<geometry_msgs::msg::PoseStamped> pose_msg;
  bool fail = false; // flag for transform lookup failure since FAIL() cannot be used inside lambdas

  gnss_to_map_convertor::GNSSToMapConvertor convertor(  // Create without transforms
      [&](auto msg) {
        pose_msg = msg;  // Record the pose message when it is set
      },
      [](auto a, auto b) -> boost::optional<geometry_msgs::msg::TransformStamped> { return boost::none; }, "map",
      "base_link", "ned_heading", worker_node->get_node_logging_interface());

  gps_msgs::msg::GPSFix fix_msg;
  fix_msg.header.frame_id = "sensor";

  gps_msgs::msg::GPSFix::UniquePtr fix_ptr(new gps_msgs::msg::GPSFix(fix_msg));

  convertor.gnssFixCb(move(fix_ptr));  // Call before any transforms are set
  ASSERT_FALSE(!!pose_msg);

  fix_ptr = gps_msgs::msg::GPSFix::UniquePtr(new gps_msgs::msg::GPSFix(fix_msg));

  convertor = gnss_to_map_convertor::GNSSToMapConvertor(  // Create with only baselink transform
      [&](auto msg) {
        pose_msg = msg;  // Record the pose message when it is set
      },
      [](auto a, auto b) -> boost::optional<geometry_msgs::msg::TransformStamped> {
        if (a.compare("sensor") == 0 && b.compare("base_link") == 0)
        {
          geometry_msgs::msg::TransformStamped msg;
          msg.transform.translation.x = 1.0;
          msg.transform.rotation.w = 1.0;
          return msg;
        }
        return boost::none;
      },
      "map", "base_link", "ned_heading", worker_node->get_node_logging_interface());

  convertor.gnssFixCb(move(fix_ptr));  // Call after baselink transform is set
  ASSERT_FALSE(!!pose_msg);

  convertor = gnss_to_map_convertor::GNSSToMapConvertor(  // Create with transforms
      [&](auto msg) {
        pose_msg = msg;  // Record the pose message when it is set
      },
      [&fail](auto a, auto b) -> boost::optional<geometry_msgs::msg::TransformStamped> {
        if (a.compare("sensor") == 0 && b.compare("base_link") == 0)
        {
          geometry_msgs::msg::TransformStamped msg;
          msg.transform.translation.x = 1.0;
          msg.transform.rotation.w = 1.0;
          return msg;
        }

        if (a.compare("sensor") == 0 && b.compare("ned_heading") == 0)
        {
          geometry_msgs::msg::TransformStamped msg;
          msg.transform.rotation.x = 1.0;
          msg.transform.rotation.w = 0.0;
          return msg;
        }
        fail = true;
        return boost::none;
      },
      "map", "base_link", "ned_heading", worker_node->get_node_logging_interface());

  // Test point at prime meridian and equator with 0 heading
  fix_msg.latitude = 0.00001;
  fix_msg.longitude = 0.00002;
  fix_msg.altitude = 0;

  fix_msg.track = 0;

  fix_ptr = gps_msgs::msg::GPSFix::UniquePtr(new gps_msgs::msg::GPSFix(fix_msg));

  convertor.gnssFixCb(move(fix_ptr));  // Call before projection is set
  ASSERT_FALSE(fail) << "System attempted to lookup transforms that were not expected ";
  ASSERT_FALSE(!!pose_msg);

  std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                          "+no_defs";
  std_msgs::msg::String msg;
  msg.data = base_proj;

  std_msgs::msg::String::UniquePtr msg_ptr = std_msgs::msg::String::UniquePtr(new std_msgs::msg::String(msg));

  convertor.geoReferenceCallback(move(msg_ptr));  // Set projection
  auto rotation = convertor.getNedInMapRotation();
  auto projector = convertor.getMapProjector();
  ASSERT_TRUE(!!rotation);
  ASSERT_TRUE(!!projector);

  fix_ptr = gps_msgs::msg::GPSFix::UniquePtr(new gps_msgs::msg::GPSFix(fix_msg));
  convertor.gnssFixCb(move(fix_ptr));  // call after projection is set

  ASSERT_TRUE(!!pose_msg);

  tf2::Vector3 solTrans(2.22638982, 2.10574276, 0);  // At offset location with baselink transformation
  tf2::Quaternion solRot;
  solRot.setRPY(0, 0, 90.0 * wgs84_utils::DEG2RAD);  // Facing north. setRPY uses extrinsic fixed frame not intrinsic frame
  tf2::Transform solution(solRot, solTrans);

  double error_bound_dist = 0.0001;
  double error_bound_rad = 0.000001;
  assertNear(solution, pose_msg->pose, error_bound_dist, error_bound_rad);

}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 