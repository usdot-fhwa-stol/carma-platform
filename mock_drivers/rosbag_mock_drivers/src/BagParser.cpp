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

#include "rosbag_mock_drivers/BagParser.h"
#include <unordered_map>

namespace mock_drivers
{
BagParser::BagParser(std::string file_path, bool dummy)
{
  file_path_ = file_path;
  mock_driver_node_ = MockDriverNode(dummy);
  bag_data_pub_ptr_ = boost::make_shared<ROSComms<cav_simulation_msgs::BagData>>(
      ROSComms<cav_simulation_msgs::BagData>(CommTypes::pub, false, 10, "bag_data"));
}

bool BagParser::getTopics()
{
  bag_.open(file_path_, rosbag::bagmode::Read);
  rosbag::View view(bag_);  // Open view of whole bag
  std::unordered_map<std::string, std::string> topics_and_types;
  for (auto connection : view.getConnections())
  {
    auto topic = topics_and_types.find(connection->topic);
    if (topic != topics_and_types.end())
      continue;
    topics_and_types[connection->topic] = connection->datatype;
  }
  for (auto pair : topics_and_types)
  {  // TODO remove
    ROS_ERROR_STREAM("Bag contains topic " << std::get<0>(pair) << " with type " << std::get<1>(pair));
  }
  bag_.close();
}

bool BagParser::publishCallback()
{
  // open the bag file for a specific time frame (the rate should be set to be faster than the fastest topic)
  static ros::Time startTime;
  static ros::Duration timeFrame = ros::Duration(1.0 / rate_);
  cav_simulation_msgs::BagData message;

  bag_.open(file_path_, rosbag::bagmode::Read);

  if (startTime.isZero())
  {
    startTime = rosbag::View(bag_).getBeginTime();
  }

  for (rosbag::MessageInstance const m : rosbag::View(bag_, startTime, (startTime + timeFrame)))
  {
    // assign the bag data topics to the specific parts of the bag_data message

    // camera
    if (m.getTopic() == "camera/camera_info")
    {
      message.camera_info = *m.instantiate<sensor_msgs::CameraInfo>();
      message.camera_info_flag = true;
    }
    if (m.getTopic() == "camera/image_raw")
    {
      message.image_raw = *m.instantiate<sensor_msgs::Image>();
      message.image_raw_flag = true;
    }
    if (m.getTopic() == "camera/image_rects")
    {
      message.image_rects = *m.instantiate<sensor_msgs::Image>();
      message.image_rects_flag = true;
    }
    if (m.getTopic() == "camera/projection_matrix")
    {
      message.projection_matrix = *m.instantiate<autoware_msgs::ProjectionMatrix>();
      message.projection_matrix_flag = true;
    }

    // can
    if (m.getTopic() == "can/acc_engaged")
    {
      message.acc_engaged = *m.instantiate<std_msgs::Bool>();
      message.acc_engaged_flag = true;
    }
    if (m.getTopic() == "can/acceleration")
    {
      message.acceleration = *m.instantiate<std_msgs::Float64>();
      message.acceleration_flag = true;
    }
    if (m.getTopic() == "can/antilock_brakes_active")
    {
      message.antilock_brakes_active = *m.instantiate<std_msgs::Bool>();
      message.antilock_brakes_active_flag = true;
    }
    if (m.getTopic() == "can/brake_lights")
    {
      message.brake_lights = *m.instantiate<std_msgs::Bool>();
      message.brake_lights_flag = true;
    }
    if (m.getTopic() == "can/brake_position")
    {
      message.brake_position = *m.instantiate<std_msgs::Float64>();
      message.brake_position_flag = true;
    }
    if (m.getTopic() == "can/engine_speed")
    {
      message.engine_speed = *m.instantiate<std_msgs::Float64>();
      message.engine_speed_flag = true;
    }
    if (m.getTopic() == "can/fuel_flow")
    {
      message.fuel_flow = *m.instantiate<std_msgs::Float64>();
      message.fuel_flow_flag = true;
    }
    if (m.getTopic() == "can/odometer")
    {
      message.odometer = *m.instantiate<std_msgs::Float64>();
      message.odometer_flag = true;
    }
    if (m.getTopic() == "can/parking_brake")
    {
      message.parking_brake = *m.instantiate<std_msgs::Bool>();
      message.parking_brake_flag = true;
    }
    if (m.getTopic() == "can/speed")
    {
      message.speed = *m.instantiate<std_msgs::Float64>();
      message.speed_flag = true;
    }
    if (m.getTopic() == "can/stability_ctrl_active")
    {
      message.stability_ctrl_active = *m.instantiate<std_msgs::Bool>();
      message.stability_ctrl_active_flag = true;
    }
    if (m.getTopic() == "can/stability_ctrl_enabled")
    {
      message.stability_ctrl_enabled = *m.instantiate<std_msgs::Bool>();
      message.stability_ctrl_enabled_flag = true;
    }
    if (m.getTopic() == "can/steering_wheel_angle")
    {
      message.steering_wheel_angle = *m.instantiate<std_msgs::Float64>();
      message.steering_wheel_angle_flag = true;
    }
    if (m.getTopic() == "can/throttle_position")
    {
      message.throttle_position = *m.instantiate<std_msgs::Float64>();
      message.throttle_position_flag = true;
    }
    if (m.getTopic() == "can/traction_ctrl_active")
    {
      message.traction_ctrl_active = *m.instantiate<std_msgs::Bool>();
      message.traction_ctrl_active_flag = true;
    }
    if (m.getTopic() == "can/traction_ctrl_enabled")
    {
      message.traction_ctrl_enabled = *m.instantiate<std_msgs::Bool>();
      message.traction_ctrl_enabled_flag = true;
    }
    if (m.getTopic() == "can/transmission_state")
    {
      message.transmission_state = *m.instantiate<j2735_msgs::TransmissionState>();
      message.transmission_state_flag = true;
    }
    if (m.getTopic() == "can/turn_signal_state")
    {
      message.turn_signal_state = *m.instantiate<cav_msgs::TurnSignal>();
      message.turn_signal_state_flag = true;
    }
    if (m.getTopic() == "can/vehicle/twist")
    {
      message.vehicle_twist = *m.instantiate<geometry_msgs::TwistStamped>();
      message.vehicle_twist_flag = true;
    }
    if (m.getTopic() == "can/vehicle_status")
    {
      message.vehicle_status = *m.instantiate<autoware_msgs::VehicleStatus>();
      message.vehicle_status_flag = true;
    }
    if (m.getTopic() == "can/velocity_accel")
    {
      message.velocity_accel = *m.instantiate<automotive_platform_msgs::VelocityAccel>();
      message.velocity_accel_flag = true;
    }

    // comms
    if (m.getTopic() == "comms/inbound_binary_msg")
    {
      message.inbound_binary_msg = *m.instantiate<cav_msgs::ByteArray>();
      message.inbound_binary_msg_flag = true;
    }

    // controller
    if (m.getTopic() == "controller/robot_status")
    {
      message.robot_status = *m.instantiate<cav_msgs::RobotEnabled>();
      message.robot_status_flag = true;
    }

    // gnss
    if (m.getTopic() == "gnss/gnss_fixed_fused")
    {
      message.gnss_fixed_fused = *m.instantiate<gps_common::GPSFix>();
      message.gnss_fixed_fused_flag = true;
    }

    // imu
    if (m.getTopic() == "imu/raw_data")
    {
      message.raw_data = *m.instantiate<sensor_msgs::Imu>();
      message.raw_data_flag = true;
    }

    // lidar
    if (m.getTopic() == "lidar/points_raw")
    {
      message.points_raw = *m.instantiate<sensor_msgs::PointCloud2>();
      message.points_raw_flag = true;
    }

    // radar
    if (m.getTopic() == "radar/status")
    {
      message.status = *m.instantiate<radar_msgs::RadarStatus>();
      message.status_flag = true;
    }
    if (m.getTopic() == "radar/tracks_raw")
    {
      message.tracks_raw = *m.instantiate<radar_msgs::RadarTrackArray>();
      message.tracks_raw_flag = true;
    }

    // roadway_sensor
    if (m.getTopic() == "roadway_sensor/detected_objects")
    {
      message.detected_objects = *m.instantiate<derived_object_msgs::ObjectWithCovariance>();
      message.detected_objects_flag = true;
    }
    if (m.getTopic() == "roadway_sensor/lane_models")
    {
      message.lane_models = *m.instantiate<derived_object_msgs::LaneModels>();
      message.lane_models_flag = true;
    }
  }

  startTime += timeFrame;

  mock_driver_node_.publishData<cav_simulation_msgs::BagData>("bag_data", message);

  bag_.close();  // TODO we cannot be opening and closing the bag repeatedly. We need to open once then read
                 // continuously and publish the data as fast as possible

  return true;
}

int BagParser::run()
{
  mock_driver_node_.init();

  mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_simulation_msgs::BagData>>>(bag_data_pub_ptr_);

  mock_driver_node_.setSpinCallback(std::bind(&BagParser::publishCallback, this));

  getTopics();

  mock_driver_node_.spin(rate_);

  return 0;
}
}  // namespace mock_drivers