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

#pragma once

#include "rosbag_mock_drivers/MockDriver.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <autoware_msgs/ProjectionMatrix.h>

namespace mock_drivers
{
class MockCameraDriver : public MockDriver
{
  template <class T>
  using ConstPtr = boost::shared_ptr<const T>;

  template <class T>
  using ConstPtrRef = const ConstPtr<T>&;

  template <class T>
  using ROSCommsPtr = boost::shared_ptr<ROSComms<T>>;

  template <class T>
  using ConstPtrRefROSComms = ROSComms<ConstPtrRef<T>>;

  template <class T>
  using ConstPtrRefROSCommsPtr = ROSCommsPtr<ConstPtrRef<T>>;

private:

  const std::string bag_prefix_ = "/bag/hardware_interface/";
  const std::string camera_info_topic_ = "camera/camera_info";
  const std::string image_raw_topic_ = "camera/image_raw";
  const std::string image_rects_topic_ = "camera/image_rects";
  const std::string projection_matrix_topic_ = "camera/projection_matrix";

public:
  MockCameraDriver(bool dummy = false);
  int run();
  void parserCB(const cav_simulation_msgs::BagData::ConstPtr& msg);
  bool driverDiscovery();

  //            void addPassthroughPub(const std::string& sub_topic, ConstPtrRefROSCommsPtr<T>& sub_ptr, const
  //            std::string& pub_topic, ROSCommsPtr<T>& pub_ptr, bool latch, size_t queue_size) {

  /*! \brief Function adds both a publisher and subscriber */  // void (*sub_cb)(ConstPtrRef<T>)
  template <typename T, bool has_header = true>
  void addPassthroughPub(const std::string& sub_topic, const std::string& pub_topic, bool latch, size_t queue_size)
  {
    // Create pointers for publishers
    ROSCommsPtr<T> pub_ptr = boost::make_shared<ROSComms<T>>(CommTypes::pub, latch, queue_size, pub_topic);

    mock_driver_node_.addPub(pub_ptr);

    std::function<void(ConstPtrRef<T>)> callback = std::bind(
        [&](ConstPtrRef<T> in) {
          T out;
          if (has_header)
          {
            out.header.stamp = ros::Time::now();
          }
          mock_driver_node_.publishData<const T&, has_header>(pub_topic, out);
        },
        std::placeholders::_1);

    ConstPtrRefROSCommsPtr<T> outbound_sub_ptr_ =
        boost::make_shared<ConstPtrRefROSComms<T>>(callback, CommTypes::sub, false, queue_size, sub_topic);

    mock_driver_node_.addSub(outbound_sub_ptr_);
  }
};

}  // namespace mock_drivers