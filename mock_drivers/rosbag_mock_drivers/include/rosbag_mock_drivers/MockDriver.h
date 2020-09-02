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

#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <cav_msgs/DriverStatus.h>

#include "rosbag_mock_drivers/ROSComms.h"
#include "rosbag_mock_drivers/MockDriverNode.h"
#include "rosbag_mock_drivers/comm_types.h"
#include "rosbag_mock_drivers/DriverType.h"

namespace mock_drivers
{
/*! \brief The template node for the mock drivers that will handle all of the driver logic
 *
 * This class will have virtual functions that define what to do when the mock driver is run,
 * as well as a callback function for when the driver gets a message from the bag parser node.
 *
 * It will also have build in default mock driver publishers and subscribers baked in, such
 * as the driver discovery publisher.
 */

class MockDriver
{
protected:
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

  MockDriverNode mock_driver_node_;
  const std::string bag_prefix_ = "/bag/hardware_interface/";

  const std::string driver_discovery_topic_ = "driver_discovery";

 ROSCommsPtr<cav_msgs::DriverStatus> driver_discovery_pub_ptr_ =
      boost::make_shared<ROSComms<cav_msgs::DriverStatus>>(CommTypes::pub, false, 10, driver_discovery_topic_);

  ros::Time last_discovery_pub_ = ros::Time(0);


  virtual bool onSpin();

  virtual int onRun() = 0;

  void driverDiscovery();

  /*! \brief Returns the mock driver node for the mock driver (used for testing) */
  MockDriverNode getMockDriverNode();

  /*! \brief Function adds both a publisher and subscriber */
  // TODO: This function can be simplified using C++ 17
  template <typename T>
  void addPassthroughPub(const std::string& sub_topic, const std::string& pub_topic, bool latch, size_t queue_size);

  /*! \brief Function adds both a publisher and subscriber */  // void (*sub_cb)(ConstPtrRef<T>)
  template <typename T>
  void addPassthroughPubNoHeader(const std::string& sub_topic, const std::string& pub_topic, bool latch,
                                 size_t queue_size);

public:

  virtual ~MockDriver() = 0;

  /*! \brief A function to initialize the publishers and subsricers and start the node */
  int run();

  /*! \brief A function to call at 1 Hz to publish to the driver discovery topic */

  virtual std::vector<DriverType> getDriverTypes() = 0;
  virtual uint8_t getDriverStatus() = 0;
  virtual unsigned int getRate() = 0;


  bool spinCallback();
};

}  // namespace mock_drivers

#include "impl/MockDriver.tpp"