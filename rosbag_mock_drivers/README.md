# rosbag_mock_drivers

This package contains a node which can mimic the behavior of different types of drivers in the CARMA Platform.
These fake drivers are refereed to as mock_drivers.
The mock drivers in this package function using real rosbag data.
The general behavior is that the rosbag is played under the ```/bag/``` prefix and
then the node will serve as a relay between that bag topic and the CARMA Platform expected topic.
If the message type included a header, then the timestamp will be updated to reflect the ros::Time::now().
This allows the nodes to function both with and without the usage of the ```--clock``` option for the rosbag playback. For example, the Lidar mock driver has the following behavior:

* It subscribes to the ```/bag/hardware_interface/lidar/points_raw``` topic.
* It publishes to the ```/hardware_interface/lidar/points_raw``` topic.
* When a message is received from the bag it is forwarded with an updated timestamp.
* Status heartbeats are sent on the ```/hardware_interface/driver_discovery``` at an ~1 Hz frequency.

## Input Data

The drivers require a rosbag located at the ```/opt/carma/data/mock_drivers/drivers.bag``` file location in order to properly function.
This file is expected to contain all topics expected by the mock drivers which are being run.

## Creating new mock drivers

If a developer wants to implement a new mock driver they should extend the MockDriver class found in the ```include/rosbag_mock_drivers/MockDriver.h``` file. There new class should be located in ```include/rosbag_mock_drivers/``` This would give them something similar to the following:

``` c++
#pragma once

#include "rosbag_mock_drivers/MockDriver.h"

namespace mock_drivers
{

/*! \brief Description of my new driver */
class MockNewDriver : public MockDriver
{
private:
  const std::string topic_name_ = "my_topic";

protected:
  int onRun() override; // Initialization function

public:
  MockLidarDriver(bool dummy = false); // Dummy flag will disable creation of real publishers/subscribers for unit tests
  ~MockLidarDriver() {};
  std::vector<DriverType> getDriverTypes() override; // Returns the type of driver
  uint8_t getDriverStatus() override; // Returns current status
  unsigned int getRate() override; // Returns the desired spin rate. This needs to be at least as fast as incoming bag data.
};

}  // namespace mock_drivers
```

The implementation for this class should go in the ```src/``` folder as a new .cpp file , and would look something like the following:

``` c++

#include "rosbag_mock_drivers/MockNewDriver.h"
#include <my_msgs/NewMsg.h>

namespace mock_drivers
{
std::vector<DriverType> MockNewDriver::getDriverTypes()
{
  return { DriverType::NEW_TYPE };
}

uint8_t MockNewDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockNewDriver::MockNewDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

unsigned int MockNewDriver::getRate()
{
  return 20;
}

int MockNewDriver::onRun()
{
  // driver publisher and subscriber
  // bag_prefix_ is a constant defined in MockDriver.h for identifying bag topics more easily
  addPassthroughPub<my_msgs::NewMsg>(bag_prefix_ + topic_name_, topic_name_, false, 10); // Add a publisher and subscriber in one call for pass through behavior.

  return 0;
}

}  // namespace mock_drivers
```

Once the classes are created, the src/main.cpp file will need to be updated to take an argument specifying the usage of this new type of driver.

``` c++
  else if (strcmp("my_driver", argv[1]) == 0)
  {
    mock_drivers::MockNewDriver node;
    node.run();
  }
```

Additionally, for full integration with carma the carma-config development config will need to be updated mainly the ```drivers.launch``` and ```docker-compose.yml``` launch files. Additionally, the ```carma/launch/mock_drivers.launch``` file should be updated to launch this new mock_driver.

## Testing

Due to the nature of these drivers they are currently testing exclusively using rostest. There are no regular unit tests at this time, but it may be beneficial to add them as future work.
