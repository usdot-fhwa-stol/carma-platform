


#include <ros/ros.h>
#include <cpp_mock_drivers/TestMockDriver.h>

#include <cpp_mock_drivers/MockCameraDriver.h>
#include <cpp_mock_drivers/MockCANDriver.h>
#include <cpp_mock_drivers/MockCommsDriver.h>
#include <cpp_mock_drivers/MockControllerDriver.h>
#include <cpp_mock_drivers/MockDataDriver.h>
#include <cpp_mock_drivers/MockGNSSDriver.h>
#include <cpp_mock_drivers/MockIMUDriver.h>
#include <cpp_mock_drivers/MockLidarDriver.h>
#include <cpp_mock_drivers/MockRadarDriver.h>
#include <cpp_mock_drivers/MockRoadwaySensorDriver.h>



int main(int argc, char **argv)
{
    //TODO:
    // Initialize a mock driver
    // success?

    ros::init(argc, argv, "mock_drivers");

    mock_drivers::MockCommsDriver node;
    mock_drivers::MockRadarDriver node2;
    mock_drivers::MockLidarDriver node3;
    mock_drivers::MockCANDriver node4;
    mock_drivers::MockControllerDriver node5;

    // mock_drivers::TestMockDriver node;

    node5.run();
    // node.publishData();
    
    return 0;
}