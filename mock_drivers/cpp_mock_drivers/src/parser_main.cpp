


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
#include <cpp_mock_drivers/BagParser.h>



int main(int argc, char **argv)
{
    //TODO:
    // Initialize a mock driver
    // success?

    ros::init(argc, argv, "bag_parser");

    mock_drivers::BagParser parser_node;

    parser_node.run();
    // node.publishData();
    
    return 0;
}