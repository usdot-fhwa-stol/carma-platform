


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

    ros::init(argc, argv, "mock_drivers");

    if (strcmp("camera", argv[1]) == 0){
        mock_drivers::MockCameraDriver node;
        node.run();
    }
    
    else if (strcmp("can", argv[1]) == 0){
        mock_drivers::MockCANDriver node;
        node.run();
    }
    
    else if (strcmp("comms", argv[1]) == 0){
        mock_drivers::MockCommsDriver node;
        node.run();
    }
    
    else if (strcmp("controller", argv[1]) == 0){
        mock_drivers::MockControllerDriver node;
        node.run();
    }
    
    else if (strcmp("data", argv[1]) == 0){
        mock_drivers::MockDataDriver node;
        node.run();
    }
    
    else if (strcmp("gnss", argv[1]) == 0){
        mock_drivers::MockGNSSDriver node;
        node.run();
    }

    else if (strcmp("imu", argv[1]) == 0){
        mock_drivers::MockIMUDriver node;
        node.run();
    }

    else if (strcmp("lidar", argv[1]) == 0){
        mock_drivers::MockLidarDriver node;
        node.run();
    }

    else if (strcmp("radar", argv[1]) == 0){
        mock_drivers::MockRadarDriver node;
        node.run();
    }

    else if (strcmp("roadway_sensor", argv[1]) == 0){
        mock_drivers::MockRoadwaySensorDriver node;
        node.run();
    }
    
    return 0;
}