



#include <ros/ros.h>
#include <TestMockDriver.h>



int main(int argc, char **argv)
{
    //TODO:
    // Initialize a mock driver
    // success?

    ros::init(argc, argv, "mock_drivers");
    mock_drivers::TestMockDriver node;

    node.run();
    
    return 0;
}