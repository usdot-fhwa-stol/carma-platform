#include <ros/ros.h>
#include "IMockDriver.cpp"
#include "AbstractMockDriver.cpp"
#include "MockLightBarDriver.cpp"
using namespace std;
using namespace mock_drivers;
int main(int argc, char** argv){

    ros::init(argc,argv,"MockLightbarDriver");
    ros::NodeHandle n;
    MockLightBarDriver f (n);
    f.readAndPublishData();
    ros::spin();

}