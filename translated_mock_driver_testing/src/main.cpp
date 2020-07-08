#include <ros/ros.h>
#include "translated_mock_driver_testing/MockLightBarDriver.hpp"
using namespace std;
using namespace translated_mock_driver_testing;



int main(int argc, char** argv){
    

    ros::init(argc,argv,"MockLightBarDriver");
    ros::NodeHandle n;
    MockLightBarDriver f (n);
    ROS_INFO("running"); 
    f.publishData();
    f.publishDriverStatus();
    ros::spin();
}