#include <ros/ros.h>
#include "translated_mock_driver_testing/MockLightBarDriver.hpp"
using namespace std;
using namespace translated_mock_driver_testing;



int main(int argc, char** argv){
    
    //look into service clients for calling services in MockLightBarDriver

    ros::init(argc,argv,"MockLightBarDriver");
    ros::NodeHandle n ("~");
    ros::ServiceClient client = n.serviceClient<cav_srvs::GetLights>("lightbar/get_lights");
    cav_srvs::GetLights light;
    MockLightBarDriver f (n);
    ROS_INFO("running");
    //client.call(light);
        f.publishData();
        f.publishDriverStatus();
        ros::spin();
}