#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include <cav_srvs/GetDriverApi.h>
#include <cav_srvs/GetDriverApiRequest.h>
#include <cav_srvs/GetDriverApiResponse.h>
#include <cav_srvs/Bind.h>
#include <cav_srvs/BindRequest.h>
#include <cav_srvs/BindResponse.h>
#include <cav_msgs/DriverStatus.h>
#include <cav_srvs/GetDriverStatus.h>
#include <cav_srvs/GetDriverStatusRequest.h>
#include <cav_srvs/GetDriverStatusResponse.h>
#include <cav_srvs/GetLights.h>
#include <cav_srvs/GetLightsRequest.h>
#include <cav_srvs/GetLightsResponse.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/SetLightsRequest.h>
#include <cav_srvs/SetLightsResponse.h>
#include <cav_msgs/LightBarStatus.h>
using namespace std;

namespace translated_mock_driver_testing{
    class MockLightBarDriver  {
        


          private:

              cav_msgs::DriverStatus driverStatus;
               //topics published
              ros::Publisher lbPub;
              string lbStatusTopic;
              ros::Publisher discoveryPub;

             //service server
             ros::ServiceServer getLightsService;
            ros::ServiceServer setLightsService;
            ros::ServiceServer getApiService;
            ros::ServiceServer bindService;
            ros::ServiceServer getDriverStatusService;
            string lbGetLightService;
            string lbSetLightService;
            short EXPECTED_DATA_COL_COUNT;
            short SAMPLE_ID_IDX ;

            //lightbar states
              bool greenFlash;
              bool yellowFlash;
              bool leftArrow;
              bool rightArrow;
              bool sidesSolid;
             bool greenSolid;
              bool yellowDim;
              bool takedown;


            //get driver status call back
            bool getDriverStatus_cb(cav_srvs::GetDriverStatusRequest& req, cav_srvs::GetDriverStatusResponse& resp);

            //bind call back
            bool bind_cb(cav_srvs::BindRequest& req, cav_srvs::BindResponse& resp);


            //get api service call back
            bool getApiService_cb(cav_srvs::GetDriverApiRequest& req, cav_srvs::GetDriverApiResponse& resp);

            //set light service call back
            bool setLightService_cb(cav_srvs::SetLightsRequest& req, cav_srvs::SetLightsResponse& resp);

            //get light service call back
            bool getLightService_cb(cav_srvs::GetLightsRequest& req, cav_srvs::GetLightsResponse& resp);

            /**
            * Helper function to build the lightbar status message
            * @return The lightbar status message
            */
            cav_msgs::LightBarStatus getLightBarStatus();

            cav_msgs::DriverStatus getDriverStatus();

            vector<string> getDriverApi();

          public:
             MockLightBarDriver(ros::NodeHandle node);

            //publishes lightbar status
            void publishData();
            
            //publishes driver status
            void publishDriverStatus();
            
            //name of node
            string getNodeName();

            vector<string> getDriverTypesList();
    };

}