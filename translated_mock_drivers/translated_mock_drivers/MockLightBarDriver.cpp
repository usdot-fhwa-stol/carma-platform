#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include "IMockDriver.cpp"
#include "AbstractMockDriver.cpp"
#include <cav_srvs/GetLights.h>
#include <cav_srvs/GetLightsRequest.h>
#include <cav_srvs/GetLightsResponse.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/SetLightsRequest.h>
#include <cav_srvs/SetLightsResponse.h>
#include <cav_msgs/LightBarStatus.h>
using namespace std;

namespace mock_drivers{
    class MockLightBarDriver : public AbstractMockDriver {
         //topics published
         ros::Publisher lbPub;
         string lbStatusTopic = "lightbar/light_bar_status";

         //service server
         ros::ServiceServer getLightsService;
         ros::ServiceServer setLightsService;
         string lbGetLightService = "lightbar/get_lights";
         string lbSetLightService = "lightbar/set_lights";
         short EXPECTED_DATA_COL_COUNT = 132;
         short SAMPLE_ID_IDX = 1;

         //lightbar states
          bool greenFlash = false;
          bool yellowFlash = false;
          bool leftArrow = false;
          bool rightArrow = false;
          bool sidesSolid = false;
          bool greenSolid = false;
          bool yellowDim = false;
          bool takedown = false;


          public:
            MockLightBarDriver(ros::NodeHandle node) : AbstractMockDriver(node) {
                //topics published
                lbPub = node.advertise<cav_msgs::LightBarStatus>(lbStatusTopic, 100);

                //service server
                setLightsService = node.advertiseService(lbSetLightService, &MockLightBarDriver::setLightService_cb, this);
                getLightsService = node.advertiseService(lbGetLightService, &MockLightBarDriver::getLightService_cb, this);
            }

            //set light service call back
            bool setLightService_cb(cav_srvs::SetLightsRequest& req, cav_srvs::SetLightsResponse& resp){
                cav_msgs::LightBarStatus lightstatus = req.set_state;
                greenFlash = lightstatus.green_flash == 1;
                yellowFlash = lightstatus.flash == 1;
                leftArrow = lightstatus.left_arrow == 1;
                rightArrow = lightstatus.right_arrow== 1;
                sidesSolid = lightstatus.sides_solid == 1;
                greenSolid = lightstatus.green_solid == 1;
                yellowDim = lightstatus.yellow_solid == 1;
                takedown = lightstatus.takedown == 1;
                return true;
            }

            //get light service call back
            bool getLightService_cb(cav_srvs::GetLightsRequest& req, cav_srvs::GetLightsResponse& resp){
                resp.status = getLightBarStatus();
                return true;
            }

            /**
            * Helper function to build the lightbar status message
            * @return The lightbar status message
            */
            cav_msgs::LightBarStatus getLightBarStatus(){
                cav_msgs::LightBarStatus status;
                status.green_solid = greenSolid? 1:0;
                status.yellow_solid = yellowDim? 1:0;
                status.right_arrow = rightArrow? 1:0;
                status.left_arrow = leftArrow? 1:0;
                status.sides_solid = sidesSolid? 1:0;
                status.flash = yellowFlash? 1:0;
                status.green_flash = greenFlash? 1:0;
                status.takedown = takedown? 1:0;
                return status;

            }

            vector<string> getDriverApi() override {
                vector<string> api;
                api.push_back(lbStatusTopic);
                api.push_back(lbGetLightService);
                api.push_back(lbSetLightService);
                return api;

            } 
        
            void publishData(list<vector<string>> data) override {
                cav_msgs::LightBarStatus status = getLightBarStatus();
                lbPub.publish(status);
            }

            short getExpectedColCount() override {
                return EXPECTED_DATA_COL_COUNT;
            }

            short getSampleIdIdx() override {
                return SAMPLE_ID_IDX;
            }

            vector<string> getDriverTypesList() override {
                vector<string> list;
                list.push_back("lightbar");
                return list;
            }
    };

}