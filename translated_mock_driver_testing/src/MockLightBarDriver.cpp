#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include "translated_mock_driver_testing/MockLightBarDriver.hpp"
#include <cav_srvs/GetLights.h>
#include <cav_srvs/GetLightsRequest.h>
#include <cav_srvs/GetLightsResponse.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/SetLightsRequest.h>
#include <cav_srvs/SetLightsResponse.h>
#include <cav_msgs/LightBarStatus.h>
using namespace std;

namespace translated_mock_driver_testing{
      
// add the services in abstract mock driver look into initializing node handle with (~) and ~/names; fix launch file
        
            MockLightBarDriver::MockLightBarDriver(ros::NodeHandle node)  {
                
                
                lbStatusTopic = "lightbar/light_bar_status";
                lbGetLightService = "lightbar/get_lights";
                lbSetLightService = "lightbar/set_lights";
                EXPECTED_DATA_COL_COUNT = 132;
                SAMPLE_ID_IDX = 1;

                //lightbar states
                greenFlash = false;
                yellowFlash = false;
                leftArrow = false;
                rightArrow = false;
                sidesSolid = false;
                greenSolid = false;
                yellowDim = false;
                takedown = false;
                
                
                
                //topics published
                lbPub = node.advertise<cav_msgs::LightBarStatus>(lbStatusTopic, 100);
                discoveryPub = node.advertise<cav_msgs::DriverStatus>("driver_discovery",100);  
                //service server
                getApiService = node.advertiseService("/get_driver_api",&MockLightBarDriver::getApiService_cb, this);
                bindService = node.advertiseService("/bind",&MockLightBarDriver::bind_cb, this);
                getDriverStatusService = node.advertiseService("/get_status",&MockLightBarDriver::getDriverStatus_cb,this);
                setLightsService = node.advertiseService(lbSetLightService, &MockLightBarDriver::setLightService_cb, this);
                getLightsService = node.advertiseService(lbGetLightService, &MockLightBarDriver::getLightService_cb, this);

                driverStatus.status = cav_msgs::DriverStatus::OPERATIONAL;
                publishDriverStatus();
                ROS_INFO("launched node");
            }

            //get driver status call back
            bool MockLightBarDriver::getDriverStatus_cb(cav_srvs::GetDriverStatusRequest& req, cav_srvs::GetDriverStatusResponse& resp){
                resp.status = getDriverStatus();
                ROS_INFO("GET DRIVER STATUS CALL BACK CALLED");
                return true;
            }

            //bind service call back
            bool MockLightBarDriver::bind_cb(cav_srvs::BindRequest& req, cav_srvs::BindResponse& resp){
                ROS_INFO("bind received");
                return true;
            }

            //get api service call back
            bool MockLightBarDriver::getApiService_cb(cav_srvs::GetDriverApiRequest& req, cav_srvs::GetDriverApiResponse& resp){
                resp.api_list = getDriverApi();
                ROS_INFO("GET DRIVER API CALL BACK CALLED");
                return true;
            }

            //set light service call back
            bool MockLightBarDriver::setLightService_cb(cav_srvs::SetLightsRequest& req, cav_srvs::SetLightsResponse& resp){
                cav_msgs::LightBarStatus lightstatus = req.set_state;
                greenFlash = lightstatus.green_flash == 1;
                yellowFlash = lightstatus.flash == 1;
                leftArrow = lightstatus.left_arrow == 1;
                rightArrow = lightstatus.right_arrow== 1;
                sidesSolid = lightstatus.sides_solid == 1;
                greenSolid = lightstatus.green_solid == 1;
                yellowDim = lightstatus.yellow_solid == 1;
                takedown = lightstatus.takedown == 1;
                ROS_INFO("SET LIGHTS CALL BACK CALLED");
                return true;
            }

            //get light service call back
            bool MockLightBarDriver::getLightService_cb(cav_srvs::GetLightsRequest& req, cav_srvs::GetLightsResponse& resp){
                resp.status = getLightBarStatus();
                ROS_INFO("GET LIGHTS CALL BACK CALLED");
                return true;
            }

            /**
            * Helper function to build the lightbar status message
            * @return The lightbar status message
            */
            cav_msgs::LightBarStatus MockLightBarDriver::getLightBarStatus(){
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

            cav_msgs::DriverStatus MockLightBarDriver::getDriverStatus(){
                cav_msgs::DriverStatus status;
            status.name = getNodeName();
            status.status = driverStatus.status;
            status.can = false;
            status.radar = false;
            status.gnss = false;
            status.imu = false;
            status.lidar = false;
            status.roadway_sensor = false;
            status.comms = false;
            status.controller = false;
            status.camera = false;
            status.lightbar = true;
            return status;

            }

            vector<string> MockLightBarDriver::getDriverApi()  {
                vector<string> api;
                api.push_back(lbStatusTopic);
                api.push_back(lbGetLightService);
                api.push_back(lbSetLightService);
                return api;

            } 
        
            void MockLightBarDriver::publishData()  {
                cav_msgs::LightBarStatus status = getLightBarStatus();
                ROS_INFO_STREAM("PUBLISHING "<< status);
                lbPub.publish(status);
            }

            void MockLightBarDriver::publishDriverStatus(){
                discoveryPub.publish(getDriverStatus());
            }

            short MockLightBarDriver::getExpectedColCount()  {
                return EXPECTED_DATA_COL_COUNT;
            }

            short MockLightBarDriver::getSampleIdIdx() {
                return SAMPLE_ID_IDX;
            }

            string MockLightBarDriver::getNodeName(){
                return "MockLightBarDriver";
            }

            vector<string> MockLightBarDriver::getDriverTypesList() {
                vector<string> list;
                list.push_back("lightbar");
                return list;
            }


}