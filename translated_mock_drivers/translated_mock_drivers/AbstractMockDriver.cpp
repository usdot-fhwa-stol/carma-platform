#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
#include <fstream>
#include <cav_msgs/DriverStatus.h>
#include <cav_srvs/BindRequest.h>
#include <cav_srvs/BindResponse.h>
#include <cav_srvs/GetDriverApi.h>
#include <cav_srvs/GetDriverApiRequest.h>
#include <cav_srvs/GetDriverApiResponse.h>
#include <cav_srvs/Bind.h>
#include <cav_srvs/GetDriverStatus.h>
#include <cav_srvs/GetDriverStatusRequest.h>
#include <cav_srvs/GetDriverStatusResponse.h>
#include <cav_srvs/GetDriverApi.h>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include "IMockDriver.cpp"
using namespace std;

namespace mock_drivers{

class AbstractMockDriver : public IMockDriver {

    protected:
         ros::NodeHandle nh;                // instance variables
         cav_msgs::DriverStatus driverStatus;
         string rosRunID;
         string dataFilePath;
         fstream file;
         string delimiter = ","; // comma for csv files
         ros::Publisher discoveryPub;
         //vector<string> api;
         string name;
         
         
         
         
         void finalize() { //figure this out 
             file.close();
             

         }
         

    public:

          /**
            * Constructor establishes the publishers and subscribers for the ROS network.
            *
            * @param connectedNode the ros node which this driver provides implementations for
            */
        AbstractMockDriver(ros::NodeHandle node){
            nh = node;
            name = node.getNamespace();
            //getting parameters
            rosRunID = node.getParam("/run_id",rosRunID);
            dataFilePath = node.getParam("~/data_file_path",dataFilePath);
            //topics published
            discoveryPub = node.advertise<cav_msgs::DriverStatus>("driver_discovery",100);                // adding topics and servers to api
           // api.push_back(discoveryPub.getTopic());

            //service server
            ros::ServiceServer bindServ = node.advertiseService("~/bind", &AbstractMockDriver::bind_callback, this); 
           // api.push_back(bindServ.getService());
            ros::ServiceServer driverAPIServ = node.advertiseService("~/get_driver_api", &AbstractMockDriver::driver_api_callback, this);
           // api.push_back(driverAPIServ.getService());
            ros::ServiceServer statusServ = node.advertiseService("~/get_status", &AbstractMockDriver::driver_api_callback, this); 
            //api.push_back(statusServ.getService());


        }
       
        //call back response methods once service server recieves response


        //log that bind info has been received ussing ROS INFO STREAM
        bool bind_callback(cav_srvs::BindRequest& request, cav_srvs::BindResponse& response){
            ROS_INFO_STREAM("request for bind received");
            return true;
        }

        //store api elements
         bool driver_api_callback(cav_srvs::GetDriverApiRequest& request, cav_srvs::GetDriverApiResponse& response){
            response.api_list = getDriverApi();         
            return true;
        }

        //set the driver status
         bool status_callback(cav_srvs::GetDriverStatusRequest& request, cav_srvs::GetDriverStatusResponse& response){
            response.status = getDriverStatus();
            return true;
        }

        /**
        * Function which should be called in the onStart function of a containing ROS Node
        * This implementation opens a data file to use for simulation
        * @param node The node which is being started
        */
        void onStart (ros::NodeHandle node) override{
           try{

               file.open(dataFilePath,ios::in);
               driverStatus.status = cav_msgs::DriverStatus::OPERATIONAL;
           }
           catch (ifstream::failure& e){
               //log that file is not found
               driverStatus.status = cav_msgs::DriverStatus::DEGRADED;
                ROS_WARN_STREAM("file is not found");
           }
        }

        /**
        * Helper function to build a driver status message
        * @return The driver status message
        */
        cav_msgs::DriverStatus getDriverStatus(){
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
            status.lightbar = false;
            for (string type : getDriverTypesList()) {
               if (type == "can") {
                   status.can = true;
               } else if (type == "radar") {
                   status.radar = true;
               } else if (type == "gnss") {
                   status.gnss = true;
               } else if (type == "imu") {
                   status.imu = true;
               } else if (type == "lidar") {
                   status.lidar = true;
               } else if (type == "roadway_sensor") {
                   status.roadway_sensor = true;
               } else if (type == "comms") {
                   status.comms = true;
               } else if (type == "controller") {
                   status.controller = true;
               } else if (type == "camera") {
                   status.camera = true;
               } else if (type == "lightbar") {
                   status.radar = true;
               } else {
                   ROS_WARN_STREAM("function getDriverStatus received an unknown driver type");
               }
            }
            return status;
        }

        void onInterruption() override{
            file.close();

        }

        void readAndPublishData() override { //include logging stuff and figure out what sample indexes are
            list<vector<string>> data;
            string dataLine;
            vector<string> elements;
            bool exitBeforeEOF = false;
            int prevSampleIndex = -1;
            int currSampleIndex;
            int pos = 0;
            string token;

            //Skip the header line of all data files need to verify this
            file.ignore(10000,'\n');

            while (getline(file,dataLine)){
                //split elements on delimiter
                elements.clear();
                while ((pos = dataLine.find(delimiter)) != std::string::npos){
                     token = dataLine.substr(0,pos);
                    elements.push_back(token);
                    dataLine.erase(0, pos + delimiter.length());
                }
            
                //update sample index
                if (elements.size() != getExpectedColCount()) {
                    ROS_WARN_STREAM("incorrect number of data elements for published data");
                    continue; //skip error line
                }

                currSampleIndex = stoi(elements[getSampleIdIdx()]);
                //If this is the first sample
                if (prevSampleIndex == -1) {
                    prevSampleIndex = currSampleIndex;
                }
                // If the end of this sample set then exit the loop
                if (currSampleIndex != prevSampleIndex) {
                    exitBeforeEOF = true;
                    break;
                }
                //add the line element to data
                data.push_back(elements);


            }
            //publishes data
            ROS_INFO_STREAM("publishing data");
            publishData(data);

        }
        string getNodeName(){
            return name;
        }
        
        void publishDriverStatus(){
            discoveryPub.publish(getDriverStatus());
        }
        
        void closeDataFile(){
            file.close();
        }
        /**
         * Gets the column number for the sample id
        * @return The column number in the data file
        */
        virtual short getSampleIdIdx();
        
          /**
            * Publishes the provided data array
            * @param data The data to be published usually provided as a direct line from a data file
            */
        virtual void publishData(list<vector<string>> data);

        /**
        * Gets the expected number of row elements in a data line
        * @return The number of expected elements
         */

        virtual short getExpectedColCount();

        /**
        * Gets a list of driver information which this driver can provide (can, comms, sensor, position, controller)
        * @return The list of driver types which this driver satisfies
        */
        virtual vector<string> getDriverTypesList();
        
        virtual vector<string> getDriverApi();
        
        long getPublishDelay(){
            return 100;

        }

};
}




