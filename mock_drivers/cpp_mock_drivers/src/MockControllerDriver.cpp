/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "cpp_mock_drivers/MockControllerDriver.h"

namespace mock_drivers{

    bool MockControllerDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockControllerDriver";
        discovery_msg.status = 1;

        discovery_msg.can = false;
        discovery_msg.radar = false;
        discovery_msg.gnss = false;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = false;
        discovery_msg.comms = false;
        discovery_msg.controller = true;
        discovery_msg.camera = false;
        discovery_msg.imu = false;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("/hardware_interface/driver_discovery", discovery_msg);

        return true;
    }

    void MockControllerDriver::parserCB(const carma_simulation_msgs::BagData::ConstPtr& msg){
        // generate messages from bag data
        if(msg->robot_status_bool.data){
            cav_msgs::RobotEnabled robot_status = msg->robot_status;
            // publish the data
            mock_driver_node_.publishDataNoHeader<cav_msgs::RobotEnabled>("/hardware_interface/controller/robot_status", robot_status);
        }
    }

    void MockControllerDriver::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg){
        // TODO: add function for vehicle command callback (if necessary)
    }

    bool MockControllerDriver::enableRoboticSrv(cav_srvs::SetEnableRobotic::Request& req, cav_srvs::SetEnableRobotic::Response& res){
        // TODO: add function for the enable robotic service (if necessary)
        return true;
    }

    MockControllerDriver::MockControllerDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        robot_status_ptr_ = boost::make_shared<ROSComms<cav_msgs::RobotEnabled>>(ROSComms<cav_msgs::RobotEnabled>(CommTypes::pub, false, 10, "robot_status"));
        
        std::function<void(const autoware_msgs::VehicleCmd::ConstPtr&)> callback_ptr_1 = std::bind(&MockControllerDriver::vehicleCmdCallback, this, std::placeholders::_1);
        vehicle_cmd_ptr_ = boost::make_shared<ROSComms<const autoware_msgs::VehicleCmd::ConstPtr&>>(ROSComms<const autoware_msgs::VehicleCmd::ConstPtr&>(callback_ptr_1, CommTypes::sub, false, 10, "vehicle_cmd"));
    
        std::function<bool(cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&)> service_ptr_1 = std::bind(&MockControllerDriver::enableRoboticSrv, this, std::placeholders::_1, std::placeholders::_2);
        enable_robotic_ptr_ = boost::make_shared<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>>(ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>(service_ptr_1, CommTypes::srv, "enable_robotic"));
    }

    int MockControllerDriver::run(){

        mock_driver_node_.init();

        // bag parser subscriber
        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        // driver publisher, subscriber, and service
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::RobotEnabled>>>(robot_status_ptr_);
        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const autoware_msgs::VehicleCmd::ConstPtr&>>>(vehicle_cmd_ptr_);
        mock_driver_node_.addSrv<boost::shared_ptr<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>>>(enable_robotic_ptr_);

        // driver discovery publisher
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockControllerDriver::driverDiscovery, this));

        mock_driver_node_.spin(1);

        return 0;
    }

}