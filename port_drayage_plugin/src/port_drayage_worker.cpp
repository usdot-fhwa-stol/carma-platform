/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include "port_drayage_plugin/port_drayage_worker.h"
#include "ros/ros.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace port_drayage_plugin
{
    OperationID::Operation OperationID::get_operation_ID() const {
        return _operation_enum;
    }

    std::string OperationID::operation_to_string() const {

        // Convert operation enum into a human-readable string
        switch(_operation_enum) {
            case Operation::PICKUP:             return "PICKUP";
            case Operation::DROPOFF:            return "DROPOFF";
            case Operation::ENTER_STAGING_AREA: return "ENTER_STAGING_AREA";
            case Operation::EXIT_STAGING_AREA:  return "EXIT_STAGING_AREA";
            case Operation::ENTER_PORT:         return "ENTER_PORT";
            case Operation::EXIT_PORT:          return "EXIT_PORT";
            case Operation::PORT_CHECKPOINT:    return "PORT_CHECKPOINT";
            case Operation::HOLDING_AREA:       return "HOLDING_AREA";
            default:
                ROS_WARN_STREAM("Conversion of an unsupported operation enum value to a string.");
                return "UNSUPPORTED_OPERATION_ID";
        }
    }

    bool PortDrayageWorker::check_for_stop(const cav_msgs::ManeuverPlanConstPtr& plan, const geometry_msgs::TwistStampedConstPtr& speed) const {
        if (plan == nullptr || speed == nullptr) {
            ROS_DEBUG("Checking for stop when PortDrayagePlugin not properly initialized. Speed or plan is null");
            return false;
        }

        if (plan->maneuvers.size() > 0 && plan->maneuvers[0].type == cav_msgs::Maneuver::STOP_AND_WAIT) {
            cav_msgs::Maneuver stop_maneuver = _cur_plan->maneuvers[0];
            if (stop_maneuver.stop_and_wait_maneuver.parameters.planning_strategic_plugin == PORT_DRAYAGE_PLUGIN_ID) {
                double longitudinal_speed = speed->twist.linear.x; 
                if (longitudinal_speed <= _stop_speed_epsilon) {
                    return true;
                }
            }
        }

        return false;
    }

    // @SONAR_STOP@
    bool PortDrayageWorker::spin() {
        if (check_for_stop(_cur_plan, _cur_speed)) {
            _pdsm.process_event(PortDrayageEvent::ARRIVED_AT_DESTINATION);
        }

        return true;
    }

    void PortDrayageWorker::set_maneuver_plan(const cav_msgs::ManeuverPlanConstPtr& plan) {
        _cur_plan = plan;
    }

    void PortDrayageWorker::set_current_speed(const geometry_msgs::TwistStampedConstPtr& speed) {
        _cur_speed = speed;
    }

    void PortDrayageWorker::initialize() {
        _pdsm.set_on_arrived_at_destination_callback(std::bind(&PortDrayageWorker::on_arrived_at_destination, this));
        _pdsm.set_on_received_new_destination_callback(std::bind(&PortDrayageWorker::on_received_new_destination, this));
    }
    // @SONAR_START@

    void PortDrayageWorker::on_arrived_at_destination() {
        cav_msgs::MobilityOperation msg = compose_arrival_message();
        _publish_mobility_operation(msg);
    }

    void PortDrayageWorker::on_received_new_destination() {       
        //  Populate the service request with the destination coordinates from the last received port drayage mobility operation message
        cav_srvs::SetActiveRoute route_req = compose_set_active_route_request(_latest_mobility_operation_msg.dest_latitude, _latest_mobility_operation_msg.dest_longitude);

        // Call service client to set the new active route
        bool is_route_generation_successful = _set_active_route(route_req);

        if (is_route_generation_successful) {
            // Publish UI Instructions to trigger a pop-up on the Web UI for the user to engage on the newly received route if desired
            cav_msgs::UIInstructions ui_instructions_msg = compose_ui_instructions(_latest_mobility_operation_msg.operation, _previously_completed_operation);
            _publish_ui_instructions(ui_instructions_msg);
        }
        else {
            // Throw exception if route generation was not successful
            ROS_DEBUG_STREAM("Route generation failed. Routing could not be completed.");
            throw std::invalid_argument("Route generation failed. Routing could not be completed.");
        }
    }

    cav_srvs::SetActiveRoute PortDrayageWorker::compose_set_active_route_request(boost::optional<double> dest_latitude, boost::optional<double> dest_longitude) const {
        cav_srvs::SetActiveRoute route_req;
        if (dest_latitude && dest_longitude) {
            route_req.request.choice = cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY;
            route_req.request.routeID = _latest_mobility_operation_msg.operation;

            cav_msgs::Position3D destination_point;
            destination_point.latitude = *_latest_mobility_operation_msg.dest_latitude;
            destination_point.longitude = *_latest_mobility_operation_msg.dest_longitude;
            destination_point.elevation_exists = false;
            
            route_req.request.destination_points.push_back(destination_point);
        }
        else {
            ROS_DEBUG_STREAM("No destination points were received. Routing could not be completed.");
            throw std::invalid_argument("No destination points were received. Routing could not be completed");
        }

        return route_req;
    }

    cav_msgs::UIInstructions PortDrayageWorker::compose_ui_instructions(const std::string& current_operation, const std::string& previous_operation) {
        // Create the text that will be displayed in the Web UI popup
        std::string popup_text = "";

        // Add text that indicates the previous action was completed (if it was a pickup or dropoff action)
        if (previous_operation == _pickup_operation) {
            popup_text += "The pickup action was completed successfully. ";
        }
        else if (previous_operation == _dropoff_operation) {
            popup_text += "The dropoff action was completed successfully. ";
        }
        else if (previous_operation == _holding_area_operation) {
            popup_text += "The inspection was completed successfully. ";
        }

        // Add text to notify the user that the system can be engaged on the newly received route
        popup_text += "A new Port Drayage route with operation type '" + current_operation + "' has been received. "
                      "Select YES to engage the system on the route, or select NO to remain "
                      "disengaged.";

        // Create and populate the UI Instructions message
        cav_msgs::UIInstructions ui_instructions_msg;
        ui_instructions_msg.stamp = ros::Time::now();
        ui_instructions_msg.msg = popup_text;
        ui_instructions_msg.type = cav_msgs::UIInstructions::ACK_REQUIRED; // The popup will be displayed until the user interacts with it
        ui_instructions_msg.response_service = SET_GUIDANCE_ACTIVE_SERVICE_ID; 

        return ui_instructions_msg;
    }

    cav_msgs::MobilityOperation PortDrayageWorker::compose_arrival_message() const {
        cav_msgs::MobilityOperation msg;

        msg.m_header.plan_id = "";
        msg.m_header.sender_id = _host_id;
        msg.m_header.sender_bsm_id = _host_bsm_id;
        msg.m_header.recipient_id = "";
        msg.m_header.timestamp = ros::Time::now().toNSec();

        msg.strategy = PORT_DRAYAGE_STRATEGY_ID;

        // Encode JSON with Boost Property Tree
        using boost::property_tree::ptree;
        ptree pt;
        pt.put("cmv_id", _cmv_id);

        // Add current vehicle location (latitude and longitude)
        ptree location;
        location.put("latitude", _current_gps_position.latitude); 
        location.put("longitude", _current_gps_position.longitude); 
        pt.put_child("location", location);

        // Add flag to indicate whether CMV is carring cargo
        pt.put("cargo", _cargo_id != "");

        // If CMV has arrived at its initial destination, assign 'operation' field for its initial arrival message
        if (_pdsm.get_state() == PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION) {
            // The CMV's initial destination is either the Staging Area Entrance or the Port Entrance
            if (_starting_at_staging_area) {
                pt.put("operation", _enter_staging_area_operation);
            }
            else {
                pt.put("operation", _enter_port_operation);
            }

            // Add cargo_id if CMV is carrying cargo
            if (_cargo_id != "") {
                pt.put("cargo_id", _cargo_id);
            }
        }
        // If CMV has arrived at a received destination, add necessary fields based on the destination type that it has arrived at
        else if (_pdsm.get_state() == PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION) {
            // Assign the 'operation' using the 'operation' from the last received port drayage message
            pt.put("operation", _latest_mobility_operation_msg.operation);

            // Assign the 'action_id' using the 'action_id' from the last received port drayage message
            if (_latest_mobility_operation_msg.current_action_id) {
                pt.put("action_id", *_latest_mobility_operation_msg.current_action_id);
            }
            else {
                ROS_WARN_STREAM("CMV has arrived at a received destination, but does not have an action_id to broadcast.");
            } 

            // Assign specific fields for arrival at a Pickup location
            if (_latest_mobility_operation_msg.operation == _pickup_operation) {
                // Assign 'cargo_id' using the value received in the previous port drayage message
                if (_latest_mobility_operation_msg.cargo_id) {
                    pt.put("cargo_id", *_latest_mobility_operation_msg.cargo_id);
                }
                else {
                    ROS_WARN_STREAM("CMV has arrived at loading area, but does not have a cargo_id to broadcast.");
                }

                if (_cargo_id != "") {
                    ROS_WARN_STREAM("CMV has arrived at a loading area, but it is already carrying cargo.");
                }
            }
            // Assign specific fields for arrival at a Dropoff location
            else if (_latest_mobility_operation_msg.operation == _dropoff_operation) {
                // Assign 'cargo_id' using the ID of the cargo currently being carried
                if (_cargo_id != "") {
                    pt.put("cargo_id", _cargo_id);
                }
                else {
                    ROS_WARN_STREAM("CMV has arrived at a dropoff area, but does not have a cargo_id to broadcast.");
                }
            }
            // Assign 'cargo_id' using the ID of the cargo currently being carried if the CMV is not arriving at a Pickup location
            else {
                if (_cargo_id != "") {
                    pt.put("cargo_id", _cargo_id);
                }
            }
        }

        std::stringstream body_stream;
        boost::property_tree::json_parser::write_json(body_stream, pt);
        msg.strategy_params = body_stream.str();

        return msg;
    }

    void PortDrayageWorker::on_inbound_mobility_operation(const cav_msgs::MobilityOperationConstPtr& msg) {
        // Check if the received message is a new message for port drayage
        if((msg->strategy == PORT_DRAYAGE_STRATEGY_ID) && (msg->strategy_params != _previous_strategy_params)) {
            // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
            using boost::property_tree::ptree;
            ptree pt;
            std::istringstream strategy_params_ss(msg->strategy_params);
            boost::property_tree::json_parser::read_json(strategy_params_ss, pt);

            std::string mobility_operation_cmv_id = pt.get<std::string>("cmv_id");

            // Check if the received MobilityOperation message is intended for this vehicle's cmv_id   
            if(mobility_operation_cmv_id == _cmv_id) {
                // Since a new message indicates the previous action was completed, update all cargo-related data members based on the previous action that was completed
                update_cargo_information_after_action_completion(_latest_mobility_operation_msg);

                // Store the previously received operation
                _previously_completed_operation = _latest_mobility_operation_msg.operation;

                ROS_DEBUG_STREAM("Processing new port drayage MobilityOperation message for cmv_id " << mobility_operation_cmv_id);
                mobility_operation_message_parser(msg->strategy_params);  
                _previous_strategy_params = msg->strategy_params;
                
                _pdsm.process_event(PortDrayageEvent::RECEIVED_NEW_DESTINATION);
            }
            else {
                ROS_DEBUG_STREAM("Ignoring received port drayage MobilityOperation message intended for cmv_id " << mobility_operation_cmv_id);
            }
        }
    }

    void PortDrayageWorker::update_cargo_information_after_action_completion(const PortDrayageMobilityOperationMsg& previous_port_drayage_msg) {
        // If the previously received message was for 'Pickup' or 'Dropoff', update this object's _cargo_id member accordingly
        // Note: This assumes the previous 'Pickup' or 'Dropoff' action was successful
        if (previous_port_drayage_msg.operation == _pickup_operation) {

            if (previous_port_drayage_msg.cargo_id) {
                _cargo_id = *previous_port_drayage_msg.cargo_id;
                ROS_DEBUG_STREAM("CMV completed pickup action. CMV is now carrying cargo ID " << _cargo_id);
            }
            else {
                ROS_WARN_STREAM("CMV has completed pickup, but there is no Cargo ID associated with the picked up cargo.");
            }
        }
        else if (previous_port_drayage_msg.operation == _dropoff_operation) {
            ROS_DEBUG_STREAM("CMV completed dropoff action. CMV is no longer carrying cargo ID " << _cargo_id);
            _cargo_id = ""; // Empty string is used when no cargo is being carried
        }
    }

    void PortDrayageWorker::mobility_operation_message_parser(std::string mobility_operation_strategy_params) {
        // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
        using boost::property_tree::ptree;
        ptree pt;
        std::istringstream mobility_operation_strategy_params_ss(mobility_operation_strategy_params);
        boost::property_tree::json_parser::read_json(mobility_operation_strategy_params_ss, pt);

        // Parse 'operation' field and assign the PortDrayageEvent type for this message accordingly
        _latest_mobility_operation_msg.operation = pt.get<std::string>("operation");
        ROS_DEBUG_STREAM("operation: " << _latest_mobility_operation_msg.operation);

        // If this CMV is commanded to pickup new cargo, check that it isn't already carrying cargo
        if (_latest_mobility_operation_msg.operation == _pickup_operation && _cargo_id != "") {
            ROS_WARN_STREAM("Received 'PICKUP' operation, but CMV is already carrying cargo.");
        }

        // If this CMV is commanded to dropoff cargo, check that it is actually carrying cargo
        if (_latest_mobility_operation_msg.operation == _dropoff_operation && _cargo_id == "") {
            ROS_WARN_STREAM("Received 'DROPOFF' operation, but CMV isn't currently carrying cargo.");
        }

        // Parse 'cargo_id' field if it exists in strategy_params
        if (pt.count("cargo_id") != 0){
            _latest_mobility_operation_msg.cargo_id = pt.get<std::string>("cargo_id");
            ROS_DEBUG_STREAM("cargo id: " << *_latest_mobility_operation_msg.cargo_id);

            // Log message if the cargo ID being dropped off does not match the cargo ID currently being carried
            if (_latest_mobility_operation_msg.operation == _dropoff_operation && _cargo_id != pt.get<std::string>("cargo_id")) {
                ROS_WARN_STREAM("CMV commanded to dropoff an invalid Cargo ID. Currently carrying " << _cargo_id << ", commanded to dropoff " << pt.get<std::string>("cargo_id"));
            }
        }
        else{
            // If this message is for 'PICKUP', then the 'cargo_id' field is required
            if(_latest_mobility_operation_msg.operation == _pickup_operation) {
                ROS_WARN_STREAM("Received 'PICKUP' operation, but no cargo_id was included.");
            }
            else if (_latest_mobility_operation_msg.operation == _dropoff_operation) {
                ROS_WARN_STREAM("Received 'DROPOFF' operation, but no cargo_id was included.");
            }

            _latest_mobility_operation_msg.cargo_id = boost::optional<std::string>();
        }
        
        // Parse 'action_id' field  if it exists in strategy_params
        if (pt.count("action_id") != 0){
            _latest_mobility_operation_msg.current_action_id = pt.get<std::string>("action_id");
            ROS_DEBUG_STREAM("action id: " << *_latest_mobility_operation_msg.current_action_id);
        }
        else{
            _latest_mobility_operation_msg.current_action_id = boost::optional<std::string>();
        }

        // Parse starting longitude/latitude fields if 'location' field exists in strategy_params:
        if (pt.count("location") != 0){
            _latest_mobility_operation_msg.start_longitude = pt.get<double>("location.longitude");
            _latest_mobility_operation_msg.start_latitude = pt.get<double>("location.latitude"); 
            ROS_DEBUG_STREAM("start long: " << *_latest_mobility_operation_msg.start_longitude);
            ROS_DEBUG_STREAM("start lat: " << *_latest_mobility_operation_msg.start_latitude);
        }
        else {
            _latest_mobility_operation_msg.start_longitude = boost::optional<double>();
            _latest_mobility_operation_msg.start_latitude = boost::optional<double>();
        }

        // Parse destination longitude/latitude fields if 'destination' field exists in strategy_params:
        if (pt.count("destination") != 0) {
            _latest_mobility_operation_msg.dest_longitude = pt.get<double>("destination.longitude"); 
            _latest_mobility_operation_msg.dest_latitude = pt.get<double>("destination.latitude"); 
            ROS_DEBUG_STREAM("dest long: " << *_latest_mobility_operation_msg.dest_longitude);
            ROS_DEBUG_STREAM("dest lat: " << *_latest_mobility_operation_msg.dest_latitude);
        }
        else {
            _latest_mobility_operation_msg.dest_longitude = boost::optional<double>();
            _latest_mobility_operation_msg.dest_latitude = boost::optional<double>();
        }
    }      
    
    void PortDrayageWorker::on_guidance_state(const cav_msgs::GuidanceStateConstPtr& msg) {
        // Drayage operations have started when the CMV has been engaged for the first time
        if ((msg->state == cav_msgs::GuidanceState::ENGAGED) && (_pdsm.get_state() == PortDrayageState::INACTIVE) && _enable_port_drayage) {
            ROS_DEBUG_STREAM("CMV has been engaged for the first time. Processing DRAYAGE_START event.");
            _pdsm.process_event(PortDrayageEvent::DRAYAGE_START);
        }
    }

    void PortDrayageWorker::on_route_event(const cav_msgs::RouteEventConstPtr& msg) {
        // CMV has officially arrived at its destination if the previous route was completed and is no longer active
        if (_latest_route_event != nullptr) {
            if (_latest_route_event->event == cav_msgs::RouteEvent::ROUTE_COMPLETED && msg->event == cav_msgs::RouteEvent::ROUTE_LOADED) {
                if (_pdsm.get_state() == PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION || _pdsm.get_state() == PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION) {
                    ROS_DEBUG_STREAM("CMV completed its previous route, and the previous route is no longer active.");
                    ROS_DEBUG_STREAM("Processing ARRIVED_AT_DESTINATION event.");
                    _pdsm.process_event(PortDrayageEvent::ARRIVED_AT_DESTINATION);
                }           
            }
        }

        // Update the latest received route event data member
        _latest_route_event = msg;
    }

    PortDrayageState PortDrayageWorker::get_port_drayage_state() {
        return _pdsm.get_state();
    }

    void PortDrayageWorker::on_new_pose(const geometry_msgs::PoseStampedConstPtr& msg) {
        if (!_map_projector) {
            ROS_DEBUG_STREAM("Ignoring pose message as projection string has not been defined");
            return;
        }

        // Convert pose message contents to a GPS coordinate
        lanelet::GPSPoint coord = _map_projector->reverse( { msg->pose.position.x, msg->pose.position.y, msg->pose.position.z } );

        // Update the locally-stored GPS position of the CMV
        _current_gps_position.latitude = coord.lat;
        _current_gps_position.longitude = coord.lon;
    }        

    void PortDrayageWorker::on_new_georeference(const std_msgs::StringConstPtr& msg) {
        // Build projector from proj string
        _map_projector = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  
    }        


} // namespace port_drayage_plugin
