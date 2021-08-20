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
    bool PortDrayageWorker::check_for_stop(const cav_msgs::ManeuverPlanConstPtr& plan, const geometry_msgs::TwistStampedConstPtr& speed) const {
        if (plan == nullptr || speed == nullptr) {
            ROS_WARN("Checking for stop when PortDrayagePlugin not properly initialized. Speed or plan is null");
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

        // Throw exception if route generation was not successful
        if (!is_route_generation_successful) {
            ROS_DEBUG_STREAM("Route generation failed. Routing could not be completed.");
            throw std::invalid_argument("Route generation failed. Routing could not be completed.");
        }
    }

    cav_srvs::SetActiveRoute PortDrayageWorker::compose_set_active_route_request(boost::optional<double> dest_latitude, boost::optional<double> dest_longitude) const {
        cav_srvs::SetActiveRoute route_req;
        if (dest_latitude && dest_longitude) {
            route_req.request.choice = cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY;

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

    cav_msgs::MobilityOperation PortDrayageWorker::compose_arrival_message() const {
        cav_msgs::MobilityOperation msg;

        msg.header.plan_id = "";
        msg.header.sender_id = _host_id;
        msg.header.sender_bsm_id = _host_bsm_id;
        msg.header.recipient_id = "";
        msg.header.timestamp = ros::Time::now().toNSec();

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
        pt.put("cargo", _has_cargo);

        // If CMV has not received any port drayage messages yet, assign 'operation' field for its initial arrival message
        if (!_has_received_first_mobility_operation_msg) {
            pt.put("operation", PORT_DRAYAGE_INITIAL_ARRIVAL_OPERATION_ID);
        }
        // If CMV has already received at least one port drayage message, add necessary fields based on the destination type that it has arrived at
        else{
            // Assign the 'operation' using the 'operation' from the last received port drayage message
            pt.put("operation", _latest_mobility_operation_msg.operation);

            // Assign the 'action_id' using the 'action_id' from the last received port drayage message
            if (_latest_mobility_operation_msg.current_action_id) {
                pt.put("action_id", *_latest_mobility_operation_msg.current_action_id);
            }
            else {
                ROS_DEBUG_STREAM("CMV has arrived at a received destination, but does not have an action_id to broadcast.");
                throw std::invalid_argument("CMV has arrived at a received destination, but does not have an action_id to broadcast.");
            } 

            // Assign 'cargo_id' field and conduct cargo-related checks if CMV is arriving at a Pickup location
            if (_latest_mobility_operation_msg.operation == PORT_DRAYAGE_PICKUP_OPERATION_ID) {
                // Assign 'cargo_id' using the value received in the previous port drayage message
                if (_latest_mobility_operation_msg.cargo_id) {
                    pt.put("cargo_id", *_latest_mobility_operation_msg.cargo_id);
                }
                else {
                    ROS_DEBUG_STREAM("CMV has arrived at loading area, but does not have a cargo_id to broadcast.");
                    throw std::invalid_argument("CMV has arrived at loading area, but does not have a cargo_id to broadcast.");
                }

                // Throw exception if CMV is arriving at a Pickup location but is already carrying cargo
                if (_has_cargo) {
                    ROS_DEBUG_STREAM("CMV has arrived at a loading area, but it is already carrying cargo.");
                    throw std::invalid_argument("CMV has arrived at a loading area, but it is already carrying cargo.");
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

            // Note: Size of 'unsigned long' is implementation/compiler/architecture specific. Behavior may be undefined if code is run on something with size of 'unsigned long' smaller than 4 bytes.
            unsigned long mobility_operation_cmv_id = pt.get<unsigned long>("cmv_id");

            // Check if the received MobilityOperation message is intended for this vehicle's cmv_id   
            if(mobility_operation_cmv_id == _cmv_id) {
                _has_received_first_mobility_operation_msg = true;

                // Since a new message indicates a the previous action was completed, update all cargo-related data members based on the previous action that was completed
                update_cargo_information_after_action_completion(_latest_mobility_operation_msg);

                ROS_DEBUG_STREAM("Processing new port drayage MobilityOperation message for cmv_id " << mobility_operation_cmv_id);
                mobility_operation_message_parser(msg->strategy_params);  
                _previous_strategy_params = msg->strategy_params;
                
                // Process event based on the PortDrayageEvent associated with the received MobilityOperation message
                switch(_latest_mobility_operation_msg.port_drayage_event_type) {
                    case PortDrayageEvent::RECEIVED_NEW_DESTINATION:
                        ROS_DEBUG_STREAM("Processing RECEIVED_NEW_DESTINATION event for operation type " << _latest_mobility_operation_msg.operation);
                        _pdsm.process_event(PortDrayageEvent::RECEIVED_NEW_DESTINATION);
                        break;
                    default:
                        ROS_DEBUG_STREAM("Not processing an event for operation type " << _latest_mobility_operation_msg.operation);
                        break;
                }
            }
            else {
                ROS_DEBUG_STREAM("Ignoring received port drayage MobilityOperation message intended for cmv_id " << mobility_operation_cmv_id);
            }
        }
    }

    void PortDrayageWorker::update_cargo_information_after_action_completion(PortDrayageMobilityOperationMsg previous_port_drayage_msg) {
        // If the previously received message was for 'Pickup' or 'Dropoff', update this object's _has_cargo flag and _cargo_id members accordingly
        // Note: This assumes the previous 'Pickup' or 'Dropoff' action was successful
        if (_latest_mobility_operation_msg.operation == PORT_DRAYAGE_PICKUP_OPERATION_ID) {
            _has_cargo = true;

            if (_latest_mobility_operation_msg.cargo_id) {
                _cargo_id = *_latest_mobility_operation_msg.cargo_id;
            }
            else {
                ROS_DEBUG_STREAM("CMV has completed pickup, but there is no Cargo ID associated with the picked up cargo.");
                throw std::invalid_argument("CMV has completed pickup, but there is no Cargo ID associated with the picked up cargo.");
            }

            ROS_DEBUG_STREAM("CMV completed pickup action. CMV is now carrying cargo " << _cargo_id);
        }
        else if (_latest_mobility_operation_msg.operation == PORT_DRAYAGE_DROPOFF_OPERATION_ID) {
            ROS_DEBUG_STREAM("CMV completed dropoff action. CMV is no longer carrying cargo " << _cargo_id);
            _has_cargo = false;
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
        if (_latest_mobility_operation_msg.operation == PORT_DRAYAGE_PICKUP_OPERATION_ID) {
            _latest_mobility_operation_msg.port_drayage_event_type = PortDrayageEvent::RECEIVED_NEW_DESTINATION;
            if (_has_cargo) {
                ROS_DEBUG_STREAM("Received 'PICKUP' operation, but CMV is already carrying cargo.");
                throw std::invalid_argument("Received 'PICKUP' operation, but CMV is already carrying cargo.");
            }
        }

        // Parse 'cargo_id' field if it exists in strategy_params
        if (pt.count("cargo_id") != 0){
            _latest_mobility_operation_msg.cargo_id = pt.get<std::string>("cargo_id");
            ROS_DEBUG_STREAM("cargo id: " << *_latest_mobility_operation_msg.cargo_id);
        }
        else{
            // If this message is for 'PICKUP', then the 'cargo_id' field is required
            if(_latest_mobility_operation_msg.operation == PORT_DRAYAGE_PICKUP_OPERATION_ID) {
                ROS_DEBUG_STREAM("Received 'PICKUP' operation, but no cargo_id was included.");
                throw std::invalid_argument("Received 'PICKUP' operation, but no cargo_id was included.");
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


    void PortDrayageWorker::on_guidance_state(const cav_msgs::GuidanceStateConstPtr& msg) {
        if ((msg->state == cav_msgs::GuidanceState::ENGAGED) && (_pdsm.get_state() == PortDrayageState::INACTIVE) && _enable_port_drayage) {
            ROS_DEBUG_STREAM("CMV has been engaged for the first time. Processing DRAYAGE_START event.");
            _pdsm.process_event(PortDrayageEvent::DRAYAGE_START);
        }
    }

    void PortDrayageWorker::on_route_event(const cav_msgs::RouteEventConstPtr& msg) {
        if (msg->event == cav_msgs::RouteEvent::ROUTE_COMPLETED) {
            // Throw exception if no vehicle speed has been received yet
            if (_cur_speed == nullptr) {
                ROS_DEBUG_STREAM("CMV has completed its route, but no vehicle speed has been received.");
                throw std::invalid_argument("CMV has completed its route, but no vehicle speed has been received.");
            }

            // If CMV has come to a stop at the end of its route. Process an 'ARRIVED_AT_DESTINATION' event.
            double longitudinal_speed = _cur_speed->twist.linear.x;
            if (fabs(longitudinal_speed) < _stop_speed_epsilon) {
                ROS_DEBUG_STREAM("CMV has come to a stop at the end of the route. Processing ARRIVED_AT_DESTINATION event.");
                _pdsm.process_event(PortDrayageEvent::ARRIVED_AT_DESTINATION);
            }
            // Throw an exception if CMV has not come to a stop at the end of its route.
            else {
                ROS_DEBUG_STREAM("CMV did not come to a stop at the end of the route.");
                throw std::invalid_argument("CMV did not come to a stop at the end of the route.");
            }
        }
    }

    const PortDrayageState PortDrayageWorker::get_port_drayage_state() {
        return _pdsm.get_state();
    }

} // namespace port_drayage_plugin
