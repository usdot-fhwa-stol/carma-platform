/*
 * Copyright (C) 2020-2022 LEIDOS.
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

#include "port_drayage_plugin/port_drayage_worker.hpp"

namespace port_drayage_plugin
{
    OperationID::Operation OperationID::getOperationID() const {
        return operation_enum_;
    }

    std::string OperationID::operationToString() const {

        // Convert operation enum into a human-readable string
        switch(operation_enum_) {
            case Operation::PICKUP:             return "PICKUP";
            case Operation::DROPOFF:            return "DROPOFF";
            case Operation::ENTER_STAGING_AREA: return "ENTER_STAGING_AREA";
            case Operation::EXIT_STAGING_AREA:  return "EXIT_STAGING_AREA";
            case Operation::ENTER_PORT:         return "ENTER_PORT";
            case Operation::EXIT_PORT:          return "EXIT_PORT";
            case Operation::PORT_CHECKPOINT:    return "PORT_CHECKPOINT";
            case Operation::HOLDING_AREA:       return "HOLDING_AREA";
            default:
                RCLCPP_WARN_STREAM(rclcpp::get_logger("OperationID"), "Conversion of an unsupported operation enum value to a string.");
                return "UNSUPPORTED_OPERATION_ID";
        }
    }

    PortDrayageWorker::PortDrayageWorker(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
                rclcpp::Clock::SharedPtr clock,
                std::function<void(carma_v2x_msgs::msg::MobilityOperation)> mobility_operations_publisher, 
                std::function<void(carma_msgs::msg::UIInstructions)> ui_instructions_publisher,
                std::function<bool(std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>)> call_set_active_route)
        : logger_(logger), clock_(clock),
          publish_mobility_operation_(mobility_operations_publisher),
          publish_ui_instructions_(ui_instructions_publisher),
          call_set_active_route_service_(call_set_active_route),
          pdsm_(logger)
    {
        pdsm_.setOnArrivedAtDestinationCallback(std::bind(&PortDrayageWorker::onArrivedAtDestination, this));
        pdsm_.setOnReceivedNewDestinationCallback(std::bind(&PortDrayageWorker::onReceivedNewDestination, this));
    }

    void PortDrayageWorker::setVehicleID(const std::string& cmv_id) {
        cmv_id_ = cmv_id;
    }
    
    void PortDrayageWorker::setCargoID(const std::string& cargo_id) {
        cargo_id_ = cargo_id;
    }

    void PortDrayageWorker::setEnablePortDrayageFlag(bool enable_port_drayage) {
        enable_port_drayage_ = enable_port_drayage;
    }

    void PortDrayageWorker::setStartingAtStagingAreaFlag(bool starting_at_staging_area) {
        starting_at_staging_area_ = starting_at_staging_area;
    }

    void PortDrayageWorker::onArrivedAtDestination() {
        carma_v2x_msgs::msg::MobilityOperation msg = composeArrivalMessage();
        publish_mobility_operation_(msg);
    }

    void PortDrayageWorker::onReceivedNewDestination() {       
        //  Populate the service request with the destination coordinates from the last received port drayage mobility operation message
        auto route_req = composeSetActiveRouteRequest(latest_mobility_operation_msg_.dest_latitude, latest_mobility_operation_msg_.dest_longitude);

        // Call service client to set the new active route
        bool is_route_generation_successful = call_set_active_route_service_(route_req);

        if (is_route_generation_successful) {
            // Publish UI Instructions to trigger a pop-up on the Web UI for the user to engage on the newly received route if desired
            carma_msgs::msg::UIInstructions ui_instructions_msg = composeUIInstructions(latest_mobility_operation_msg_.operation, previously_completed_operation_);
            publish_ui_instructions_(ui_instructions_msg);
        }
        else {
            // Throw exception if route generation was not successful
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Route generation failed. Routing could not be completed.");
            throw std::invalid_argument("Route generation failed. Routing could not be completed.");
        }
    }

    std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> PortDrayageWorker::composeSetActiveRouteRequest(boost::optional<double> dest_latitude, boost::optional<double> dest_longitude) const {
        auto route_req = std::make_shared<carma_planning_msgs::srv::SetActiveRoute::Request>();
        if (dest_latitude && dest_longitude) {
            route_req->choice = carma_planning_msgs::srv::SetActiveRoute::Request::DESTINATION_POINTS_ARRAY;
            route_req->route_id = latest_mobility_operation_msg_.operation;

            carma_v2x_msgs::msg::Position3D destination_point;
            destination_point.latitude = *latest_mobility_operation_msg_.dest_latitude;
            destination_point.longitude = *latest_mobility_operation_msg_.dest_longitude;
            destination_point.elevation_exists = false;
            
            route_req->destination_points.push_back(destination_point);
        }
        else {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "No destination points were received. Routing could not be completed.");
            throw std::invalid_argument("No destination points were received. Routing could not be completed");
        }

        return route_req;
    }

    carma_msgs::msg::UIInstructions PortDrayageWorker::composeUIInstructions(const std::string& current_operation, const std::string& previous_operation) {
        // Create the text that will be displayed in the Web UI popup
        std::string popup_text = "";

        // Add text that indicates the previous action was completed (if it was a pickup or dropoff action)
        if (previous_operation == OperationID::PICKUP) {
            popup_text += "The pickup action was completed successfully. ";
        }
        else if (previous_operation == OperationID::DROPOFF) {
            popup_text += "The dropoff action was completed successfully. ";
        }
        else if (previous_operation == OperationID::HOLDING_AREA) {
            popup_text += "The inspection was completed successfully. ";
        }

        // Add text to notify the user that the system can be engaged on the newly received route
        popup_text += "A new Port Drayage route with operation type '" + current_operation + "' has been received. "
                      "Select YES to engage the system on the route, or select NO to remain "
                      "disengaged.";

        // Create and populate the UI Instructions message
        carma_msgs::msg::UIInstructions ui_instructions_msg;
        ui_instructions_msg.stamp = clock_->now();
        ui_instructions_msg.msg = popup_text;
        ui_instructions_msg.type = carma_msgs::msg::UIInstructions::ACK_REQUIRED; // The popup will be displayed until the user interacts with it
        ui_instructions_msg.response_service = SET_GUIDANCE_ACTIVE_SERVICE_ID; 

        return ui_instructions_msg;
    }

    carma_v2x_msgs::msg::MobilityOperation PortDrayageWorker::composeArrivalMessage() const {
        carma_v2x_msgs::msg::MobilityOperation msg;

        msg.m_header.plan_id = "";
        msg.m_header.sender_id = cmv_id_;
        msg.m_header.recipient_id = "";
        msg.m_header.timestamp = clock_->now().nanoseconds()/1000000;

        msg.strategy = PORT_DRAYAGE_STRATEGY_ID;

        // Encode JSON with Boost Property Tree
        using boost::property_tree::ptree;
        ptree pt;
        pt.put("cmv_id", cmv_id_);

        // Add current vehicle location (latitude and longitude)
        ptree location;
        location.put("latitude", current_gps_position_.latitude); 
        location.put("longitude", current_gps_position_.longitude); 
        pt.put_child("location", location);

        // Add flag to indicate whether CMV is carring cargo
        pt.put("cargo", cargo_id_ != "");

        // If CMV has arrived at its initial destination, assign 'operation' field for its initial arrival message
        if (pdsm_.getState() == PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION) {
            // The CMV's initial destination is either the Staging Area Entrance or the Port Entrance
            if (starting_at_staging_area_) {
                OperationID pickup_operation = OperationID::ENTER_STAGING_AREA;
                pt.put("operation", pickup_operation);
            }
            else {
                OperationID enter_port_operation = OperationID::ENTER_PORT;
                pt.put("operation", enter_port_operation);
            }

            // Add cargo_id if CMV is carrying cargo
            if (cargo_id_ != "") {
                pt.put("cargo_id", cargo_id_);
            }
        }
        // If CMV has arrived at a received destination, add necessary fields based on the destination type that it has arrived at
        else if (pdsm_.getState() == PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION) {
            // Assign the 'operation' using the 'operation' from the last received port drayage message
            pt.put("operation", latest_mobility_operation_msg_.operation);

            // Assign the 'action_id' using the 'action_id' from the last received port drayage message
            if (latest_mobility_operation_msg_.current_action_id) {
                pt.put("action_id", *latest_mobility_operation_msg_.current_action_id);
            }
            else {
                RCLCPP_WARN_STREAM(logger_->get_logger(), "CMV has arrived at a received destination, but does not have an action_id to broadcast.");
            } 

            // Assign specific fields for arrival at a Pickup location
            if (latest_mobility_operation_msg_.operation == OperationID::PICKUP) {
                // Assign 'cargo_id' using the value received in the previous port drayage message
                if (latest_mobility_operation_msg_.cargo_id) {
                    pt.put("cargo_id", *latest_mobility_operation_msg_.cargo_id);
                }
                else {
                    RCLCPP_WARN_STREAM(logger_->get_logger(), "CMV has arrived at loading area, but does not have a cargo_id to broadcast.");
                }

                if (cargo_id_ != "") {
                    RCLCPP_WARN_STREAM(logger_->get_logger(), "CMV has arrived at a loading area, but it is already carrying cargo.");
                }
            }
            // Assign specific fields for arrival at a Dropoff location
            else if (latest_mobility_operation_msg_.operation == OperationID::DROPOFF) {
                // Assign 'cargo_id' using the ID of the cargo currently being carried
                if (cargo_id_ != "") {
                    pt.put("cargo_id", cargo_id_);
                }
                else {
                    RCLCPP_WARN_STREAM(logger_->get_logger(), "CMV has arrived at a dropoff area, but does not have a cargo_id to broadcast.");
                }
            }
            // Assign 'cargo_id' using the ID of the cargo currently being carried if the CMV is not arriving at a Pickup location
            else {
                if (cargo_id_ != "") {
                    pt.put("cargo_id", cargo_id_);
                }
            }
        }

        std::stringstream body_stream;
        boost::property_tree::json_parser::write_json(body_stream, pt);
        msg.strategy_params = body_stream.str();

        return msg;
    }

    void PortDrayageWorker::onInboundMobilityOperation(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg) {
        // Check if the received message is a new message for port drayage
        if((msg->strategy == PORT_DRAYAGE_STRATEGY_ID) && (msg->strategy_params != previous_strategy_params_)) {
            // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
            using boost::property_tree::ptree;
            ptree pt;
            std::istringstream strategy_params_ss(msg->strategy_params);
            boost::property_tree::json_parser::read_json(strategy_params_ss, pt);

            std::string mobility_operation_cmv_id = pt.get<std::string>("cmv_id");

            // Check if the received MobilityOperation message is intended for this vehicle's cmv_id   
            if(mobility_operation_cmv_id == cmv_id_) {
                // Since a new message indicates the previous action was completed, update all cargo-related data members based on the previous action that was completed
                updateCargoInformationAfterActionCompletion(latest_mobility_operation_msg_);

                // Store the previously received operation
                previously_completed_operation_ = latest_mobility_operation_msg_.operation;

                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Processing new port drayage MobilityOperation message for cmv_id " << mobility_operation_cmv_id);
                mobilityOperationMessageParser(msg->strategy_params);  
                previous_strategy_params_ = msg->strategy_params;
                
                pdsm_.processEvent(PortDrayageEvent::RECEIVED_NEW_DESTINATION);
            }
            else {
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Ignoring received port drayage MobilityOperation message intended for cmv_id " << mobility_operation_cmv_id);
            }
        }
    }

    void PortDrayageWorker::updateCargoInformationAfterActionCompletion(const PortDrayageMobilityOperationMsg& previous_port_drayage_msg) {
        // If the previously received message was for 'Pickup' or 'Dropoff', update this object's cargo_id_ member accordingly
        // Note: This assumes the previous 'Pickup' or 'Dropoff' action was successful
        if (previous_port_drayage_msg.operation == OperationID::PICKUP) {

            if (previous_port_drayage_msg.cargo_id) {
                cargo_id_ = *previous_port_drayage_msg.cargo_id;
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "CMV completed pickup action. CMV is now carrying cargo ID " << cargo_id_);
            }
            else {
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "CMV has completed pickup, but there is no Cargo ID associated with the picked up cargo.");
            }
        }
        else if (previous_port_drayage_msg.operation == OperationID::DROPOFF) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "CMV completed dropoff action. CMV is no longer carrying cargo ID " << cargo_id_);
            cargo_id_ = ""; // Empty string is used when no cargo is being carried
        }
    }

    void PortDrayageWorker::mobilityOperationMessageParser(std::string mobility_operation_strategy_params) {
        // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
        using boost::property_tree::ptree;
        ptree pt;
        std::istringstream mobility_operation_strategy_params_ss(mobility_operation_strategy_params);
        boost::property_tree::json_parser::read_json(mobility_operation_strategy_params_ss, pt);

        // Parse 'operation' field and assign the PortDrayageEvent type for this message accordingly
        latest_mobility_operation_msg_.operation = pt.get<std::string>("operation");
        RCLCPP_DEBUG_STREAM(logger_->get_logger(), "operation: " << latest_mobility_operation_msg_.operation);

        // If this CMV is commanded to pickup new cargo, check that it isn't already carrying cargo
        if (latest_mobility_operation_msg_.operation == OperationID::PICKUP && cargo_id_ != "") {
            RCLCPP_WARN_STREAM(logger_->get_logger(), "Received 'PICKUP' operation, but CMV is already carrying cargo.");
        }

        // If this CMV is commanded to dropoff cargo, check that it is actually carrying cargo
        if (latest_mobility_operation_msg_.operation == OperationID::DROPOFF && cargo_id_ == "") {
            RCLCPP_WARN_STREAM(logger_->get_logger(), "Received 'DROPOFF' operation, but CMV isn't currently carrying cargo.");
        }

        // Parse 'cargo_id' field if it exists in strategy_params
        if (pt.count("cargo_id") != 0){
            latest_mobility_operation_msg_.cargo_id = pt.get<std::string>("cargo_id");
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "cargo id: " << *latest_mobility_operation_msg_.cargo_id);

            // Log message if the cargo ID being dropped off does not match the cargo ID currently being carried
            if (latest_mobility_operation_msg_.operation == OperationID::DROPOFF && cargo_id_ != pt.get<std::string>("cargo_id")) {
                RCLCPP_WARN_STREAM(logger_->get_logger(), "CMV commanded to dropoff an invalid Cargo ID. Currently carrying " << cargo_id_ << ", commanded to dropoff " << pt.get<std::string>("cargo_id"));
            }
        }
        else{
            // If this message is for 'PICKUP', then the 'cargo_id' field is required
            if(latest_mobility_operation_msg_.operation == OperationID::PICKUP) {
                RCLCPP_WARN_STREAM(logger_->get_logger(), "Received 'PICKUP' operation, but no cargo_id was included.");
            }
            else if (latest_mobility_operation_msg_.operation == OperationID::DROPOFF) {
                RCLCPP_WARN_STREAM(logger_->get_logger(), "Received 'DROPOFF' operation, but no cargo_id was included.");
            }

            latest_mobility_operation_msg_.cargo_id = boost::optional<std::string>();
        }
        
        // Parse 'action_id' field  if it exists in strategy_params
        if (pt.count("action_id") != 0){
            latest_mobility_operation_msg_.current_action_id = pt.get<std::string>("action_id");
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "action id: " << *latest_mobility_operation_msg_.current_action_id);
        }
        else{
            latest_mobility_operation_msg_.current_action_id = boost::optional<std::string>();
        }

        // Parse starting longitude/latitude fields if 'location' field exists in strategy_params:
        if (pt.count("location") != 0){
            latest_mobility_operation_msg_.start_longitude = pt.get<double>("location.longitude");
            latest_mobility_operation_msg_.start_latitude = pt.get<double>("location.latitude"); 
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "start long: " << *latest_mobility_operation_msg_.start_longitude);
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "start lat: " << *latest_mobility_operation_msg_.start_latitude);
        }
        else {
            latest_mobility_operation_msg_.start_longitude = boost::optional<double>();
            latest_mobility_operation_msg_.start_latitude = boost::optional<double>();
        }

        // Parse destination longitude/latitude fields if 'destination' field exists in strategy_params:
        if (pt.count("destination") != 0) {
            latest_mobility_operation_msg_.dest_longitude = pt.get<double>("destination.longitude"); 
            latest_mobility_operation_msg_.dest_latitude = pt.get<double>("destination.latitude"); 
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "dest long: " << *latest_mobility_operation_msg_.dest_longitude);
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "dest lat: " << *latest_mobility_operation_msg_.dest_latitude);
        }
        else {
            latest_mobility_operation_msg_.dest_longitude = boost::optional<double>();
            latest_mobility_operation_msg_.dest_latitude = boost::optional<double>();
        }
    }      
    
    void PortDrayageWorker::onGuidanceState(carma_planning_msgs::msg::GuidanceState::UniquePtr msg) {
        // Drayage operations have started when the CMV has been engaged for the first time
        if ((msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED) && (pdsm_.getState() == PortDrayageState::INACTIVE) && enable_port_drayage_) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "CMV has been engaged for the first time. Processing DRAYAGE_START event.");
            pdsm_.processEvent(PortDrayageEvent::DRAYAGE_START);
        }
    }

    void PortDrayageWorker::onRouteEvent(carma_planning_msgs::msg::RouteEvent::UniquePtr msg) {
        // CMV has officially arrived at its destination if the previous route was completed and is no longer active
        if (latest_route_event_ != nullptr) {
            if (latest_route_event_->event == carma_planning_msgs::msg::RouteEvent::ROUTE_COMPLETED && msg->event == carma_planning_msgs::msg::RouteEvent::ROUTE_LOADED) {
                if (pdsm_.getState() == PortDrayageState::EN_ROUTE_TO_INITIAL_DESTINATION || pdsm_.getState() == PortDrayageState::EN_ROUTE_TO_RECEIVED_DESTINATION) {
                    RCLCPP_DEBUG_STREAM(logger_->get_logger(), "CMV completed its previous route, and the previous route is no longer active.");
                    RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Processing ARRIVED_AT_DESTINATION event.");
                    pdsm_.processEvent(PortDrayageEvent::ARRIVED_AT_DESTINATION);
                }           
            }
        }

        // Update the latest received route event data member
        latest_route_event_ = std::move(msg);
    }

    PortDrayageState PortDrayageWorker::getPortDrayageState() {
        return pdsm_.getState();
    }

    void PortDrayageWorker::onNewPose(geometry_msgs::msg::PoseStamped::UniquePtr msg) {
        if (!map_projector_) {
            RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Ignoring pose message as projection string has not been defined");
            return;
        }

        // Convert pose message contents to a GPS coordinate
        lanelet::GPSPoint coord = map_projector_->reverse( { msg->pose.position.x, msg->pose.position.y, msg->pose.position.z } );

        // Update the locally-stored GPS position of the CMV
        current_gps_position_.latitude = coord.lat;
        current_gps_position_.longitude = coord.lon;
    }        

    void PortDrayageWorker::onNewGeoreference(std_msgs::msg::String::UniquePtr msg) {
        // Build projector from proj string
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  
    }        


} // namespace port_drayage_plugin