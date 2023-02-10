#pragma once

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

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_planning_msgs/msg/route_event.hpp>
#include <carma_planning_msgs/srv/set_active_route.hpp>
#include <carma_msgs/msg/ui_instructions.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/string.hpp"
#include "port_drayage_plugin/port_drayage_state_machine.hpp"

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace port_drayage_plugin
{    
    /**
     * \brief Convenience struct for storing all data contained in a received MobilityOperation message's
     * strategy_params field with strategy "carma/port_drayage"
     */
    struct PortDrayageMobilityOperationMsg
    {
        boost::optional<std::string> cargo_id;
        std::string operation = "";
        boost::optional<std::string> current_action_id; // Identifier for the action this message is related to
        boost::optional<double> dest_longitude;  // Destination longitude for the carma vehicle
        boost::optional<double> dest_latitude;   // Destination latitude for the carma vehicle
        boost::optional<double> start_longitude; // Starting longitude of the carma vehicle
        boost::optional<double> start_latitude;  // Starting latitude of the carma vehicle
    };

    /**
     * \brief Convenience struct for storing the vehicle's current latitude/longitude coordinates
     */
    struct LatLonCoordinate
    {
        double latitude = 0.0;
        double longitude = 0.0;
    };

    /**
     * \brief Helper class containing an enum for valid port drayage MobilityOperation message operation IDs and
     * a function that converts each operation enum value to a human-readable string.
     */
    class OperationID
    {
        public:
            /**
             * \brief Enum containing possible operation IDs used to define destinations for port drayage.
             */
            enum Operation {
                PICKUP,
                DROPOFF,
                ENTER_STAGING_AREA,
                EXIT_STAGING_AREA,
                ENTER_PORT,
                EXIT_PORT,
                PORT_CHECKPOINT,
                HOLDING_AREA,
                DEFAULT_OPERATION
            };

            /**
             * \brief Standard constructor for OperationID
             * \param op Operation enum associated with this object.
             */
            OperationID(enum Operation op) :
                operation_enum_(op) {};

            /**
             * \brief Getter function to obtain the Operation enum associated with this object.
             * \return Operation enum associated with this object.
             */
            OperationID::Operation getOperationID() const;

            /**
             * \brief Function to convert this object's 'operation_enum_' to a human-readable string.
             * \return A human-readable string representing this object's 'operation_enum_'.
             */
            std::string operationToString() const;

            /**
             * \brief Stream operator for this object.
             */
            friend std::ostream& operator<<(std::ostream& output, const OperationID& oid){
                return output << oid.operationToString();
            }

            /**
             * \brief Overloaded == operator for comparision with String objects.
             */
            friend bool operator==(const std::string& lhs, const OperationID& rhs) {
                return lhs == rhs.operationToString();
            }

        private:
            // Data member containing this object's Operation enum value
            const Operation operation_enum_ = Operation::DEFAULT_OPERATION; 
    };

    /**
     * Implementation class for all the business logic of the PortDrayagePlugin
     */
    class PortDrayageWorker
    {
        private:
            std::shared_ptr<carma_planning_msgs::msg::RouteEvent> latest_route_event_ = nullptr;
            PortDrayageStateMachine pdsm_;
            std::string previously_completed_operation_;
            std::string cmv_id_;
            std::string cargo_id_; // Empty if CMV is not currently carrying cargo
            std::function<void(carma_v2x_msgs::msg::MobilityOperation)> publish_mobility_operation_;
            std::function<void(carma_msgs::msg::UIInstructions)> publish_ui_instructions_;
            std::function<bool(std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>)> call_set_active_route_service_;
            std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_ = nullptr;
            bool starting_at_staging_area_; // Flag indicating CMV's first destination; 'true' indicates Staging Area Entrance; 'false' indicates Port Entrance.
            bool enable_port_drayage_; // Flag to enable to port drayage operations. If false, state machine will remain in 'INACTIVE' state

            // Data member for storing the strategy_params field of the last processed port drayage MobilityOperation message intended for this vehicle's cmv_id
            std::string previous_strategy_params_;

            // Constants
            const std::string PORT_DRAYAGE_PLUGIN_ID = "port_drayage_plugin";
            const std::string PORT_DRAYAGE_STRATEGY_ID = "carma/port_drayage";
            const std::string SET_GUIDANCE_ACTIVE_SERVICE_ID = "/guidance/set_guidance_active";
            
            // Clock for this object
            rclcpp::Clock::SharedPtr clock_;

            // Logger interface for this object
            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

        public:

            /**
             * \brief Standard constructor for the PortDrayageWorker
             * 
             * \param mobility_operations_publisher A function containing the logic
             * necessary to publish a Mobility Operations  message. 
             * 
             * \param ui_instructions_publisher A function containing the logic
             * necessary to publish a UI Instructions message. 
             * 
             * \param call_set_active_route A function containing the logic
             * necessary to call the SetActiveRoute service client.
             */
            PortDrayageWorker(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
                rclcpp::Clock::SharedPtr clock,
                std::function<void(carma_v2x_msgs::msg::MobilityOperation)> mobility_operations_publisher, 
                std::function<void(carma_msgs::msg::UIInstructions)> ui_instructions_publisher,
                std::function<bool(std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request>)> call_set_active_route);

            /**
             * \brief Setter function to set this object's cmv_id_ string
             * \param cmv_id The provided CMV ID for this object 
             */
            void setVehicleID(const std::string& cmv_id);

            /**
             * \brief Setter function to set this object's cargo_id_ string
             * \param cargo_id The provided Cargo ID for this object 
             */
            void setCargoID(const std::string& cargo_id);

            /**
             * \brief Setter function to set this object's enable_port_drayage flag
             * \param enable_port_drayage The provided boolean flag 
             */
            void setEnablePortDrayageFlag(bool enable_port_drayage);

            /**
             * \brief Setter function to set this object's starting_at_staging_area flag
             * \param starting_at_staging_area The provided boolean flag 
             */
            void setStartingAtStagingAreaFlag(bool starting_at_staging_area);

            /**
             * \brief Callback for usage by the PortDrayageStateMachine when the vehicle has arrived at a destination
             */
            void onArrivedAtDestination();

            /**
             * \brief Callback for usage by the PortDrayageStateMachine when the vehicle has received a new destination
             */
            void onReceivedNewDestination();

            /**
             * \brief Create a SetActiveRoute service request to set a new active route for the system based on
             *        the destination points contained in the most recently-received Port Drayage MobilityOperation message
             *        intended for this vehicle.
             * \param dest_latitude The destination point's latitude
             * \param dest_longitude The destination point's longitude
             */
            std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> composeSetActiveRouteRequest(boost::optional<double> dest_latitude, boost::optional<double> dest_longitude) const;

            /**
             * \brief Creates a UIInstructions message that can be used to create a pop-up on the Web UI to notify a user that a new
             *        route has been received for a specified destination type, and that the system can be engaged on that route.
             * \param current_operation The 'operation' identifier associated with the latest received and processed Port Drayage MobilityOperation message.
             * \param previous_operation The previously completed 'operation' identifier. This is an empty string if no 'operation' was previously completed.
             */
            carma_msgs::msg::UIInstructions composeUIInstructions(const std::string& current_operation, const std::string& previous_operation);

            /**
             * \brief Assemble the current dataset into a MobilityOperations
             * message with a JSON formatted body containing CMV ID and cargo ID
             */
            carma_v2x_msgs::msg::MobilityOperation composeArrivalMessage() const;

            /**
            * \brief Callback for map projection string to define lat/lon <--> map conversion
            * \param msg The proj string defining the projection.
             */
            void onNewGeoreference(std_msgs::msg::String::UniquePtr msg);   

            /**
             * \brief Callback for the pose subscriber. The pose will be converted into lat/lon and stored locally.
             * \param msg Latest pose message
             */
            void onNewPose(geometry_msgs::msg::PoseStamped::UniquePtr msg);          

            /**
             * \brief Callback to process a received MobilityOperation message
             * \param msg a received MobilityOperation message
             */
            void onInboundMobilityOperation(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg);

            /**
             * \brief Method to update worker's cargo-related data members depending on whether
             *  the previously completed action was for a pickup or dropoff.
             * \param previous_port_drayage_msg The contents of the previously received MobilityOperation
             *  port drayage message for this CMV stored in a PortDrayageMobilityOperationMsg object.
             */
            void updateCargoInformationAfterActionCompletion(const PortDrayageMobilityOperationMsg& previous_port_drayage_msg);

            /**
             * \brief Function to help parse the text included in an inbound MobilityOperation message's 
             *  strategy_params field according to the JSON schema intended for MobilityOperation messages
             *  with strategy type 'carma/port_drayage'. Stores the parsed information in _latest_mobility_operation_msg.
             * \param mobility_operation_strategy_params the strategy_params field of a MobilityOperation message
             */
            void mobilityOperationMessageParser(std::string mobility_operation_strategy_params);

            /**
             * \brief Callback to process the current status of the guidance state machine. 
             * \param msg a received GuidanceState message
             */
            void onGuidanceState(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg); 

            /**
             * \brief Callback to process each Route Event
             * \param msg a received RouteEvent message 
             */
            void onRouteEvent(const carma_planning_msgs::msg::RouteEvent::UniquePtr msg);

            /**
             * \brief Get the current state of the port drayage state machine
             * \return The current state value of the port drayage state machine
             */
            PortDrayageState getPortDrayageState();

            // PortDrayageMobilityOperationMsg object for storing strategy_params data of a received port drayage MobilityOperation message intended for this vehicle's cmv_id
            PortDrayageMobilityOperationMsg latest_mobility_operation_msg_;

            // LatLonCoordinate object for storing the vehicle's current gps latitude/longitude coordinates
            LatLonCoordinate current_gps_position_;
    };
} // namespace port_drayage_plugin