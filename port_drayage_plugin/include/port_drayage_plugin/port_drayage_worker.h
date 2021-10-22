#pragma once

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

#include <cav_msgs/ManeuverPlan.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/GuidanceState.h>
#include <cav_msgs/RouteEvent.h>
#include <cav_msgs/UIInstructions.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>
#include <cav_srvs/SetActiveRoute.h>
#include <boost/optional.hpp>
#include "port_drayage_plugin/port_drayage_state_machine.h"

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
                _operation_enum(op) {};

            /**
             * \brief Getter function to obtain the Operation enum associated with this object.
             * \return Operation enum associated with this object.
             */
            OperationID::Operation get_operation_ID() const;

            /**
             * \brief Function to convert this object's '_operation_enum' to a human-readable string.
             * \return A human-readable string representing this object's '_operation_enum'.
             */
            std::string operation_to_string() const;

            /**
             * \brief Stream operator for this object.
             */
            friend std::ostream& operator<<(std::ostream& output, const OperationID& oid){
                return output << oid.operation_to_string();
            }

            /**
             * \brief Overloaded == operator for comparision with String objects.
             */
            friend bool operator==(const std::string& lhs, const OperationID& rhs) {
                return lhs == rhs.operation_to_string();
            }

        private:
            // Data member containing this object's Operation enum value
            const Operation _operation_enum = Operation::DEFAULT_OPERATION; 
    };

    /**
     * Implementation class for all the business logic of the PortDrayagePlugin
     * 
     * Should not contain any reference to ROS publishing/subscribing or params
     * and all of that data (as needed) should be pushed into it via methods. * This implementation uses lambdas and std::function to give it the ability
     * to publish the outbound MobilityOperation message w/o the need for it
     * to know about the ROS underlying implementation.
     */
    class PortDrayageWorker
    {
        private:
            cav_msgs::ManeuverPlanConstPtr _cur_plan;
            geometry_msgs::TwistStampedConstPtr _cur_speed;
            cav_msgs::RouteEventConstPtr _latest_route_event;
            double _stop_speed_epsilon;
            PortDrayageStateMachine _pdsm;
            std::string _host_id;
            std::string _host_bsm_id;
            std::string _previously_completed_operation;
            OperationID _pickup_operation{OperationID::PICKUP}; 
            OperationID _dropoff_operation{OperationID::DROPOFF};
            OperationID _enter_staging_area_operation{OperationID::ENTER_STAGING_AREA};
            OperationID _enter_port_operation{OperationID::ENTER_PORT};
            OperationID _holding_area_operation{OperationID::HOLDING_AREA};
            unsigned long _cmv_id;
            std::string _cargo_id; // Empty if CMV is not currently carrying cargo
            std::function<void(cav_msgs::MobilityOperation)> _publish_mobility_operation;
            std::function<void(cav_msgs::UIInstructions)> _publish_ui_instructions;
            std::function<bool(cav_srvs::SetActiveRoute)> _set_active_route;
            std::shared_ptr<lanelet::projection::LocalFrameProjector> _map_projector = nullptr;
            bool _starting_at_staging_area; // Flag indicating CMV's first destination; 'true' indicates Staging Area Entrance; 'false' indicates Port Entrance.
            bool _enable_port_drayage; // Flag to enable to port drayage operations. If false, state machine will remain in 'INACTIVE' state

            // Data member for storing the strategy_params field of the last processed port drayage MobilityOperation message intended for this vehicle's cmv_id
            std::string _previous_strategy_params;

            // Constants
            const std::string PORT_DRAYAGE_PLUGIN_ID = "port_drayage_plugin";
            const std::string PORT_DRAYAGE_STRATEGY_ID = "carma/port_drayage";
            const std::string SET_GUIDANCE_ACTIVE_SERVICE_ID = "/guidance/set_guidance_active";

        public:

            /**
             * \brief Standard constructor for the PortDrayageWorker
             * 
             * \param cmv_id The Carrier Motor Vehicle ID (an unsigned long) for the host
             * vehicle
             * 
             * \param cargo_id The identification string for the cargo carried
             * by the host vehicle. If no cargo is being carried this should be
             * empty.
             * 
             * \param host_id The CARMA ID string for the host vehicle
             * 
             * \param starting_at_staging_area Flag indicating whether CMV's first 
             * destination is the Staging Area Entrance. If 'false, CMV's first destination
             * is the Port Entrance.
             * 
             * \param mobility_operations_publisher A lambda containing the logic
             * necessary to publish a MobilityOperations message. This lambda should
             * contain all the necessary ROS logic so that it does not leak into
             * the implementation of this class
             * 
             * \param stop_speed_epsilon An epsilon factor to be used when
             * comparing the current vehicle's speed to 0.0
             * 
             * \param enable_port_drayage A boolean flag indicating whether port drayage
             * operations are enabled or not. If false, the port drayage state machine
             * will remain in an INACTIVE state.
             * 
             * \param call_set_active_route_client A lambda containing the logic
             * necessary to call the SetActiveRoute service client. This lambda should
             * contain all the necessary ROS logic so that it does not leak into 
             * the implementation of this class.
             */
            PortDrayageWorker(
                unsigned long cmv_id,
                std::string cargo_id,
                std::string host_id,
                bool starting_at_staging_area,
                std::function<void(cav_msgs::MobilityOperation)> mobility_operations_publisher, 
                std::function<void(cav_msgs::UIInstructions)> ui_instructions_publisher,
                double stop_speed_epsilon,
                bool enable_port_drayage,
                std::function<bool(cav_srvs::SetActiveRoute)> call_set_active_route_client) :
                _cmv_id(cmv_id),
                _cargo_id(cargo_id),
                _host_id(host_id),
                _starting_at_staging_area(starting_at_staging_area),
                _publish_mobility_operation(mobility_operations_publisher),
                _publish_ui_instructions(ui_instructions_publisher),
                _set_active_route(call_set_active_route_client),
                _stop_speed_epsilon(stop_speed_epsilon),
                _enable_port_drayage(enable_port_drayage) {
                    initialize();       
                };

            /**
             * \brief Initialize the PortDrayageWorker, setting up it's relation
             * to the state machine.
             */
            void initialize();

            /**
             * \brief Callback for usage by the PortDrayageStateMachine when the vehicle has arrived at a destination
             */
            void on_arrived_at_destination();

            /**
             * \brief Callback for usage by the PortDrayageStateMachine when the vehicle has received a new destination
             */
            void on_received_new_destination();

            /**
             * \brief Create a SetActiveRoute service request to set a new active route for the system based on
             *        the destination points contained in the most recently-received Port Drayage MobilityOperation message
             *        intended for this vehicle.
             * \param dest_latitude The destination point's latitude
             * \param dest_longitude The destination point's longitude
             */
            cav_srvs::SetActiveRoute compose_set_active_route_request(boost::optional<double> dest_latitude, boost::optional<double> dest_longitude) const;

            /**
             * \brief Creates a UIInstructions message that can be used to create a pop-up on the Web UI to notify a user that a new
             *        route has been received for a specified destination type, and that the system can be engaged on that route.
             * \param current_operation The 'operation' identifier associated with the latest received and processed Port Drayage MobilityOperation message.
             * \param previous_operation The previously completed 'operation' identifier. This is an empty string if no 'operation' was previously completed.
             */
            cav_msgs::UIInstructions compose_ui_instructions(const std::string& current_operation, const std::string& previous_operation);

            /**
             * \brief Assemble the current dataset into a MobilityOperations
             * message with a JSON formatted body containing CMV ID and cargo ID
             */
            cav_msgs::MobilityOperation compose_arrival_message() const;

            /**
             * \brief Set the current plan from the arbitrator
             */
            void set_maneuver_plan(const cav_msgs::ManeuverPlanConstPtr& plan);

            /**
             * \brief Set the current speed as measured by the vehicle's sensors
             */
            void set_current_speed(const geometry_msgs::TwistStampedConstPtr& speed);

            /**
            * \brief Callback for map projection string to define lat/lon <--> map conversion
            * \param msg The proj string defining the projection.
             */
            void on_new_georeference(const std_msgs::StringConstPtr& msg);   

            /**
             * \brief Callback for the pose subscriber. The pose will be converted into lat/lon and stored locally.
             * \param msg Latest pose message
             */
            void on_new_pose(const geometry_msgs::PoseStampedConstPtr& msg);          

            /**
             * \brief Callback to process a received MobilityOperation message
             * \param msg a received MobilityOperation message
             */
            void on_inbound_mobility_operation(const cav_msgs::MobilityOperationConstPtr& msg);

            /**
             * \brief Method to update worker's cargo-related data members depending on whether
             *  the previously completed action was for a pickup or dropoff.
             * \param previous_port_drayage_msg The contents of the previously received MobilityOperation
             *  port drayage message for this CMV stored in a PortDrayageMobilityOperationMsg object.
             */
            void update_cargo_information_after_action_completion(const PortDrayageMobilityOperationMsg& previous_port_drayage_msg);

            /**
             * \brief Function to help parse the text included in an inbound MobilityOperation message's 
             *  strategy_params field according to the JSON schema intended for MobilityOperation messages
             *  with strategy type 'carma/port_drayage'. Stores the parsed information in _latest_mobility_operation_msg.
             * \param mobility_operation_strategy_params the strategy_params field of a MobilityOperation message
             */
            void mobility_operation_message_parser(std::string mobility_operation_strategy_params);

            /**
             * \brief Callback to process the current status of the guidance state machine. 
             * \param msg a received GuidanceState message
             */
            void on_guidance_state(const cav_msgs::GuidanceStateConstPtr& msg); 

            /**
             * \brief Callback to process each Route Event
             * \param msg a received RouteEvent message 
             */
            void on_route_event(const cav_msgs::RouteEventConstPtr& msg);

            /**
             * \brief Get the current state of the port drayage state machine
             * \return The current state value of the port drayage state machine
             */
            PortDrayageState get_port_drayage_state();

            /**
             * \brief Spin and process data
             */
            bool spin();
            
            /**
             * \brief Check to see if the vehicle has stopped under the command of the 
             * Port Drayage Plugin.
             * 
             * \param plan The current maneuver plan
             * \param speed The current vehicle speed
             * 
             * \return True if the vehicle has been stopped by the PDP, false o.w.
             */
            bool check_for_stop(const cav_msgs::ManeuverPlanConstPtr& plan, const geometry_msgs::TwistStampedConstPtr& speed) const;

            // PortDrayageMobilityOperationMsg object for storing strategy_params data of a received port drayage MobilityOperation message intended for this vehicle's cmv_id
            PortDrayageMobilityOperationMsg _latest_mobility_operation_msg;

            // LatLonCoordinate object for storing the vehicle's current gps latitude/longitude coordinates
            LatLonCoordinate _current_gps_position;

    };
} // namespace port_drayage_plugin
