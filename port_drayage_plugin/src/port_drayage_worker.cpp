/*
 * Copyright (C) 2018-2020 LEIDOS.
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
    }
    // @SONAR_START@

    void PortDrayageWorker::on_arrived_at_destination() {
        cav_msgs::MobilityOperation msg = compose_arrival_message();
        _publish_mobility_operation(msg);
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
        pt.put("cargo_id", _cargo_id);
        pt.put("operation", PORT_DRAYAGE_ARRIVAL_OPERATION_ID);
        std::stringstream body_stream;
        boost::property_tree::json_parser::write_json(body_stream, pt);
        msg.strategy_params = body_stream.str();

        return msg;
    }

    void PortDrayageWorker::on_inbound_mobility_operation(const cav_msgs::MobilityOperationConstPtr& mobility_operation_msg)
    {
        // Check if the received message is a new message for port drayage
        if((mobility_operation_msg->strategy == PORT_DRAYAGE_STRATEGY_ID) && (mobility_operation_msg->strategy_params != _previous_strategy_params))
        {
            // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
            using boost::property_tree::ptree;
            ptree pt;
            std::istringstream strategy_params_ss(mobility_operation_msg->strategy_params);
            boost::property_tree::json_parser::read_json(strategy_params_ss, pt);
            std::string received_cmv_id = pt.get<std::string>("cmv_id");

            // Check if the received MobilityOperation message is intended for this specific vehicle   
            if(received_cmv_id == _cmv_id)
            {
                ROS_DEBUG_STREAM("Processing new port drayage MobilityOperation message for cmv_id " << received_cmv_id);
                _pdsm.process_event(PortDrayageEvent::RECEIVED_NEW_DESTINATION);
                mobility_operation_message_parser(mobility_operation_msg->strategy_params);  
                _previous_strategy_params = mobility_operation_msg->strategy_params;
            }
            else
            {
                ROS_DEBUG_STREAM("Ignoring received port drayage MobilityOperation message intended for cmv_id " << received_cmv_id);
            }

        }
    }

    void PortDrayageWorker::mobility_operation_message_parser(std::string mobility_operation_strategy_params)
    {
        // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
        using boost::property_tree::ptree;
        ptree pt;
        std::istringstream mobility_operation_strategy_params_ss(mobility_operation_strategy_params);
        boost::property_tree::json_parser::read_json(mobility_operation_strategy_params_ss, pt);

        _received_cargo_id = pt.get<std::string>("cargo_id");
        _received_operation = pt.get<std::string>("operation");
        _received_cargo_flag = pt.get<bool>("cargo");
        _received_start_long = pt.get<double>("location.longitude");
        _received_start_lat = pt.get<double>("location.latitude");
        _received_dest_long = pt.get<double>("destination.longitude");
        _received_dest_lat = pt.get<double>("destination.latitude");
        _received_curr_action_id = pt.get<std::string>("action_id");
        _received_next_action_id = pt.get<std::string>("next_action");

        ROS_DEBUG_STREAM("cargo id: " << _received_cargo_id);
        ROS_DEBUG_STREAM("operation: " << _received_operation);
        ROS_DEBUG_STREAM("cargo flag: " << _received_cargo_flag);
        ROS_DEBUG_STREAM("start long: " << _received_start_long);
        ROS_DEBUG_STREAM("start lat: " << _received_start_lat);
        ROS_DEBUG_STREAM("dest long: " << _received_dest_long);
        ROS_DEBUG_STREAM("dest lat: " << _received_dest_lat);
        ROS_DEBUG_STREAM("current action id: " << _received_curr_action_id);
        ROS_DEBUG_STREAM("next action id: " << _received_next_action_id);
    }

    double PortDrayageWorker::get_received_destination_long()
    {
        return _received_dest_long;
    }

    double PortDrayageWorker::get_received_destination_lat()
    {
        return _received_dest_lat;
    }

    std::string PortDrayageWorker::get_received_cargo_id()
    {
        return _received_cargo_id;
    }

} // namespace port_drayage_plugin
