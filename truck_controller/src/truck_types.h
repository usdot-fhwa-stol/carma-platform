#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cav_driver_utils/can/socketcan_interface/socketcan_interface.h>
#include <cav_driver_utils/can/ros_socketcan_bridge/ros_socketcan_bridge.h>

#include <iostream>
#include <array>

#include <boost/crc.hpp>

namespace truck
{
enum class SocketCanSelect
{
    CavSocketCan = 0,
    RosSocketCan = 1
};

struct TruckSettings
{
    SocketCanSelect socketcan_selection;
    std::string socketcan_device;
    std::string ros_socketcan_topic_recv;
    std::string ros_socketcan_topic_send;

    TruckSettings() :
        socketcan_selection(SocketCanSelect::CavSocketCan),
        socketcan_device("dbw_ifc"),
        ros_socketcan_topic_recv("ros_can_in"),
        ros_socketcan_topic_send("ros_can_out")
    {

    }
};

struct TruckDbwSendRates
{
    float send_control_mode_message;
    float send_engine_brake_message;
    float send_pid_gains_message;
    float send_led_status_message;
    float send_front_light_status_message;
    float send_rear_light_status_message;

    TruckDbwSendRates() :
        send_control_mode_message(0.020),            // 20 ms nominal
        send_engine_brake_message(0.100),            // 100 ms nominal
        send_pid_gains_message(1.000),               // 1000 ms nominal
        send_led_status_message(1.000),              // 1000 ms nominal
        send_front_light_status_message(0.500),      // Not Specified
        send_rear_light_status_message(0.500)        // Not Specified
    {

    }
};

struct TruckRosRecvRates
{
    float recv_roboticmode_pedalposition;
    float recv_control_message_echo;
    float recv_prim_brake_axiomatic;
    float recv_sec_brake_axiomatic;
    float recv_engine_brake_control;
    float recv_speed_control_pid_params;
    float recv_speed_control_pid_echo;
    float recv_general_ecu_status;
    float recv_raw_input_channel;
    float rev_raw_output_channel;
    float recv_manual_check_conditions;
    float recv_brake_fault_checks;
    float recv_emergency_fault_checks;
    float recv_led_status_echo;
    float recv_settings_crc;

    TruckRosRecvRates() :
        recv_roboticmode_pedalposition(0.040),
        recv_control_message_echo(0.040),
        recv_prim_brake_axiomatic(0.200),
        recv_sec_brake_axiomatic(0.200),
        recv_engine_brake_control(0.200),
        recv_speed_control_pid_params(2.000),
        recv_speed_control_pid_echo(2.000),
        recv_general_ecu_status(2.000),
        recv_raw_input_channel(0.200),
        rev_raw_output_channel(0.200),
        recv_manual_check_conditions(2.000),
        recv_brake_fault_checks(2.000),
        recv_emergency_fault_checks(2.000),
        recv_led_status_echo(2.000),
        recv_settings_crc (500.00)
    {

    }
};

/** ---------- CAN Message Specific Enums ----------**/

enum class ControlTypeCommandMode
{
    Disable_ACC_System = 0,
    Robotic_WrenchEffort_Ctrl = 1,
    Robotic_Speed_Ctrl = 2,
    Reserved = 3
};

enum class RoboticModeState
{
    Manual_Mode = 0,
    Robotic_Mode = 1,
    Degraded_Robotic_Mode = 2,
    Robotic_Throttle_Override = 3
};

enum class AxiomaticHealthStatus
{
    Module_OK = 0,
    Module_Fault = 1,
    Module_Comms_Lost = 2,
    No_Change = 3
};

enum class EngineBrakeMsgModeCmd
{
    Pass_Through = 0,
    Torc_Bypass = 1,
    Error = 2,
    No_Change = 3
};

enum class EngineBrakeCmd
{
    Disable_Engine_Braking = 0,
    Enable_Engine_Braking = 1,
    Error = 2,
    No_Change = 3
};

enum class EngineBrakeLevelCmd
{
    Low_Level = 0,
    Medium_Level = 1,
    High_Level = 2,
    No_Change = 3
};

enum class ClearFaultsMode
{
    Clear_Faults_Disabled = 0,
    Clear_Faults_Enabled = 1,
    Error_Clear_Disabled = 2,
    Clear_Faults_Disabled_2 = 3
};

enum class GenericStates
{
    Off = 0,
    On = 1,
    Error = 2,
    Unknown = 3,
	Ignore = 0xF
};

/** ---------- CAN Messages ---------- **/

inline uint8_t getNextCounter(uint8_t counter)
{
    uint8_t next_count = 0;
    if(counter >= 15)
    {
        next_count = 0;
    }
    else
    {
        next_count = counter + 1;
    }
    return next_count;
}

inline bool checkMessageCrc(std::array<uint8_t, 8> data, uint8_t size)
{
    boost::crc_basic<8> crc8(0xD5, 0xFF, 0, true, true); 
    unsigned char message[size - 1] = {};
    for(int i = 0; i < (size - 1); i++)
    {
        message[i] = static_cast<unsigned char>(data[i]);
    }

    crc8.process_bytes(message, size - 1);  // Pass in Message Data (except last byte)

    // Compare calculated result to Message result
    if(static_cast<uint8_t>(crc8.checksum()) == static_cast<uint8_t>(data[size - 1]))
    {
        return true;    // True: if CRCs match
    }
    else
    {
        return false;   // False: if CRCs mismatch
    }
}

inline bool checkMessageCounter(std::array<uint8_t, 8> data, uint8_t size, uint8_t curr_count)
{
    uint8_t msg_count = static_cast<uint8_t>((data[size - 2] >> 4) & 0x0F);  // Pull Msg count out of Msg


    if((curr_count + 8) > 15)   // Range rolls over to 0
    {
        if((msg_count > curr_count) & (msg_count <= 15))    // if the new count is greater than the old count it also has to be below 15
        {
            return true;
        }
        else if((msg_count < curr_count) & (msg_count < (curr_count - 7)))  // if the new count is less than the old count it needs to be within range
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else // Range does not roll over
    {
        if((msg_count > curr_count) & (msg_count < (curr_count + 8)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

/** PropB_10 : PGN 65296 : Truck Command (From: Control SW, To: Throttle Controller) **/
struct PropB_10_Message
{
    float wrench_effort;
    float speed_control;
    float max_accel;
    uint8_t reserved_propb_10;
    ControlTypeCommandMode control_type_command_mode;
    uint8_t rolling_counter;
    uint8_t check_value;

    uint8_t propb_10_cur_count_;
    uint8_t propb_10_size = 8U;

    PropB_10_Message() :
        wrench_effort(0.0),
        speed_control(0.0),
        max_accel(0.0),
        reserved_propb_10(0xFF),
        control_type_command_mode(ControlTypeCommandMode::Disable_ACC_System),
        rolling_counter(0x0),
        check_value(0xFF),
        propb_10_cur_count_(0)
    {

    }

    cav::CANFrameStamped packData()
    {
        // Create an 8-bit Boost CRC Calculator 
        boost::crc_basic<8> crc8(0xD5, 0xFF, 0x00, true, true);

        // Create an empty array of message size (bytes)
        unsigned char data[propb_10_size] = {};
        // Byte 1.1 - 2.8: Wrench Effort
        int16_t wrench_eff = static_cast<int16_t>(wrench_effort / 0.025) + 4000.0;   // 0.025% per bit | -4000 offset
        data[0] = static_cast<uint8_t>(wrench_eff) & 0xFF;
        data[1] = static_cast<uint8_t>(wrench_eff >> 8) & 0xFF;
        // Byte 3.1 - 4.8: Speed Control
        int16_t spd_ctrl = static_cast<int16_t>(speed_control / 0.03125);   // 0.03125 km/h per bit
        data[2] = static_cast<uint8_t>(spd_ctrl) & 0xFF;
        data[3] = static_cast<uint8_t>(spd_ctrl >> 8) & 0xFF;
        // Byte 5.1 - 6.8: Max Accel
        int16_t max_acc  = static_cast<int16_t>(max_accel / 0.03125);   // 0.03125 (km/h)/s per bit
        data[4] = static_cast<uint8_t>(max_acc) & 0xFF;
        data[5] = static_cast<uint8_t>(max_acc >> 8) & 0xFF;
        // Byte 7.1 - 7.2: Robotic Override Enable
        data[6] |= static_cast<uint8_t>(reserved_propb_10) & 0x03;
        // Byte 7.3 - 7.4: Control Type Command Mode
        data[6] |= (static_cast<uint8_t>(control_type_command_mode) << 2) & 0x0C;
        // Byte 7.5 - 7.8: Counter
        uint8_t next_counter = getNextCounter(propb_10_cur_count_);
        propb_10_cur_count_ = next_counter;
        data[6] |= (static_cast<uint8_t>(propb_10_cur_count_) << 4) & 0xF0;
        // Byte 8.1 - 8.8: CheckValue
        crc8.process_bytes(data, 7);
        data[7] = static_cast<uint8_t>(crc8.checksum());
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = 0x0CFF1027;
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = propb_10_size;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    float getWrenchEffort()                     { return wrench_effort; }
    float getSpeedControl()                     { return speed_control; }
    float getMaxAccel()                         { return max_accel; }
    uint8_t getReservedPropB10()                { return reserved_propb_10; }
    ControlTypeCommandMode getCtrlCmdMode()     { return control_type_command_mode; }
    uint8_t getRollingCounter()                 { return rolling_counter; }
    uint8_t getCheckValue()                     { return check_value; }

    void setWrenchEffort(float data)                    { wrench_effort = data; }
    void setSpeedControl(float data)                    { speed_control = data; }
    void setMaxAccel(float data)                        { max_accel = data; }
    void setReservedPropB10(uint8_t data)               { reserved_propb_10 = data; }
    void setCtrlCmdMode(ControlTypeCommandMode data)    { control_type_command_mode = data; }
    void setRollingCounter(uint8_t data)                { rolling_counter = data; }
    void setCheckValue(uint8_t data)                    { check_value = data; }
};  // End 'PropB_10_Message' message struct

/** PropB_11 : PGN 65297 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_11_Message
{
    float generated_accel_pedal_percentage = 0.0;
    float expected_accel_pedal_percentage = 0.0;
    float physical_accel_pedal_percentage = 0.0;
    RoboticModeState robotic_mode_state_status = RoboticModeState::Manual_Mode;
    uint8_t reserved_propb_11 = 0xFF;
    uint8_t rolling_counter = 0x0;
    uint8_t check_value = 0xFF;

    uint8_t propb_11_size = 8U;
    bool is_valid;

    void setData(std::array<uint8_t, 8> data)
    {
        is_valid = (checkMessageCrc(data, propb_11_size) && checkMessageCounter(data, propb_11_size, rolling_counter));
        if(is_valid)
        {
            generated_accel_pedal_percentage    = static_cast<float>(data[0] + (static_cast<uint16_t>(data[1]) << 8)) * 0.025; // 2 bytes (1.1 - 2.8) | 0.025 (%)/bit | 0
            expected_accel_pedal_percentage     = static_cast<float>(data[2] + (static_cast<uint16_t>(data[3]) << 8)) * 0.025; // 2 bytes (3.1 - 4.8) | 0.025 (%)/bit | 0
            physical_accel_pedal_percentage     = static_cast<float>(data[4] + (static_cast<uint16_t>(data[5]) << 8)) * 0.025; // 2 bytes (5.1 - 6.8) | 0.025 (%)/bit | 0
            robotic_mode_state_status           = static_cast<RoboticModeState>(data[6] & 0x03); // 2 bits (7.1 - 7.2) | 4 Possible States
            reserved_propb_11                   = static_cast<uint8_t>((data[6] >> 2) & 0x03); // 2 bits (7.3 - 7.4) | RESERVED
        }
        rolling_counter                     = static_cast<uint8_t>((data[6] >> 4) & 0x0F); // 4 bits (7.5 - 7.8) | 16 Possible States
        check_value                         = static_cast<uint8_t>(data[7]); // 1 byte (8.1 - 8.8) | Checksum Value
    }

    void printData()
    {
        std::cout << "\tis_valid: " << is_valid << std::endl;
        std::cout << "\tgenerated_accel_pedal_percentage: " << generated_accel_pedal_percentage                 << std::endl;
        std::cout << "\texpected_accel_pedal_percentage: "  << expected_accel_pedal_percentage                  << std::endl;
        std::cout << "\tphysical_accel_pedal_percentage: "  << physical_accel_pedal_percentage                  << std::endl;
        std::cout << "\trobotic_mode_state_status: "        << static_cast<uint16_t>(robotic_mode_state_status) << std::endl;
        std::cout << "\treserved_propb_11: "                << static_cast<uint16_t>(reserved_propb_11)         << std::endl;
        std::cout << "\trolling_counter: "                  << static_cast<uint16_t>(rolling_counter)           << std::endl;
        std::cout << "\tcheck_value: "                      << static_cast<uint16_t>(check_value)               << std::endl;
    }

    float getGenAccelPedalPercent()                 { return generated_accel_pedal_percentage; }
    float getExpAccelPedalPercent()                 { return expected_accel_pedal_percentage; }
    float getPhyAccelPedalPercent()                 { return physical_accel_pedal_percentage; }
    RoboticModeState getRoboticModeStateStatus()    { return robotic_mode_state_status; }
    uint8_t getReservedPropB11()                    { return reserved_propb_11; }
    uint8_t getRollingCounter()                     { return rolling_counter; }
    uint8_t getCheckValue()                         { return check_value; }

    void setGenAccelPedalPercent(float data)                { generated_accel_pedal_percentage = data; }
    void setExpAccelPedalPercent(float data)                { expected_accel_pedal_percentage = data; }
    void setPhyAccelPedalPercent(float data)                { physical_accel_pedal_percentage = data; }
    void setRoboticModeStateStatus(RoboticModeState data)   { robotic_mode_state_status = data; }
    void setReservedPropB11(uint8_t data)                   { reserved_propb_11 = data; }
    void setRollingCounter(uint8_t data)                    { rolling_counter = data; }
    void setCheckValue(uint8_t data)                        { check_value = data; }
};  // End 'PropB_11_Message' message struct

/** PropB_12 : PGN 65298 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_12_Message
{
    float wrench_effort_echo = 0.0;
    float speed_control_echo = 0.0;
    float max_accel_echo = 0.0;
    uint8_t reserved_propb_12 = 0xFF;
    ControlTypeCommandMode ctrl_type_cmd_mode = ControlTypeCommandMode::Disable_ACC_System;
    uint8_t rolling_counter = 0x0;
    uint8_t check_value = 0xFF;

    uint8_t propb_12_size = 8U;
    bool is_valid;

    void setData(std::array<uint8_t, 8> data)
    {
        is_valid = (checkMessageCrc(data, propb_12_size) && checkMessageCounter(data, propb_12_size, rolling_counter));
        if(is_valid)
        {
            wrench_effort_echo  = (static_cast<float>(data[0] + (static_cast<uint16_t>(data[1]) << 8)) * 0.025) - 4000.0; // 2 bytes (1.1 - 2.8) | 0.025 (%)/bit | -4000
            speed_control_echo  = static_cast<float>(data[2] + (static_cast<uint16_t>(data[3]) << 8)) * 0.03125; // 2 bytes (3.1 - 4.8) | 0.03125 (km/h)/bit | 0
            max_accel_echo      = static_cast<float>(data[4] + (static_cast<uint16_t>(data[5]) << 8)) * 0.03125; // 2 bytes (5.1 - 6.8) | 0.03125 ((km/h)/s)/bit | 0
            reserved_propb_12   = static_cast<uint8_t>(data[6] & 0x03); // 2 bits (7.1 - 7.2) | RESERVED
            ctrl_type_cmd_mode  = static_cast<ControlTypeCommandMode>((data[6] >> 2) & 0x03); // 2 bits (7.3 - 7.4) | 4 Possible States
        }
        rolling_counter = static_cast<uint8_t>((data[6] >> 4) & 0x0F); // 4 bits (7.5 - 7.8) | 16 Possible States
        check_value     = static_cast<uint8_t>(data[7]); // 1 byte (8.1 - 8.8) | Checksum Value
    }

    void printData()
    {
        std::cout << "\tis_valid: " << is_valid << std::endl;
        std::cout << "\twrench_effort_echo: "   << wrench_effort_echo                           << std::endl;
        std::cout << "\tspeed_control_echo: "   << speed_control_echo                           << std::endl;
        std::cout << "\tmax_accel_echo: "       << max_accel_echo                               << std::endl;
        std::cout << "\treserved_propb_12: "    << static_cast<uint16_t>(reserved_propb_12)     << std::endl;
        std::cout << "\tctrl_type_cmd_mode: "   << static_cast<uint16_t>(ctrl_type_cmd_mode)    << std::endl;
        std::cout << "\trolling_counter: "      << static_cast<uint16_t>(rolling_counter)       << std::endl;
        std::cout << "\tcheck_value: "          << static_cast<uint16_t>(check_value)           << std::endl;
    }

    float getWrenchEffortEcho()                         { return wrench_effort_echo; }
    float getSpeedControlEcho()                         { return speed_control_echo; }
    float getMaxAccelEcho()                             { return max_accel_echo; }
    uint8_t getReservedPropB12()                        { return reserved_propb_12; }
    ControlTypeCommandMode getControlTypeCmdModeEcho()  { return ctrl_type_cmd_mode; }
    uint8_t getRollingCounter()                         { return rolling_counter; }
    uint8_t getCheckValue()                             { return check_value; }

    void setWrenchEffortEcho(float data)                        { wrench_effort_echo = data; }
    void setSpeedControlEcho(float data)                        { speed_control_echo = data; }
    void setMaxAccelEcho(float data)                            { max_accel_echo = data; }
    void setReservedPropB12(uint8_t data)                       { reserved_propb_12 = data; }
    void setControlTypeCmdModeEcho(ControlTypeCommandMode data) { ctrl_type_cmd_mode = data; }
    void setRollingCounter(uint8_t data)                        { rolling_counter = data; }
    void setCheckValue(uint8_t data)                            { check_value = data; }
};  // End 'PropB_12_Message' message struct

/** PropB_20 : PGN 65312 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_20_Message
{
    float system_air_pressure = 0.0;
    float robotic_truck_applied_brake_pressure = 0.0;
    float robotic_trailer_applied_brake_pressure = 0.0;
    float cmd_brake_application_level = 0.0;
    uint8_t reserved_propb_20 = 0xFF;
    AxiomaticHealthStatus axiomatic_health_status = AxiomaticHealthStatus::Module_OK;
    uint8_t rolling_counter = 0x0;
    uint8_t check_value = 0xFF;

    uint8_t propb_20_size = 8U;
    bool is_valid;

    void setData(std::array<uint8_t, 8> data)
    {
        is_valid = (checkMessageCrc(data, propb_20_size) && checkMessageCounter(data, propb_20_size, rolling_counter));
        if(is_valid)
        {
            system_air_pressure             = static_cast<float>(data[0] + ((data[1] & 0x0F) << 8)) * 0.05; // 12 bits (1.1 - 2.4) | 0.05 (psi)/bit | 0
            robotic_truck_applied_brake_pressure    = static_cast<float>(((data[1] >> 4) & 0x0F) + (data[2] << 4)) * 0.05; // 12 bits (2.5 - 3.8) | 0.05 (psi)/bit | 0
            robotic_trailer_applied_brake_pressure  = static_cast<float>(data[3] + ((data[4] * 0x0F) << 8)) * 0.05; // 12 bits (4.1 - 5.4) | 0.05 (psi)/bit | 0
            cmd_brake_application_level     = static_cast<float>(((data[4] >> 4) & 0x0F) + ((data[5] & 0x3F) << 4)) * 0.1; // 10 bits (5.5 - 6.6) | 0.1 (%)/bit | 0
            reserved_propb_20               = static_cast<uint8_t>(((data[5] >> 6) & 0x3) + ((data[6] & 0x03) << 2)); // 2 bits (6.7 - 7.2) | RESERVED
            axiomatic_health_status         = static_cast<AxiomaticHealthStatus>((data[6] >> 2) & 0x03); // 2 bits (7.3 - 7.4) | 4 Possible States
        }
        rolling_counter     = static_cast<uint8_t>((data[6] >> 4) & 0x0F); // 4 bits (7.5 - 7.8) | 16 Possible States
        check_value         = static_cast<uint8_t>(data[7]); // 1 byte (7.1 - 7.8) | Checksum Value
    }

    void printData()
    {
        std::cout << "\tis_valid: " << is_valid << std::endl;
        std::cout << "\tsystem_air_pressure: "                      << system_air_pressure                            << std::endl;
        std::cout << "\robotic_truck_applied_brake_pressure: "      << robotic_truck_applied_brake_pressure           << std::endl;
        std::cout << "\robotic_trailer_applied_brake_pressure: "    << robotic_trailer_applied_brake_pressure         << std::endl;
        std::cout << "\tcmd_brake_application_level: "              << cmd_brake_application_level                    << std::endl;
        std::cout << "\treserved_propb_20: "                        << reserved_propb_20                              << std::endl;
        std::cout << "\taxiomatic_health_status: "                  << static_cast<uint16_t>(axiomatic_health_status) << std::endl;
        std::cout << "\trolling_counter: "                          << static_cast<uint16_t>(rolling_counter)         << std::endl;
        std::cout << "\tcheck_value: "                              << static_cast<uint16_t>(check_value)             << std::endl;
    }

    float getSystemAirPres()                            { return system_air_pressure; }
    float getRoboticTruckAppliedBrakePres()             { return robotic_truck_applied_brake_pressure; }
    float getRoboticTrailerAppliedBrakePres()           { return robotic_trailer_applied_brake_pressure; }
    float getCmdBrakeAppliedLevel()                     { return cmd_brake_application_level; }
    float getReservedPropB20()                          { return reserved_propb_20; }
    AxiomaticHealthStatus getAxiomaticHealthStatus()    { return axiomatic_health_status; }
    uint8_t getRollingCounter()                         { return rolling_counter; }
    uint8_t getCheckValue()                             { return check_value; }

    void setSystemAirPres(float data)                           { system_air_pressure = data; }
    void setRoboticTruckAppliedBrakePres(float data)            { robotic_truck_applied_brake_pressure = data; }
    void setRoboticTrailerAppliedBrakePres(float data)          { robotic_trailer_applied_brake_pressure = data; }
    void setCmdBrakeAppliedLevel(float data)                    { cmd_brake_application_level = data; }
    void getReservedPropB20(uint8_t data)                       { reserved_propb_20 = data; }
    void setAxiomaticHealthStatus(AxiomaticHealthStatus data)   { axiomatic_health_status = data; }
    void setRollingCounter(uint8_t data)                        { rolling_counter = data; }
    void setCheckValue(uint8_t data)                            { check_value = data; }
};  // End 'PropB_20_Message' message struct

/** PropB_21 : PGN 65313 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_21_Message
{
    float system_air_pressure = 0.0;
    float robotic_truck_applied_brake_pressure = 0.0;
    float robotic_trailer_applied_brake_pressure = 0.0;
    float cmd_brake_application_level = 0.0;
    uint8_t reserved_propb_21 = 0xFF;
    AxiomaticHealthStatus axiomatic_health_status = AxiomaticHealthStatus::Module_OK;
    uint8_t rolling_counter = 0x0;
    uint8_t check_value = 0xFF;

    uint8_t propb_21_size = 8U;
    bool is_valid;

    void setData(std::array<uint8_t, 8> data)
    {
        is_valid = (checkMessageCrc(data, propb_21_size) && checkMessageCounter(data, propb_21_size, rolling_counter));
        if(is_valid)
        {
            system_air_pressure             = static_cast<float>(data[0] + ((data[1] & 0x0F) << 8)) * 0.05; // 12 bits (1.1 - 2.4) | 0.05 (psi)/bit | 0
            robotic_truck_applied_brake_pressure    = static_cast<float>(((data[1] >> 4) & 0x0F) + (data[2] << 4)) * 0.05; // 12 bits (2.5 - 3.8) | 0.05 (psi)/bit | 0
            robotic_trailer_applied_brake_pressure  = static_cast<float>(data[3] + ((data[4] * 0x0F) << 8)) * 0.05; // 12 bits (4.1 - 5.4) | 0.05 (psi)/bit | 0
            cmd_brake_application_level     = static_cast<float>(((data[4] >> 4) & 0x0F) + ((data[5] & 0x3F) << 4)) * 0.1; // 10 bits (5.5 - 6.6) | 0.1 (%)/bit | 0
            reserved_propb_21               = static_cast<uint8_t>(((data[5] >> 6) & 0x3) + ((data[6] & 0x03) << 2)); // 2 bits (6.7 - 7.2) | RESERVED
            axiomatic_health_status         = static_cast<AxiomaticHealthStatus>((data[6] >> 2) & 0x03); // 2 bits (7.3 - 7.4) | 4 Possible States
        }
        rolling_counter     = static_cast<uint8_t>((data[6] >> 4) & 0x0F); // 4 bits (7.5 - 7.8) | 16 Possible States
        check_value         = static_cast<uint8_t>(data[7]); // 1 byte (7.1 - 7.8) | Checksum Value
    }

    void printData()
    {
        std::cout << "\tis_valid: " << is_valid << std::endl;
        std::cout << "\tsystem_air_pressure: "                      << system_air_pressure                            << std::endl;
        std::cout << "\robotic_truck_applied_brake_pressure: "      << robotic_truck_applied_brake_pressure           << std::endl;
        std::cout << "\robotic_trailer_applied_brake_pressure: "    << robotic_trailer_applied_brake_pressure         << std::endl;
        std::cout << "\tcmd_brake_application_level: "              << cmd_brake_application_level                    << std::endl;
        std::cout << "\treserved_propb_21: "                        << reserved_propb_21                              << std::endl;
        std::cout << "\taxiomatic_health_status: "                  << static_cast<uint16_t>(axiomatic_health_status) << std::endl;
        std::cout << "\trolling_counter: "                          << static_cast<uint16_t>(rolling_counter)         << std::endl;
        std::cout << "\tcheck_value: "                              << static_cast<uint16_t>(check_value)             << std::endl;
    }

    float getSystemAirPres()                            { return system_air_pressure; }
    float getRoboticTruckAppliedBrakePres()             { return robotic_truck_applied_brake_pressure; }
    float getRoboticTrailerAppliedBrakePres()           { return robotic_trailer_applied_brake_pressure; }
    float getCmdBrakeAppliedLevel()                     { return cmd_brake_application_level; }
    float getReservedPropB21()                          { return reserved_propb_21; }
    AxiomaticHealthStatus getAxiomaticHealthStatus()    { return axiomatic_health_status; }
    uint8_t getRollingCounter()                         { return rolling_counter; }
    uint8_t getCheckValue()                             { return check_value; }

    void setSystemAirPres(float data)                           { system_air_pressure = data; }
    void setRoboticTruckAppliedBrakePres(float data)            { robotic_truck_applied_brake_pressure = data; }
    void setRoboticTrailerAppliedBrakePres(float data)          { robotic_trailer_applied_brake_pressure = data; }
    void setCmdBrakeAppliedLevel(float data)                    { cmd_brake_application_level = data; }
    void getReservedPropB21(uint8_t data)                       { reserved_propb_21 = data; }
    void setAxiomaticHealthStatus(AxiomaticHealthStatus data)   { axiomatic_health_status = data; }
    void setRollingCounter(uint8_t data)                        { rolling_counter = data; }
    void setCheckValue(uint8_t data)                            { check_value = data; }
};  // End 'PropB_21_Message' message struct

/** PropB_26 : PGN 65318 : Truck Command (From: Throttle Controller, To: Aux Controller) **/
struct PropB_26_Message
{
    EngineBrakeMsgModeCmd engine_brake_msg_mode_command;
    EngineBrakeCmd engine_brake_command;
    EngineBrakeLevelCmd engine_brake_level_command;
    uint8_t reserved_propb_26;
    uint8_t rolling_counter;
    uint8_t check_value;

    uint8_t propb_26_cur_count_;
    uint8_t propb_26_size = 3U;

    PropB_26_Message() :
        engine_brake_msg_mode_command(EngineBrakeMsgModeCmd::Pass_Through),
        engine_brake_command(EngineBrakeCmd::Disable_Engine_Braking),
        engine_brake_level_command(EngineBrakeLevelCmd::Low_Level),
        reserved_propb_26(0xFF),
        rolling_counter(0x0),
        check_value(0xFF),
        propb_26_cur_count_(0)
    {

    }

    cav::CANFrameStamped packData()
    {
        // Create an 8-bit Boost CRC Calculator 
        boost::crc_basic<8> crc8(0xD5, 0xFF, 0, true, true);

        // Create an empty array of message size (bytes)
        unsigned char data[propb_26_size] = {0x00, 0x00, 0x00};
        // Byte 1.1 - 1.2: Engine Brake Message Mode Command
        data[0] |= static_cast<uint8_t>(engine_brake_msg_mode_command) & 0x03;
        // Byte 1.3 - 1.4: Engine Brake Command
        data[0] |= (static_cast<uint8_t>(engine_brake_command) << 2) & 0x0C;
        // Byte 1.5 - 1.6: Engine Brake Level Command
        data[0] |= (static_cast<uint8_t>(engine_brake_level_command) << 4) & 0x30;
        // Byte 1.7 - 2.4: RESERVED
        data[0] |= (static_cast<uint8_t>(reserved_propb_26) << 6) & 0xC0;
        data[1] |= (static_cast<uint8_t>(reserved_propb_26) >> 2) & 0x0F;
        // Byte 2.5 - 2.8: Counter
        uint8_t next_counter = getNextCounter(propb_26_cur_count_);
        propb_26_cur_count_ = next_counter;
        data[1] |= (static_cast<uint8_t>(propb_26_cur_count_) << 4) & 0xF0;
        // Byte 3.1 - 3.8: CheckValue
        crc8.process_bytes(data, 7);
        data[2] = static_cast<uint8_t>(crc8.checksum());
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = 0x0CFF265A;
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = propb_26_size;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    EngineBrakeMsgModeCmd getEngBrakeCmdMode()  { return engine_brake_msg_mode_command; }
    EngineBrakeCmd getEngBrakeCmd()             { return engine_brake_command; }
    EngineBrakeLevelCmd getEngBrakeLevelCmd()   { return engine_brake_level_command; }
    uint8_t getReservedPropB26()                { return reserved_propb_26; }
    uint8_t getRollingCounter()                 { return rolling_counter; }
    uint8_t getCheckValue()                     { return check_value; }

    void setEngBrakeCmdMode(EngineBrakeMsgModeCmd data) { engine_brake_msg_mode_command = data; }
    void setEngBrakeCmd(EngineBrakeCmd data)            { engine_brake_command = data; }
    void setEngBrakeLevelCmd(EngineBrakeLevelCmd data)  { engine_brake_level_command = data; }
    void setReservedPropB26(uint8_t data)               { reserved_propb_26 = data; }
    void setRollingCounter(uint8_t data)                { rolling_counter = data; }
    void setCheckValue(uint8_t data)                    { check_value = data; }

};  // End 'PropB_26_Message' message struct

/** PropB_27 : PGN 65319 : Truck Feedback (From: Aux Controller, To: All) **/
struct PropB_27_Message
{
    EngineBrakeMsgModeCmd engine_brake_msg_mode_feedback = EngineBrakeMsgModeCmd::Pass_Through;
    EngineBrakeCmd generated_engine_brake_command = EngineBrakeCmd::Disable_Engine_Braking;
    EngineBrakeCmd expected_engine_brake_command = EngineBrakeCmd::Disable_Engine_Braking;
    EngineBrakeCmd physical_engine_brake_command = EngineBrakeCmd::Disable_Engine_Braking;
    EngineBrakeLevelCmd generated_engine_brake_level_command = EngineBrakeLevelCmd::Low_Level;
    EngineBrakeLevelCmd expected_engine_brake_level_command = EngineBrakeLevelCmd::Low_Level;
    EngineBrakeLevelCmd physical_engine_brake_level_command = EngineBrakeLevelCmd::Low_Level;
    uint8_t reserved_propb_27 = 0xFF;
    uint8_t rolling_counter = 0x0;
    uint8_t check_value = 0xFF;

    uint8_t propb_27_size = 4U;
    bool is_valid;

    void setData(std::array<uint8_t, 8> data)
    {
        is_valid = (checkMessageCrc(data, propb_27_size) && checkMessageCounter(data, propb_27_size, rolling_counter));
        if(is_valid)
        {
            engine_brake_msg_mode_feedback          = static_cast<EngineBrakeMsgModeCmd>(data[0] & 0x03); // 2 bits (1.1 - 1.2) | 4 Possible States
            generated_engine_brake_command          = static_cast<EngineBrakeCmd>((data[0] >> 2) & 0x03); // 2 bits (1.3 - 1.4) | 4 Possible States
            expected_engine_brake_command           = static_cast<EngineBrakeCmd>((data[0] >> 4) & 0x03); // 2 bits (1.6 - 1.6) | 4 Possible States
            physical_engine_brake_command           = static_cast<EngineBrakeCmd>((data[0] >> 6) & 0x03); // 2 bits (1.7 - 1.8) | 4 Possible States
            generated_engine_brake_level_command    = static_cast<EngineBrakeLevelCmd>(data[1] & 0x03); // 2 bits (2.1 - 2.2) | 4 Possible States
            expected_engine_brake_level_command     = static_cast<EngineBrakeLevelCmd>((data[1] >> 2) & 0x03); // 2 bits (2.3 - 2.4) | 4 Possible States
            physical_engine_brake_level_command     = static_cast<EngineBrakeLevelCmd>((data[1] >> 4) & 0x03); // 2 bits (2.5 - 2.6) | 4 Possible States
            reserved_propb_27                       = static_cast<uint8_t>(((data[1] >> 6) & 0x03) + ((data[2] & 0x0F) << 2)); // 6 bits (2.7 - 3.4) | RESERVED
        }
        rolling_counter                         = static_cast<uint8_t>((data[2] >> 4) & 0x0F); // 4 bits (3.5 - 3.8) | 16 Possible States
        check_value                             = static_cast<uint8_t>(data[3]); // 1 byte (4.1 - 4.8) | Checksum Value
    }

    void printData()
    {
        std::cout << "\tis_valid: " << is_valid << std::endl;
        std::cout << "\tengine_brake_msg_mode_feedback: "       << static_cast<uint16_t>(engine_brake_msg_mode_feedback)        << std::endl;
        std::cout << "\tgenerated_engine_brake_command: "       << static_cast<uint16_t>(generated_engine_brake_command)        << std::endl;
        std::cout << "\texpected_engine_brake_command: "        << static_cast<uint16_t>(expected_engine_brake_command)         << std::endl;
        std::cout << "\tphysical_engine_brake_command: "        << static_cast<uint16_t>(physical_engine_brake_command)         << std::endl;
        std::cout << "\tgenerated_engine_brake_level_command: " << static_cast<uint16_t>(generated_engine_brake_level_command)  << std::endl;
        std::cout << "\texpected_engine_brake_level_command: "  << static_cast<uint16_t>(expected_engine_brake_level_command)   << std::endl;
        std::cout << "\tphysical_engine_brake_level_command: "  << static_cast<uint16_t>(physical_engine_brake_level_command)   << std::endl;
        std::cout << "\treserved_propb_27: "                    << static_cast<uint16_t>(reserved_propb_27)                     << std::endl;
        std::cout << "\trolling_counter: "                      << static_cast<uint16_t>(rolling_counter)                       << std::endl;
        std::cout << "\tcheck_value: "                          << static_cast<uint16_t>(check_value)                           << std::endl;
    }

    EngineBrakeMsgModeCmd getEngineBrakeMsgModeFb()     { return engine_brake_msg_mode_feedback; }
    EngineBrakeCmd getGenEngineBrakeCmd()               { return generated_engine_brake_command; }
    EngineBrakeCmd getExpEngineBrakeCmd()               { return expected_engine_brake_command; }
    EngineBrakeCmd getPhyEngineBrakeCmd()               { return physical_engine_brake_command; }
    EngineBrakeLevelCmd getGenEngineBrakeLevelCmd()     { return generated_engine_brake_level_command; }
    EngineBrakeLevelCmd getExpEngineBrakeLevelCmd()     { return expected_engine_brake_level_command; }
    EngineBrakeLevelCmd getPhyEngineBrakeLevelCmd()     { return physical_engine_brake_level_command; }
    uint8_t getReservedPropB27()                        { return reserved_propb_27; }
    uint8_t getRollingCounter()                         { return rolling_counter; }
    uint8_t getCheckValue()                             { return check_value; }

    void setEngineBrakeMsgModeFb(EngineBrakeMsgModeCmd data)    { engine_brake_msg_mode_feedback = data; }
    void setGenEngineBrakeCmd(EngineBrakeCmd data)              { generated_engine_brake_command = data; }
    void setExpEngineBrakeCmd(EngineBrakeCmd data)              { expected_engine_brake_command = data; }
    void setPhyEngineBrakeCmd(EngineBrakeCmd data)              { physical_engine_brake_command = data; }
    void setGenEngineBrakeLevelCmd(EngineBrakeLevelCmd data)    { generated_engine_brake_level_command = data; }
    void setExpEngineBrakeLevelCmd(EngineBrakeLevelCmd data)    { expected_engine_brake_level_command = data; }
    void setPhyEngineBrakeLevelCmd(EngineBrakeLevelCmd data)    { physical_engine_brake_level_command = data; }
    void setReservedPropB27(uint8_t data)                       { reserved_propb_27 = data; }
    void setRollingCounter(uint8_t data)                        { rolling_counter = data; }
    void setCheckValue(uint8_t data)                            { check_value = data; }
};  // End 'PropB_27_Message' message struct

/** PropB_30 : PGN 65328 : Truck Feedback (From: Control SW, To: Throttle Controller) **/
struct PropB_30_Message
{
private:
    int32_t proportional_error = 0;
    int32_t integrator_error = 0;
    int32_t derivative_error = 0;
    int32_t divisor = 0;

public:
    void setData(std::array<uint8_t, 8> data)
    {
        proportional_error  = static_cast<int32_t>(static_cast<int16_t>(data[0] + (static_cast<uint16_t>(data[1]) << 8))) + 32768; // 2 bytes (1.1 - 2.8) | ? | 32768
        integrator_error    = static_cast<int32_t>(static_cast<int16_t>(data[2] + (static_cast<uint16_t>(data[3]) << 8))) + 32768; // 2 bytes (3.1 - 4.8) | ? | 32768
        derivative_error    = static_cast<int32_t>(static_cast<int16_t>(data[4] + (static_cast<uint16_t>(data[5]) << 8))) + 32768; // 2 bytes (5.1 - 6.8) | ? | 32768
        divisor    = static_cast<int32_t>(static_cast<int16_t>(data[6] + (static_cast<uint16_t>(data[7]) << 8))) + 32768; // 2 bytes (7.1 - 7.8) | ? | 32768
    }

    void printData()
    {
        std::cout << "\tproportional_error: " << proportional_error     << std::endl;
        std::cout << "\tintegrator_error: "   << integrator_error       << std::endl;
        std::cout << "\tderivative_error: "   << derivative_error       << std::endl;
        std::cout << "\tunknown_propb_30: "   << divisor       << std::endl;
    }

    int32_t getProportionalError()    { return proportional_error; }
    int32_t getIntegratorError()      { return integrator_error; }
    int32_t getDerivativeError()      { return derivative_error; }
    int32_t getUnknownPropB30()       { return divisor; }

    void setProportionalError(int32_t data)   { proportional_error = data; }
    void setIntegratorError(int32_t data)     { integrator_error = data; }
    void setDerivativeError(int32_t data)     { derivative_error = data; }
    void setUnknownPropB30(int32_t data)      { divisor = data; }

};  // End 'PropB_30_Message' message struct


/** PropB_31 : PGN 65329 : Truck Command (From: Control SW, To: Throttle Controller) **/
/** PropB_31 : PGN 65329 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_31_Message
{
    int16_t proportional_gain_comp_numerator = 0U;
    int16_t integrator_gain_comp_numerator = 0U;
    int16_t derivative_gain_comp_numerator = 0U;
    int16_t pid_shared_divisor = 0U;

    uint8_t propb_31_size = 8U;

    PropB_31_Message() :
        proportional_gain_comp_numerator(0),
        integrator_gain_comp_numerator(0),
        derivative_gain_comp_numerator(0),
        pid_shared_divisor(0)
    {

    }

    cav::CANFrameStamped packData()
    {
        // Create an empty array of message size (bytes)
        unsigned char data[propb_31_size] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        // Byte 1.1 - 2.8: Proportional Gain Component Numerator
        uint16_t prop_gain_num = static_cast<uint16_t>(static_cast<int32_t>(proportional_gain_comp_numerator) + 32768);
        data[0] = static_cast<uint8_t>(prop_gain_num) & 0xFF;
        data[1] = (static_cast<uint8_t>(prop_gain_num) >> 8) & 0xFF;
        // Byte 3.1 - 4.8: Integrator Gain Component Numerator
        uint16_t intr_gain_num = static_cast<uint16_t>(static_cast<int32_t>(integrator_gain_comp_numerator) + 32768);
        data[2] = static_cast<uint8_t>(intr_gain_num) & 0xFF;
        data[3] = (static_cast<uint8_t>(intr_gain_num) >> 8) & 0xFF;
        // Byte 5.1 - 6.8: Derivative Gain Component Numerator
        uint16_t derv_gain_num = static_cast<uint16_t>(static_cast<int32_t>(derivative_gain_comp_numerator) + 32768);
        data[4] = static_cast<uint8_t>(derv_gain_num) & 0xFF;
        data[5] = (static_cast<uint8_t>(derv_gain_num) >> 8) & 0xFF;
        // Byte 7.1 - 8.8: PID Shared Divisor
        uint16_t pid_div = static_cast<uint16_t>(static_cast<int32_t>(pid_shared_divisor) + 32768);
        data[6] = static_cast<uint8_t>(pid_div) & 0xFF;
        data[7] = (static_cast<uint8_t>(pid_div) >> 8) & 0xFF;
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = 0x0CFF3127;
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = propb_31_size;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    void setData(std::array<uint8_t, 8> data)
    {
        proportional_gain_comp_numerator    = static_cast<int32_t>(static_cast<int16_t>(data[0] + (static_cast<uint16_t>(data[1]) << 8))) + 32768; // 2 bytes (1.1 - 2.8) | ? | 32768
        integrator_gain_comp_numerator      = static_cast<int32_t>(static_cast<int16_t>(data[2] + (static_cast<uint16_t>(data[3]) << 8))) + 32768; // 2 bytes (3.1 - 4.8) | ? | 32768
        derivative_gain_comp_numerator      = static_cast<int32_t>(static_cast<int16_t>(data[4] + (static_cast<uint16_t>(data[5]) << 8))) + 32768; // 2 bytes (5.1 - 6.8) | ? | 32768
        pid_shared_divisor                  = static_cast<int32_t>(static_cast<int16_t>(data[6] + (static_cast<uint16_t>(data[7]) << 8))) + 32768; // 2 bytes (7.1 - 7.8) | ? | 32768
    }

    void printData()
    {
        std::cout << "\tproportional_gain_comp_numerator: " << proportional_gain_comp_numerator << std::endl;
        std::cout << "\tintegrator_gain_comp_numerator: "   << integrator_gain_comp_numerator   << std::endl;
        std::cout << "\tderivative_gain_comp_numerator: "   << derivative_gain_comp_numerator   << std::endl;
        std::cout << "\tpid_shared_divisor: "               << pid_shared_divisor               << std::endl;
    }

    int32_t getProportionalGainCompNum()    { return proportional_gain_comp_numerator; }
    int32_t getIntegratorGainCompNum()      { return integrator_gain_comp_numerator; }
    int32_t getDerivativeGainCompNum()      { return derivative_gain_comp_numerator; }
    int32_t getPIDSharedDivisor()           { return pid_shared_divisor; }

    void setProportionalGainCompNum(int32_t data)   { proportional_gain_comp_numerator = data; }
    void setIntegratorGainCompNum(int32_t data)     { integrator_gain_comp_numerator = data; }
    void setDerivativeGainCompNum(int32_t data)     { derivative_gain_comp_numerator = data; }
    void setPIDSharedDivisor(int32_t data)          { pid_shared_divisor = data; }
};  // End 'PropB_31_Message' message struct

/** PropB_F0 : PGN 65520 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_F0_Message
{
    float module_temperature = 0.0;
    float module_input_voltage = 0.0;
    float module_max_loop_time = 0.0;
    float module_avg_loop_time = 0.0;

    void setData(std::array<uint8_t, 8> data)
    {
        module_temperature      = (static_cast<float>(static_cast<int16_t>(data[0] + (static_cast<uint16_t>(data[1]) << 8))) * (1.0/128.0)) - 273; // 2 bytes (1.1 - 2.8) | 1/128 (C)/bit | -273
        module_input_voltage    = static_cast<float>(static_cast<int16_t>(data[2] + (static_cast<uint16_t>(data[3]) << 8))) * 0.001; // 2 bytes (3.1 - 4.8) | 0.001 (V)/bit | 0
        module_max_loop_time    = static_cast<float>(static_cast<int16_t>(data[4] + (static_cast<uint16_t>(data[5]) << 8))) * 5.0; // 2 bytes (5.1 - 6.8) | 5 (msec)/bit | 0
        module_avg_loop_time    = static_cast<float>(static_cast<int16_t>(data[6] + (static_cast<uint16_t>(data[7]) << 8))) * 5.0; // 2 bytes (7.1 - 7.8) | 5 (msec)/bit | 0
    }

    void printData()
    {
        std::cout << "\tmodule_temperature: "   << module_temperature   << std::endl;
        std::cout << "\tmodule_input_voltage: " << module_input_voltage << std::endl;
        std::cout << "\tmodule_max_loop_time: " << module_max_loop_time << std::endl;
        std::cout << "\tmodule_avg_loop_time: " << module_avg_loop_time << std::endl;
    }

    float getModuleTemperature()    { return module_temperature; }
    float getModuleInputVoltage()   { return module_input_voltage; }
    float getModuleMaxLoopTime()    { return module_max_loop_time; }
    float getModuleAvgLoopTime()    { return module_avg_loop_time; }

    void setModuleTemperature(float data)   { module_temperature = data; }
    void setModuleInputVoltage(float data)  { module_input_voltage = data; }
    void setModuleMaxLoopTime(float data)   { module_max_loop_time = data; }
    void setModuleAvgLoopTime(float data)   { module_avg_loop_time = data; }
};  // End 'PropB_F0_Message' message struct

/** PropB_F1 : PGN 65521 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_F1_Message
{
    float ain_1_raw = 0.0;
    float ain_2_raw = 0.0;
    float ain_3_raw = 0.0;
    float ain_4_raw = 0.0;

    void setData(std::array<uint8_t, 8> data)
    {
        ain_1_raw   = static_cast<float>(static_cast<int16_t>(data[0] + (static_cast<uint16_t>(data[1]) << 8))) * 0.001; // 2 bytes (1.1 - 2.8) | 0.001 (V)/bit | 0
        ain_2_raw   = static_cast<float>(static_cast<int16_t>(data[2] + (static_cast<uint16_t>(data[3]) << 8))) * 0.001; // 2 bytes (3.1 - 4.8) | 0.001 (V)/bit | 0
        ain_3_raw   = static_cast<float>(static_cast<int16_t>(data[4] + (static_cast<uint16_t>(data[5]) << 8))) * 0.001; // 2 bytes (5.1 - 6.8) | 0.001 (V)/bit | 0
        ain_4_raw   = static_cast<float>(static_cast<int16_t>(data[6] + (static_cast<uint16_t>(data[7]) << 8))) * 0.001; // 2 bytes (7.1 - 8.8) | 0.001 (V)/bit | 0
    }

    void printData()
    {
        std::cout << "\tain_1_raw: "    << ain_1_raw    << std::endl;
        std::cout << "\tain_2_raw: "    << ain_2_raw    << std::endl;
        std::cout << "\tain_3_raw: "    << ain_3_raw    << std::endl;
        std::cout << "\tain_4_raw: "    << ain_4_raw    << std::endl;
    }

    float getAIn1Raw()  { return ain_1_raw; }
    float getAIn2Raw()  { return ain_2_raw; }
    float getAIn3Raw()  { return ain_3_raw; }
    float getAIn4Raw()  { return ain_4_raw; }

    void setAIn1Raw(float data)     { ain_1_raw = data; }
    void setAIn2Raw(float data)     { ain_2_raw = data; }
    void setAIn3Raw(float data)     { ain_3_raw = data; }
    void setAIn4Raw(float data)     { ain_4_raw = data; }
};  // End 'PropB_F1_Message' message struct

/** PropB_F2 : PGN 65522 : Truck Feedback (From: Throttle Controller, To: Control SW) **/
struct PropB_F2_Message
{
    float aout_1_raw = 0.0;
    float aout_2_raw = 0.0;
    float aout_3_raw = 0.0;
    float aout_4_raw = 0.0;

    void setData(std::array<uint8_t, 8> data)
    {
        aout_1_raw   = static_cast<float>(static_cast<int16_t>(data[0] + (static_cast<uint16_t>(data[1]) << 8))) * 0.001; // 2 bytes (1.1 - 2.8) | 0.001 (V)/bit | 0
        aout_2_raw   = static_cast<float>(static_cast<int16_t>(data[2] + (static_cast<uint16_t>(data[3]) << 8))) * 0.001; // 2 bytes (3.1 - 4.8) | 0.001 (V)/bit | 0
        aout_3_raw   = static_cast<float>(static_cast<int16_t>(data[4] + (static_cast<uint16_t>(data[5]) << 8))) * 0.001; // 2 bytes (5.1 - 6.8) | 0.001 (V)/bit | 0
        aout_4_raw   = static_cast<float>(static_cast<int16_t>(data[6] + (static_cast<uint16_t>(data[7]) << 8))) * 0.001; // 2 bytes (7.1 - 8.8) | 0.001 (V)/bit | 0
    }

    void printData()
    {
        std::cout << "\taout_1_raw: "   << aout_1_raw    << std::endl;
        std::cout << "\taout_2_raw: "   << aout_2_raw    << std::endl;
        std::cout << "\taout_3_raw: "   << aout_3_raw    << std::endl;
        std::cout << "\taout_4_raw: "   << aout_4_raw    << std::endl;
    }

    float getAOut1Raw()     { return aout_1_raw; }
    float getAOut2Raw()     { return aout_2_raw; }
    float getAOut3Raw()     { return aout_3_raw; }
    float getAOut4Raw()     { return aout_4_raw; }

    void setAOut1Raw(float data)    { aout_1_raw = data; }
    void setAOut2Raw(float data)    { aout_2_raw = data; }
    void setAOut3Raw(float data)    { aout_3_raw = data; }
    void setAOut4Raw(float data)    { aout_4_raw = data; }
};  // End 'PropB_F2_Message' message struct

/** PropB_F4 : PGN 65524 : Truck Feedback (From: All, To: Control SW) **/
struct PropB_F4_Message
{
    uint8_t service_brake = 0U;
    uint8_t door_ajar = 0U;
    uint8_t parking_brake = 0U;
    uint8_t gear_not_manual = 0U;
    uint64_t reserved_propb_f4 = 0U;

    void setData(std::array<uint8_t, 8> data)
    {
        service_brake       = static_cast<uint8_t>(data[0] & 0x03); // 2 bits (1.1 - 1.2) | 4 Possible States
        door_ajar           = static_cast<uint8_t>((data[0] >> 2) & 0x03); // 2 bits (1.3 - 1.4) | 4 Possible States
        parking_brake       = static_cast<uint8_t>((data[0] >> 4) & 0x03); // 2 bits (1.5 - 1.6) | 4 Possible States
        gear_not_manual     = static_cast<uint8_t>((data[0] >> 6) & 0x03); // 2 bits (1.7 - 1.8) | 4 Possible States
        reserved_propb_f4   = static_cast<uint64_t>(data[1] + (static_cast<uint64_t>(data[2]) << 8) + 
        (static_cast<uint64_t>(data[3]) << 16) + (static_cast<uint64_t>(data[4]) << 24) + (static_cast<uint64_t>(data[5]) << 32) +
        (static_cast<uint64_t>(data[6]) << 40) + (static_cast<uint64_t>(data[7]) << 48));  // 7 bytes (2.1 - 8.8) | RESERVED
    }

    void printData()
    {
        std::cout << "\tservice_brake: "        << static_cast<uint16_t>(service_brake)     << std::endl;
        std::cout << "\tdoor_ajar: "            << static_cast<uint16_t>(door_ajar)         << std::endl;
        std::cout << "\tparking_brake: "        << static_cast<uint16_t>(parking_brake)     << std::endl;
        std::cout << "\tgear_not_manual: "      << static_cast<uint16_t>(gear_not_manual)   << std::endl;
        std::cout << "\treserved_propb_f4: "    << static_cast<uint64_t>(reserved_propb_f4) << std::endl;
    }

    uint8_t getServiceBrake()       { return service_brake; }
    uint8_t getDoorAjar()           { return door_ajar; }
    uint8_t getParkingBrake()       { return parking_brake; }
    uint8_t getGearNotManual()      { return gear_not_manual; }
    uint64_t getReservedPropBF4()   { return reserved_propb_f4; }

    void setServiceBrake(uint8_t data)      { service_brake = data; }
    void setDoorAjar(uint8_t data)          { door_ajar = data; }
    void setParkingBrake(uint8_t data)      { parking_brake = data; }
    void setGearNotManual(uint8_t data)     { gear_not_manual = data; }
    void setReservedPropBF4(uint64_t data)  { reserved_propb_f4 = data; }
};  // End 'PropB_F4_Message' message struct */

/** PropB_F5 : PGN 65525 : Truck Feedback (From: All, To: Control SW) **/
struct PropB_F5_Message
{
    uint8_t air_loss_primary = 0U;
    uint8_t air_loss_secondary = 0U;
    uint8_t valve_failure_primary = 0U;
    uint8_t valve_failure_secondary = 0U;
    uint8_t controller_power_primary = 0U;
    uint8_t controller_power_secondary = 0U;
    uint8_t controller_comms_primary = 0U;
    uint8_t controller_comms_secondary = 0U;
    uint64_t reserved_propb_f5;
    
    void setData(std::array<uint8_t, 8> data)
    {
        air_loss_primary            = static_cast<uint8_t>(data[0] & 0x03); // 2 bits (1.1 - 1.2) | 4 Possible States
        air_loss_secondary          = static_cast<uint8_t>((data[0] >> 2) & 0x03); // 2 bits (1.3 - 1.4) | 4 Possible States
        valve_failure_primary       = static_cast<uint8_t>((data[0] >> 4) & 0x03); // 2 bits (1.5 - 1.6) | 4 Possible States
        valve_failure_secondary     = static_cast<uint8_t>((data[0] >> 6) & 0x033); // 2 bits (1.7 - 1.8) | 4 Possible States
        controller_power_primary    = static_cast<uint8_t>(data[1] & 0x03); // 2 bits (2.1 - 2.2) | 4 Possible States
        controller_power_secondary  = static_cast<uint8_t>((data[1] >> 2) & 0x03); // 2 bits (2.3 - 2.4) | 4 Possible States
        controller_comms_primary    = static_cast<uint8_t>((data[1] >> 4) & 0x03); // 2 bits (2.5 - 2.6) | 4 Possible States
        controller_comms_secondary  = static_cast<uint8_t>((data[1] >> 6) & 0x03); // 2 bits (2.7 - 2.8) | 4 Possible States
        reserved_propb_f5           = static_cast<uint64_t>(data[2] + (static_cast<uint64_t>(data[3]) << 8) + (static_cast<uint64_t>(data[4]) << 16) + 
        (static_cast<uint64_t>(data[5]) << 24) + (static_cast<uint64_t>(data[6]) << 32) + (static_cast<uint64_t>(data[7]) << 40));  // 6 bytes (3.1 - 8.8) | RESERVED
    }

    void printData()
    {
        std::cout << "\tair_loss_primary: "             << static_cast<uint16_t>(air_loss_primary)            << std::endl;
        std::cout << "\tair_loss_secondary: "           << static_cast<uint16_t>(air_loss_secondary)          << std::endl;
        std::cout << "\tvalve_failure_primary: "        << static_cast<uint16_t>(valve_failure_primary)       << std::endl;
        std::cout << "\tvalve_failure_secondary: "      << static_cast<uint16_t>(valve_failure_secondary)     << std::endl;
        std::cout << "\tcontroller_power_primary: "     << static_cast<uint16_t>(controller_power_primary)    << std::endl;
        std::cout << "\tcontroller_power_secondary: "   << static_cast<uint16_t>(controller_power_secondary)  << std::endl;
        std::cout << "\tcontroller_comms_primary: "     << static_cast<uint16_t>(controller_comms_primary)    << std::endl;
        std::cout << "\tcontroller_comms_secondary: "   << static_cast<uint16_t>(controller_comms_secondary)  << std::endl;
        std::cout << "\treserved_propb_f5: "            << static_cast<uint64_t>(reserved_propb_f5)           << std::endl;
    }

    uint8_t getAirLossPrimary()             { return air_loss_primary; }
    uint8_t getAirLossSecondary()           { return air_loss_secondary; }
    uint8_t getValveFailurePrimary()        { return valve_failure_primary; }
    uint8_t getValveFailureSecondary()      { return valve_failure_secondary; }
    uint8_t getControllerPowerPrimary()     { return controller_power_primary; }
    uint8_t getControllerPowerSecondary()   { return controller_power_secondary; }
    uint8_t getControllerCommsPrimary()     { return controller_comms_primary; }
    uint8_t getControllerCommsSecondary()   { return controller_comms_secondary; }
    uint64_t getReservedPropBF5()           { return reserved_propb_f5; }

    void setAirLossPrimary(uint8_t data)            { air_loss_primary = data; }
    void setAirLossSecondary(uint8_t data)          { air_loss_secondary = data; }
    void setValveFailurePrimary(uint8_t data)       { valve_failure_primary = data; }
    void setValveFailureSecondary(uint8_t data)     { valve_failure_secondary = data; }
    void setControllerPowerPrimary(uint8_t data)    { controller_power_primary = data; }
    void setControllerPowerSecondary(uint8_t data)  { controller_power_secondary = data; }
    void setControllerCommsPrimary(uint8_t data)    { controller_comms_primary = data; }
    void setControllerCommsSecondary(uint8_t data)  { controller_comms_secondary = data; }
    void setReservedPropBF5(uint64_t data)          { reserved_propb_f5 = data; }
};  // End 'PropB_F5_Message' message struct */

/** PropB_F6 : PGN 65526 : Truck Feedback (From: All, To: Control SW) **/
struct PropB_F6_Message
{
    uint8_t firmware_load = 0U;
    uint8_t settings_invalid = 0U;
    uint8_t vehicle_can2_error_passive = 0U;
    uint8_t vehicle_can2_bus_off = 0U;
    uint8_t dbw_can1_error_passive = 0U;
    uint8_t dbw_can1_bus_off = 0U;
    uint8_t vbus_low_voltage = 0U;
    uint8_t system_reset_watchdog = 0U;
    uint8_t uncalibrated_settings = 0U;
    uint8_t vehicle_speed_msg_expirerd = 0U;
    uint8_t brake_pressure_error_threshold = 0U;
    uint8_t brake_emergency_fault = 0U;
    uint8_t aux_throt_module_timeout = 0U;
    uint8_t accel_sync_signal_lost = 0U;
    uint8_t accel_in_sensor_threshold = 0U;
    uint8_t accel_out_sensor_threshold = 0U;
    uint8_t accel_in_out_mismatch = 0U;
    uint8_t test_mode_timeout = 0U;
    uint8_t axiom_can4_error_passive = 0U;
    uint8_t axiom_can4_bus_off = 0U;
    uint8_t vehicle_can3_error_passive = 0U;
    uint8_t vehicle_can3_bus_off = 0U;
    uint8_t command_msg_timeout = 0U;
    uint8_t axiomatic_msg_timeout = 0U;
    uint8_t truck_feedback_msg_timeout = 0U;
    uint16_t reserved_propb_f6 = 0U;
    
    void setData(std::array<uint8_t, 8> data)
    {
        firmware_load                   = static_cast<uint8_t>(data[0] & 0x03); // 2 bits (1.1 - 1.2) | 4 Possible States
        settings_invalid                = static_cast<uint8_t>((data[0] >> 2) & 0x03); // 2 bits (1.3 - 1.4) | 4 Possible States
        vehicle_can2_error_passive      = static_cast<uint8_t>((data[0] >> 4) & 0x03); // 2 bits (1.5 - 1.6) | 4 Possible States
        vehicle_can2_bus_off            = static_cast<uint8_t>((data[0] >> 6) & 0x03); // 2 bits (1.7 - 1.8) | 4 Possible States
        dbw_can1_error_passive          = static_cast<uint8_t>(data[1] & 0x03); // 2 bits (2.1 - 2.2) | 4 Possible States
        dbw_can1_bus_off                = static_cast<uint8_t>((data[1] >> 2) & 0x03); // 2 bits (2.3 - 2.4) | 4 Possible States
        vbus_low_voltage                = static_cast<uint8_t>((data[1] >> 4) & 0x03); // 2 bits (2.5 - 2.6) | 4 Possible States
        system_reset_watchdog           = static_cast<uint8_t>((data[1] >> 6) & 0x03); // 2 bits (2.7 - 2.8) | 4 Possible States
        uncalibrated_settings           = static_cast<uint8_t>(data[2] & 0x03); // 2 bits (3.1 - 3.2) | 4 Possible States
        vehicle_speed_msg_expirerd      = static_cast<uint8_t>((data[2] >> 2) & 0x03); // 2 bits (3.3 - 3.4) | 4 Possible States
        brake_pressure_error_threshold  = static_cast<uint8_t>((data[2] >> 4) & 0x03); // 2 bits (3.5 - 3.6) | 4 Possible States
        brake_emergency_fault           = static_cast<uint8_t>((data[2] >> 6) & 0x03); // 2 bits (3.7 - 3.8) | 4 Possible States
        aux_throt_module_timeout        = static_cast<uint8_t>(data[3] & 0x03); // 2 bits (4.1 - 4.2) | 4 Possible States
        accel_sync_signal_lost          = static_cast<uint8_t>((data[3] >> 2) & 0x03); // 2 bits (4.3 - 4.4) | 4 Possible States
        accel_in_sensor_threshold       = static_cast<uint8_t>((data[3] >> 4) & 0x03); // 2 bits (4.5 - 4.6) | 4 Possible States
        accel_out_sensor_threshold      = static_cast<uint8_t>((data[3] >> 6) & 0x03); // 2 bits (4.7 - 4.8) | 4 Possible States
        accel_in_out_mismatch           = static_cast<uint8_t>(data[4] & 0x03); // 2 bits (5.1 - 5.2) | 4 Possible States
        test_mode_timeout               = static_cast<uint8_t>((data[4] >> 2) & 0x03); // 2 bits (5.3 - 5.4) | 4 Possible States
        axiom_can4_error_passive        = static_cast<uint8_t>((data[4] >> 4) & 0x03); // 2 bits (5.5 - 5.6) | 4 Possible States
        axiom_can4_bus_off              = static_cast<uint8_t>((data[4] >> 6) & 0x03); // 2 bits (5.7 - 5.8) | 4 Possible States
        vehicle_can3_error_passive      = static_cast<uint8_t>(data[5] & 0x03); // 2 bits (6.1 - 6.2) | 4 Possible States
        vehicle_can3_bus_off            = static_cast<uint8_t>((data[5] >> 2) & 0x03); // 2 bits (6.3 - 6.4) | 4 Possible States
        command_msg_timeout             = static_cast<uint8_t>((data[5] >> 4) & 0x03); // 2 bits (6.5 - 6.6) | 4 Possible States
        axiomatic_msg_timeout           = static_cast<uint8_t>((data[5] >> 6) & 0x03); // 2 bits (6.7 - 6.8) | 4 Possible States
        truck_feedback_msg_timeout      = static_cast<uint8_t>(data[6] & 0x03); // 2 bits (7.1 - 7.2) | 4 Possible States
        reserved_propb_f6               = static_cast<uint16_t>(((data[6] >> 2) & 0x3F) + (static_cast<uint16_t>(data[7]) << 6)); // 2 bytes (7.3 - 8.8) | RESERVED
    }

    void printData()
    {
        std::cout << "\tfirmware_load: "                    << static_cast<uint16_t>(firmware_load)                   << std::endl;
        std::cout << "\tsettings_invalid: "                 << static_cast<uint16_t>(settings_invalid)                << std::endl;
        std::cout << "\tvehicle_can2_error_passive: "       << static_cast<uint16_t>(vehicle_can2_error_passive)      << std::endl;
        std::cout << "\tvehicle_can2_bus_off: "             << static_cast<uint16_t>(vehicle_can2_bus_off)            << std::endl;
        std::cout << "\tdbw_can1_error_passive: "           << static_cast<uint16_t>(dbw_can1_error_passive)          << std::endl;
        std::cout << "\tdbw_can1_bus_off: "                 << static_cast<uint16_t>(dbw_can1_bus_off)                << std::endl;
        std::cout << "\tvbus_low_voltage: "                 << static_cast<uint16_t>(vbus_low_voltage)                << std::endl;
        std::cout << "\tsystem_reset_watchdog: "            << static_cast<uint16_t>(system_reset_watchdog)           << std::endl;
        std::cout << "\tuncalibrated_settings: "            << static_cast<uint16_t>(uncalibrated_settings)           << std::endl;
        std::cout << "\tvehicle_speed_msg_expirerd: "       << static_cast<uint16_t>(vehicle_speed_msg_expirerd)      << std::endl;
        std::cout << "\tbrake_pressure_error_threshold: "   << static_cast<uint16_t>(brake_pressure_error_threshold)  << std::endl;
        std::cout << "\tbrake_emergency_fault: "            << static_cast<uint16_t>(brake_emergency_fault)           << std::endl;
        std::cout << "\taux_throt_module_timeout: "         << static_cast<uint16_t>(aux_throt_module_timeout)        << std::endl;
        std::cout << "\taccel_sync_signal_lost: "           << static_cast<uint16_t>(accel_sync_signal_lost)          << std::endl;
        std::cout << "\taccel_in_sensor_threshold: "        << static_cast<uint16_t>(accel_in_sensor_threshold)       << std::endl;
        std::cout << "\taccel_out_sensor_threshold: "       << static_cast<uint16_t>(accel_out_sensor_threshold)      << std::endl;
        std::cout << "\taccel_in_out_mismatch: "            << static_cast<uint16_t>(accel_in_out_mismatch)           << std::endl;
        std::cout << "\ttest_mode_timeout: "                << static_cast<uint16_t>(test_mode_timeout)               << std::endl;
        std::cout << "\taxiom_can4_error_passive: "         << static_cast<uint16_t>(axiom_can4_error_passive)        << std::endl;
        std::cout << "\taxiom_can4_bus_off: "               << static_cast<uint16_t>(axiom_can4_bus_off)              << std::endl;
        std::cout << "\tvehicle_can3_error_passive: "       << static_cast<uint16_t>(vehicle_can3_error_passive)      << std::endl;
        std::cout << "\tvehicle_can3_bus_off: "             << static_cast<uint16_t>(vehicle_can3_bus_off)            << std::endl;
        std::cout << "\tcommand_msg_timeout: "              << static_cast<uint16_t>(command_msg_timeout)             << std::endl;
        std::cout << "\taxiomatic_msg_timeout: "            << static_cast<uint16_t>(axiomatic_msg_timeout)           << std::endl;
        std::cout << "\truck_feedback_msg_timeout: "        << static_cast<uint16_t>(truck_feedback_msg_timeout)      << std::endl;
        std::cout << "\treserved_propb_f6: "                << static_cast<uint16_t>(reserved_propb_f6)               << std::endl;
    }

    uint8_t getFirmwareLoad()                   { return firmware_load; }
    uint8_t getSettingsInvalid()                { return settings_invalid; }
    uint8_t getVehicleCanErrorPassive()         { return vehicle_can2_error_passive; }
    uint8_t getVehicleCanBusOff()               { return vehicle_can2_bus_off; }
    uint8_t getDbwCanErrorPassive()             { return dbw_can1_error_passive; }
    uint8_t getDbwCanBusOff()                   { return dbw_can1_bus_off; }
    uint8_t getVBusLowVoltage()                 { return vbus_low_voltage; }
    uint8_t getSystemResetWatchdog()            { return system_reset_watchdog; }
    uint8_t getUncalibratedSettings()           { return uncalibrated_settings; }
    uint8_t getVehicleSpeedMsgExpired()         { return vehicle_speed_msg_expirerd; }
    uint8_t getBrakePressureErrorThreshold()    { return brake_pressure_error_threshold; }
    uint8_t getBrakeEmergencyFault()            { return brake_emergency_fault; }
    uint8_t getAuxiliaryModuleTimeout()         { return aux_throt_module_timeout; }
    uint8_t getSyncSignalLost()                 { return accel_sync_signal_lost; }
    uint8_t getAnalog0SensorThreshold()         { return accel_in_sensor_threshold; }
    uint8_t getAnalog1SensorThreshold()         { return accel_out_sensor_threshold; }
    uint8_t getAnalogSensorMismatch()           { return accel_in_out_mismatch; }
    uint8_t getTestModeTimeout()                { return test_mode_timeout; }
    uint8_t getAxiomCan4ErrorPassive()          { return axiom_can4_error_passive; }
    uint8_t getAxiomCan4BussOff()               { return axiom_can4_bus_off; }
    uint8_t getVehicleCan3ErrorPassive()        { return vehicle_can3_error_passive; }
    uint8_t getVehicleCan3BusOff()              { return vehicle_can3_bus_off; }
    uint8_t getCommandMsgTimeout()              { return command_msg_timeout; }
    uint8_t getAxiomaticMsgTimeout()            { return axiomatic_msg_timeout; }
    uint8_t getTruckFeedbackMsgTimeout()        { return truck_feedback_msg_timeout; }
    uint16_t getReservedPropBF6()               { return reserved_propb_f6; }

    void setFirmwareLoad(uint8_t data)                  { firmware_load = data; }
    void setSettingsInvalid(uint8_t data)               { settings_invalid = data; }
    void setVehicleCanErrorPassive(uint8_t data)        { vehicle_can2_error_passive = data; }
    void setVehicleCanBusOff(uint8_t data)              { vehicle_can2_bus_off = data; }
    void setDbwCanErrorPassive(uint8_t data)            { dbw_can1_error_passive = data; }
    void setDbwCanBusOff(uint8_t data)                  { dbw_can1_bus_off = data; }
    void setVBusLowVoltage(uint8_t data)                { vbus_low_voltage = data; }
    void setSystemResetWatchdog(uint8_t data)           { system_reset_watchdog = data; }
    void setUncalibratedSettings(uint8_t data)          { uncalibrated_settings = data; }
    void setVehicleSpeedMsgExpired(uint8_t data)        { vehicle_speed_msg_expirerd = data; }
    void setBrakePressureErrorThreshold(uint8_t data)   { brake_pressure_error_threshold = data; }
    void setBrakeEmergencyFault(uint8_t data)           { brake_emergency_fault = data; }
    void setAuxiliaryModuleTimeout(uint8_t data)        { aux_throt_module_timeout = data; }
    void setSyncSignalLost(uint8_t data)                { accel_sync_signal_lost = data; }
    void setAnalog0SensorThreshold(uint8_t data)        { accel_in_sensor_threshold = data; }
    void setAnalog1SensorThreshold(uint8_t data)        { accel_out_sensor_threshold = data; }
    void setAnalogSensorMismatch(uint8_t data)          { accel_in_out_mismatch = data; }
    void setTestModeTimeout(uint8_t data)               { test_mode_timeout = data; }
    void setAxiomCan4ErrorPassive(uint8_t data)         { axiom_can4_error_passive = data; }
    void setAxiomCan4BussOff(uint8_t data)              { axiom_can4_bus_off = data; }
    void setVehicleCan3ErrorPassive(uint8_t data)       { vehicle_can3_error_passive = data; }
    void setVehicleCan3BusOff(uint8_t data)             { vehicle_can3_bus_off = data; }
    void setCommandMsgTimeout(uint8_t data)             { command_msg_timeout = data; }
    void setAxiomaticsgTimeout(uint8_t data)            { axiomatic_msg_timeout = data; }
    void setTruckFeedbackMsgTimeout(uint8_t data)       { truck_feedback_msg_timeout = data; }
    void setReservedPropBF6(uint16_t data)              { reserved_propb_f6 = data; }
};  // End 'PropB_F6_Message' message struct */

/** Req : PGN 59904 : Truck Command (From: All, To: All) **/
struct Request_Message
{
    uint32_t request_pgn = 0U;

    uint8_t req_msg_size = 3U;

    Request_Message() :
        request_pgn(0)
    {

    }

    cav::CANFrameStamped packData()
    {
        // Create an empty array of message size (bytes)
        unsigned char data[req_msg_size] = {};
        // Byte 1.1 - 3.8: PGN of Requested Message
        data[0] = static_cast<uint8_t>(request_pgn) & 0xFF;
        data[1] = (static_cast<uint8_t>(request_pgn) >> 8) & 0xFF;
        data[2] = (static_cast<uint8_t>(request_pgn) >> 16) & 0xFF;
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = 0x0CEAFF27;    // REQUEST From: Control SW, To: All
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = req_msg_size;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    uint32_t getPgnRequest()    { return request_pgn; }

    void setPgnRequest(uint32_t pgn_request) {    }
};  // End 'Request_Message' message struct

/** PropA_F1D0 : PGN 61184 : Truck Command (From: Control SW, To: All) **/
struct PropA_F1D0_Message
{
    uint16_t propa_F1D0_extension_id;
    ClearFaultsMode clear_faults_mode;
    uint8_t reserved_propa_F1D0;
    uint8_t rolling_counter;

    uint8_t propa_F1D0_cur_count_;
    uint8_t propa_F1D0_size = 3U;

    PropA_F1D0_Message() : 
        propa_F1D0_extension_id(0xD0F1),
        clear_faults_mode(ClearFaultsMode::Clear_Faults_Disabled),
        reserved_propa_F1D0(0xFF),
        rolling_counter(0),
        propa_F1D0_cur_count_(0)
    {

    }

    cav::CANFrameStamped packData()
    {
        // Create an empty array of message size (bytes)
        unsigned char data[propa_F1D0_size] = {};
        // Byte 1.1 - 2.8: PropA Extension ID
        data[0] = static_cast<uint8_t>(propa_F1D0_extension_id) & 0xFF;
        data[1] = static_cast<uint8_t>(propa_F1D0_extension_id >> 8);
        // Byte 3.1 - 3.2: Clear Faults Mode
        data[2] |= static_cast<uint8_t>(clear_faults_mode) & 0x03;
        // Byte 3.3 - 3.4: RESERVED
        data[2] |= static_cast<uint8_t>(reserved_propa_F1D0) & 0x0C;
        // Byte 3.5 - 3.8: Counter
        uint8_t next_counter = getNextCounter(propa_F1D0_cur_count_);
        propa_F1D0_cur_count_ = next_counter;
        data[2] |= (static_cast<uint8_t>(propa_F1D0_cur_count_) << 4) & 0xF0;
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = 0x0CEFFF27;    // PropA From: Control SW, To: All
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = propa_F1D0_size;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    uint16_t getExtensionID()               { return propa_F1D0_extension_id; }
    ClearFaultsMode getClearFaultsMode()    { return clear_faults_mode; }
    uint8_t getReservedPropAF1D0()          { return reserved_propa_F1D0; }
    uint8_t getRollingCounter()             { return rolling_counter; } 

    void setExtensionID(uint16_t data)              { propa_F1D0_extension_id = data; }
    void setClearFaultsMode(ClearFaultsMode data)   { clear_faults_mode = data; }
    void setReservedPropAF1D0(uint8_t data)         { reserved_propa_F1D0 = data; }
    void setRollingCounter(uint8_t data)            { rolling_counter = data; } 

};  // End 'PropA_F1D0_Message' message struct */

/** PropB_ED : PGN 65517 : Truck Command (From: Aux Controller, To: Control SW) **/
/** PropB_ED : PGN 65517 : Truck Feedback (From: Control SW, To: Aux Controller) **/
struct PropB_ED_Message
{
    GenericStates fault_led_state = GenericStates::Off;
    GenericStates robotic_led_state = GenericStates::Off;
    GenericStates emo_led_state = GenericStates::Off;
    GenericStates audible_buzzer_state = GenericStates::Off;
    
    PropB_ED_Message() : 
        fault_led_state(GenericStates::Off),
        robotic_led_state(GenericStates::Off),
        emo_led_state(GenericStates::Off),
        audible_buzzer_state(GenericStates::Off)
    {

    }

    cav::CANFrameStamped packData()
    {
        // Create an empty array of message size (bytes)
        unsigned char data[1] = {};
        // Byte 1.1 - 1.2: Fault LED State
        data[0] |= static_cast<uint8_t>(fault_led_state) & 0x03;
        // Byte 1.3 - 1.4: Robotic LED State
        data[0] |= (static_cast<uint8_t>(robotic_led_state) << 2) & 0x0C;
        // Byte 1.5 - 1.6: Emo Button LED State
        data[0] |= (static_cast<uint8_t>(emo_led_state) << 4) & 0x30;
        // Byte 1.7 - 1.8: Audible Buzzer State
        data[0] |= (static_cast<uint8_t>(audible_buzzer_state) << 6) & 0xC0;
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = 0x0CFFED28;
        can_frame.is_rtr = false;
        can_frame.is_extended = true;
        can_frame.is_error = false;
        can_frame.dlc = 1;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    void setData(std::array<uint8_t, 8> data)
    {
        fault_led_state         = static_cast<GenericStates>(data[0] & 0x03); // 2 bits (1.1 - 1.2) | 4 Possible States
        robotic_led_state       = static_cast<GenericStates>((data[0] >> 2) & 0x03); // 2 bits (1.3 - 1.4) | 4 Possible States
        emo_led_state           = static_cast<GenericStates>((data[0] >> 4) & 0x03); // 2 bits (1.5 - 1.6) | 4 Possible States
        audible_buzzer_state    = static_cast<GenericStates>((data[0] >> 6) & 0x03); // 2 bits (1.7 - 1.8) | 4 Possible States
    }

    void printData()
    {
        std::cout << "\tfault_led_state: "      << static_cast<uint16_t>(fault_led_state)      << std::endl;
        std::cout << "\trobotic_led_state: "    << static_cast<uint16_t>(robotic_led_state)    << std::endl;
        std::cout << "\temo_led_state: "        << static_cast<uint16_t>(emo_led_state)        << std::endl;
        std::cout << "\taudible_buzzer_state: " << static_cast<uint16_t>(audible_buzzer_state) << std::endl;
    }

    GenericStates getFaultLEDState()        { return fault_led_state; }
    GenericStates getRoboticLEDState()      { return robotic_led_state; }
    GenericStates getEmoLEDState()          { return emo_led_state; }
    GenericStates getAudibleBuzzerState()   { return audible_buzzer_state; }

    void setFaultLEDState(GenericStates data)       { fault_led_state = data; }
    void setRoboticLEDState(GenericStates data)     { robotic_led_state = data; }
    void setEmoLEDState(GenericStates data)         { emo_led_state = data; }
    void setAudibleBuzzerState(GenericStates data)  { audible_buzzer_state = data; }
};  // End 'PropB_ED_Message' message struct */

/** PropB_FC : PGN 65532 : Truck Feedback (From: All, To: Control SW) **/
struct PropB_FC_Message
{
    uint32_t module_nvram_settings_crc = 0U;
    uint32_t module_ram_settings_crc = 0U;

    uint8_t propb_ED_size = 8U;
    
    void setData(std::array<uint8_t, 8> data)
    {
        module_nvram_settings_crc   = static_cast<uint32_t>(data[0] + (static_cast<uint16_t>(data[1]) << 8) + (data[2] << 16) + (data[3] << 24)); // 4 bytes (1.1 - 4.8) | ?
        module_ram_settings_crc     = static_cast<uint32_t>(data[4] + (static_cast<uint16_t>(data[5]) << 8) + (data[6] << 16) + (data[7] << 24)); // 4 bytes (5.1 - 8.8) | ?
    }

    void printData()
    {
        std::cout << "\tmodule_nvram_settings_crc: "    << module_nvram_settings_crc    << std::endl;
        std::cout << "\tmodule_ram_settings_crc: "      << module_ram_settings_crc      << std::endl;
    }

    uint32_t getModuleNVRamSettingsCRC()    { return module_nvram_settings_crc; }
    uint32_t getModuleRamSettingsCRC()      { return module_ram_settings_crc; }

    void setModuleNVRamSettingsCRC(uint32_t data)   { module_nvram_settings_crc = data; }
    void setModuleRamSettingsCRC(uint32_t data)     { module_ram_settings_crc = data; }
};  // End 'PropB_FC_Message' message struct */ 

/** Light_Status_Message : PGN 65536 :  **/
struct Light_Status_Message
{
    GenericStates flashing = GenericStates::Ignore;
    GenericStates left_blinker = GenericStates::Ignore;
    GenericStates right_blinker = GenericStates::Ignore;
    GenericStates take_down = GenericStates::Ignore;
    GenericStates green_flasher = GenericStates::Ignore;
    GenericStates green_solid = GenericStates::Ignore;

    Light_Status_Message() : 
        flashing(GenericStates::Ignore),
        left_blinker(GenericStates::Ignore),
        right_blinker(GenericStates::Ignore),
        take_down(GenericStates::Ignore),
        green_flasher(GenericStates::Ignore),
        green_solid(GenericStates::Ignore)
    {

    }

    cav::CANFrameStamped packData(uint32_t message_id)
    {
        // Create an empty array of message size (bytes)
        unsigned char data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        if(message_id == 0x50A)
        {
			// Byte 1.1 - 1.4: Takedown 
            data[0] = static_cast<uint8_t>(flashing) | 0xF0;
			data[1] = 0xFF;
			data[2] = 0xFF;
			data[3] = 0xFF;
			data[4] = 0xFF;
            // Byte 6.1 - 6.4: Flashing
            data[5] = static_cast<uint8_t>(right_blinker) | 0xF0;
			data[6] = 0xFF;
            // Byte 8.1 - 8.4: Left Blinker
            data[7] = static_cast<uint8_t>(left_blinker) | (static_cast<uint8_t>(take_down) << 4);
            // Byte 8.5 - 8.8: Right Blinker
            //data[7] &= (static_cast<uint8_t>(right_blinker) << 4) & 0xF0;

            // NOT USED
            //data[4] = static_cast<uint8_t>(green_flasher);
            // NOT USED
            //data[5] = static_cast<uint8_t>(green_solid);
        }
        if(message_id == 0x50B)
        {
            // Byte 1.5 - 1.8: Flashing
            data[0] = (static_cast<uint8_t>(left_blinker) << 4) | 0x0F;
            // Byte 2.1 - 2.4: Left Blinker
            data[1] = static_cast<uint8_t>(right_blinker) | (static_cast<uint8_t>(take_down) << 4);
			data[2] = 0xFF;
			data[3] = 0xFF;
			data[4] = 0xFF;
            // Byte 2.5 - 2.8: Right Blinker
            //data[1] &= (static_cast<uint8_t>(right_blinker) << 4) & 0xF0;
            // Byte 6.5 - 6.8: Takedown 
            data[5] = (static_cast<uint8_t>(flashing) << 4) | 0x0F;
			data[6] = 0xFF;
			data[7] = 0xFF;
            // NOT USED
            //data[4] = static_cast<uint8_t>(green_flasher);
            // NOT USED
            //data[5] = static_cast<uint8_t>(green_solid);
        }
        
        // Create the CAN Frame
        cav::CANFrameStamped can_frame;
        can_frame.id = message_id;
        can_frame.is_rtr = false;
        can_frame.is_extended = false;
        can_frame.is_error = false;
        can_frame.dlc = 8;
        for(int i = 0; i < can_frame.dlc; i++)
        {
            can_frame.data[i] = data[i];
        }
        return can_frame;
    }

    void printData()
    {
        std::cout << "\tflashing: "         << static_cast<uint8_t>(flashing)         << std::endl;
        std::cout << "\tleft_blinker: "     << static_cast<uint8_t>(left_blinker)     << std::endl;
        std::cout << "\tright_blinker: "    << static_cast<uint8_t>(right_blinker)    << std::endl;
        std::cout << "\ttake_down: "        << static_cast<uint8_t>(take_down)        << std::endl;
        std::cout << "\tgreen_flasher: "    << static_cast<uint8_t>(green_flasher)    << std::endl;
        std::cout << "\tgreen_solid: "      << static_cast<uint8_t>(green_solid)      << std::endl;
    }

    GenericStates getFlashingState()        { return flashing; }
    GenericStates getLeftBlinkerState()     { return left_blinker; }
    GenericStates getRightBlinkerState()    { return right_blinker; }
    GenericStates getTakeDownState()        { return take_down; }
    GenericStates getGreenFlasherState()    { return green_flasher; }
    GenericStates getGreenSolidState()      { return green_solid; }

    void setFlashingState(GenericStates data)       { flashing = data; }
    void setLeftBlinkerState(GenericStates data)    { left_blinker = data; }
    void setRightBlinkerState(GenericStates data)   { right_blinker = data; }
    void setTakeDownState(GenericStates data)       { take_down = data; }
    void setGreenFlasherState(GenericStates data)   { green_flasher = data; }
    void setGreenSolidState(GenericStates data)     { green_solid = data; }
};  // End 'Light_Status_Message' message struct */ 

} // End 'truck' namespace
