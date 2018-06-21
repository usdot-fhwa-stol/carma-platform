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

#include "truck_types.h"

#include <cav_driver_utils/can/can_interface.h>
#include <cav_driver_utils/synchronized_async_process_queue.h>
#include <cav_driver_utils/can/error_code.h>

#include <boost/endian/conversion.hpp>
#include <boost/signals2/signal.hpp>

#include <limits>

class TruckDBWController
{
//region CustomTypes
public:

    /** ---------- SEND MESSAGES ---------- **/
    // PropB_10 Message Structs
    enum class CommandMode : uint8_t
    {
        DisableACCSystem = 0,
        RoboticWrenchEffortControl = 1,
        RoboticSpeedControl = 2,
        None = 3,
        Reserved = 4

    };

    struct ControlMessage
    {
        float wrench_effort;
        float speed_control;
        float max_accel;
        CommandMode mode;

        ControlMessage() : wrench_effort(0),
                           speed_control(0),
                           max_accel(0),
                           mode(CommandMode::None)
        {

        }

    };

    // PropB_26 Message Structs
    enum class EngineBrakeMsgModeEnum : uint8_t
    {
        PassThrough = 0,
        TorcBypass = 1,
        Error = 2,
        NoChange = 3

    };

    enum class EngineBrakeCommandEnum : uint8_t
    {
        DisableEngineBraking = 0,
        EnableEngineBraking = 1,
        Error = 2,
        NoChange = 3

    };
    enum class EngineBrakeLevelCommandEnum : uint8_t
    {
        Low = 0,
        Medium = 1,
        High = 2,
        NoChange = 3

    };

    struct EngineBrakeCommand
    {

        EngineBrakeMsgModeEnum mode;
        EngineBrakeCommandEnum command;
        EngineBrakeLevelCommandEnum level;

        EngineBrakeCommand()
            : mode(EngineBrakeMsgModeEnum::PassThrough),
              command(EngineBrakeCommandEnum::DisableEngineBraking),
              level(EngineBrakeLevelCommandEnum::Low)
        {

        }

        inline bool operator==(const EngineBrakeCommand &rhs) const
        {
            return ((mode == rhs.mode) &&
                (command == rhs.command) &&
                (level == rhs.level));
        }
        inline bool operator!=(const EngineBrakeCommand &rhs) const
        {
            return !(*this == rhs);
        }

    };

    // PropB_31 Message Struct
    struct PidParams
    {
        int16_t kp;
        int16_t ki;
        int16_t kd;
        int16_t divisor;

        inline bool operator==(const PidParams &rhs) const
        {
            return ((kp == rhs.kp) && (ki == rhs.ki) && (kd == rhs.kd) && (divisor == rhs.divisor));
        }

        inline bool operator!=(const PidParams &rhs) const
        {
            return !(*this == rhs);
        }

        void setF(double p, double i, double d)
        {
            double max = std::max({p, i, d});
            if (max != 0.0)
            {
                int max_div = boost::math::iround(std::numeric_limits<int16_t>::max() / max);
                divisor = (int16_t) std::min<int>(max_div, std::numeric_limits<int16_t>::max());
            } else
            {
                divisor = std::numeric_limits<int16_t>::max();
            }

            kp = (int16_t) boost::math::iround(p * divisor);
            ki = (int16_t) boost::math::iround(i * divisor);
            kd = (int16_t) boost::math::iround(d * divisor);
        }

    };

    // PropB_ED Message Struct
    enum class GenericStatusEnum : uint8_t
    {
        Off = 0,
        On = 1,
        Error = 2,
        Unknown = 3

    };

    struct LEDStatus
    {
        GenericStatusEnum FaultLed;
        GenericStatusEnum RoboticLed;
        GenericStatusEnum EmoLed;
        GenericStatusEnum AudibleBuzzer;

        LEDStatus() :
            FaultLed(GenericStatusEnum::Off),
            RoboticLed(GenericStatusEnum::Off),
            EmoLed(GenericStatusEnum::Off),
            AudibleBuzzer(GenericStatusEnum::Off)
        {

        }

        inline bool operator==(const LEDStatus &rhs) const
        {
            return ((FaultLed == rhs.FaultLed) && (RoboticLed == rhs.RoboticLed) &&
                (EmoLed == rhs.EmoLed) && (AudibleBuzzer == rhs.AudibleBuzzer));
        }
        inline bool operator!=(const LEDStatus &rhs) const
        {
            return !(*this == rhs);
        }

    };

    enum class LightId
    {
        Front,
        Rear,
        Trailer

    };

    // Light Status Message Struct
    struct LightStatus
    {
        GenericStatusEnum Flashing;
        GenericStatusEnum LeftBlinker;
        GenericStatusEnum RightBlinker;
        GenericStatusEnum TakeDown;
        GenericStatusEnum GreenFlash;
        GenericStatusEnum GreenSolid;

        LightStatus() :
            Flashing(GenericStatusEnum::Off),
            LeftBlinker(GenericStatusEnum::Off),
            RightBlinker(GenericStatusEnum::Off),
            TakeDown(GenericStatusEnum::Off),
            GreenFlash(GenericStatusEnum::Off),
            GreenSolid(GenericStatusEnum::Off)
        {

        }

        inline bool operator==(const LightStatus &rhs) const
        {
            return ((Flashing == rhs.Flashing) && (LeftBlinker == rhs.LeftBlinker) && (RightBlinker == rhs.RightBlinker)
                &&
                    (TakeDown == rhs.TakeDown) && (GreenFlash == rhs.GreenFlash) && (GreenSolid == rhs.GreenSolid));
        }
        inline bool operator!=(const LightStatus &rhs) const
        {
            return !(*this == rhs);
        }

    };

    /** ---------- RECV MESSAGES ---------- **/

    // PropB_11 Message Struct
    enum class RoboticMode : uint8_t
    {
        ManualMode = 0,
        RoboticMode = 1,
        DegradedRoboticMode = 2,
        RoboticThrottleOverride = 3
    };




    struct RoboticMode_PedalPosition
    {
        float gen_accel_pedal_percent;
        float exp_accel_pedal_percent;
        float phy_accel_pedal_percent;
        RoboticMode status;

    };


    // PropB_12 Message Struct
    enum class ControlModeEcho : uint8_t
    {
        DisableACCSystem = 0,
        RoboticWrenchCtrl = 1,
        RoboticSpeedCtrl = 2,
        Reserved = 3

    };


    struct ControlMessageEcho
    {
        float wrench_effort;
        float speed_ctrl;
        float max_accel;
        ControlModeEcho mode;

    };


    // PropB_20 Message Struct
    enum class PrimAxiomaticHealthStatus : uint8_t
    {
        Ok = 0,
        Fault = 1,
        CommsLost = 2,
        NoChange = 3

    };


    struct PrimaryBrakeAxiomatic
    {
        float sys_air_pres;
        float robotic_truck_brake_pres;
        float robotic_trailer_brake_pres;
        float cmd_brake_appy_level;
        PrimAxiomaticHealthStatus status;

    };


    // PropB_21 Message Struct
    enum class SecAxiomaticHealthStatus : uint8_t
    {
        Ok = 0,
        Fault = 1,
        CommsLost = 2,
        NoChange = 3

    };


    struct SecondaryBrakeAxiomatic
    {
        float sys_air_pres;
        float robotic_truck_brake_pres;
        float robotic_trailer_brake_pres;
        float cmd_brake_appy_level;
        SecAxiomaticHealthStatus status;

    };


    // PropB_27 Message Struct
    enum class EngineBrakeMsgMode : uint8_t
    {
        PassThru = 0,
        TorcBypass = 1,
        Error = 2,
        NoChange = 3

    };


    enum class EngineBrakeCmd : uint8_t
    {
        DisengageEngineBrake = 0,
        EngangeEngineBrake = 1,
        Error = 2,
        NoChange = 3

    };


    enum class EngineBrakeLevel : uint8_t
    {
        Low = 0,
        Medium = 1,
        High = 2,
        NoChange = 3

    };


    struct EngineBrakeControl
    {
        EngineBrakeMsgMode eng_brake_msg_mode;
        EngineBrakeCmd gen_eng_brake_cmd;
        EngineBrakeCmd exp_eng_brake_cmd;
        EngineBrakeCmd phy_eng_brake_cmd;
        EngineBrakeLevel gen_eng_brake_level;
        EngineBrakeLevel exp_eng_brake_level;
        EngineBrakeLevel phy_eng_brake_level;

    };


    // PropB_30 Message Struct
    struct SpeedControllerPIDParams
    {
        int32_t p;
        int32_t i;
        int32_t d;
        int32_t divisor;
    };


    // PropB_31 Message Struct
    struct SpeedControllerPIDEcho
    {
        int32_t p;
        int32_t i;
        int32_t d;
        int32_t divisor;

    };


    // PropB_F0 Message Struct
    struct GeneralECUStatus
    {
        float module_temp;
        float module_in_voltage;
        float module_max_loop_time;
        float module_avg_loop_time;

    };


    // PropB_F1 Message Struct
    struct RawInputChannel
    {
        float ain_1;
        float ain_2;
        float ain_3;
        float ain_4;

    };


    // PropB_F2 Message Struct
    struct RawOutputChannel
    {
        float aout_1;
        float aout_2;
        float aout_3;
        float aout_4;

    };


    // PropB_F4 Message Struct
    struct ManualConditionChecks
    {
        uint8_t service_brake;
        uint8_t door_ajar;
        uint8_t parking_brake;
        uint8_t gear_not_manual;
    };


    // PropB_F5 Message Struct
    struct BrakeFaultConditionChecks
    {
        uint8_t air_loss_primary;
        uint8_t air_loss_secondary;
        uint8_t valve_failure_primary;
        uint8_t valve_failure_secondary;
        uint8_t controller_power_primary;
        uint8_t controller_power_secondary;
        uint8_t controller_comms_primary;
        uint8_t controller_comms_secondary;
    };


    // PropB_F6 Message Struct
    struct EmergencyFaultConditionChecks
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
    };


    // PropB_ED Message Struct
    enum class LEDStates : uint8_t
    {
        Off = 0,
        On = 1,
        Error = 2,
        Unknown = 3

    };


    struct LEDStatusEcho
    {
        LEDStates fault_led;
        LEDStates robotic_led;
        LEDStates emo_led;
        LEDStates audible_buzzer;
    };


    // PropB_FC Message Struct
    struct SettingsCrc
    {
        uint32_t nvram_settings_crc;
        uint32_t ram_settings_crc;

    };

//endregion


public:
    /**
     * @brief Signal called when device sends an error
     */
    boost::signals2::signal<void(const boost::system::error_code &)> onError;

    /**
     * @brief Signal called when device sends CAN error
     */
    boost::signals2::signal<void(const cav::can::ErrorCode_t &)> onCANError;

    /**
     * @brief Constructs a TruckCanClient
     * @param device - shared_ptr to a device that implements cav::CANInterface
     */
    TruckDBWController(std::shared_ptr<cav::CANInterface> device);
    TruckDBWController(const TruckDBWController &) = delete;
    TruckDBWController(TruckDBWController &&) = delete;
    ~TruckDBWController();

    /**
     * @brief Initializes the device and begins listening for frames
     */
    void initialize();


    //Callbacks signaled on receipt of message on bus.
    boost::signals2::signal<void(TruckDBWController::RoboticMode_PedalPosition)> onRoboticMode_PedalPercentageRecv;
    boost::signals2::signal<void(TruckDBWController::ControlMessageEcho)> onControlMessageEchoRecv;
    boost::signals2::signal<void(TruckDBWController::PrimaryBrakeAxiomatic)> onPrimaryBrakeAxiomaticRecv;
    boost::signals2::signal<void(TruckDBWController::SecondaryBrakeAxiomatic)> onSecondaryBrakeAxiomaticRecv;
    boost::signals2::signal<void(TruckDBWController::EngineBrakeControl)> onEngineBrakeControlFeedbackRecv;
    boost::signals2::signal<void(TruckDBWController::SpeedControllerPIDParams)> onSpeedControllerPidRecv;
    boost::signals2::signal<void(TruckDBWController::SpeedControllerPIDEcho)> onSpeedConrollerPidEchoRecv;
    boost::signals2::signal<void(TruckDBWController::GeneralECUStatus)> onGeneralEcuStatusRecv;
    boost::signals2::signal<void(TruckDBWController::RawInputChannel)> onRawInputChannelRecv;
    boost::signals2::signal<void(TruckDBWController::RawOutputChannel)> onRawOutputChannelRecv;
    boost::signals2::signal<void(TruckDBWController::ManualConditionChecks)> onManualConditionChecksRecv;
    boost::signals2::signal<void(TruckDBWController::BrakeFaultConditionChecks)> onBrakeFaultChecksRecv;
    boost::signals2::signal<void(TruckDBWController::EmergencyFaultConditionChecks)> onEmergencyFaultChecksRecv;
    boost::signals2::signal<void(TruckDBWController::LEDStatusEcho)> onLedStatusEchoRecv;
    boost::signals2::signal<void(TruckDBWController::SettingsCrc)> onSettingsCrcRecv;

    /**
     * @brief Set functions for fields within sent messages
     */
    void sendWrenchEffortCommand(float wrench_effort);
    void sendSpeedAccelCommand(float speed_ctrl, float max_accel);
    void sendDisableRoboticControl();
    void sendEngineBrakeCommand(EngineBrakeCommand engine_brake_msg);
    void sendPidParam(PidParams params);
    void sendLEDStatus(LEDStatus led_status);
    void sendLightStatus(LightId id, LightStatus light_status_struct);
    void sendClearFaults();

private:

    /**
    *  @Brief Struct containing all messages received
    */
    struct TruckDBWRecvMessages
    {
        truck::PropB_11_Message propB_11_message;
        truck::PropB_12_Message propB_12_message;
        truck::PropB_20_Message propB_20_message;
        truck::PropB_21_Message propB_21_message;
        truck::PropB_27_Message propB_27_message;
        truck::PropB_30_Message propB_30_message;
        truck::PropB_31_Message propB_31_message;
        truck::PropB_F0_Message propB_F0_message;
        truck::PropB_F1_Message propB_F1_message;
        truck::PropB_F2_Message propB_F2_message;
        truck::PropB_F4_Message propB_F4_message;
        truck::PropB_F5_Message propB_F5_message;
        truck::PropB_F6_Message propB_F6_message;
        truck::PropB_ED_Message propB_ED_message;
        truck::PropB_FC_Message propB_FC_message;
    };

    /**
    *  @Brief Struct containing all messages received
    */
    struct TruckDBWSendMessages
    {
        truck::PropB_10_Message propB_10_message;
        truck::PropB_26_Message propB_26_message;
        truck::PropB_31_Message propB_31_message;
        truck::Request_Message request_message;
        truck::PropA_F1D0_Message propA_F1D0_message;
        truck::PropB_ED_Message propB_ED_message;
        truck::Light_Status_Message light_status_message;
        truck::PropA_F1D0_Message clear_faults_message;
    };

    /**
     * @brief Pertinent CAN Ids on the bus
     */
    std::vector<uint32_t> can_ids =
        {
            0x0CFF115A,     // PropB_11 FEEDBACK - PGN 65297 from Throttle Controller (90)
            0x0CFF125A,     // PropB_12 FEEDBACK - PGN 65298 from Throttle Controller (90)
            0x0CFF205A,     // PropB_20 FEEDBACK - PGN 65312 from Throttle Controller (90)
            0x0CFF215A,     // PropB_21 FEEDBACK - PGN 65313 from Throttle Controller (90)
            0x0CFF2728,     // PropB_27 FEEDBACK - PGN 65319 from Aux Controller (40)
            0x0CFF3027,     // PropB_30 FEEDBACK - PGN 65328 from Control SW (39)
            0x0CFF315A,     // PropB_31 FEEDBACK - PGN 65329 from Throttle Control (90)
            0x0CFFF05A,     // PropB_F0 FEEDBACK - PGN 65520 from Throttle Control (90)
            0x0CFFF15A,     // PropB_F1 FEEDBACK - PGN 65521 from Throttle Control (90)
            0x0CFFF25A,     // PropB_F2 FEEDBACK - PGN 65522 from Throttle Control (90)
            0x0CFFF4FF,     // PropB_F4 FEEDBACK - PGN 65524 from All Modules
            0x0CFFF5FF,     // PropB_F5 FEEDBACK - PGN 65525 from All Modules
            0x0CFFF6FF,     // PropB_F6 FEEDBACK - PGN 65526 from All Modules
            0x0CFFED27,     // PropB_ED FEEDBACK - PGN 65517 from Control SW (39)
            0x0CFFFCFF,     // PropB_FC FEEDBACK - PGN 65532 from All Modules
            0x0CEA2127,     // TP Request - Only Recv Not Handled
        };

    std::shared_ptr<cav::CANInterface> device_; // Device to send and receive messages on

    /**
     * Use Async Process Queue to call process function for incoming messages
     */
    typedef cav::SynchronizedAsyncProcessQueue<std::shared_ptr<cav::CANFrameStamped const>> _sync_q_type;
    std::unique_ptr<_sync_q_type> process_q_;

    TruckDBWRecvMessages truck_recv_;   // Instance of Messages being Received
    TruckDBWSendMessages truck_send_;   // Instance of Messages being Sent

    /**
     * @brief Handles a frame received. The frame is queued into an asynchronous process queue to avoid
     * congesting the receive thread and to separate the work
     * @param msg
     */
    void frameReceivedHandler(std::shared_ptr<cav::CANFrameStamped const> msg);

    /**
     * @brief Handles an error frame received. The frame is queued into an asynchronous process queue to avoid
     * congesting the receive thread and to separate the work
     * @param msg
     */
    void errorFrameReceivedHandler(std::shared_ptr<cav::CANFrameStamped const> msg);

    /**
     * @brief processes can messages, called by the async process queue
     * @param msg
     */
    void process(std::shared_ptr<cav::CANFrameStamped const> msg);

    /**
     * @brief handler for errors signaled by device
     * @param ec
     */
    void errorReceivedHandler(const boost::system::error_code &ec);

    RoboticMode_PedalPosition convertPropB11();
    ControlMessageEcho convertPropB12();
    PrimaryBrakeAxiomatic convertPropB20();
    SecondaryBrakeAxiomatic convertPropB21();
    EngineBrakeControl convertPropB27();
    SpeedControllerPIDParams convertPropB30();
    SpeedControllerPIDEcho convertPropB31();
    GeneralECUStatus convertPropBF0();
    RawInputChannel convertPropBF1();
    RawOutputChannel convertPropBF2();
    ManualConditionChecks convertPropBF4();
    BrakeFaultConditionChecks convertPropBF5();
    EmergencyFaultConditionChecks convertPropBF6();
    LEDStatusEcho convertPropBED();
    SettingsCrc convertPropBFC();
};


inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::RoboticMode& mode)
{
    switch(mode)
    {
        case TruckDBWController::RoboticMode::RoboticMode:
            return os << "Robotic Mode";

        case TruckDBWController::RoboticMode::DegradedRoboticMode:
            return os << "Degraded Robotic Mode";

        case TruckDBWController::RoboticMode::ManualMode:
            return os << "Manual Mode";

        case TruckDBWController::RoboticMode::RoboticThrottleOverride:
            return os << "Robotic Throttle Override";

        default:
            return os << "Invalid";
    }
}

inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::ControlModeEcho& mode)
{
    switch(mode)
    {
        case TruckDBWController::ControlModeEcho::DisableACCSystem:
            return os << "Disable ACC System";

        case TruckDBWController::ControlModeEcho::RoboticWrenchCtrl:
            return os << "Robotic Wrench Effort";

        case TruckDBWController::ControlModeEcho::RoboticSpeedCtrl:
            return os << "Robotic Speed Control";

        default:
            return os << "Invalid";
    }
}


inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::PrimAxiomaticHealthStatus & mode)
{
    switch(mode)
    {
        case TruckDBWController::PrimAxiomaticHealthStatus::Ok:
            return os << "Ok";

        case TruckDBWController::PrimAxiomaticHealthStatus::NoChange:
            return os << "No Change";

        case TruckDBWController::PrimAxiomaticHealthStatus::CommsLost:
            return os << "Comms Lost";

        case TruckDBWController::PrimAxiomaticHealthStatus::Fault:
            return os << "Fault";

        default:
            return os << "Invalid";
    }
}

inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::SecAxiomaticHealthStatus & mode)
{
    switch(mode)
    {
        case TruckDBWController::SecAxiomaticHealthStatus::Ok:
            return os << "Ok";

        case TruckDBWController::SecAxiomaticHealthStatus::NoChange:
            return os << "No Change";

        case TruckDBWController::SecAxiomaticHealthStatus::CommsLost:
            return os << "Comms Lost";

        case TruckDBWController::SecAxiomaticHealthStatus::Fault:
            return os << "Fault";

        default:
            return os << "Invalid";
    }
}

inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::EngineBrakeMsgMode & mode)
{
    switch(mode)
    {
        case TruckDBWController::EngineBrakeMsgMode::TorcBypass:
            return os << "Torc Bypass";

        case TruckDBWController::EngineBrakeMsgMode::NoChange:
            return os << "No Change";

        case TruckDBWController::EngineBrakeMsgMode::PassThru:
            return os << "Pass Through";

        case TruckDBWController::EngineBrakeMsgMode::Error:
            return os << "Error";

        default:
            return os << "Invalid";
    }
}

inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::EngineBrakeCmd & mode)
{
    switch(mode)
    {
        case TruckDBWController::EngineBrakeCmd::EngangeEngineBrake:
            return os << "Engage Engine Brake";

        case TruckDBWController::EngineBrakeCmd::NoChange:
            return os << "No Change";

        case TruckDBWController::EngineBrakeCmd::DisengageEngineBrake:
            return os << "Disengage Engine Brake";

        case TruckDBWController::EngineBrakeCmd::Error:
            return os << "Error";

        default:
            return os << "Invalid";
    }
}

inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::EngineBrakeLevel & mode)
{
    switch(mode)
    {
        case TruckDBWController::EngineBrakeLevel::High:
            return os << "High";

        case TruckDBWController::EngineBrakeLevel::NoChange:
            return os << "No Change";

        case TruckDBWController::EngineBrakeLevel::Medium:
            return os << "Medium";

        case TruckDBWController::EngineBrakeLevel::Low:
            return os << "Low";

        default:
            return os << "Invalid";
    }
}

inline std::ostream& operator<<(std::ostream&os,const TruckDBWController::LEDStates & mode)
{
    switch(mode)
    {
        case TruckDBWController::LEDStates::On:
            return os << "On";

        case TruckDBWController::LEDStates::Off:
            return os << "Off";

        case TruckDBWController::LEDStates::Unknown:
            return os << "Unknown";

        case TruckDBWController::LEDStates::Error:
            return os << "Error";

        default:
            return os << "Invalid";
    }
}