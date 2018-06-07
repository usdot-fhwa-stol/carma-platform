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

#include "truck_dbw_controller.h"
#include <cmath>
#include <utility>

TruckDBWController::TruckDBWController(std::shared_ptr<cav::CANInterface> device) :
    device_(std::move(device)),
    process_q_(new _sync_q_type(std::bind(&TruckDBWController::process, this, std::placeholders::_1)))
{
}

TruckDBWController::~TruckDBWController()
{
    device_.reset();
    process_q_.reset();
}

void TruckDBWController::initialize()
{
    device_->setFilters(can_ids);
    device_->onFrameReceived.connect(std::bind(&TruckDBWController::frameReceivedHandler, this, std::placeholders::_1));
    device_->onErrorFrameReceived.connect(std::bind(&TruckDBWController::errorFrameReceivedHandler, this, std::placeholders::_1));
    device_->onError.connect(std::bind(&TruckDBWController::errorReceivedHandler, this, std::placeholders::_1));
}

// PropB_10 Messages
void TruckDBWController::sendWrenchEffortCommand(float wrench_effort)
{
	if(wrench_effort < -100.0)	// Check Lower Bound
	{
		wrench_effort = -100.0;	// Latch to Lower Bound
	}
	else if(wrench_effort > 100.0)	// Check Upper Bound
	{
		wrench_effort = 100.0;		// Latch to Upper Bound
	}
	truck_send_.propB_10_message.setWrenchEffort(wrench_effort);
	truck_send_.propB_10_message.setSpeedControl(0.0);	// Reset Speed Control -> NOT USING IT
	truck_send_.propB_10_message.setMaxAccel(0.0);	// Reset Max Accel -> NOT USING IT

	truck_send_.propB_10_message.setCtrlCmdMode(truck::ControlTypeCommandMode::Robotic_WrenchEffort_Ctrl);
	device_->write(truck_send_.propB_10_message.packData());
}
void TruckDBWController::sendSpeedAccelCommand(float speed_ctrl, float max_accel)
{
	truck_send_.propB_10_message.setWrenchEffort(0.0);	// Reset Wrench Effort -> NOT USING IT
	truck_send_.propB_10_message.setSpeedControl(speed_ctrl);
	truck_send_.propB_10_message.setMaxAccel(max_accel);

	truck_send_.propB_10_message.setCtrlCmdMode(truck::ControlTypeCommandMode::Robotic_Speed_Ctrl);
	device_->write(truck_send_.propB_10_message.packData());
}
void TruckDBWController::sendDisableRoboticControl()
{
	// Reset all values and disable ACC System
	truck_send_.propB_10_message.setWrenchEffort(0.0);
	truck_send_.propB_10_message.setSpeedControl(0.0);
	truck_send_.propB_10_message.setMaxAccel(0.0);

	truck_send_.propB_10_message.setCtrlCmdMode(truck::ControlTypeCommandMode::Disable_ACC_System);
	device_->write(truck_send_.propB_10_message.packData());
}

// PropB_26 Messages
void TruckDBWController::sendEngineBrakeCommand(EngineBrakeCommand engine_brake_msg)
{
	truck_send_.propB_26_message.setEngBrakeCmdMode(static_cast<truck::EngineBrakeMsgModeCmd>(engine_brake_msg.mode));
	truck_send_.propB_26_message.setEngBrakeCmd(static_cast<truck::EngineBrakeCmd>(engine_brake_msg.command));
	truck_send_.propB_26_message.setEngBrakeLevelCmd(static_cast<truck::EngineBrakeLevelCmd>(engine_brake_msg.level));
	
	device_->write(truck_send_.propB_26_message.packData());
}

// PropB_31 Messages
void TruckDBWController::sendPidParam(PidParams params)
{
	truck_send_.propB_31_message.setProportionalGainCompNum(params.kp);
	truck_send_.propB_31_message.setIntegratorGainCompNum(params.ki);
	truck_send_.propB_31_message.setDerivativeGainCompNum(params.kd);
	truck_send_.propB_31_message.setPIDSharedDivisor(params.divisor);

	device_->write(truck_send_.propB_31_message.packData());
}

// PropB_ED Messages
void TruckDBWController::sendLEDStatus(LEDStatus led_status)
{
	truck_send_.propB_ED_message.setFaultLEDState(static_cast<truck::GenericStates>(led_status.FaultLed));
	truck_send_.propB_ED_message.setRoboticLEDState(static_cast<truck::GenericStates>(led_status.RoboticLed));
	truck_send_.propB_ED_message.setEmoLEDState(static_cast<truck::GenericStates>(led_status.EmoLed));
	truck_send_.propB_ED_message.setAudibleBuzzerState(static_cast<truck::GenericStates>(led_status.AudibleBuzzer));

	device_->write(truck_send_.propB_ED_message.packData());
}

// Light Status Messages
void TruckDBWController::sendLightStatus(LightId id, LightStatus light_status_struct)
{

    int32_t light_message_id;
    switch(id)
    {
        case LightId::Front:
            light_message_id = 0x50A;
            break;
        case LightId::Rear:
            light_message_id = 0x50B;
            break;
        case LightId::Trailer:
            break;
    }

	truck_send_.light_status_message.setFlashingState(static_cast<truck::GenericStates>(light_status_struct.Flashing));
	truck_send_.light_status_message.setLeftBlinkerState(static_cast<truck::GenericStates>(light_status_struct.LeftBlinker));
	truck_send_.light_status_message.setRightBlinkerState(static_cast<truck::GenericStates>(light_status_struct.RightBlinker));
	truck_send_.light_status_message.setTakeDownState(static_cast<truck::GenericStates>(light_status_struct.TakeDown));
	truck_send_.light_status_message.setGreenFlasherState(static_cast<truck::GenericStates>(light_status_struct.GreenFlash));
	truck_send_.light_status_message.setGreenSolidState(static_cast<truck::GenericStates>(light_status_struct.GreenSolid));

	device_->write(truck_send_.light_status_message.packData(light_message_id));
}

// Clear Faults Messages
void TruckDBWController::sendClearFaults()
{
    truck_send_.clear_faults_message.setClearFaultsMode(truck::ClearFaultsMode::Clear_Faults_Enabled);
    device_->write(truck_send_.clear_faults_message.packData());
}

//***********************************************************************************//

void TruckDBWController::frameReceivedHandler(std::shared_ptr<cav::CANFrameStamped const> msg)
{
    process_q_->push(msg);
}

void TruckDBWController::errorFrameReceivedHandler(std::shared_ptr<cav::CANFrameStamped const> msg)
{
    process_q_->push(msg);
}

void TruckDBWController::process(const std::shared_ptr<cav::CANFrameStamped const> msg)
{
    // Check if current msg contains error
    if(msg->is_error)
    {
        cav::can::ErrorCode_t ec;
        ec.code = msg->id;
        onCANError(ec);
        return;
    }

    // Switch Statement to Properly hand Msg based on ID
    switch(msg->id | 0xFF)  // TODO : Look into this
    {
        case 0x0CFF11FF: // PropB_11 FEEDBACK - PGN 65297 from Throttle Controller (90)
        {
            truck_recv_.propB_11_message.setData(msg->data);
            onRoboticMode_PedalPercentageRecv(convertPropB11());
            break;
        }
        case 0x0CFF12FF: // PropB_12 FEEDBACK - PGN 65298 from Throttle Controller (90)
        {
            truck_recv_.propB_12_message.setData(msg->data);
            onControlMessageEchoRecv(convertPropB12());
            break;
        }
        case 0x0CFF20FF: // PropB_20 FEEDBACK - PGN 65312 from Throttle Controller (90)
        {
            truck_recv_.propB_20_message.setData(msg->data);
            onPrimaryBrakeAxiomaticRecv(convertPropB20());
            break;
        }
        case 0x0CFF21FF: // PropB_21 FEEDBACK - PGN 65313 from Throttle Controller (90)
        {
            truck_recv_.propB_21_message.setData(msg->data);
            onSecondaryBrakeAxiomaticRecv(convertPropB21());
            break;
        }
        case 0x0CFF27FF: // PropB_27 FEEDBACK - PGN 65319 from Aux Controller (40)
        {
            truck_recv_.propB_27_message.setData(msg->data);
            onEngineBrakeControlFeedbackRecv(convertPropB27());
            break;
        }
        case 0x0CFF30FF: // PropB_30 FEEDBACK - PGN 65328 from Control SW (39)
        {
            truck_recv_.propB_30_message.setData(msg->data);
            onSpeedControllerPidRecv(convertPropB30());
            break;
        }
        case 0x0CFF31FF: // PropB_31 FEEDBACK - PGN 65329 from Throttle Control (90)
        {
            truck_recv_.propB_31_message.setData(msg->data);
            onSpeedConrollerPidEchoRecv(convertPropB31());
            break;
        }
        case 0x0CFFF0FF: // PropB_F0 FEEDBACK - PGN 65520 from Throttle Control (90)
        {
            truck_recv_.propB_F0_message.setData(msg->data);
            onGeneralEcuStatusRecv(convertPropBF0());
            break;
        }
        case 0x0CFFF1FF: // PropB_F1 FEEDBACK - PGN 65521 from Throttle Control (90)
        {
            truck_recv_.propB_F1_message.setData(msg->data);
            onRawInputChannelRecv(convertPropBF1());
            break;
        }
        case 0x0CFFF2FF: // PropB_F2 FEEDBACK - PGN 65522 from Throttle Control (90)
        {
            truck_recv_.propB_F2_message.setData(msg->data);
            onRawOutputChannelRecv(convertPropBF2());
            break;
        }
        case 0x0CFFF4FF: // PropB_F4 FEEDBACK - PGN 65524 from ALL Modules
        {
            truck_recv_.propB_F4_message.setData(msg->data);
            onManualConditionChecksRecv(convertPropBF4());
            break;
        }
        case 0x0CFFF5FF: // PropB_F5 FEEDBACK - PGN 65525 from ALL Modules
        {
            truck_recv_.propB_F5_message.setData(msg->data);
            onBrakeFaultChecksRecv(convertPropBF5());
            break;
        }
        case 0x0CFFF6FF: // PropB_F6 FEEDBACK - PGN 65526 from ALL Modules
        {
            truck_recv_.propB_F6_message.setData(msg->data);
            onEmergencyFaultChecksRecv(convertPropBF6());
            break;
        }
        case 0x0CFFEDFF: // PropB_ED FEEDBACK - PGN 65517 from Control SW (39)
        {
            truck_recv_.propB_ED_message.setData(msg->data);
            onLedStatusEchoRecv(convertPropBED());
            break;
        }
        case 0x0CFFFCFF: // PropB_FC FEEDBACK - PGN 65532 from ALL Modules
        {
            truck_recv_.propB_FC_message.setData(msg->data);
            onSettingsCrcRecv(convertPropBFC());
            break;
        }
        case 0x0CEA21FF:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

void TruckDBWController::errorReceivedHandler(const boost::system::error_code &ec)
{
    onError(ec);
}

TruckDBWController::RoboticMode_PedalPosition TruckDBWController::convertPropB11()
{
	RoboticMode_PedalPosition temp;
	temp.gen_accel_pedal_percent = truck_recv_.propB_11_message.getGenAccelPedalPercent();
	temp.exp_accel_pedal_percent = truck_recv_.propB_11_message.getExpAccelPedalPercent();
	temp.phy_accel_pedal_percent = truck_recv_.propB_11_message.getPhyAccelPedalPercent();
	temp.status                  = static_cast<RoboticMode>(truck_recv_.propB_11_message.getRoboticModeStateStatus());
	return temp;
}

TruckDBWController::ControlMessageEcho TruckDBWController::convertPropB12()
{
	ControlMessageEcho temp;
	temp.wrench_effort  = truck_recv_.propB_12_message.getWrenchEffortEcho();
	temp.speed_ctrl     = truck_recv_.propB_12_message.getSpeedControlEcho();
	temp.max_accel      = truck_recv_.propB_12_message.getMaxAccelEcho();
	temp.mode           = static_cast<ControlModeEcho>(truck_recv_.propB_12_message.getControlTypeCmdModeEcho());
	return temp;
}

TruckDBWController::PrimaryBrakeAxiomatic TruckDBWController::convertPropB20()
{
	PrimaryBrakeAxiomatic temp;
	temp.sys_air_pres               = truck_recv_.propB_20_message.getSystemAirPres();
	temp.robotic_truck_brake_pres   = truck_recv_.propB_20_message.getRoboticTruckAppliedBrakePres();
    temp.robotic_trailer_brake_pres = truck_recv_.propB_20_message.getRoboticTrailerAppliedBrakePres();
	temp.cmd_brake_appy_level       = truck_recv_.propB_20_message.getCmdBrakeAppliedLevel();
	temp.status                     = static_cast<PrimAxiomaticHealthStatus>(truck_recv_.propB_20_message.getAxiomaticHealthStatus());
	return temp;
}

TruckDBWController::SecondaryBrakeAxiomatic TruckDBWController::convertPropB21()
{
	SecondaryBrakeAxiomatic temp;
	temp.sys_air_pres               = truck_recv_.propB_21_message.getSystemAirPres();
	temp.robotic_truck_brake_pres   = truck_recv_.propB_21_message.getRoboticTruckAppliedBrakePres();
    temp.robotic_trailer_brake_pres = truck_recv_.propB_21_message.getRoboticTrailerAppliedBrakePres();
	temp.cmd_brake_appy_level       = truck_recv_.propB_21_message.getCmdBrakeAppliedLevel();
	temp.status                     = static_cast<SecAxiomaticHealthStatus>(truck_recv_.propB_21_message.getAxiomaticHealthStatus());
	return temp;
}

TruckDBWController::EngineBrakeControl TruckDBWController::convertPropB27()
{
	EngineBrakeControl temp;
	temp.eng_brake_msg_mode     = static_cast<EngineBrakeMsgMode>(truck_recv_.propB_27_message.getEngineBrakeMsgModeFb());
	temp.gen_eng_brake_cmd      = static_cast<EngineBrakeCmd>(truck_recv_.propB_27_message.getGenEngineBrakeCmd());
	temp.exp_eng_brake_cmd      = static_cast<EngineBrakeCmd>(truck_recv_.propB_27_message.getExpEngineBrakeCmd());
	temp.phy_eng_brake_cmd      = static_cast<EngineBrakeCmd>(truck_recv_.propB_27_message.getPhyEngineBrakeCmd());
	temp.gen_eng_brake_level    = static_cast<EngineBrakeLevel>(truck_recv_.propB_27_message.getGenEngineBrakeLevelCmd());
	temp.exp_eng_brake_level    = static_cast<EngineBrakeLevel>(truck_recv_.propB_27_message.getExpEngineBrakeLevelCmd());
	temp.phy_eng_brake_level    = static_cast<EngineBrakeLevel>(truck_recv_.propB_27_message.getPhyEngineBrakeLevelCmd());
	return temp;
}

TruckDBWController::SpeedControllerPIDParams TruckDBWController::convertPropB30()
{
	SpeedControllerPIDParams temp;
	temp.p = truck_recv_.propB_30_message.getProportionalError();
	temp.i = truck_recv_.propB_30_message.getIntegratorError();
	temp.d = truck_recv_.propB_30_message.getDerivativeError();
	return temp;
}

TruckDBWController::SpeedControllerPIDEcho TruckDBWController::convertPropB31()
{
	SpeedControllerPIDEcho temp;
	temp.p = truck_recv_.propB_31_message.getProportionalGainCompNum();
	temp.i = truck_recv_.propB_31_message.getIntegratorGainCompNum();
	temp.d = truck_recv_.propB_31_message.getDerivativeGainCompNum();
	temp.divisor = truck_recv_.propB_31_message.getPIDSharedDivisor();
	return temp;
}

TruckDBWController::GeneralECUStatus TruckDBWController::convertPropBF0()
{
	GeneralECUStatus temp;
	temp.module_temp = truck_recv_.propB_F0_message.getModuleTemperature();
	temp.module_in_voltage = truck_recv_.propB_F0_message.getModuleInputVoltage();
	temp.module_max_loop_time = truck_recv_.propB_F0_message.getModuleMaxLoopTime();
	temp.module_avg_loop_time = truck_recv_.propB_F0_message.getModuleAvgLoopTime();
	return temp;
}

TruckDBWController::RawInputChannel TruckDBWController::convertPropBF1()
{
	RawInputChannel temp;
	temp.ain_1 = truck_recv_.propB_F1_message.getAIn1Raw();
	temp.ain_2 = truck_recv_.propB_F1_message.getAIn2Raw();
	temp.ain_3 = truck_recv_.propB_F1_message.getAIn3Raw();
	temp.ain_4 = truck_recv_.propB_F1_message.getAIn4Raw();
	return temp;
}

TruckDBWController::RawOutputChannel TruckDBWController::convertPropBF2()
{
	RawOutputChannel temp;
	temp.aout_1 = truck_recv_.propB_F2_message.getAOut1Raw();
	temp.aout_2 = truck_recv_.propB_F2_message.getAOut2Raw();
	temp.aout_3 = truck_recv_.propB_F2_message.getAOut3Raw();
	temp.aout_4 = truck_recv_.propB_F2_message.getAOut4Raw();
	return temp;
}

TruckDBWController::ManualConditionChecks TruckDBWController::convertPropBF4()
{
	ManualConditionChecks temp;
    temp.service_brake = truck_recv_.propB_F4_message.service_brake;
    temp.door_ajar = truck_recv_.propB_F4_message.door_ajar;
    temp.parking_brake = truck_recv_.propB_F4_message.parking_brake;
    temp.gear_not_manual = truck_recv_.propB_F4_message.gear_not_manual;
	
	return temp;
}

TruckDBWController::BrakeFaultConditionChecks TruckDBWController::convertPropBF5()
{
	BrakeFaultConditionChecks temp;
    temp.air_loss_primary = truck_recv_.propB_F5_message.air_loss_primary;
    temp.air_loss_secondary = truck_recv_.propB_F5_message.air_loss_secondary;
    temp.valve_failure_primary= truck_recv_.propB_F5_message.valve_failure_primary;
    temp.valve_failure_secondary= truck_recv_.propB_F5_message.valve_failure_secondary;
    temp.controller_power_primary= truck_recv_.propB_F5_message.controller_power_primary;
    temp.controller_power_secondary= truck_recv_.propB_F5_message.controller_power_secondary;
    temp.controller_comms_primary= truck_recv_.propB_F5_message.controller_comms_primary;
    temp.controller_comms_secondary= truck_recv_.propB_F5_message.controller_comms_secondary;
	
	return temp;
}

TruckDBWController::EmergencyFaultConditionChecks TruckDBWController::convertPropBF6()
{

	EmergencyFaultConditionChecks temp;
    temp.firmware_load = truck_recv_.propB_F6_message.firmware_load ;
    temp.settings_invalid = truck_recv_.propB_F6_message.settings_invalid ;
    temp.vehicle_can2_error_passive = truck_recv_.propB_F6_message.vehicle_can2_error_passive ;
    temp.vehicle_can2_bus_off = truck_recv_.propB_F6_message.vehicle_can2_bus_off ;
    temp.dbw_can1_error_passive = truck_recv_.propB_F6_message.dbw_can1_error_passive ;
    temp.dbw_can1_bus_off = truck_recv_.propB_F6_message.dbw_can1_bus_off ;
    temp.vbus_low_voltage = truck_recv_.propB_F6_message.vbus_low_voltage ;
    temp.system_reset_watchdog = truck_recv_.propB_F6_message.system_reset_watchdog ;
    temp.uncalibrated_settings = truck_recv_.propB_F6_message.uncalibrated_settings ;
    temp.vehicle_speed_msg_expirerd = truck_recv_.propB_F6_message.vehicle_speed_msg_expirerd ;
    temp.brake_pressure_error_threshold = truck_recv_.propB_F6_message.brake_pressure_error_threshold ;
    temp.brake_emergency_fault = truck_recv_.propB_F6_message.brake_emergency_fault ;
    temp.aux_throt_module_timeout = truck_recv_.propB_F6_message.aux_throt_module_timeout ;
    temp.accel_sync_signal_lost = truck_recv_.propB_F6_message.accel_sync_signal_lost ;
    temp.accel_in_sensor_threshold = truck_recv_.propB_F6_message.accel_in_sensor_threshold ;
    temp.accel_out_sensor_threshold = truck_recv_.propB_F6_message.accel_out_sensor_threshold ;
    temp.accel_in_out_mismatch = truck_recv_.propB_F6_message.accel_in_out_mismatch ;
    temp.test_mode_timeout = truck_recv_.propB_F6_message.test_mode_timeout ;
    temp.axiom_can4_error_passive = truck_recv_.propB_F6_message.axiom_can4_error_passive ;
    temp.axiom_can4_bus_off = truck_recv_.propB_F6_message.axiom_can4_bus_off ;
    temp.vehicle_can3_error_passive = truck_recv_.propB_F6_message.vehicle_can3_error_passive ;
    temp.vehicle_can3_bus_off = truck_recv_.propB_F6_message.vehicle_can3_bus_off ;
    temp.command_msg_timeout = truck_recv_.propB_F6_message.command_msg_timeout ;
    temp.axiomatic_msg_timeout = truck_recv_.propB_F6_message.axiomatic_msg_timeout ;
    temp.truck_feedback_msg_timeout = truck_recv_.propB_F6_message.truck_feedback_msg_timeout ;
    
	return temp;
}

TruckDBWController::LEDStatusEcho TruckDBWController::convertPropBED()
{
    LEDStatusEcho temp;
	temp.fault_led = static_cast<LEDStates>(truck_recv_.propB_ED_message.getFaultLEDState());
	temp.robotic_led = static_cast<LEDStates>(truck_recv_.propB_ED_message.getRoboticLEDState());
	temp.emo_led = static_cast<LEDStates>(truck_recv_.propB_ED_message.getEmoLEDState());
	temp.audible_buzzer = static_cast<LEDStates>(truck_recv_.propB_ED_message.getAudibleBuzzerState());
	return temp;
}

TruckDBWController::SettingsCrc TruckDBWController::convertPropBFC()
{
	SettingsCrc temp;
	temp.nvram_settings_crc = truck_recv_.propB_FC_message.getModuleNVRamSettingsCRC();
	temp.ram_settings_crc = truck_recv_.propB_FC_message.getModuleRamSettingsCRC();
	return temp;
}
