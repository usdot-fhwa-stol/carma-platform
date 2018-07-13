#pragma once

#include "delphi_message_definitions.h"

namespace delphi
{
enum class SpeedDirection
{
    Forward = 0,
    Reverse = 1
};

enum class SteeringWheelSign
{
    CounterClockwise = 0,
    Clockwise = 1
};

enum class Validity
{
    UnavailableOrInvalid = 0,
    AvailableAndValid = 1
};

enum class SensorModeCommand
{
    DoNotRadiate = 0,
    Radiate = 1
};

enum class WindShieldWiperStatus
{
    Off = 0,
    On = 1
};

enum class RawDataEnable
{
    Filtered = 0,
    Raw = 1
};


enum class RadiatingStatus
{
    NotRadiating = 0,
    Radiating = 1
};

enum class ErrorStatus
{
    NotFailed = 0,
    Failed = 1
};

enum class BlockedStatus
{
    NotBlocked = 0,
    Blocked = 1
};

enum class OverHeatStatus
{
    NotOverTemp = 0,
    OverTemp = 1
};

enum class GroupingMode
{
    None = 0,
    MovingTargetsOnly = 1,
    StationaryTargetsOnly = 2,
    StationayAndMovingTargets = 3
};

enum class MRLRMode
{
    Reserved = 0,
    MROnly = 1,
    LROnly = 2,
    MRAndLR = 3
};

enum class MedRangeMode
{
    NoUpdate = 0,
    MRUpdate = 1,
    LRUpdate = 2,
    BothMRandLRUpdate = 3
};

enum class TrackStatus
{
    NoTarget = 0,
    NewTarget = 1,
    NewUpdatedTarget = 2,
    UpdatedTarget = 3,
    CoastedTarget = 4,
    MergedTarget = 5,
    InvalidCOastedTarget = 6,
    NewCoastedTarget = 7
};

enum class InterfaceVersion
{
    TDPwith64tracks = 0,
    ADPDV1with64tracks =1,
    ADPDV1forDV1MOART = 2,
    ADPDV2 = 3,
    ADPDV3 = 4,
    ADPPV = 5,
    ADPPV_2011 =6,
    Reserved7 = 7,
    Reserved8 = 8,
    Reserved9 = 9,
    Reserved10 = 10,
    Reserved11 = 11,
    Reserved12 = 12,
    Reserved13 = 13,
    Reserved14 = 14,
    Reserved15 = 15
};

enum class PowerMode
{
    DSPInit = 0,
    RadiateOff = 1,
    RadiateOn = 2,
    DSPShutdown = 3,
    DSPOff = 4,
    HostShutdown = 5,
    Test = 6,
    Invalid = 7
};

enum class AlignmentStatus
{
    Off = 0,
    Bust = 1,
    Success = 2,
    FailNoTarget = 3,
    FailDevTooLarge = 4,
    FailVarTooLarge = 5
};

enum class TurnSignalStatus {
    None = 0,
    Left = 1,
    Right = 2
};

struct DelphiRX4F0
{
    float speed;
    SpeedDirection speed_direction;
    float yaw_rate;
    Validity yaw_rate_validity;
    int32_t radius_curvature;
    uint16_t steering_wheel_angle;
    SteeringWheelSign steering_wheel_sign;
    Validity steering_angle_validity;

    uint16_t steering_wheel_angle_rate;
    SteeringWheelSign steering_wheel_angle_rate_sign;


    DelphiRX4F0() :
            speed(0.0),
            speed_direction(SpeedDirection::Forward),
            yaw_rate(0.0),
            yaw_rate_validity(Validity::UnavailableOrInvalid),
            radius_curvature(8191),
            steering_wheel_angle(0),
            steering_wheel_sign(SteeringWheelSign::CounterClockwise),
            steering_angle_validity(Validity::UnavailableOrInvalid),
            steering_wheel_angle_rate(0),
            steering_wheel_angle_rate_sign(SteeringWheelSign::CounterClockwise)
    {

    }

};

struct DelphiRX4F1
{
    float lateral_mounting_offset;
    float alignment_angle_offset;
    float angle_misalignment;
    Validity vehicle_speed_validity;
    bool mmr_upside_down;
    bool blockage_disable;
    bool use_angle_misalignment;
    bool clear_faults;
    bool lr_transmit;
    bool mr_transmit;
    int32_t short_track_roc;
    TurnSignalStatus turn_signal_status;
    SensorModeCommand radiate_command;
    WindShieldWiperStatus wiper_status;
    RawDataEnable raw_data_enable;
    GroupingMode grouping_mode;
    uint16_t scan_index_ack;
    uint8_t maximum_tracks;
    int8_t high_yaw_angle;

    DelphiRX4F1() :
            scan_index_ack(0),
            lateral_mounting_offset(0.0),
            alignment_angle_offset(0.0),
            angle_misalignment(0.0),
            maximum_tracks(64),
            radiate_command(SensorModeCommand::DoNotRadiate),
            wiper_status(WindShieldWiperStatus::Off),
            raw_data_enable(RawDataEnable::Filtered),
            grouping_mode(GroupingMode::None),
            vehicle_speed_validity(Validity::UnavailableOrInvalid),
            mmr_upside_down(false),
            use_angle_misalignment(false),
            clear_faults(false),
            high_yaw_angle(0),
            lr_transmit(true),
            mr_transmit(true),
            short_track_roc(0)


    {

    }
};

struct ESRStatus1
{
    float vehicle_speed_calc;
    float yaw_rate_calc;
    float radius_curvature_calc;
    bool comm_error;
    uint16_t scan_index;
    uint8_t rolling_count;
    uint8_t dsp_timestamp;

    void setFromDefinition(const ESRStatus1Definition& def)
    {
        scan_index = static_cast<uint16_t>(def.scanIndex());
        rolling_count = static_cast<uint8_t>(def.rollingCount());
        dsp_timestamp = static_cast<uint8_t>(def.dspTimeStamp());
        vehicle_speed_calc = def.vehicleSpeedCalc();
        yaw_rate_calc = def.yawRateCalc();
        radius_curvature_calc = def.radiusCurvatureCalc();
        comm_error = static_cast<bool>(def.commError());
    }


};

struct ESRStatus2
{


    float yaw_rate_bias;
    float veh_spd_comp_factor;
    RadiatingStatus xcvr_operational;
    ErrorStatus internal_error;
    BlockedStatus range_perf_error;
    OverHeatStatus overheat_error;
    RawDataEnable raw_data_mode;
    GroupingMode grouping_mode;
    uint16_t steering_angle_ack;
    uint16_t sw_version;
    int8_t temperature;
    uint8_t rolling_count;
    uint8_t maximum_tracks_ack;

    void setFromDefinition(const ESRStatus2Definition& def)
    {
        yaw_rate_bias = def.yawRateBias();
        veh_spd_comp_factor = def.vehSpdCompFactor();
        xcvr_operational = static_cast<RadiatingStatus>(def.xcvrOperational());
        internal_error = static_cast<ErrorStatus>(def.internalError());
        overheat_error = static_cast<OverHeatStatus>(def.overheatError());
        raw_data_mode = static_cast<RawDataEnable>(def.rawDataMode());
        grouping_mode = static_cast<GroupingMode>(def.groupingMode());
        steering_angle_ack = static_cast<uint16_t>(def.steeringWheelAngleAck());
        sw_version = static_cast<uint16_t>(def.hostSWVersion());
        temperature = static_cast<int8_t>(def.temperature());
        rolling_count = static_cast<uint8_t>(def.rollingCount());
        maximum_tracks_ack = static_cast<uint8_t>(def.maximumTracksAck());

    }


};

struct ESRStatus3
{
    uint32_t host_sw_version;
    uint32_t serial_num;
    InterfaceVersion interface_version;
    uint8_t sw_version_pld;
    uint8_t hardware_version;


    void setFromDefinition(const ESRStatus3Definition& def)
    {
        host_sw_version = def.swVersionHost();
        hardware_version = static_cast<uint8_t>(def.hwVersion());
        interface_version = static_cast<InterfaceVersion>(def.interfaceVersion());
        serial_num = def.serialNum();
        sw_version_pld = static_cast<uint8_t>(def.swVersionPLD());
    }
};


struct ESRStatus4
{
    float auto_align_angle;
    MRLRMode mode;
    bool partial_blockage;
    bool sidelobe_blockage;
    bool LR_only_grating_lobe_detected;
    bool truck_target_detected;
    uint8_t rolling_count;
    uint8_t path_ID_ACC;
    uint8_t path_ID_CMBB_move;
    uint8_t path_ID_CMBB_stat;
    uint8_t path_ID_FCW_move;
    uint8_t path_ID_FCW_stat;

    void setFromDefinition(const ESRStatus4Definition& def)
    {
        rolling_count = static_cast<uint8_t>(def.rollingCount());
        path_ID_ACC = static_cast<uint8_t>(def.pathIDACC());
        path_ID_CMBB_move = static_cast<uint8_t>(def.pathIDCMBBMove());
        path_ID_CMBB_stat = static_cast<uint8_t>(def.pathIDCMBBStat());
        path_ID_FCW_move = static_cast<uint8_t>(def.pathIDFCWMove());
        path_ID_FCW_stat = static_cast<uint8_t>(def.pathIDCMBBStat());
        auto_align_angle = def.autoAlignAngle();
        mode = static_cast<MRLRMode>(def.MRLRMode());
        partial_blockage = static_cast<bool>(def.partialBlockage());
        sidelobe_blockage = static_cast<bool>(def.sidelobeBlockge());
        LR_only_grating_lobe_detected = static_cast<bool>(def.LROnlyGratingLobeDet());
        truck_target_detected = static_cast<bool>(def.truckTargetDet());
    }
};

struct ESRStatus5
{
    uint8_t swbatt_a2d;
    uint8_t ignp_a2d;
    uint8_t temp1_a2d;
    uint8_t temp2_a2d;
    uint8_t supply_5va_a2d;
    uint8_t supply_5vd_a2d;
    uint8_t supply_3p3v_a2d;
    uint8_t supply_10v_a2d;

    void setFromDefinition(const ESRStatus5Definition& def)
    {
        swbatt_a2d = static_cast<uint8_t>(def.swBattA2D());
        ignp_a2d = static_cast<uint8_t>(def.IGNPA2D());
        temp1_a2d = static_cast<uint8_t>(def.temp1A2D());
        temp2_a2d = static_cast<uint8_t>(def.temp2A2D());
        supply_5va_a2d = static_cast<uint8_t>(def.supply5VAA2D());
        supply_5vd_a2d = static_cast<uint8_t>(def.supply5VDXA2D());
        supply_3p3v_a2d = static_cast<uint8_t>(def.supply3P3VA2D());
        supply_10v_a2d = static_cast<uint8_t>(def.supply10vA2D());
    }

};

struct ESRStatus6
{
    float vertical_misalignment;
    float factory_misalignment;
    PowerMode system_power_mode;
    AlignmentStatus factory_align_status1;
    AlignmentStatus factory_align_status2;
    bool recommend_unconverge;
    bool found_target;
    bool vertical_alignment_updated;
    uint8_t service_alignment_updates_done;
    uint8_t sw_version_dsp_3rd_byte;
    uint8_t supply_1p8v_a2d;
    uint8_t supply_n5v_a2d;
    uint8_t wave_diff_a2d;

    void setFromDefinition(const ESRStatus6Definition& def)
    {
        vertical_misalignment = def.verticalMisalignment();
        factory_misalignment = def.factoryMisalignment();
        system_power_mode = static_cast<PowerMode>(def.systemPowerMode());
        factory_align_status1 = static_cast<AlignmentStatus>(def.factoryAlignStatus1());
        factory_align_status2 = static_cast<AlignmentStatus>(def.factorAlignStatus2());
        recommend_unconverge = static_cast<bool>(def.recommendUnconverge());
        found_target = static_cast<bool>(def.foundTarget());
        vertical_alignment_updated = static_cast<bool>(def.verticalAlignUpdated());
        service_alignment_updates_done = static_cast<uint8_t>(def.servAlignUpdatesDone());
        sw_version_dsp_3rd_byte = static_cast<uint8_t>(def.swVersionDSP3rdByte());
        supply_1p8v_a2d = static_cast<uint8_t>(def.supply1P8VA2D());
        supply_n5v_a2d = static_cast<uint8_t>(def.supplyN5VA2D());
        wave_diff_a2d = static_cast<uint8_t>(def.waveDiffA2D());
    }
};

struct ESRStatus7
{
    uint8_t active_fault0;
    uint8_t active_fault1;
    uint8_t active_fault2;
    uint8_t active_fault3;
    uint8_t active_fault4;
    uint8_t active_fault5;
    uint8_t active_fault6;
    uint8_t active_fault7;

    void setFromDefinition(const ESRStatus7Definition& def)
    {
        active_fault0 = static_cast<uint8_t>(def.activeFault0());
        active_fault1 = static_cast<uint8_t>(def.activeFault1());
        active_fault2 = static_cast<uint8_t>(def.activeFault2());
        active_fault3 = static_cast<uint8_t>(def.activeFault3());
        active_fault4 = static_cast<uint8_t>(def.activeFault4());
        active_fault5 = static_cast<uint8_t>(def.activeFault5());
        active_fault6 = static_cast<uint8_t>(def.activeFault6());
        active_fault7 = static_cast<uint8_t>(def.activeFault7());

    }
};

struct ESRStatus8
{
    uint8_t history_fault0;
    uint8_t history_fault1;
    uint8_t history_fault2;
    uint8_t history_fault3;
    uint8_t history_fault4;
    uint8_t history_fault5;
    uint8_t history_fault6;
    uint8_t history_fault7;

    void setFromDefinition(const ESRStatus8Definition& def)
    {
        history_fault0 = static_cast<uint8_t>(def.historyFault0());
        history_fault1 = static_cast<uint8_t>(def.historyFault1());
        history_fault2 = static_cast<uint8_t>(def.historyFault2());
        history_fault3 = static_cast<uint8_t>(def.historyFault3());
        history_fault4 = static_cast<uint8_t>(def.historyFault4());
        history_fault5 = static_cast<uint8_t>(def.historyFault5());
        history_fault6 = static_cast<uint8_t>(def.historyFault6());
        history_fault7 = static_cast<uint8_t>(def.historyFault7());
    }
};

struct ESRStatus9
{
    uint16_t avg_pwr_cwblkg;
    float sideslip_angle;
    uint8_t serial_num_3rd_byte;
    uint8_t water_spray_id;
    float filtered_xohp_of_acc_tgt;
    uint8_t path_id_acc_2;
    uint8_t path_id_acc_3;

    void setFromDefinition(const ESRStatus9Definition def)
    {
        avg_pwr_cwblkg = static_cast<uint16_t>(def.avgPwrCWBlkg());
        sideslip_angle = def.sideslipAngle();
        serial_num_3rd_byte = static_cast<uint8_t>(def.serialNum3rdByte());
        water_spray_id = static_cast<uint8_t>(def.waterSprayTargetId());
        filtered_xohp_of_acc_tgt = def.filteredXOHPACCCIPV();
        path_id_acc_2 = static_cast<uint8_t>(def.pathIDACC2());
        path_id_acc_3 = static_cast<uint8_t>(def.pathIDACC3());
    }

};


struct ESRTrack
{
    uint8_t rolling_count;
    float range;
    float range_rate;
    float range_accel;
    float angle;
    float width;
    bool grouping_changed;
    bool oncoming;
    float lat_rate;
    MedRangeMode mode;
    TrackStatus track_status;
    bool bridge_object;

    bool moving;
    bool movable_fast;
    bool movable_slow;
    int8_t power;

    void setFromDefinition(const ESRTrackDefinition& def)
    {
        rolling_count = static_cast<uint8_t>(def.rollingCount());
        range = def.range();
        range_rate = def.rangeRate();
        range_accel = def.rangeAccel();
        angle = def.angle();
        width = def.width();
        grouping_changed = static_cast<bool>(def.groupingChanged());
        oncoming = static_cast<bool>(def.onComing());
        lat_rate = def.latRate();
        mode = static_cast<MedRangeMode>(def.rangeMode());
        track_status = static_cast<TrackStatus>(def.status());
        bridge_object = static_cast<bool>(def.bridgeObject());
    }

    void setMovePower(const ESRTrackMotionPowerDefinition::TrackMovePower& movePower)
    {
        moving = static_cast<bool>(movePower.moving);
        movable_slow = static_cast<bool>(movePower.movable_slow);
        movable_fast = static_cast<bool>(movePower.movable_fast);
        power = movePower.power;
    }

};


} //namespace delphi