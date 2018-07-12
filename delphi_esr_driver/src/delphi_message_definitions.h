#pragma once

#include <cav_driver_utils/can/can_message_definition.h>

#include <string>

namespace delphi
{

class ESRStatus1Definition
{
private:
    enum names_idx : unsigned int
    {
        DSP_TIMESTAMP,
        COMM_ERROR,
        YAW_RATE_CALC,
        VEHICLE_SPEED_CALC,
        SCAN_INDEX,
        ROLLING_COUNT_1,
        RADIUS_CURVATURE_CALC
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t timestamp_, comm_error_, scan_index_, rolling_count_;
    int32_t radius_curv_calc_;
    float yaw_rate_calc, vehicle_speed_calc_;

public:

    ESRStatus1Definition()
    {
        using namespace cav::can;
        DataField rollingCount(FieldDataTypes::UNSIGNED,63,2,false,1,0);
        DataField dspTimeStamp(FieldDataTypes::UNSIGNED,61,7,false,2,0);
        DataField commError(FieldDataTypes::UNSIGNED,54,1,false,1,0);
        DataField radiusCurvCalc(FieldDataTypes::SIGNED,53,14,false,1,0);
        DataField scanIndex(FieldDataTypes::UNSIGNED,39,16,false,1,0);
        DataField yawRateCalc(FieldDataTypes::SIGNED,23,12,false,0.0625,0);
        DataField vehicleSpeedCalc(FieldDataTypes::UNSIGNED,10,11,false,0.0625,0);

        def_.addFieldDefinition(DSP_TIMESTAMP,dspTimeStamp);
        def_.addFieldDefinition(COMM_ERROR, commError);
        def_.addFieldDefinition(YAW_RATE_CALC,yawRateCalc);
        def_.addFieldDefinition(VEHICLE_SPEED_CALC,vehicleSpeedCalc);
        def_.addFieldDefinition(ROLLING_COUNT_1,rollingCount);
        def_.addFieldDefinition(RADIUS_CURVATURE_CALC,radiusCurvCalc);
        def_.addFieldDefinition(SCAN_INDEX,scanIndex);
    }

    ESRStatus1Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus1Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        timestamp_ = def_.getUnsignedIntValue(DSP_TIMESTAMP);
        comm_error_ = def_.getUnsignedIntValue(COMM_ERROR);
        scan_index_ = def_.getUnsignedIntValue(SCAN_INDEX);
        rolling_count_ = def_.getUnsignedIntValue(ROLLING_COUNT_1);

        radius_curv_calc_ = (int32_t)def_.getUnsignedIntValue(RADIUS_CURVATURE_CALC);
        yaw_rate_calc = def_.getValue(YAW_RATE_CALC);
        vehicle_speed_calc_ = def_.getValue(VEHICLE_SPEED_CALC);
    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline int32_t radiusCurvatureCalc()const { return radius_curv_calc_; }
    inline float yawRateCalc()const { return yaw_rate_calc; }
    inline float vehicleSpeedCalc()const { return vehicle_speed_calc_; }

    inline uint32_t dspTimeStamp()const { return timestamp_; }
    inline uint32_t commError()const { return comm_error_; }
    inline uint32_t scanIndex()const { return scan_index_; }
    inline uint32_t rollingCount()const { return rolling_count_; }
};


class ESRStatus2Definition
{
private:
    enum names_idx : unsigned int
    {
        GROUPING_MODE,
        INTERNAL_ERROR,
        MAXIMUM_TRACKS_ACK,
        OVERHEAT_ERROR,
        RANGE_PERF_ERROR,
        RAW_DATA_MODE,
        ROLLING_COUNT_2,
        STEERING_ANGLE_ACK,
        SW_VERSION_DSP,
        TEMPERATURE,
        VEH_SPD_COMP_FCTOR,
        XCVR_OPERATIONAL,
        YAW_RATE_BIAS
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t maximum_tracks_ack_, rolling_count_2_, overheat_error_, range_perf_error_, internal_error_,xcvr_operational_,raw_data_mode_, host_sw_version_,grouping_mode_, steering_wheel_angle_ack_;
    int32_t temperature_;
    float vehicle_speed_compensation_,yaw_rate_bias_;

public:

    ESRStatus2Definition()
    {
        using namespace cav::can;
        DataField maximumTracksAck(FieldDataTypes::UNSIGNED,63,6,false,1,1);
        DataField rollingCount2(FieldDataTypes::UNSIGNED,57,2,false,1,0);
        DataField overheatError(FieldDataTypes::UNSIGNED,55,1,false,1,0);
        DataField rangePerfError(FieldDataTypes::UNSIGNED,54,1,false,1,0);
        DataField internalError(FieldDataTypes::UNSIGNED,53,1,false,1,0);
        DataField xcvrOperational(FieldDataTypes::UNSIGNED,52,1,false,1,0);
        DataField rawDataMode(FieldDataTypes::UNSIGNED,51,1,false,1,0);
        DataField steeringWheelAngleAck(FieldDataTypes::UNSIGNED,50,11,false,1,0);
        DataField temp(FieldDataTypes::SIGNED,39,8,false,1,0);
        DataField vehSpdCompFactor(FieldDataTypes::SIGNED,31,6,false,0.001953125,1);
        DataField groupingMode(FieldDataTypes::UNSIGNED,25,2,false,1,0);
        DataField yawRateBias(FieldDataTypes::SIGNED,23,8,false,0.125,0);
        DataField swVersionDSP(FieldDataTypes::UNSIGNED,15,16,false,1,0);

        def_.addFieldDefinition(GROUPING_MODE,groupingMode);
        def_.addFieldDefinition(INTERNAL_ERROR, internalError);
        def_.addFieldDefinition(MAXIMUM_TRACKS_ACK,maximumTracksAck);
        def_.addFieldDefinition(OVERHEAT_ERROR,overheatError);
        def_.addFieldDefinition(RANGE_PERF_ERROR,rangePerfError);
        def_.addFieldDefinition(RAW_DATA_MODE,rawDataMode);
        def_.addFieldDefinition(ROLLING_COUNT_2,rollingCount2);
        def_.addFieldDefinition(STEERING_ANGLE_ACK,steeringWheelAngleAck);
        def_.addFieldDefinition(SW_VERSION_DSP,swVersionDSP);
        def_.addFieldDefinition(TEMPERATURE,temp);
        def_.addFieldDefinition(VEH_SPD_COMP_FCTOR,vehSpdCompFactor);
        def_.addFieldDefinition(XCVR_OPERATIONAL,xcvrOperational);
        def_.addFieldDefinition(YAW_RATE_BIAS,yawRateBias);
    }

    ESRStatus2Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus2Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        grouping_mode_  = def_.getUnsignedIntValue(GROUPING_MODE);
        internal_error_ = def_.getUnsignedIntValue(INTERNAL_ERROR);
        maximum_tracks_ack_= def_.getUnsignedIntValue(MAXIMUM_TRACKS_ACK);
        overheat_error_ = def_.getUnsignedIntValue(OVERHEAT_ERROR);
        range_perf_error_= def_.getUnsignedIntValue(RANGE_PERF_ERROR);
        raw_data_mode_= def_.getUnsignedIntValue(RAW_DATA_MODE);
        rolling_count_2_ = def_.getUnsignedIntValue(ROLLING_COUNT_2);
        xcvr_operational_= def_.getUnsignedIntValue(XCVR_OPERATIONAL);
        host_sw_version_= def_.getUnsignedIntValue(SW_VERSION_DSP);


        steering_wheel_angle_ack_ = def_.getUnsignedIntValue(STEERING_ANGLE_ACK);
        temperature_ = (int32_t) def_.getUnsignedIntValue(TEMPERATURE);
        vehicle_speed_compensation_= def_.getValue(VEH_SPD_COMP_FCTOR);
        yaw_rate_bias_ = def_.getValue(YAW_RATE_BIAS);
    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t maximumTracksAck() const {
        return maximum_tracks_ack_;
    }

    inline uint32_t rollingCount() const {
        return rolling_count_2_;
    }

    inline uint32_t overheatError() const {
        return overheat_error_;
    }

    inline uint32_t rangePerfError() const {
        return range_perf_error_;
    }

    inline uint32_t internalError() const {
        return internal_error_;
    }

    inline uint32_t xcvrOperational() const {
        return xcvr_operational_;
    }

    inline uint32_t rawDataMode() const {
        return raw_data_mode_;
    }

    inline uint32_t hostSWVersion() const {
        return host_sw_version_;
    }

    inline uint32_t groupingMode() const {
        return grouping_mode_;
    }

    inline uint32_t steeringWheelAngleAck() const {
        return steering_wheel_angle_ack_;
    }

    inline int32_t temperature() const {
        return temperature_;
    }

    inline float vehSpdCompFactor() const {
        return vehicle_speed_compensation_;
    }

    inline float yawRateBias() const {
        return yaw_rate_bias_;
    }
};


class ESRStatus3Definition
{
private:
    enum names_idx : unsigned int
    {
        HW_VERSION,
        INTERFACE_VERSION,
        SERIAL_NUM,
        SW_VERSION_HOST,
        SW_VERSION_PLD
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t rolling_count_, hw_version_, interface_version_, serial_num_, sw_version_host_, sw_version_pld_;

public:

    ESRStatus3Definition()
    {
        using namespace cav::can;
        DataField interfaceVersion(FieldDataTypes::UNSIGNED,63,4,false,1,0);
        DataField hwVersion(FieldDataTypes::UNSIGNED,59,4,false,1,0);
        DataField swVersionHost(FieldDataTypes::UNSIGNED,55,24,false,1,0);
        DataField serialNumber(FieldDataTypes::SIGNED,31,24,false,1,0);
        DataField swVersionPLD(FieldDataTypes::UNSIGNED,7,8,false,1,0);

        def_.addFieldDefinition(HW_VERSION,hwVersion);
        def_.addFieldDefinition(INTERFACE_VERSION, interfaceVersion);
        def_.addFieldDefinition(SERIAL_NUM,serialNumber);
        def_.addFieldDefinition(SW_VERSION_HOST,swVersionHost);
        def_.addFieldDefinition(SW_VERSION_PLD,swVersionPLD);
    }

    ESRStatus3Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus3Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        hw_version_ = def_.getUnsignedIntValue(HW_VERSION);
        interface_version_ = def_.getUnsignedIntValue(INTERFACE_VERSION);
        serial_num_ = def_.getUnsignedIntValue(SERIAL_NUM);
        sw_version_host_ = def_.getUnsignedIntValue(SW_VERSION_HOST);
        sw_version_pld_ = def_.getUnsignedIntValue(SW_VERSION_PLD);

    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t hwVersion() const {
        return hw_version_;
    }

    inline uint32_t interfaceVersion() const {
        return interface_version_;
    }

    inline uint32_t serialNum() const {
        return serial_num_;
    }

    inline uint32_t swVersionHost() const {
        return sw_version_host_;
    }

    inline uint32_t swVersionPLD() const {
        return sw_version_pld_;
    }
}; //class ESRStatus3Definition



class ESRStatus4Definition
{
private:
    enum names_idx : unsigned int
    {
        AUTO_ALIGN_ANGLE,
        LR_ONLY_GRATING_LOBE_DET,
        MR_LR_MODE,
        PARTIAL_BLOCKAGE,
        PATH_ID_ACC,
        PATH_ID_ACC_STAT,
        PATH_ID_CMBB_MOVE,
        PATH_ID_CMBB_STAT,
        PATH_ID_FCW_MOVE,
        PATH_ID_FCW_STAT,
        ROLLING_COUNT_3,
        SIDELOBE_BLOCKAGE,
        TRUCK_TARGET_DET
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t lr_only_grating_lob_det_,mr_lr_mode_,partial_blockage_, path_id_acc_, path_id_acc_stat_, path_id_cmbb_move_,
            path_id_cmbb_stat_,path_id_fcw_move_,path_id_fcw_stat_,rolling_count_,sidelobe_blockage_,truck_target_det_;

    float auto_align_angle_;
public:

    ESRStatus4Definition()
    {
        using namespace cav::can;
        DataField truckTgtDet(FieldDataTypes::UNSIGNED,63,1,false,1,0);
        DataField lrOnlyGratingLobeDet(FieldDataTypes::UNSIGNED,62,1,false,1,0);
        DataField sidelobeBlockage(FieldDataTypes::UNSIGNED,61,1,false,1,0);
        DataField partialBlockage(FieldDataTypes::UNSIGNED,60,1,false,1,0);
        DataField mrlrMode(FieldDataTypes::UNSIGNED,59,2,false,1,0);
        DataField rollingCount(FieldDataTypes::UNSIGNED,57,2,false,1,0);
        DataField pathIDAcc(FieldDataTypes::UNSIGNED,55,8,false,1,0);
        DataField pathIDCMBBMove(FieldDataTypes::UNSIGNED,47,8,false,1,0);
        DataField pathIDCMBBStat(FieldDataTypes::UNSIGNED,39,8,false,1,0);
        DataField pathIDFCWMove(FieldDataTypes::UNSIGNED,31,8,false,1,0);
        DataField pathIDFCWStat(FieldDataTypes::UNSIGNED,23,8,false,1,0);
        DataField autoAlignAngle(FieldDataTypes::SIGNED,15,8,false,0.0625,0);
        DataField pathIDACCStat(FieldDataTypes::UNSIGNED,7,8,false,1,0);

        def_.addFieldDefinition(AUTO_ALIGN_ANGLE,autoAlignAngle);
        def_.addFieldDefinition(LR_ONLY_GRATING_LOBE_DET, lrOnlyGratingLobeDet);
        def_.addFieldDefinition(MR_LR_MODE,mrlrMode);
        def_.addFieldDefinition(PARTIAL_BLOCKAGE,partialBlockage);
        def_.addFieldDefinition(PATH_ID_ACC,pathIDAcc);
        def_.addFieldDefinition(PATH_ID_ACC_STAT,pathIDACCStat);
        def_.addFieldDefinition(PATH_ID_CMBB_MOVE,pathIDCMBBMove);
        def_.addFieldDefinition(PATH_ID_CMBB_STAT,pathIDCMBBStat);
        def_.addFieldDefinition(PATH_ID_FCW_MOVE,pathIDFCWMove);
        def_.addFieldDefinition(PATH_ID_FCW_STAT,pathIDFCWStat);
        def_.addFieldDefinition(ROLLING_COUNT_3,rollingCount);
        def_.addFieldDefinition(SIDELOBE_BLOCKAGE,sidelobeBlockage);
        def_.addFieldDefinition(TRUCK_TARGET_DET,truckTgtDet);
    }

    ESRStatus4Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus4Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        lr_only_grating_lob_det_ = def_.getUnsignedIntValue(LR_ONLY_GRATING_LOBE_DET);
        mr_lr_mode_ = def_.getUnsignedIntValue(MR_LR_MODE);
        partial_blockage_ = def_.getUnsignedIntValue(PARTIAL_BLOCKAGE);
        rolling_count_ = def_.getUnsignedIntValue(ROLLING_COUNT_3);
        path_id_acc_ = def_.getUnsignedIntValue(PATH_ID_ACC);
        path_id_acc_stat_ = def_.getUnsignedIntValue(PATH_ID_ACC_STAT);
        path_id_cmbb_move_ = def_.getUnsignedIntValue(PATH_ID_CMBB_MOVE);
        path_id_cmbb_stat_ = def_.getUnsignedIntValue(PATH_ID_CMBB_STAT);
        path_id_fcw_move_ = def_.getUnsignedIntValue(PATH_ID_FCW_MOVE);
        path_id_fcw_stat_ = def_.getUnsignedIntValue(PATH_ID_FCW_STAT);
        sidelobe_blockage_ = def_.getUnsignedIntValue(SIDELOBE_BLOCKAGE);
        truck_target_det_ = def_.getUnsignedIntValue(TRUCK_TARGET_DET);

        auto_align_angle_ = def_.getValue(AUTO_ALIGN_ANGLE);
    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t LROnlyGratingLobeDet() const {
        return lr_only_grating_lob_det_;
    }

    inline uint32_t MRLRMode() const {
        return mr_lr_mode_;
    }

    inline uint32_t partialBlockage() const {
        return partial_blockage_;
    }

    inline uint32_t pathIDACC() const {
        return path_id_acc_;
    }

    inline uint32_t pathIDACCStat() const {
        return path_id_acc_stat_;
    }

    inline uint32_t pathIDCMBBMove() const {
        return path_id_cmbb_move_;
    }

    inline uint32_t pathIDCMBBStat() const {
        return path_id_cmbb_stat_;
    }

    inline uint32_t pathIDFCWMove() const {
        return path_id_fcw_move_;
    }

    inline uint32_t pathIDFCWStat() const {
        return path_id_fcw_stat_;
    }

    inline uint32_t rollingCount() const {
        return rolling_count_;
    }

    inline uint32_t sidelobeBlockge() const {
        return sidelobe_blockage_;
    }

    inline uint32_t truckTargetDet() const {
        return truck_target_det_;
    }

    inline float autoAlignAngle() const {
        return auto_align_angle_;
    }
};


class ESRStatus5Definition
{
private:
    enum names_idx : unsigned int
    {
        IGNP_A2D,
        SUPPLY_10V_A2D,
        SUPPLY_3P3V_A2D,
        SUPPLY_5VA_A2D,
        SUPPLY_5VDX_A2D,
        SWBATT_A2D,
        TEMP1_A2D,
        TEMP2_A2D

    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t ignp_a2d_, supply_10v_a2d_, supply_3p3v_a2d_,supply_5va_a2d_,supply_5vdx_a2d_,swbatt_a2d_, temp1_a2d_, temp2_a2d_;

public:

    ESRStatus5Definition()
    {
        using namespace cav::can;
        DataField swBattA2D(FieldDataTypes::UNSIGNED,63,8,false,1,0);
        DataField ignpA2D(FieldDataTypes::UNSIGNED,55,8,false,1,0);
        DataField temp1A2D(FieldDataTypes::UNSIGNED,47,8,false,1,0);
        DataField temp2A2D(FieldDataTypes::UNSIGNED,39,8,false,1,0);
        DataField supply5vaA2D(FieldDataTypes::UNSIGNED,31,8,false,1,0);
        DataField supply5vdxA2D(FieldDataTypes::UNSIGNED,23,8,false,1,0);
        DataField supply3p3vA2d(FieldDataTypes::UNSIGNED,15,8,false,1,0);
        DataField supply10vA2D(FieldDataTypes::UNSIGNED,7,8,false,1,0);

        def_.addFieldDefinition(IGNP_A2D,ignpA2D);
        def_.addFieldDefinition(SUPPLY_10V_A2D, supply10vA2D);
        def_.addFieldDefinition(SUPPLY_3P3V_A2D,supply3p3vA2d);
        def_.addFieldDefinition(SUPPLY_5VA_A2D,supply5vaA2D);
        def_.addFieldDefinition(SUPPLY_5VDX_A2D,supply5vdxA2D);
        def_.addFieldDefinition(SWBATT_A2D,swBattA2D);
        def_.addFieldDefinition(TEMP1_A2D,temp1A2D);
        def_.addFieldDefinition(TEMP2_A2D,temp2A2D);
    }

    ESRStatus5Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus5Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        ignp_a2d_ = def_.getUnsignedIntValue(IGNP_A2D);
        supply_10v_a2d_ = def_.getUnsignedIntValue(SUPPLY_10V_A2D);
        supply_3p3v_a2d_ = def_.getUnsignedIntValue(SUPPLY_3P3V_A2D);
        supply_5va_a2d_ = def_.getUnsignedIntValue(SUPPLY_5VA_A2D);
        supply_5vdx_a2d_ = def_.getUnsignedIntValue(SUPPLY_5VDX_A2D);
        swbatt_a2d_ = def_.getUnsignedIntValue(SWBATT_A2D);
        temp1_a2d_ = def_.getUnsignedIntValue(TEMP1_A2D);
        temp2_a2d_ = def_.getUnsignedIntValue(TEMP2_A2D);
    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t IGNPA2D() const {
        return ignp_a2d_;
    }

    inline uint32_t supply10vA2D() const {
        return supply_10v_a2d_;
    }

    inline uint32_t supply3P3VA2D() const {
        return supply_3p3v_a2d_;
    }

    inline uint32_t supply5VAA2D() const {
        return supply_5va_a2d_;
    }

    inline uint32_t supply5VDXA2D() const {
        return supply_5vdx_a2d_;
    }

    inline uint32_t swBattA2D() const {
        return swbatt_a2d_;
    }

    inline uint32_t temp1A2D() const {
        return temp1_a2d_;
    }

    inline uint32_t temp2A2D() const {
        return temp2_a2d_;
    }

};


class ESRStatus6Definition
{
private:
    enum names_idx : unsigned int
    {
        FACTORY_ALIGN_STATUS_1,
        FACTORY_ALIGN_STATUS_2,
        FACTORY_MISALIGNMENT,
        FOUND_TARGET,
        RECOMMEND_UNCONVERGE,
        SERV_ALIGN_UPDATES_DONE,
        SUPPLY_1P8V_A2D,
        SUPPLY_N5VA_A2D,
        SW_VERSION_DSP_3RD_BYTE,
        SYSTEM_POWER_MODE,
        VERTICAL_ALIGN_UPDATED,
        VERTICAL_MISALIGNMENT,
        WAVE_DIFF_A2D

    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t factory_align_status1_,factory_align_status2_, found_target_, recommend_unconverge_, serv_align_updates_done_,
            supply_1p8v_a2d_,supply_n5va_a2d_,sw_version_dsp_3rd_byte_,system_power_mode_,vertical_align_updated_, wave_diff_a2d_;

    float factory_misalignment_, vertical_misalignment_;

public:

    ESRStatus6Definition()
    {
        using namespace cav::can;
        DataField supply1P8VA2D(FieldDataTypes::UNSIGNED,63,8,false,1,0);
        DataField supplyN5VA2D(FieldDataTypes::UNSIGNED,55,8,false,1,0);
        DataField waveDiffA2D(FieldDataTypes::UNSIGNED,47,8,false,1,0);
        DataField swVersionDsp3rdByte(FieldDataTypes::UNSIGNED,39,4,false,1,0);
        DataField verticalAlignmentUpdated(FieldDataTypes::UNSIGNED,35,1,false,1,0);
        DataField systemPowerMode(FieldDataTypes::UNSIGNED,34,3,false,1,0);
        DataField foundTarget(FieldDataTypes::UNSIGNED,31,1,false,1,0);
        DataField recommendUnconverge(FieldDataTypes::UNSIGNED,30,1,false,1,0);
        DataField factoryAlignStatus1(FieldDataTypes::UNSIGNED,29,3,false,1,0);
        DataField factoryAlignStatus2(FieldDataTypes::UNSIGNED,26,3,false,1,0);
        DataField factoryMisalignment(FieldDataTypes::SIGNED,23,8,false,0.0625,0);
        DataField alignUpdatesDone(FieldDataTypes::UNSIGNED,15,8,false,1,0);
        DataField verticalMisalignment(FieldDataTypes::SIGNED,7,8,false,0.0625,0);

        def_.addFieldDefinition(FACTORY_ALIGN_STATUS_1,factoryAlignStatus1);
        def_.addFieldDefinition(FACTORY_ALIGN_STATUS_2, factoryAlignStatus2);
        def_.addFieldDefinition(FACTORY_MISALIGNMENT,factoryMisalignment);
        def_.addFieldDefinition(FOUND_TARGET,foundTarget);
        def_.addFieldDefinition(RECOMMEND_UNCONVERGE,recommendUnconverge);
        def_.addFieldDefinition(SERV_ALIGN_UPDATES_DONE,alignUpdatesDone);
        def_.addFieldDefinition(SUPPLY_1P8V_A2D,supply1P8VA2D);
        def_.addFieldDefinition(SUPPLY_N5VA_A2D,supplyN5VA2D);
        def_.addFieldDefinition(SW_VERSION_DSP_3RD_BYTE,swVersionDsp3rdByte);
        def_.addFieldDefinition(SYSTEM_POWER_MODE,systemPowerMode);
        def_.addFieldDefinition(VERTICAL_ALIGN_UPDATED,verticalAlignmentUpdated);
        def_.addFieldDefinition(VERTICAL_MISALIGNMENT,verticalMisalignment);
        def_.addFieldDefinition(WAVE_DIFF_A2D,waveDiffA2D);
    }

    ESRStatus6Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus6Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        factory_align_status1_ = def_.getUnsignedIntValue(FACTORY_ALIGN_STATUS_1);
        factory_align_status2_ = def_.getUnsignedIntValue(FACTORY_ALIGN_STATUS_2);
        found_target_ = def_.getUnsignedIntValue(FOUND_TARGET);
        recommend_unconverge_ = def_.getUnsignedIntValue(RECOMMEND_UNCONVERGE);
        serv_align_updates_done_ = def_.getUnsignedIntValue(SERV_ALIGN_UPDATES_DONE);
        supply_1p8v_a2d_ = def_.getUnsignedIntValue(SUPPLY_1P8V_A2D);
        supply_n5va_a2d_ = def_.getUnsignedIntValue(SUPPLY_N5VA_A2D);
        sw_version_dsp_3rd_byte_ = def_.getUnsignedIntValue(SW_VERSION_DSP_3RD_BYTE);
        system_power_mode_ = def_.getUnsignedIntValue(SYSTEM_POWER_MODE);
        vertical_align_updated_ = def_.getUnsignedIntValue(VERTICAL_ALIGN_UPDATED);
        wave_diff_a2d_ = def_.getUnsignedIntValue(WAVE_DIFF_A2D);

        factory_misalignment_= def_.getValue(FACTORY_MISALIGNMENT);
        vertical_misalignment_ = def_.getValue(VERTICAL_MISALIGNMENT);

    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    uint32_t factoryAlignStatus1() const {
        return factory_align_status1_;
    }

    uint32_t factorAlignStatus2() const {
        return factory_align_status2_;
    }

    uint32_t foundTarget() const {
        return found_target_;
    }

    uint32_t recommendUnconverge() const {
        return recommend_unconverge_;
    }

    uint32_t servAlignUpdatesDone() const {
        return serv_align_updates_done_;
    }

    uint32_t supply1P8VA2D() const {
        return supply_1p8v_a2d_;
    }

    uint32_t supplyN5VA2D() const {
        return supply_n5va_a2d_;
    }

    uint32_t swVersionDSP3rdByte() const {
        return sw_version_dsp_3rd_byte_;
    }

    uint32_t systemPowerMode() const {
        return system_power_mode_;
    }

    uint32_t verticalAlignUpdated() const {
        return vertical_align_updated_;
    }

    uint32_t waveDiffA2D() const {
        return wave_diff_a2d_;
    }

    float factoryMisalignment() const {
        return factory_misalignment_;
    }

    float verticalMisalignment() const {
        return vertical_misalignment_;
    }
};


class ESRStatus7Definition
{
private:
    enum names_idx : unsigned int
    {
        FAULT0,
        FAULT1,
        FAULT2,
        FAULT3,
        FAULT4,
        FAULT5,
        FAULT6,
        FAULT7,
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t fault0_,fault1_,fault2_,fault3_,fault4_,fault5_,fault6_,fault7_;

public:

    ESRStatus7Definition()
    {
        using namespace cav::can;
        DataField fault0(FieldDataTypes::UNSIGNED,63,8,false,1,0);
        DataField fault1(FieldDataTypes::UNSIGNED,55,8,false,1,0);
        DataField fault2(FieldDataTypes::UNSIGNED,47,8,false,1,0);
        DataField fault3(FieldDataTypes::UNSIGNED,39,8,false,1,0);
        DataField fault4(FieldDataTypes::UNSIGNED,31,8,false,1,0);
        DataField fault5(FieldDataTypes::UNSIGNED,23,8,false,1,0);
        DataField fault6(FieldDataTypes::UNSIGNED,15,8,false,1,0);
        DataField fault7(FieldDataTypes::UNSIGNED,7,8,false,1,0);

        def_.addFieldDefinition(FAULT0,fault0);
        def_.addFieldDefinition(FAULT1,fault1);
        def_.addFieldDefinition(FAULT2,fault2);
        def_.addFieldDefinition(FAULT3,fault3);
        def_.addFieldDefinition(FAULT4,fault4);
        def_.addFieldDefinition(FAULT5,fault5);
        def_.addFieldDefinition(FAULT6,fault6);
        def_.addFieldDefinition(FAULT7,fault7);
    }

    ESRStatus7Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus7Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        fault0_ = def_.getUnsignedIntValue(FAULT0);
        fault1_ = def_.getUnsignedIntValue(FAULT1);
        fault2_ = def_.getUnsignedIntValue(FAULT2);
        fault3_ = def_.getUnsignedIntValue(FAULT3);
        fault4_ = def_.getUnsignedIntValue(FAULT4);
        fault5_ = def_.getUnsignedIntValue(FAULT5);
        fault6_ = def_.getUnsignedIntValue(FAULT6);
        fault7_ = def_.getUnsignedIntValue(FAULT7);
    }


    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t activeFault0() const {
        return fault0_;
    }

    inline uint32_t activeFault1() const {
        return fault1_;
    }

    inline uint32_t activeFault2() const {
        return fault2_;
    }

    inline uint32_t activeFault3() const {
        return fault3_;
    }

    inline uint32_t activeFault4() const {
        return fault4_;
    }

    inline uint32_t activeFault5() const {
        return fault5_;
    }

    inline uint32_t activeFault6() const {
        return fault6_;
    }

    inline uint32_t activeFault7() const {
        return fault7_;
    }
};


class ESRStatus8Definition
{
private:
    enum names_idx : unsigned int
    {
        FAULT0,
        FAULT1,
        FAULT2,
        FAULT3,
        FAULT4,
        FAULT5,
        FAULT6,
        FAULT7,
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t fault0_,fault1_,fault2_,fault3_,fault4_,fault5_,fault6_,fault7_;

public:

    ESRStatus8Definition()
    {
        using namespace cav::can;
        DataField fault0(FieldDataTypes::UNSIGNED,63,8,false,1,0);
        DataField fault1(FieldDataTypes::UNSIGNED,55,8,false,1,0);
        DataField fault2(FieldDataTypes::UNSIGNED,47,8,false,1,0);
        DataField fault3(FieldDataTypes::UNSIGNED,39,8,false,1,0);
        DataField fault4(FieldDataTypes::UNSIGNED,31,8,false,1,0);
        DataField fault5(FieldDataTypes::UNSIGNED,23,8,false,1,0);
        DataField fault6(FieldDataTypes::UNSIGNED,15,8,false,1,0);
        DataField fault7(FieldDataTypes::UNSIGNED,7,8,false,1,0);

        def_.addFieldDefinition(FAULT0,fault0);
        def_.addFieldDefinition(FAULT1,fault1);
        def_.addFieldDefinition(FAULT2,fault2);
        def_.addFieldDefinition(FAULT3,fault3);
        def_.addFieldDefinition(FAULT4,fault4);
        def_.addFieldDefinition(FAULT5,fault5);
        def_.addFieldDefinition(FAULT6,fault6);
        def_.addFieldDefinition(FAULT7,fault7);
    }

    ESRStatus8Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus8Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        fault0_ = def_.getUnsignedIntValue(FAULT0);
        fault1_ = def_.getUnsignedIntValue(FAULT1);
        fault2_ = def_.getUnsignedIntValue(FAULT2);
        fault3_ = def_.getUnsignedIntValue(FAULT3);
        fault4_ = def_.getUnsignedIntValue(FAULT4);
        fault5_ = def_.getUnsignedIntValue(FAULT5);
        fault6_ = def_.getUnsignedIntValue(FAULT6);
        fault7_ = def_.getUnsignedIntValue(FAULT7);
    }


    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t historyFault0() const {
        return fault0_;
    }

    inline uint32_t historyFault1() const {
        return fault1_;
    }

    inline uint32_t historyFault2() const {
        return fault2_;
    }

    inline uint32_t historyFault3() const {
        return fault3_;
    }

    inline uint32_t historyFault4() const {
        return fault4_;
    }

    inline uint32_t historyFault5() const {
        return fault5_;
    }

    inline uint32_t historyFault6() const {
        return fault6_;
    }

    inline uint32_t historyFault7() const {
        return fault7_;
    }
};



class ESRStatus9Definition
{
private:
    enum names_idx : unsigned int
    {
        AVG_PWR_CWBLKG,
        FILTERED_XOHP_ACC_CIPV,
        PATH_ID_ACC_2,
        PATH_ID_ACC_3,
        SERIAL_NUM_3RD_BYTE,
        SIDESLIP_ANGLE,
        WATER_SPRAY_TARGET_ID,
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t avg_pwr_cwblkg_,path_id_acc2_,path_id_acc3_,serial_num_3rd_byte_,water_spray_target_id_;

    float sideslip_angle_, filtered_xohp_acc_cipv_;

public:

    ESRStatus9Definition()
    {
        using namespace cav::can;
        DataField avgPwrCWBlckG(FieldDataTypes::UNSIGNED,63,12,false,1,0);
        DataField sideslipAngle(FieldDataTypes::SIGNED,49,10,false,0.125,0);
        DataField serialNum3rdByte(FieldDataTypes::UNSIGNED,39,8,false,1,0);
        DataField waterSprayTgtID(FieldDataTypes::UNSIGNED,31,7,false,1,0);
        DataField filteredXOHPAccCPV(FieldDataTypes::SIGNED,24,9,false,0.03125,0);
        DataField pathIDACC2(FieldDataTypes::UNSIGNED,15,8,false,1,0);
        DataField pathIDACC3(FieldDataTypes::UNSIGNED,7,8,false,1,0);

        def_.addFieldDefinition(AVG_PWR_CWBLKG,avgPwrCWBlckG);
        def_.addFieldDefinition(FILTERED_XOHP_ACC_CIPV,filteredXOHPAccCPV);
        def_.addFieldDefinition(PATH_ID_ACC_2,pathIDACC2);
        def_.addFieldDefinition(PATH_ID_ACC_3,pathIDACC3);
        def_.addFieldDefinition(SERIAL_NUM_3RD_BYTE,serialNum3rdByte);
        def_.addFieldDefinition(SIDESLIP_ANGLE,sideslipAngle);
        def_.addFieldDefinition(WATER_SPRAY_TARGET_ID,waterSprayTgtID);
    }

    ESRStatus9Definition(unsigned char const* const data, const unsigned char data_length) : ESRStatus9Definition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        avg_pwr_cwblkg_ = def_.getUnsignedIntValue(AVG_PWR_CWBLKG);
        path_id_acc2_ = def_.getUnsignedIntValue(PATH_ID_ACC_2);
        path_id_acc3_ = def_.getUnsignedIntValue(PATH_ID_ACC_3);
        serial_num_3rd_byte_ = def_.getUnsignedIntValue(SERIAL_NUM_3RD_BYTE);
        water_spray_target_id_ = def_.getUnsignedIntValue(WATER_SPRAY_TARGET_ID);

        sideslip_angle_ = def_.getValue(SIDESLIP_ANGLE);
        filtered_xohp_acc_cipv_ = def_.getValue(FILTERED_XOHP_ACC_CIPV);
    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    uint32_t avgPwrCWBlkg() const {
        return avg_pwr_cwblkg_;
    }

    uint32_t pathIDACC2() const {
        return path_id_acc2_;
    }

    uint32_t pathIDACC3() const {
        return path_id_acc3_;
    }

    uint32_t serialNum3rdByte() const {
        return serial_num_3rd_byte_;
    }

    uint32_t waterSprayTargetId() const {
        return water_spray_target_id_;
    }

    float sideslipAngle() const {
        return sideslip_angle_;
    }

    float filteredXOHPACCCIPV() const {
        return filtered_xohp_acc_cipv_;
    }
};


class ESRTrackDefinition
{
private:
    enum names_idx : unsigned int
    {
        ANGLE,
        BRIDGE_OBJECT,
        GROUPING_CHANGED,
        LAT_RATE,
        MED_RANGE_MODE,
        ONCOMING,
        RANGE,
        RANGE_ACCEL,
        RANGE_RATE,
        ROLLING_COUNT,
        STATUS,
        WIDTH
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t status_,grouping_changed_,oncoming_,bridge_object_, rolling_count_,range_mode_;

    float lat_rate_, angle_, range_, width_, range_accel_,range_rate_;

public:

    ESRTrackDefinition()
    {
        using namespace cav::can;
        DataField latRate(FieldDataTypes::SIGNED,63,6,false,0.25,0);
        DataField groupingChange(FieldDataTypes::UNSIGNED,57,1,false,1,0);
        DataField trackOncoming(FieldDataTypes::UNSIGNED,56,1,false,1,0);
        DataField trackStatus(FieldDataTypes::UNSIGNED,55,3,false,1,0);
        DataField angle(FieldDataTypes::SIGNED,52,10,false,0.1,0);
        DataField range(FieldDataTypes::UNSIGNED,42,11,false,0.1,0);
        DataField bridgeObject(FieldDataTypes::UNSIGNED,31,1,false,1,0);
        DataField rollingCount(FieldDataTypes::UNSIGNED,30,1,false,1,0);
        DataField width(FieldDataTypes::UNSIGNED,29,4,false,0.5,0.0);
        DataField rangeAccel(FieldDataTypes::SIGNED,25,10,false,0.05,0);
        DataField medRangeMode(FieldDataTypes::UNSIGNED,15,2,false,1,0);
        DataField rangeRate(FieldDataTypes::SIGNED,13,14,false,0.01,0);

        def_.addFieldDefinition(ANGLE,angle);
        def_.addFieldDefinition(BRIDGE_OBJECT,bridgeObject);
        def_.addFieldDefinition(GROUPING_CHANGED,groupingChange);
        def_.addFieldDefinition(LAT_RATE,latRate);
        def_.addFieldDefinition(MED_RANGE_MODE,medRangeMode);
        def_.addFieldDefinition(ONCOMING,trackOncoming);
        def_.addFieldDefinition(RANGE,range);
        def_.addFieldDefinition(RANGE_ACCEL,rangeAccel);
        def_.addFieldDefinition(RANGE_RATE,rangeRate);
        def_.addFieldDefinition(ROLLING_COUNT,rollingCount);
        def_.addFieldDefinition(STATUS,trackStatus);
        def_.addFieldDefinition(WIDTH,width);
    }

    ESRTrackDefinition(unsigned char const* const data, const unsigned char data_length) : ESRTrackDefinition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length)
    {
        def_.setData(data,data_length);

        status_ = def_.getUnsignedIntValue(STATUS);
        grouping_changed_ = def_.getUnsignedIntValue(GROUPING_CHANGED);
        oncoming_ = def_.getUnsignedIntValue(ONCOMING);
        bridge_object_ = def_.getUnsignedIntValue(BRIDGE_OBJECT);
        rolling_count_ = def_.getUnsignedIntValue(ROLLING_COUNT);
        range_mode_ = def_.getUnsignedIntValue(MED_RANGE_MODE);

        lat_rate_ = def_.getValue(LAT_RATE);
        angle_ = def_.getValue(ANGLE);
        range_ = def_.getValue(RANGE);
        width_ = def_.getValue(WIDTH);
        range_accel_ = def_.getValue(RANGE_ACCEL);
        range_rate_ = def_.getValue(RANGE_RATE);
    }


    std::string to_string() const
    {
        return def_.dataToString();
    }

    uint32_t status() const {
        return status_;
    }

    uint32_t groupingChanged() const {
        return grouping_changed_;
    }

    uint32_t onComing() const {
        return oncoming_;
    }

    uint32_t bridgeObject() const {
        return bridge_object_;
    }

    uint32_t rollingCount() const {
        return rolling_count_;
    }

    uint32_t rangeMode() const {
        return range_mode_;
    }

    float latRate() const {
        return lat_rate_;
    }

    float angle() const {
        return angle_;
    }

    float range() const {
        return range_;
    }

    float width() const {
        return width_;
    }

    float rangeAccel() const {
        return range_accel_;
    }

    float rangeRate() const {
        return range_rate_;
    }
};

class ESRTrackMotionPowerDefinition
{
public:


    struct TrackMovePower
    {
        uint32_t movable_fast, movable_slow, moving;
        int32_t power;
    };


private:
    enum names_idx : unsigned int
    {
        ROLLING_COUNT,
        GROUP,
        TRACK0,
        TRACK1,
        TRACK2,
        TRACK3,
        TRACK4,
        TRACK5,
        TRACK6
    };

    cav::can::CANMessageDefinition<unsigned int> def_;
    uint32_t rolling_count_, group_;
    std::vector<TrackMovePower> tracks_;

public:

    ESRTrackMotionPowerDefinition() : tracks_(7)
    {
        using namespace cav::can;
        DataField rollingCount(FieldDataTypes::UNSIGNED,60,1,false,1,0);
        DataField groupID(FieldDataTypes::UNSIGNED,59,4,false,1,0);
        DataField track0(FieldDataTypes::UNSIGNED,55,8,false,1,0);
        DataField track1(FieldDataTypes::UNSIGNED,47,8,false,1,0);
        DataField track2(FieldDataTypes::UNSIGNED,39,8,false,1,0);
        DataField track3(FieldDataTypes::UNSIGNED,31,8,false,1,0);
        DataField track4(FieldDataTypes::UNSIGNED,23,8,false,1,0);
        DataField track5(FieldDataTypes::UNSIGNED,15,8,false,1,0);
        DataField track6(FieldDataTypes::UNSIGNED,7,8,false,1,0);


        def_.addFieldDefinition(ROLLING_COUNT,rollingCount);
        def_.addFieldDefinition(GROUP,groupID);
        def_.addFieldDefinition(TRACK0,track0);
        def_.addFieldDefinition(TRACK1,track1);
        def_.addFieldDefinition(TRACK2,track2);
        def_.addFieldDefinition(TRACK3,track3);
        def_.addFieldDefinition(TRACK4,track4);
        def_.addFieldDefinition(TRACK5,track5);
        def_.addFieldDefinition(TRACK6,track6);
    }

    ESRTrackMotionPowerDefinition(unsigned char const* const data, const unsigned char data_length) : ESRTrackMotionPowerDefinition()
    {
        setData(data,data_length);
    }

    void setData(unsigned char const* const data, const unsigned char data_length) {
        def_.setData(data, data_length);

        rolling_count_ = def_.getUnsignedIntValue(ROLLING_COUNT);
        group_ = def_.getUnsignedIntValue(GROUP);

        unsigned int j = TRACK0;
        for(int i = 0; i < 7; i++, j++)
        {
            uint32_t value = def_.getUnsignedIntValue(j);
            tracks_[i].movable_fast = (value & 0x80) >> 7;
            tracks_[i].movable_slow = (value & 0x40) >> 6;
            tracks_[i].moving = (value & 0x20) >> 5;
            tracks_[i].power = (value & 0x1F) - 10;
        }

    }

    std::string to_string() const
    {
        return def_.dataToString();
    }

    inline uint32_t rollingCount() const {
        return rolling_count_;
    }

    inline uint32_t group() const {
        return group_;
    }

    inline std::vector<TrackMovePower>& track() {
        return tracks_;
    }
};



}//namespace delphi
