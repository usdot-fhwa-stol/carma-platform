// Test truck_message_definitions

#include "../src/truck_types.h"

#include <array>
#include <gtest/gtest.h>

TEST(TruckMessageParsingTest, test1)
{
    std::array<uint8_t, 8> propb_11_data = {0x0A,0x00,0x64,0x00,0x10,0x00,0x1D,0x93};
    truck::PropB_11_Message propb_11;
    propb_11.setData(propb_11_data);
    //propb_11.printData();
    EXPECT_FLOAT_EQ(0.25, propb_11.getGenAccelPedalPercent());
    EXPECT_FLOAT_EQ(2.5, propb_11.getExpAccelPedalPercent());
    EXPECT_FLOAT_EQ(16 * 0.025, propb_11.getPhyAccelPedalPercent());
    EXPECT_EQ(1, static_cast<uint16_t>(propb_11.getRoboticModeStateStatus()));
    EXPECT_EQ(3, propb_11.getReservedPropB11());
    EXPECT_EQ(1, propb_11.getRollingCounter());
    EXPECT_EQ(0x93, propb_11.getCheckValue());

    std::array<uint8_t, 8> propb_12_data = {0x0A,0x00,0x64,0x00,0x10,0x00,0x17,0x96};
    truck::PropB_12_Message propb_12;
    propb_12.setData(propb_12_data);
    //propb_12.printData();
    EXPECT_FLOAT_EQ(0.25 - 4000, propb_12.getWrenchEffortEcho());
    EXPECT_FLOAT_EQ(3.125, propb_12.getSpeedControlEcho());
    EXPECT_FLOAT_EQ(16 * 0.03125, propb_12.getMaxAccelEcho());
    EXPECT_EQ(3, propb_12.getReservedPropB12());
    EXPECT_EQ(1, static_cast<uint16_t>(propb_12.getControlTypeCmdModeEcho()));
    EXPECT_EQ(1, propb_12.getRollingCounter());
    EXPECT_EQ(0x96, propb_12.getCheckValue());

    std::array<uint8_t, 8> propb_20_data = {0x0A,0x00,0x01,0x20,0x18,0xCC,0xFF,0xFF};
    truck::PropB_20_Message propb_20;
    propb_20.setData(propb_20_data);
    //propb_20.printData();
    EXPECT_FLOAT_EQ(0.5, propb_20.getSystemAirPres());
    EXPECT_FLOAT_EQ(0.8, propb_20.getRoboticAppliedBrakePres());
    EXPECT_FLOAT_EQ(3.2, propb_20.getCmdBrakeAppliedLevel());
    EXPECT_EQ(2, static_cast<uint16_t>(propb_20.getAxiomaticHealthStatus()));
    EXPECT_EQ(1, propb_20.getRollingCounter());
    EXPECT_EQ(0xCC, propb_20.getCheckValue());

    std::array<uint8_t, 8> propb_21_data = {0x0A,0x00,0x01,0x20,0x18,0xCC,0xFF,0xFF};
    truck::PropB_21_Message propb_21;
    propb_21.setData(propb_21_data);
    //propb_21.printData();
    EXPECT_FLOAT_EQ(0.5, propb_21.getSystemAirPres());
    EXPECT_FLOAT_EQ(0.8, propb_21.getRoboticAppliedBrakePres());
    EXPECT_FLOAT_EQ(3.2, propb_21.getCmdBrakeAppliedLevel());
    EXPECT_EQ(2, static_cast<uint16_t>(propb_21.getAxiomaticHealthStatus()));
    EXPECT_EQ(1, propb_21.getRollingCounter());
    EXPECT_EQ(0xCC, propb_21.getCheckValue());

    std::array<uint8_t, 8> propb_27_data = {0x55,0xD5,0x1F,0xF9,0xFF,0xFF,0xFF,0xFF};
    truck::PropB_27_Message propb_27;
    propb_27.setData(propb_27_data);
    //propb_27.printData();
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getEngineBrakeMsgModeFb()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getGenEngineBrakeCmd()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getExpEngineBrakeCmd()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getPhyEngineBrakeCmd()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getGenEngineBrakeLevelCmd()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getExpEngineBrakeLevelCmd()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_27.getPhyEngineBrakeLevelCmd()));
    EXPECT_EQ(63, propb_27.getReservedPropB27());
    EXPECT_EQ(1, propb_27.getRollingCounter());
    EXPECT_EQ(0xF9, propb_27.getCheckValue());

    std::array<uint8_t, 8> propb_30_data = {0x10,0x00,0x0A,0x00,0xF6,0xFF,0x04,0x00};
    truck::PropB_30_Message propb_30;
    propb_30.setData(propb_30_data);
    //propb_30.printData();
    EXPECT_EQ(16 + 32768, propb_30.getProportionalError());
    EXPECT_EQ(10 + 32768, propb_30.getIntegratorError());
    EXPECT_EQ(-10 + 32768, propb_30.getDerivativeError());
    EXPECT_EQ(4 + 32768, propb_30.getUnknownPropB30());

    std::array<uint8_t, 8> propb_31_data = {0x10,0x00,0x0A,0x00,0xF6,0xFF,0x04,0x00};
    truck::PropB_31_Message propb_31;
    propb_31.setData(propb_31_data);
    //propb_31.printData();
    EXPECT_EQ(16 + 32768, propb_31.getProportionalGainCompNum());
    EXPECT_EQ(10 + 32768, propb_31.getIntegratorGainCompNum());
    EXPECT_EQ(-10 + 32768, propb_31.getDerivativeGainCompNum());
    EXPECT_EQ(4 + 32768, propb_31.getPIDSharedDivisor());

    std::array<uint8_t, 8> propb_F0_data = {0x80,0x00,0x80,0x00,0xF6,0xFF,0x10,0x00};
    truck::PropB_F0_Message propb_F0;
    propb_F0.setData(propb_F0_data);
    //propb_F0.printData();
    EXPECT_FLOAT_EQ(1 - 273, propb_F0.getModuleTemperature());
    EXPECT_FLOAT_EQ(0.128, propb_F0.getModuleInputVoltage());
    EXPECT_FLOAT_EQ(-50, propb_F0.getModuleMaxLoopTime());
    EXPECT_FLOAT_EQ(80, propb_F0.getModuleAvgLoopTime());

    std::array<uint8_t, 8> propb_F1_data = {0x80,0x00,0x10,0x00,0xF6,0xFF,0x40,0x00};
    truck::PropB_F1_Message propb_F1;
    propb_F1.setData(propb_F1_data);
    //propb_F1.printData();
    EXPECT_FLOAT_EQ(0.128, propb_F1.getAIn1Raw());
    EXPECT_FLOAT_EQ(0.016, propb_F1.getAIn2Raw());
    EXPECT_FLOAT_EQ(-0.01, propb_F1.getAIn3Raw());
    EXPECT_FLOAT_EQ(0.064, propb_F1.getAIn4Raw());

    std::array<uint8_t, 8> propb_F2_data = {0x80,0x00,0x10,0x00,0xF6,0xFF,0x40,0x00};
    truck::PropB_F2_Message propb_F2;
    propb_F2.setData(propb_F2_data);
    //propb_F2.printData();
    EXPECT_FLOAT_EQ(0.128, propb_F2.getAOut1Raw());
    EXPECT_FLOAT_EQ(0.016, propb_F2.getAOut2Raw());
    EXPECT_FLOAT_EQ(-0.01, propb_F2.getAOut3Raw());
    EXPECT_FLOAT_EQ(0.064, propb_F2.getAOut4Raw());

    std::array<uint8_t, 8> propb_ED_data = {0x55,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    truck::PropB_ED_Message propb_ED;
    propb_ED.setData(propb_ED_data);
    //propb_ED.printData();
    EXPECT_EQ(1, static_cast<uint16_t>(propb_ED.getFaultLEDState()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_ED.getRoboticLEDState()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_ED.getEmoLEDState()));
    EXPECT_EQ(1, static_cast<uint16_t>(propb_ED.getAudibleBuzzerState()));

    std::array<uint8_t, 8> propb_FC_data = {0x10,0x00,0x00,0x00,0x04,0x00,0x00,0x00};
    truck::PropB_FC_Message propb_FC;
    propb_FC.setData(propb_FC_data);
    //propb_FC.printData();
    EXPECT_EQ(16, propb_FC.getModuleNVRamSettingsCRC());
    EXPECT_EQ(4, propb_FC.getModuleRamSettingsCRC());
}

int main(int argc, char**argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
