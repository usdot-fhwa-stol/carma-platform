// Test delphi_message_definitions

#include "../src/delphi_message_definitions.h"

#include <gtest/gtest.h>

TEST(DelphiMessageDefinitionParsingTest, test1)
{
    uint8_t const data[8] = {0xc7,0x12,0xad,0x16,0xb3,0xfa,0x6f,0xa5};
    delphi::ESRStatus1Definition status1;
    status1.setData(data,8);

    
    EXPECT_EQ( 28,status1.dspTimeStamp() );
    EXPECT_EQ(status1.commError() , 0);
    EXPECT_FLOAT_EQ( -5.625,status1.yawRateCalc() );
    EXPECT_FLOAT_EQ( 122.3125,status1.vehicleSpeedCalc() );
    EXPECT_EQ(5811,status1.scanIndex());
    EXPECT_EQ( 3,status1.rollingCount() );
    EXPECT_EQ( 4781,status1.radiusCurvatureCalc() );

    delphi::ESRStatus2Definition status2;
    status2.setData(data,8);

    EXPECT_FLOAT_EQ(-0.75,status2.yawRateBias());
    EXPECT_FLOAT_EQ(0.9609375,status2.vehSpdCompFactor());
    EXPECT_EQ(28581,status2.hostSWVersion());
    EXPECT_EQ(22,status2.temperature());
    EXPECT_EQ(0,status2.rawDataMode());
    EXPECT_EQ(0,status2.rangePerfError());
    EXPECT_EQ(50,status2.maximumTracksAck());
    EXPECT_EQ(0,status2.overheatError());
    EXPECT_EQ(0,status2.internalError());
    EXPECT_EQ(3,status2.groupingMode());
    EXPECT_EQ(1,status2.xcvrOperational());
    EXPECT_EQ(685,status2.steeringWheelAngleAck());
    EXPECT_EQ(3,status2.rollingCount());

    delphi::ESRStatus3Definition status3;
    status3.setData(data,8);

    EXPECT_EQ(165,status3.swVersionPLD());
    EXPECT_EQ(1223958,status3.swVersionHost());
    EXPECT_EQ(7,status3.hwVersion());
    EXPECT_EQ(12,status3.interfaceVersion());
    EXPECT_EQ(11795055,status3.serialNum());

    delphi::ESRStatus4Definition status4;
    status4.setData(data,8);

    EXPECT_EQ(1,status4.truckTargetDet());
    EXPECT_EQ(1,status4.LROnlyGratingLobeDet());
    EXPECT_EQ(0,status4.sidelobeBlockge());
    EXPECT_EQ(0,status4.partialBlockage());
    EXPECT_EQ(165,status4.pathIDACCStat());
    EXPECT_EQ(1,status4.MRLRMode());
    EXPECT_FLOAT_EQ(6.9375,status4.autoAlignAngle());
    EXPECT_EQ(3,status4.rollingCount());
    EXPECT_EQ(250,status4.pathIDFCWStat());
    EXPECT_EQ(179,status4.pathIDFCWMove());
    EXPECT_EQ(22,status4.pathIDCMBBStat());
    EXPECT_EQ(173,status4.pathIDCMBBMove());
    EXPECT_EQ(18,status4.pathIDACC());

    delphi::ESRTrackDefinition trackDefinition;
    trackDefinition.setData(data,8);

    EXPECT_EQ(1,trackDefinition.groupingChanged());
    EXPECT_EQ(1,trackDefinition.onComing());
    EXPECT_FLOAT_EQ(-3.75,trackDefinition.latRate());
    EXPECT_EQ(1,trackDefinition.bridgeObject());
    EXPECT_FLOAT_EQ(6.0,trackDefinition.width());
    EXPECT_EQ(0,trackDefinition.status());
    EXPECT_EQ(0,trackDefinition.rollingCount());
    EXPECT_FLOAT_EQ(-41.869999,trackDefinition.rangeRate());
    EXPECT_FLOAT_EQ(-0.3,trackDefinition.rangeAccel());
    EXPECT_FLOAT_EQ(130.2,trackDefinition.range());
    EXPECT_EQ(1,trackDefinition.rangeMode());
    EXPECT_FLOAT_EQ(-42.7,trackDefinition.angle());

    delphi::ESRTrackMotionPowerDefinition trkMotion;
    trkMotion.setData(data,8);

    EXPECT_EQ(7,trkMotion.group());
    EXPECT_EQ(0,trkMotion.rollingCount());
    EXPECT_EQ(0,trkMotion.track()[0].movable_slow);
    EXPECT_EQ(0,trkMotion.track()[0].movable_fast);
    EXPECT_EQ(8,trkMotion.track()[0].power);
    EXPECT_EQ(0,trkMotion.track()[0].moving);

    EXPECT_EQ(0,trkMotion.track()[1].movable_slow);
    EXPECT_EQ(3,trkMotion.track()[1].power);
    EXPECT_EQ(1,trkMotion.track()[1].moving);
    EXPECT_EQ(1,trkMotion.track()[1].movable_fast);

    EXPECT_EQ(0,trkMotion.track()[2].movable_slow);
    EXPECT_EQ(12,trkMotion.track()[2].power);
    EXPECT_EQ(0,trkMotion.track()[2].moving);
    EXPECT_EQ(0,trkMotion.track()[2].movable_fast);

    EXPECT_EQ(0,trkMotion.track()[3].movable_slow);
    EXPECT_EQ(9,trkMotion.track()[3].power);
    EXPECT_EQ(1,trkMotion.track()[3].moving);
    EXPECT_EQ(1,trkMotion.track()[3].movable_fast);

    EXPECT_EQ(1,trkMotion.track()[4].movable_slow);
    EXPECT_EQ(16,trkMotion.track()[4].power);
    EXPECT_EQ(1,trkMotion.track()[4].moving);
    EXPECT_EQ(1,trkMotion.track()[4].movable_fast);

    EXPECT_EQ(1,trkMotion.track()[5].movable_slow);
    EXPECT_EQ(5,trkMotion.track()[5].power);
    EXPECT_EQ(1,trkMotion.track()[5].moving);
    EXPECT_EQ(0,trkMotion.track()[5].movable_fast);

    EXPECT_EQ(0,trkMotion.track()[6].movable_slow);
    EXPECT_EQ(-5,trkMotion.track()[6].power);
    EXPECT_EQ(1,trkMotion.track()[6].moving);
    EXPECT_EQ(1,trkMotion.track()[6].movable_fast);

    delphi::ESRStatus5Definition status5;
    status5.setData(data,8);

    EXPECT_EQ(165,status5.supply10vA2D());
    EXPECT_EQ(22,status5.temp2A2D());
    EXPECT_EQ(173,status5.temp1A2D());
    EXPECT_EQ(199,status5.swBattA2D());
    EXPECT_EQ(250,status5.supply5VDXA2D());
    EXPECT_EQ(179,status5.supply5VAA2D());
    EXPECT_EQ(111,status5.supply3P3VA2D());
    EXPECT_EQ(18,status5.IGNPA2D());

    delphi::ESRStatus6Definition status6;
    status6.setData(data,8);

    EXPECT_EQ(1,status6.swVersionDSP3rdByte());
    EXPECT_EQ(0,status6.verticalAlignUpdated());
    EXPECT_FLOAT_EQ(-5.6875,status6.verticalMisalignment());
    EXPECT_EQ(111,status6.servAlignUpdatesDone());
    EXPECT_EQ(1,status6.foundTarget());
    EXPECT_FLOAT_EQ(-0.375,status6.factoryMisalignment());
    EXPECT_EQ(3,status6.factorAlignStatus2());
    EXPECT_EQ(6, status6.factoryAlignStatus1());
    EXPECT_EQ(0,status6.recommendUnconverge());
    EXPECT_EQ(173,status6.waveDiffA2D());
    EXPECT_EQ(6,status6.systemPowerMode());
    EXPECT_EQ(18,status6.supplyN5VA2D());
    EXPECT_EQ(199,status6.supply1P8VA2D());

    delphi::ESRStatus7Definition status7;
    status7.setData(data,8);

    EXPECT_EQ(165,status7.activeFault7());
    EXPECT_EQ(111,status7.activeFault6());
    EXPECT_EQ(250,status7.activeFault5());
    EXPECT_EQ(179,status7.activeFault4());
    EXPECT_EQ(22,status7.activeFault3());
    EXPECT_EQ(173,status7.activeFault2());
    EXPECT_EQ(199,status7.activeFault0());
    EXPECT_EQ(18,status7.activeFault1());

    delphi::ESRStatus8Definition status8;
    status8.setData(data,8);

    EXPECT_EQ(165,status8.historyFault7());
    EXPECT_EQ(111,status8.historyFault6());
    EXPECT_EQ(250,status8.historyFault5());
    EXPECT_EQ(179,status8.historyFault4());
    EXPECT_EQ(22,status8.historyFault3());
    EXPECT_EQ(173,status8.historyFault2());
    EXPECT_EQ(18,status8.historyFault1());
    EXPECT_EQ(199,status8.historyFault0());

    delphi::ESRStatus9Definition status9;
    status9.setData(data,8);

    EXPECT_EQ(165,status9.pathIDACC3());
    EXPECT_EQ(111,status9.pathIDACC2());
    EXPECT_FLOAT_EQ(-0.1875,status9.filteredXOHPACCCIPV());
    EXPECT_EQ(89,status9.waterSprayTargetId());
    EXPECT_EQ(22,status9.serialNum3rdByte());
    EXPECT_FLOAT_EQ(-42.375,status9.sideslipAngle());
    EXPECT_EQ(3185,status9.avgPwrCWBlkg());

}

TEST(DelphiMessageDefinitionParsingTest, test2)
{
    uint8_t data[8] = {0xc0,0xc7,0x67,0x53,0x0f,0xef,0xb4,0xf8};
    delphi::ESRStatus1Definition status1;
    status1.setData(data,8);

    EXPECT_EQ( 2,status1.dspTimeStamp() );
    EXPECT_EQ( 1,status1.commError() );
    EXPECT_FLOAT_EQ( -16.3125,status1.yawRateCalc() );
    EXPECT_FLOAT_EQ( 79.5,status1.vehicleSpeedCalc() );
    EXPECT_EQ(21263,status1.scanIndex());
    EXPECT_EQ( 3,status1.rollingCount() );
    EXPECT_EQ( 1895,status1.radiusCurvatureCalc() );

    delphi::ESRStatus2Definition status2;
    status2.setData(data,8);

    EXPECT_FLOAT_EQ(-2.125,status2.yawRateBias());
    EXPECT_FLOAT_EQ(1.0058594,status2.vehSpdCompFactor());
    EXPECT_EQ(46328,status2.hostSWVersion());
    EXPECT_EQ(83,status2.temperature());
    EXPECT_EQ(0,status2.rawDataMode());
    EXPECT_EQ(1,status2.rangePerfError());
    EXPECT_EQ(1,status2.overheatError());
    EXPECT_EQ(49,status2.maximumTracksAck());
    EXPECT_EQ(0,status2.internalError());
    EXPECT_EQ(3,status2.groupingMode());
    EXPECT_EQ(0,status2.xcvrOperational());
    EXPECT_EQ(1895,status2.steeringWheelAngleAck());
    EXPECT_EQ(0,status2.rollingCount());

    delphi::ESRStatus3Definition status3;
    status3.setData(data,8);

    EXPECT_EQ(248,status3.swVersionPLD());
    EXPECT_EQ(13068115,status3.swVersionHost());
    EXPECT_EQ(0,status3.hwVersion());
    EXPECT_EQ(12,status3.interfaceVersion());
    EXPECT_EQ(1044404,status3.serialNum());

    delphi::ESRStatus4Definition status4;
    status4.setData(data,8);

    EXPECT_EQ(1,status4.truckTargetDet());
    EXPECT_EQ(1,status4.LROnlyGratingLobeDet());
    EXPECT_EQ(0,status4.sidelobeBlockge());
    EXPECT_EQ(0,status4.partialBlockage());
    EXPECT_EQ(248,status4.pathIDACCStat());
    EXPECT_EQ(0,status4.MRLRMode());
    EXPECT_FLOAT_EQ(-4.75,status4.autoAlignAngle());
    EXPECT_EQ(0,status4.rollingCount());
    EXPECT_EQ(239,status4.pathIDFCWStat());
    EXPECT_EQ(15,status4.pathIDFCWMove());
    EXPECT_EQ(83,status4.pathIDCMBBStat());
    EXPECT_EQ(103,status4.pathIDCMBBMove());
    EXPECT_EQ(199,status4.pathIDACC());

    delphi::ESRTrackDefinition trackDefinition;
    trackDefinition.setData(data,8);

    EXPECT_EQ(0,trackDefinition.groupingChanged());
    EXPECT_EQ(0,trackDefinition.onComing());
    EXPECT_FLOAT_EQ(-4.0,trackDefinition.latRate());
    EXPECT_EQ(0,trackDefinition.bridgeObject());
    EXPECT_FLOAT_EQ(1.5,trackDefinition.width());
    EXPECT_EQ(6,trackDefinition.status());
    EXPECT_EQ(0,trackDefinition.rollingCount());
    EXPECT_FLOAT_EQ(-28.24,trackDefinition.rangeRate());
    EXPECT_FLOAT_EQ(-0.85000002,trackDefinition.rangeAccel());
    EXPECT_FLOAT_EQ(187.5,trackDefinition.range());
    EXPECT_EQ(2,trackDefinition.rangeMode());
    EXPECT_FLOAT_EQ(23.6,trackDefinition.angle());

    delphi::ESRTrackMotionPowerDefinition trkMotion;
    trkMotion.setData(data,8);

    EXPECT_EQ(0,trkMotion.group());
    EXPECT_EQ(0,trkMotion.rollingCount());

    EXPECT_EQ(1,trkMotion.track()[0].movable_slow);
    EXPECT_EQ(1,trkMotion.track()[0].movable_fast);
    EXPECT_EQ(-3,trkMotion.track()[0].power);
    EXPECT_EQ(0,trkMotion.track()[0].moving);

    EXPECT_EQ(1,trkMotion.track()[1].movable_slow);
    EXPECT_EQ(-3,trkMotion.track()[1].power);
    EXPECT_EQ(1,trkMotion.track()[1].moving);
    EXPECT_EQ(0,trkMotion.track()[1].movable_fast);

    EXPECT_EQ(1,trkMotion.track()[2].movable_slow);
    EXPECT_EQ(9,trkMotion.track()[2].power);
    EXPECT_EQ(0,trkMotion.track()[2].moving);
    EXPECT_EQ(0,trkMotion.track()[2].movable_fast);

    EXPECT_EQ(0,trkMotion.track()[3].movable_slow);
    EXPECT_EQ(5,trkMotion.track()[3].power);
    EXPECT_EQ(0,trkMotion.track()[3].moving);
    EXPECT_EQ(0,trkMotion.track()[3].movable_fast);

    EXPECT_EQ(1,trkMotion.track()[4].movable_slow);
    EXPECT_EQ(5,trkMotion.track()[4].power);
    EXPECT_EQ(1,trkMotion.track()[4].moving);
    EXPECT_EQ(1,trkMotion.track()[4].movable_fast);

    EXPECT_EQ(0,trkMotion.track()[5].movable_slow);
    EXPECT_EQ(10,trkMotion.track()[5].power);
    EXPECT_EQ(1,trkMotion.track()[5].moving);
    EXPECT_EQ(1,trkMotion.track()[5].movable_fast);

    EXPECT_EQ(1,trkMotion.track()[6].movable_slow);
    EXPECT_EQ(14,trkMotion.track()[6].power);
    EXPECT_EQ(1,trkMotion.track()[6].moving);
    EXPECT_EQ(1,trkMotion.track()[6].movable_fast);

    delphi::ESRStatus5Definition status5;
    status5.setData(data,8);

    EXPECT_EQ(248,status5.supply10vA2D());
    EXPECT_EQ(83,status5.temp2A2D());
    EXPECT_EQ(103,status5.temp1A2D());
    EXPECT_EQ(192,status5.swBattA2D());
    EXPECT_EQ(239,status5.supply5VDXA2D());
    EXPECT_EQ(15,status5.supply5VAA2D());
    EXPECT_EQ(180,status5.supply3P3VA2D());
    EXPECT_EQ(199,status5.IGNPA2D());

    delphi::ESRStatus6Definition status6;
    status6.setData(data,8);

    EXPECT_EQ(5,status6.swVersionDSP3rdByte());
    EXPECT_EQ(0,status6.verticalAlignUpdated());
    EXPECT_FLOAT_EQ(-0.5,status6.verticalMisalignment());
    EXPECT_EQ(180,status6.servAlignUpdatesDone());
    EXPECT_EQ(0,status6.foundTarget());
    EXPECT_FLOAT_EQ(-1.0625,status6.factoryMisalignment());
    EXPECT_EQ(7,status6.factorAlignStatus2());
    EXPECT_EQ(1, status6.factoryAlignStatus1());
    EXPECT_EQ(0,status6.recommendUnconverge());
    EXPECT_EQ(103,status6.waveDiffA2D());
    EXPECT_EQ(3,status6.systemPowerMode());
    EXPECT_EQ(199,status6.supplyN5VA2D());
    EXPECT_EQ(192,status6.supply1P8VA2D());

    delphi::ESRStatus7Definition status7;
    status7.setData(data,8);

    EXPECT_EQ(248,status7.activeFault7());
    EXPECT_EQ(180,status7.activeFault6());
    EXPECT_EQ(239,status7.activeFault5());
    EXPECT_EQ(15,status7.activeFault4());
    EXPECT_EQ(83,status7.activeFault3());
    EXPECT_EQ(103,status7.activeFault2());
    EXPECT_EQ(192,status7.activeFault0());
    EXPECT_EQ(199,status7.activeFault1());

    delphi::ESRStatus8Definition status8;
    status8.setData(data,8);

    EXPECT_EQ(248,status8.historyFault7());
    EXPECT_EQ(180,status8.historyFault6());
    EXPECT_EQ(239,status8.historyFault5());
    EXPECT_EQ(15,status8.historyFault4());
    EXPECT_EQ(83,status8.historyFault3());
    EXPECT_EQ(103,status8.historyFault2());
    EXPECT_EQ(199,status8.historyFault1());
    EXPECT_EQ(192,status8.historyFault0());


    delphi::ESRStatus9Definition status9;
    status9.setData(data,8);

    EXPECT_EQ(248,status9.pathIDACC3());
    EXPECT_EQ(180,status9.pathIDACC2());
    EXPECT_FLOAT_EQ(-0.53125,status9.filteredXOHPACCCIPV());
    EXPECT_EQ(7,status9.waterSprayTargetId());
    EXPECT_EQ(83,status9.serialNum3rdByte());
    EXPECT_FLOAT_EQ(-19.125,status9.sideslipAngle());
    EXPECT_EQ(3084,status9.avgPwrCWBlkg());

}

int main(int argc, char**argv)
{
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
