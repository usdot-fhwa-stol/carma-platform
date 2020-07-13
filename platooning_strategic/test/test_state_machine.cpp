#include "state_machine.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include "mobility_messages.cpp"
// #include "TestHelpers.h"



TEST(PlatooningStateMachineTest, test1)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;
    

    psm.pm_ = pm;


    psm.current_platoon_state = platoon_strategic::PlatoonState::FOLLOWER;

    MobilityMessages mm;
    cav_msgs::MobilityOperation res1 = mm.getOperation1();

    psm.onMobilityOperationMessage(res1);


    
}

TEST(PlatooningStateMachineTest, test2)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;

    psm.pm_ = pm;


    psm.current_platoon_state = platoon_strategic::PlatoonState::LEADER;

    MobilityMessages mm;
    cav_msgs::MobilityRequest r1 = mm.getRequest1();

    platoon_strategic::MobilityRequestResponse out = psm.onMobilityRequestMessage(r1);



    EXPECT_EQ(2, out);

    
}

TEST(PlatooningStateMachineTest, test3)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;

    psm.pm_ = pm;
    psm.applicantID = "11";

    psm.current_platoon_state = platoon_strategic::PlatoonState::LEADERWAITING;

    MobilityMessages mm;
    cav_msgs::MobilityRequest r1 = mm.getRequest2();

    platoon_strategic::MobilityRequestResponse out = psm.onMobilityRequestMessage(r1);


    EXPECT_EQ(2, psm.current_platoon_state);
    EXPECT_EQ(0, out);

    
}

TEST(PlatooningStateMachineTest, test4)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;

    psm.pm_ = pm;
    psm.applicantID = "11";

    psm.current_platoon_state = platoon_strategic::PlatoonState::CANDIDATEFOLLOWER;

    MobilityMessages mm;
    cav_msgs::MobilityRequest r1 = mm.getRequest1();

    platoon_strategic::MobilityRequestResponse out = psm.onMobilityRequestMessage(r1);

    // cav_msgs::MobilityRequest r1;
    // psm.onMobilityRequestMessage(r1);

    // cav_msgs::MobilityResponse r1;
    // psm.onMobilityResponseMessage(r1);

    EXPECT_EQ(2, out);
}

