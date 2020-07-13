#include "platoon_manager.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/CARMAUtils.h>
// #include "TestHelpers.h"



TEST(PlatoonManagerTest, test1)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;


    platoon_strategic::PlatoonManager pm;
    pm.platoon = cur_pl;

    
    pm.isFollower = false;
    pm.platoonSize = 1;
    pm.leaderID = "0";
    pm.currentPlatoonID = "a";

    std::string params = "CMDSPEED:11,DOWNTRACK:01,SPEED:11";

    pm.memberUpdates("1", "1", "1", params);

    EXPECT_EQ(0, pm.platoon.size());

    
}

TEST(PlatoonManagerTest, test2)
{
    platoon_strategic::PlatoonMember* member = new platoon_strategic::PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
    std::vector<platoon_strategic::PlatoonMember> cur_pl;

    cur_pl.push_back(*member);

    platoon_strategic::PlatoonManager pm;
    pm.platoon = cur_pl;

    pm.isFollower = true;
    pm.platoonSize = 1;
    pm.leaderID = "0";
    pm.currentPlatoonID = "a";

    std::string params = "CMDSPEED:11,DOWNTRACK:01,SPEED:11";

    ros::Time::init();

    pm.updatesOrAddMemberInfo("2", "2", 2.0, 1.0, 2.5);
    // updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed)

    EXPECT_EQ(2, pm.platoon.size());
    EXPECT_EQ("2", pm.platoon[0].staticId);

}


TEST(PlatoonManagerTest, test3)
{
    platoon_strategic::PlatoonMember* member1 = new platoon_strategic::PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
    platoon_strategic::PlatoonMember* member2 = new platoon_strategic::PlatoonMember("2", "2", 2.0, 2.1, 0.2, 200);
    std::vector<platoon_strategic::PlatoonMember> cur_pl;

    cur_pl.push_back(*member1);
    cur_pl.push_back(*member2);

    platoon_strategic::PlatoonManager pm;
    pm.platoon = cur_pl;

    pm.isFollower = false;
    pm.platoonSize = 2;
    pm.leaderID = "0";
    pm.currentPlatoonID = "a";


    ros::Time::init();

    int res = pm.getTotalPlatooningSize();

    EXPECT_EQ(3, res);

}

TEST(PlatoonManagerTest, test4)
{
    platoon_strategic::PlatoonMember* member1 = new platoon_strategic::PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
    platoon_strategic::PlatoonMember* member2 = new platoon_strategic::PlatoonMember("2", "2", 2.0, 2.1, 0.2, 200);
    std::vector<platoon_strategic::PlatoonMember> cur_pl;

    cur_pl.push_back(*member1);
    cur_pl.push_back(*member2);

    platoon_strategic::PlatoonManager pm;
    pm.platoon = cur_pl;

    pm.isFollower = true;
    pm.platoonSize = 2;
    pm.leaderID = "0";
    pm.currentPlatoonID = "a";


    ros::Time::init();

    int res = pm.allPredecessorFollowing();

    EXPECT_EQ(0, res);

}


