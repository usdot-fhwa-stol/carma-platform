
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

------------------------------------------------------------------------------*/

#include "platoon_manager.h"
#include "platoon_strategic.h"
#include "platoon_config.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/CARMAUtils.h>
// #include "TestHelpers.h"

using namespace platoon_strategic;

TEST(PlatoonManagerTest, test_construct)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    plugin.pm_.current_platoon_state = PlatoonState::LEADER;

}

TEST(PlatoonManagerTest, test_ecef_encode)
{
    ros::Time::init();

    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    cav_msgs::LocationECEF ecef_point_test;
    ecef_point_test.ecef_x = 1.0;
    ecef_point_test.ecef_y = 2.0;
    ecef_point_test.ecef_z = 3.0;
    plugin.pose_ecef_point_ = ecef_point_test;
    plugin.run_leader_waiting();

}


TEST(PlatoonManagerTest, test_split)
{
    cav_msgs::MobilityOperation msg;
    std::string strategyParams("INFO|REAR:1,LENGTH:2,SPEED:3,SIZE:4");
    std::vector<std::string> inputsParams;
    boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));
    std::vector<std::string> rearVehicleBsmId_parsed;
    boost::algorithm::split(rearVehicleBsmId_parsed, inputsParams[0], boost::is_any_of(":"));
    std::string rearVehicleBsmId = rearVehicleBsmId_parsed[1];
    std::cout << "rearVehicleBsmId: " << rearVehicleBsmId << std::endl;

    std::vector<std::string> rearVehicleDtd_parsed;
    boost::algorithm::split(rearVehicleDtd_parsed, inputsParams[1], boost::is_any_of(":"));
    double rearVehicleDtd = std::stod(rearVehicleDtd_parsed[1]);
    std::cout << "rearVehicleDtd: " << rearVehicleDtd << std::endl;
}


// TEST(PlatoonManagerTest, test_states)
// {
//     ros::Time::init();

//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
//     plugin.pm_.current_platoon_state = PlatoonState::LEADER;
//     plugin.pm_.current_downtrack_distance_ = 20;

//     cav_msgs::MobilityRequest request;
//     request.plan_type.type = 3;
//     request.strategy_params = "SIZE:1,SPEED:0,DTD:11.5599";

//     plugin.mob_req_cb(request);

//     EXPECT_EQ(plugin.pm_.current_platoon_state, PlatoonState::LEADERWAITING);
// }

TEST(PlatoonManagerTest, test_compose)
{
    std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,DTD:%2%,SPEED:%3%";
    double cmdSpeed = 1;
    double current_speed = 2;
    double current_downtrack = 4;
    boost::format fmter(OPERATION_STATUS_PARAMS);
    fmter %cmdSpeed;
    fmter %current_downtrack;
    fmter %current_speed;
    std::string statusParams = fmter.str();

    std::cout << "statusParams: " << statusParams << std::endl;
}


TEST(PlatoonStrategicPlugin, mob_resp_cb)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    plugin.pm_.current_platoon_state = PlatoonState::FOLLOWER;

    plugin.onSpin();
   
}

TEST(PlatoonStrategicPlugin, platoon_info_pub)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    plugin.pm_.current_platoon_state = PlatoonState::LEADER;

    cav_msgs::PlatooningInfo info_msg1 = plugin.composePlatoonInfoMsg();
    EXPECT_EQ(info_msg1.leader_id, "default_id");

    plugin.pm_.current_platoon_state = PlatoonState::FOLLOWER;
    plugin.pm_.isFollower = true;
    PlatoonMember member = PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
    std::vector<PlatoonMember> cur_pl;
    cur_pl.push_back(member);
    plugin.pm_.platoon = cur_pl;

    
    cav_msgs::PlatooningInfo info_msg2 = plugin.composePlatoonInfoMsg();
    EXPECT_EQ(info_msg2.leader_id, "1");
   
}

TEST(PlatoonStrategicPlugin, test_follower)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    plugin.pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
    plugin.pm_.current_plan.valid = true;
    EXPECT_EQ(plugin.pm_.isFollower, false);

    cav_msgs::MobilityResponse resp;
    resp.header.plan_id = "resp";
    resp.is_accepted = true;
    plugin.mob_resp_cb(resp);
    EXPECT_EQ(plugin.pm_.current_platoon_state, PlatoonState::FOLLOWER);
    EXPECT_EQ(plugin.pm_.isFollower, true);
}

TEST(PlatoonStrategicPlugin, test_get_leader)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicPlugin plugin(wm, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
    plugin.pm_.current_platoon_state = PlatoonState::FOLLOWER;

    PlatoonMember member = PlatoonMember("1", "1", 1.0, 1.1, 0.1, 100);
    std::vector<PlatoonMember> cur_pl;
    cur_pl.push_back(member);

    plugin.pm_.platoon = cur_pl;

    EXPECT_EQ(plugin.pm_.platoon.size(), 1);

    PlatoonMember member1 = plugin.pm_.platoon[0];

    plugin.pm_.isFollower = true;
    PlatoonMember platoon_leader = plugin.pm_.getLeader();

    EXPECT_EQ(member1.staticId, "1");

    EXPECT_EQ(platoon_leader.staticId, "1");
   
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

    EXPECT_EQ(2, pm.platoon.size());
    EXPECT_EQ("1", pm.platoon[0].staticId);

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


