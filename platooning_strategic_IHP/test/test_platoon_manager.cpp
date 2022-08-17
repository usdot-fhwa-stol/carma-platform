
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

#include "platoon_strategic_ihp/platoon_manager_ihp.h"
#include "platoon_strategic_ihp/platoon_strategic_ihp.h"
#include "platoon_strategic_ihp/platoon_config_ihp.h"
#include <gtest/gtest.h>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>

using namespace platoon_strategic_ihp;

TEST(PlatoonManagerTest, test_construct)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {}, 
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>()); 
    // Use Getter to retrieve host Platoon Manager class
    PlatoonManager pm_ = plugin.getHostPM();
    pm_.current_platoon_state = PlatoonState::LEADER;
}

TEST(PlatoonManagerTest, test_ecef_encode)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());

    carma_v2x_msgs::msg::LocationECEF ecef_point_test;
    ecef_point_test.ecef_x = 1.0;
    ecef_point_test.ecef_y = 2.0;
    ecef_point_test.ecef_z = 3.0;
    // Update ecef location
    plugin.setHostECEF(ecef_point_test);
    plugin.run_candidate_leader();
}


TEST(PlatoonManagerTest, test_split)
{
    carma_v2x_msgs::msg::MobilityOperation msg;
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
//     PlatoonPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {});
//     pm_.current_platoon_state = PlatoonState::LEADER;
//     pm_.current_downtrack_distance_ = 20;

//     carma_v2x_msgs::msg::MobilityRequest request;
//     request.plan_type.type = 3;
//     request.strategy_params = "SIZE:1,SPEED:0,DTD:11.5599";

//     plugin.mob_req_cb(request);

//     EXPECT_EQ(pm_.current_platoon_state, PlatoonState::LEADERWAITING);
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


TEST(PlatoonStrategicIHPPlugin, mob_resp_cb)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    // Use Getter to retrieve host Platoon Manager class
    PlatoonManager pm_ = plugin.getHostPM();
    pm_.current_platoon_state = PlatoonState::FOLLOWER;

    plugin.onSpin();
   
}

TEST(PlatoonStrategicIHPPlugin, platoon_info_pub)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    // Use Getter to retrieve host Platoon Manager class
    PlatoonManager pm_ = plugin.getHostPM();
    pm_.current_platoon_state = PlatoonState::LEADER;

    carma_planning_msgs::msg::PlatooningInfo info_msg1 = plugin.composePlatoonInfoMsg();
    EXPECT_EQ(info_msg1.leader_id, "default_id");

    pm_.current_platoon_state = PlatoonState::FOLLOWER;
    pm_.isFollower = true;
    PlatoonMember member = PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    std::vector<PlatoonMember> cur_pl;
    cur_pl.push_back(member);
    pm_.host_platoon_ = cur_pl;
    
    carma_planning_msgs::msg::PlatooningInfo info_msg2 = plugin.composePlatoonInfoMsg();
    EXPECT_EQ(info_msg2.leader_id, "1");
}

TEST(PlatoonStrategicIHPPlugin, test_follower)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    // Use Getter to retrieve host Platoon Manager class
    PlatoonManager pm_ = plugin.getHostPM();
    pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
    pm_.current_plan.valid = true;
    EXPECT_EQ(pm_.isFollower, false);

    auto resp = std::make_unique<carma_v2x_msgs::msg::MobilityResponse>();
    resp->m_header.plan_id = "resp";
    resp->is_accepted = true;
    plugin.mob_resp_cb(std::move(resp));
    EXPECT_EQ(pm_.current_platoon_state, PlatoonState::FOLLOWER);
    EXPECT_EQ(pm_.isFollower, true);
}

TEST(PlatoonStrategicIHPPlugin, test_get_leader)
{
    PlatoonPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatoonStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    // Use Getter to retrieve host Platoon Manager class
    PlatoonManager pm_ = plugin.getHostPM();
    pm_.current_platoon_state = PlatoonState::FOLLOWER;

    PlatoonMember member = PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    std::vector<PlatoonMember> cur_pl;
    cur_pl.push_back(member);

    pm_.host_platoon_ = cur_pl;

    EXPECT_EQ(pm_.host_platoon_.size(), 1ul);

    PlatoonMember member1 = pm_.host_platoon_[0];

    pm_.isFollower = true;
    PlatoonMember platoon_leader = pm_.getDynamicLeader();

    EXPECT_EQ(member1.staticId, "1");
    EXPECT_EQ(platoon_leader.staticId, "1");
}


TEST(PlatoonManagerTest, test2)
{
    platoon_strategic_ihp::PlatoonMember* member = new platoon_strategic_ihp::PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    std::vector<platoon_strategic_ihp::PlatoonMember> cur_pl;

    cur_pl.push_back(*member);

    platoon_strategic_ihp::PlatoonManager pm(std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    pm.host_platoon_ = cur_pl;

    pm.isFollower = true;
    pm.platoonLeaderID = "0";
    pm.currentPlatoonID = "a";

    std::string params = "CMDSPEED:11,DOWNTRACK:01,SPEED:11";

    pm.updatesOrAddMemberInfo(cur_pl, "2", 2.0, 1.0, 0.0, 2.5); //HERE

    EXPECT_EQ(2ul, cur_pl.size());
    EXPECT_EQ("2", cur_pl[0].staticId);
}


TEST(PlatoonManagerTest, test3)
{
    platoon_strategic_ihp::PlatoonMember* member1 = new platoon_strategic_ihp::PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    platoon_strategic_ihp::PlatoonMember* member2 = new platoon_strategic_ihp::PlatoonMember("2", 2.0, 2.1, 0.2, 0, 200);
    std::vector<platoon_strategic_ihp::PlatoonMember> cur_pl;

    cur_pl.push_back(*member1);
    cur_pl.push_back(*member2);

    platoon_strategic_ihp::PlatoonManager pm(std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    pm.host_platoon_ = cur_pl;

    pm.isFollower = false;
    pm.platoonLeaderID = "0";
    pm.currentPlatoonID = "a";

    int res = pm.getHostPlatoonSize();

    EXPECT_EQ(2, res);

}

TEST(PlatoonManagerTest, test4)
{
    platoon_strategic_ihp::PlatoonMember* member1 = new platoon_strategic_ihp::PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    platoon_strategic_ihp::PlatoonMember* member2 = new platoon_strategic_ihp::PlatoonMember("2", 2.0, 2.1, 0.2, 0, 200);
    std::vector<platoon_strategic_ihp::PlatoonMember> cur_pl;

    cur_pl.push_back(*member1);
    cur_pl.push_back(*member2);

    platoon_strategic_ihp::PlatoonManager pm(std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    pm.host_platoon_ = cur_pl;

    pm.isFollower = true;
    pm.platoonLeaderID = "0";
    pm.currentPlatoonID = "a";

    int res = pm.allPredecessorFollowing();

    EXPECT_EQ(0, res);
}
