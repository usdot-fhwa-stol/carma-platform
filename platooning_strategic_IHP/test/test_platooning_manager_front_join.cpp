
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

/*
 * Developed by the UCLA Mobility Lab, 10/20/2021.
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
 */

#include "platooning_strategic_ihp/platooning_manager_ihp.h"
#include "platooning_strategic_ihp/platooning_strategic_ihp.h"
#include "platooning_strategic_ihp/platooning_config_ihp.h"
#include <gtest/gtest.h>
#include <rclcpp/logging.hpp>
#include <carma_wm/WMListener.hpp>
#include <carma_wm/WorldModel.hpp>
#include <carma_wm/CARMAWorldModel.hpp>
#include <carma_wm/WMTestLibForGuidance.hpp>
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>
#include <string>
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

using carma_ros2_utils::timers::testing::TestTimer;
using carma_ros2_utils::timers::testing::TestTimerFactory;

namespace platooning_strategic_ihp
{

TEST(PlatooningManagerTestFrontJoin, test_construct_front)
{
    PlatooningPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    // Use Getter to retrieve host Platoon Manager class
    // change state
    plugin.setPMState(PlatoonState::LEADER);
}

// UCLA: use "run_candidate_leader" to test ecef encoding
TEST(PlatooningManagerTestFrontJoin, test_ecef_encode_front)
{
    PlatooningPluginConfig config;
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

    PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    carma_v2x_msgs::msg::LocationECEF ecef_point_test;
    ecef_point_test.ecef_x = 1.0;
    ecef_point_test.ecef_y = 2.0;
    ecef_point_test.ecef_z = 3.0;
    // Update ecef location
    plugin.setHostECEF(ecef_point_test);
    plugin.run_candidate_leader();
}


TEST(PlatooningManagerTestFrontJoin, test_split_front)
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


TEST(PlatooningManagerTestFrontJoin, test_compose_front)
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


TEST(PlatooningStrategicIHPPlugin, mob_resp_cb_front)
{
    PlatooningPluginConfig config;;

    PlatooningStrategicIHPPlugin plugin(carma_wm::test::getGuidanceTestMap(), config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    // change state
    plugin.setPMState(PlatoonState::FOLLOWER);

    plugin.onSpin();
}

TEST(PlatooningStrategicIHPPlugin, platoon_info_pub_front)
{
    PlatooningPluginConfig config;

    PlatooningStrategicIHPPlugin plugin(carma_wm::test::getGuidanceTestMap(), config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
        std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());

    // Use platoon manager setter to set state
    plugin.setPMState(PlatoonState::LEADER);
    // compose platoon info
    carma_planning_msgs::msg::PlatooningInfo info_msg1 = plugin.composePlatoonInfoMsg();
    // check platoon info (Host is a single ADS vehicle, so it's also the leader)
    EXPECT_EQ(info_msg1.leader_id, "default_id");

    // set host to follower
    plugin.setPMState(PlatoonState::FOLLOWER);
    // add member
    PlatoonMember member = PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    std::vector<PlatoonMember> cur_pl;
    cur_pl.push_back(member);
    // update platoon list
    plugin.updatePlatoonList(cur_pl);

    plugin.pm_.changeFromLeaderToFollower("a", "1");
    // compose platoon info
    carma_planning_msgs::msg::PlatooningInfo info_msg2 = plugin.composePlatoonInfoMsg();
    // check platoo info (when host is follower, the newly added member will be the leader)
    EXPECT_EQ(info_msg2.leader_id, "1");
}

// These tests has been temporarily disabled to support Continuous Improvement (CI) processes.
// Related GitHub Issue: <https://github.com/usdot-fhwa-stol/carma-platform/issues/2335>

// ---------------------------------------- UCLA Tests for same-lane front join and cut-in front join ---------------------------------------
// UCLA: Use the transition "leader_aborting --> follower" to test follower functions
// TEST(PlatooningStrategicIHPPlugin, test_follower_front)
// {

//     PlatooningPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
//         std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
//     // change state
//     plugin.setPMState(PlatoonState::LEADERABORTING);
//     // Use Getter to retrieve host Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm_ = plugin.getHostPM();
//     EXPECT_EQ(pm_.isFollower, false);

//     auto resp = std::make_unique<carma_v2x_msgs::msg::MobilityResponse>();

//     resp->is_accepted = true;
//     // sender id need to match with default value
//     resp->m_header.sender_id = pm_.neighbor_platoon_leader_id_;
//     // plan_id need to match with default value, which is ""
//     resp->m_header.plan_id = pm_.current_plan.planId;
//     plugin.mob_resp_cb(std::move(resp));

//     // Use Getter to retrieve an updated Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm1_ = plugin.getHostPM();
//     EXPECT_EQ(pm1_.current_platoon_state, PlatoonState::FOLLOWER);
//     EXPECT_EQ(pm1_.isFollower, true);
// }

// UCLA: Cut-in front test for platoon leader, Test the transition "lead_with_operation --> leader_aborting"
// TEST(PlatooningStrategicIHPPlugin, cutin_test_leader_2)
// {
//     PlatooningPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
//         std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());

//     // change state
//     plugin.setPMState(PlatoonState::LEADWITHOPERATION);
//     // Use Getter to retrieve host Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm_ = plugin.getHostPM();
//     EXPECT_EQ(pm_.isFollower, false);

//     // compose dummy request msg
//     carma_v2x_msgs::msg::MobilityRequest msg;
//     msg.m_header.plan_id = "req";
//     msg.plan_type.type = carma_v2x_msgs::msg::PlanType::CUT_IN_FRONT_DONE;

//     // compose dummy response msg
//     MobilityRequestResponse resp = plugin.handle_mob_req(msg);

//     // Use Getter to update Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm1_ = plugin.getHostPM();

//     // run test and compare result
//     EXPECT_EQ(pm1_.current_platoon_state, PlatoonState::LEADERABORTING);
//     EXPECT_EQ(resp, MobilityRequestResponse::ACK);
// }

// UCLA: Cut-in front test for joining vehicle. Test the transition "leader --> prepare to join"
// TEST(PlatooningStrategicIHPPlugin, cutin_test_join_1)
// {
//     PlatooningPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
//         std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
//     // Use Getter to retrieve host Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm_ = plugin.getHostPM();
//     plugin.setPMState(PlatoonState::LEADER);
//     EXPECT_EQ(pm_.isFollower, false);

//     // compose dummy response msg
//     auto resp = std::make_unique<carma_v2x_msgs::msg::MobilityResponse>();

//     // compose plan ID and leader ID
//     resp->m_header.plan_id = pm_.current_plan.planId;
//     resp->m_header.sender_id = pm_.current_plan.peerId;
//     // plan type and acceptance
//     resp->plan_type.type = carma_v2x_msgs::msg::PlanType::CUT_IN_FROM_SIDE;
//     resp->is_accepted = true;

//     // send dummy reponse
//     plugin.mob_resp_cb(std::move(resp));

//     // Use Getter to retrieve host Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm1_ = plugin.getHostPM();
//     EXPECT_EQ(pm1_.current_platoon_state, PlatoonState::PREPARETOJOIN);

// }

//UCLA: Cut-in front test for joining vehicle. Test the transition "prepare to join --> candidate leader"
// TEST(PlatooningStrategicIHPPlugin, cutin_test_join_2)
// {
//     PlatooningPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
//         std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
//     // change state
//     plugin.setPMState(PlatoonState::PREPARETOJOIN);
//     // Use Getter to retrieve host Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm_ = plugin.getHostPM();
//     EXPECT_EQ(pm_.isFollower, false);

//     // compose dummy response msg
//     auto resp = std::make_unique<carma_v2x_msgs::msg::MobilityResponse>();
//     // sender id need to match with default value
//     resp->m_header.sender_id = pm_.neighbor_platoon_leader_id_;
//     // plan_id need to match with default value, which is ""
//     resp->m_header.plan_id = pm_.current_plan.planId;
//     resp->is_accepted = true;
//     resp->plan_type.type = carma_v2x_msgs::msg::PlanType::CUT_IN_FRONT_DONE;

//     // send dummy reponse
//     plugin.mob_resp_cb(std::move(resp));

//     // run test and compare result
//     // Use Getter to update Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm1_ = plugin.getHostPM();
//     EXPECT_EQ(pm1_.current_platoon_state, PlatoonState::CANDIDATELEADER);
// }

// -------------------------------------------------------------------------------------------------------------------------------------------

// test get leader function
// TEST(PlatooningStrategicIHPPlugin, test_get_leader_front)
// {
//     PlatooningPluginConfig config;
//     std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

//     PlatooningStrategicIHPPlugin plugin(wm, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
//         std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
//     // change state
//     plugin.setPMState(PlatoonState::FOLLOWER);

//     PlatoonMember member = PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
//     std::vector<PlatoonMember> cur_pl;
//     cur_pl.push_back(member);

//     // update platoon list
//     plugin.updatePlatoonList(cur_pl);

//     // Use Getter to retrieve host Platoon Manager class
//     platooning_strategic_ihp::PlatooningManager pm_ = plugin.getHostPM();
//     // check platoon size
//     EXPECT_EQ(pm_.host_platoon_.size(), 1ul);

//     PlatoonMember member1 = pm_.host_platoon_[0];
//     //check member info
//     EXPECT_EQ(member1.staticId, "1");

//     // PM should now be follower
//     // update PM
//     platooning_strategic_ihp::PlatooningManager pm1_ = plugin.getHostPM();
//     PlatoonMember platoon_leader = pm1_.getDynamicLeader();
    // check dynamic leader info
//     EXPECT_EQ(platoon_leader.staticId, "1");

// }

// test platoon size and get leader platoon ID
TEST(PlatooningManagerTestFrontJoin, test2_front)
{
    platooning_strategic_ihp::PlatoonMember* member = new platooning_strategic_ihp::PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100); // dtd: 0.1m, smaller downtrack, at back.
    std::vector<platooning_strategic_ihp::PlatoonMember> cur_pl;

    cur_pl.push_back(*member);

    platooning_strategic_ihp::PlatooningManager pm(std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    pm.host_platoon_ = cur_pl;

    pm.isFollower = true;
    pm.platoonLeaderID = "2";
    pm.currentPlatoonID = "a";

    std::string params = "CMDSPEED:11,DOWNTRACK:01,SPEED:11";

    pm.updatesOrAddMemberInfo(cur_pl, "2", 2.0, 1.0, 0.0, 2.5); // dtd: 1.0m, larger down track, in front.

    EXPECT_EQ(2ul, cur_pl.size());
    EXPECT_EQ("2", cur_pl[0].staticId);  // sorted by dtd distance, larger downtrack means vehicle at front, hence ranked higher.
}

// add 2 member to platoon, test size
TEST(PlatooningManagerTestFrontJoin, test3_front)
{
    // note: platoon list now include leading vehicle. So platoon_size = host_platoon_.size() , here expect platoon_size == 2.
    platooning_strategic_ihp::PlatoonMember* member1 = new platooning_strategic_ihp::PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    platooning_strategic_ihp::PlatoonMember* member2 = new platooning_strategic_ihp::PlatoonMember("2", 2.0, 2.1, 0.2, 0, 200);
    std::vector<platooning_strategic_ihp::PlatoonMember> cur_pl;

    cur_pl.push_back(*member1);
    cur_pl.push_back(*member2);

    platooning_strategic_ihp::PlatooningManager pm(std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    pm.host_platoon_ = cur_pl;

    pm.isFollower = false;
    pm.platoonLeaderID = "0";
    pm.currentPlatoonID = "a";

    int res = pm.getHostPlatoonSize();

    EXPECT_EQ(2, res);
}

// test APF for 3 vehicles
TEST(PlatooningManagerTestFrontJoin, test4_front)
{
    platooning_strategic_ihp::PlatoonMember* member1 = new platooning_strategic_ihp::PlatoonMember("1", 1.0, 1.1, 0.1, 0, 100);
    platooning_strategic_ihp::PlatoonMember* member2 = new platooning_strategic_ihp::PlatoonMember("2", 2.0, 2.1, 0.2, 0, 200);
    std::vector<platooning_strategic_ihp::PlatoonMember> cur_pl;

    cur_pl.push_back(*member1);
    cur_pl.push_back(*member2);

    platooning_strategic_ihp::PlatooningManager pm(std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());
    pm.host_platoon_ = cur_pl;

    pm.isFollower = true;
    pm.platoonLeaderID = "0";
    pm.currentPlatoonID = "a";

    int res = pm.allPredecessorFollowing();

    EXPECT_EQ(0, res);
}

// These tests has been temporarily disabled to support Continuous Improvement (CI) processes.
// Related GitHub Issue: <https://github.com/usdot-fhwa-stol/carma-platform/issues/2335>

// TEST(PlatooningStrategicIHPPlugin, is_lanechange_possible)
//     {
//         std::string path = ament_index_cpp::get_package_share_directory("platooning_strategic_ihp");
//         std::string file = "/resource/map/town01_vector_map_lane_change.osm";
//         file = path.append(file);
//         int projector_type = 0;
//         std::string target_frame;
//         lanelet::ErrorMessages load_errors;
//         lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
//         lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
//         lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
//         if (map->laneletLayer.size() == 0)
//         {
//             FAIL() << "Input map does not contain any lanelets";
//         }
//         std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
//         cmw->carma_wm::CARMAWorldModel::setMap(map);
//         //Set Route
//         lanelet::Id start_id = 106;
//         lanelet::Id end_id = 110;
//         carma_wm::test::setRouteByIds({start_id, end_id}, cmw);
//         cmw->carma_wm::CARMAWorldModel::setMap(map);

//         PlatooningPluginConfig config;

//         PlatooningStrategicIHPPlugin plugin(cmw, config, [&](auto) {}, [&](auto) {}, [&](auto) {}, [&](auto) {},
//         std::make_shared<carma_ros2_utils::timers::testing::TestTimerFactory>());

//         lanelet::Id target_id1 = 111;
//         bool lanechange_possible = plugin.is_lanechange_possible(start_id, target_id1);

//         EXPECT_TRUE(lanechange_possible);

//         lanelet::Id target_id2 = 146;
//         bool lanechange_possible2 = plugin.is_lanechange_possible(start_id, target_id2);
//         EXPECT_FALSE(lanechange_possible2);


//     }

} // Namespace required for FRIEND_TEST to allow access to private members
