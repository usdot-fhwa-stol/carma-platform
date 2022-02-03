
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
#include <carma_wm/WMTestLibForGuidance.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/geometry/LineString.h>
#include <cav_msgs/MobilityResponse.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <cav_msgs/LocationECEF.h>
#include <cav_msgs/Trajectory.h>
#include <sstream>
#include <ros/package.h>

namespace platoon_strategic
{
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

        // Test JOIN_REQUIREMENTS message
        std::string strategyParams2("JOIN_REQUIREMENTS|LANE_INDEX:3");
        std::vector<std::string>inputsParams2;
        boost::algorithm::split(inputsParams2, strategyParams2, boost::is_any_of(","));
        std::vector<std::string> lane_index_parsed;
        boost::algorithm::split(lane_index_parsed, inputsParams2[0], boost::is_any_of(":"));
        int lane_index_integer = std::stoi(lane_index_parsed[1]);
        EXPECT_EQ(lane_index_integer, 3);
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

        // Test composed JOIN_REQUIREMENTS message
        std::string JOIN_REQUIREMENTS_PARAMS = "JOIN_REQUIREMENTS|LANE_INDEX:%1%";
        int lane_index = 3;
        boost::format fmter2(JOIN_REQUIREMENTS_PARAMS);
        fmter2 %lane_index;
        std::string joinRequirementsParams = fmter2.str();
        EXPECT_EQ(joinRequirementsParams, "JOIN_REQUIREMENTS|LANE_INDEX:3");
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

    TEST(PlatoonManagerTest, test5)
    {
        // File to process. 
        std::string file = "/workspaces/carma_ws/src/carma-platform/platooning_strategic/test/town01_vector_map_lane_change.osm";
        lanelet::Id start_id = 101;
        lanelet::Id end_id = 111;
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;
        // Parse geo reference info from the original lanelet map (.osm)
        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
        if (map->laneletLayer.size() == 0)
        {
            FAIL() << "Input map does not contain any lanelets";
        }
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        //Set Route
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        //get starting position
        auto llt=map.get()->laneletLayer.get(start_id);
        lanelet::BasicPoint2d curr_pose = llt.centerline2d().front();

        PlatoonPluginConfig config;
        config.maxCrosstrackError = 110;
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

        PlatoonStrategicPlugin plugin(cmw, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
        plugin.pm_.current_platoon_state = PlatoonState::LEADER;
        

        std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
        std_msgs::String proj_msg;
        proj_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(proj_msg));
        plugin.georeference_cb(msg_ptr);  // Set projection

        geometry_msgs::PoseStamped pose_msg;
        //Assign vehicle position
        pose_msg.pose.position.x = curr_pose.x();
        pose_msg.pose.position.y = curr_pose.y();
        auto mpt = boost::make_shared<const geometry_msgs::PoseStamped>(pose_msg);
        plugin.pose_cb(mpt);

        plugin.pm_.current_downtrack_distance_ = 150;

        cav_msgs::MobilityRequest request;
        request.plan_type.type = 3;
        request.strategy_params = "SIZE:1,SPEED:0,DTD:11.5599,ECEFX:1.0,ECEFY:200.0,ECEFZ:3.0";

        plugin.single_lane_road_ = false;
        plugin.in_rightmost_lane_ = true;
        
        plugin.mob_req_cb(request);

        EXPECT_EQ(plugin.pm_.current_platoon_state, PlatoonState::LEADER);
    }

    TEST(PlatoonManagerTest, test6)
    {
        // File to process. 
        std::string file = "/workspaces/carma_ws/src/carma-platform/platooning_strategic/test/town01_vector_map_lane_change.osm";
        lanelet::Id start_id = 101;
        lanelet::Id end_id = 111;
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;
        // Parse geo reference info from the original lanelet map (.osm)
        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
        if (map->laneletLayer.size() == 0)
        {
            FAIL() << "Input map does not contain any lanelets";
        }
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        //Set Route
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        //get starting position
        auto llt=map.get()->laneletLayer.get(start_id);
        lanelet::BasicPoint2d curr_pose = llt.centerline2d().front();

        PlatoonPluginConfig config;
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();

        PlatoonStrategicPlugin plugin(cmw, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
        plugin.pm_.current_platoon_state = PlatoonState::LEADER;
        std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
        std_msgs::String proj_msg;
        proj_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(proj_msg));
        plugin.georeference_cb(msg_ptr);  // Set projection

        
        geometry_msgs::PoseStamped pose_msg;
        //Assign vehicle position
        pose_msg.pose.position.x = curr_pose.x();
        pose_msg.pose.position.y = curr_pose.y();
        auto mpt = boost::make_shared<const geometry_msgs::PoseStamped>(pose_msg);
        plugin.pose_cb(mpt);

        EXPECT_EQ(plugin.single_lane_road_, false);
        EXPECT_EQ(plugin.in_rightmost_lane_, false);

    }

    TEST(PlatoonStrategicPlugin, test_platoon_formation_lane_conditions)
    {
        // Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 20;
        options.lane_width_ = 3.7;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        // Create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);

        // Set carma_wm with this map along with its speed limit and a route
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, cmw);
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);

        // Create a PlatoonStrategicPlugin for both a front and a rear vehicle and set their initial platoon state to Leader
        PlatoonPluginConfig config;
        PlatoonStrategicPlugin plugin_front(cmw, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
        plugin_front.platooning_enabled_ = true;
        plugin_front.pm_.current_platoon_state = PlatoonState::LEADER;
        PlatoonStrategicPlugin plugin_rear(cmw, config, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {}, [&](auto msg) {});
        plugin_rear.platooning_enabled_ = true;
        plugin_rear.pm_.current_platoon_state = PlatoonState::LEADER;

        // Set georeference projection for front and rear vehicle
        std::string base_proj = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        std_msgs::String georeference_msg;
        georeference_msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(georeference_msg));
        plugin_front.georeference_cb(msg_ptr);  
        plugin_rear.georeference_cb(msg_ptr);   

        //////// TEST 1: Front vehicle transitions LEADER -> LEADERWAITING after receiving a valid      /////////////
        ////////         JOIN_PLATOON_AT_REAR request if Front vehicle is in a suitable platooning lane /////////////

        // Place front vehicle in 1211 (middle lane second lanelet)
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = 4;
        msg.pose.position.y = 25;
        geometry_msgs::PoseStampedPtr mpt2(new geometry_msgs::PoseStamped(msg));
        plugin_front.pose_cb(mpt2);
        EXPECT_EQ(plugin_front.in_rightmost_lane_, false);
        plugin_front.pm_.current_downtrack_distance_ = 20;

        // Place rear vehicle in 1210 (middle lane first lanelet) and obtain the ECEF location of this point
        msg.pose.position.x = 4;
        msg.pose.position.y = 10;
        geometry_msgs::PoseStampedPtr mpt(new geometry_msgs::PoseStamped(msg));
        plugin_rear.pose_cb(mpt);
        cav_msgs::LocationECEF rear_vehicle_location = plugin_rear.pose_ecef_point_;

        // Create the rear vehicle's 'JOIN_PLATOON_AT_REAR' MobilityRequest, which will be sent to the front vehicle
        cav_msgs::MobilityRequest request;
        request.header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
        request.header.recipient_id = "default_recipient_id"; 
        request.header.sender_bsm_id = "default_host_id";
        request.header.sender_id = "default_id";
        request.header.timestamp = ros::Time::now().toNSec()/1000000;
        request.location = rear_vehicle_location; // Rear vehicle is in Lanelet 1210
        request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
        request.strategy = "Carma/Platooning";
        request.strategy_params = "SIZE:0,SPEED:4.0,DTD:15.0,ECEFX:4.0,ECEFY:10.0,ECEFZ:0.0";
        request.urgency = 50;

        // Front vehicle receives the 'JOIN_PLATOON_AT_REAR' MobilityRequest and transitions to LEADERWAITING
        plugin_front.handle_mob_req(request);
        EXPECT_EQ(plugin_front.pm_.current_platoon_state, PlatoonState::LEADERWAITING);
        EXPECT_EQ(plugin_rear.pm_.current_platoon_state, PlatoonState::LEADER);

        //////// TEST 2: Front vehicle remains LEADER after receiving a valid JOIN_PLATOON_AT_REAR      /////////////
        ////////         request if the front vehicle is in a rightmost lane. Front vehicle transitions /////////////
        ////////         LEADER -> LEADERWAITING only after entering a non-rightmost lane.              /////////////

        // Change front vehicle back to LEADER state and place it in lanelet 1221 (rightmost lane)
        plugin_front.pm_.current_platoon_state = PlatoonState::LEADER;
        msg.pose.position.x = 10;
        msg.pose.position.y = 25;
        geometry_msgs::PoseStampedPtr mpt3(new geometry_msgs::PoseStamped(msg));
        plugin_front.pose_cb(mpt3);
        EXPECT_EQ(plugin_front.in_rightmost_lane_, true);

        // Place rear vehicle in lanelet 1220 (also rightmost lane)
        msg.pose.position.x = 10;
        msg.pose.position.y = 9;
        geometry_msgs::PoseStampedPtr mpt4(new geometry_msgs::PoseStamped(msg));
        plugin_rear.pose_cb(mpt4);
        rear_vehicle_location = plugin_rear.pose_ecef_point_;
        EXPECT_EQ(plugin_front.in_rightmost_lane_, true);

        // Create the rear vehicle's 'JOIN_PLATOON_AT_REAR' MobilityRequest, which will be sent to the front vehicle
        request.location = rear_vehicle_location; // Rear vehicle is in Lanelet 1220
        request.strategy_params = "SIZE:0,SPEED:4.0,DTD:15.0,ECEFX:10.0,ECEFY:9.0,ECEFZ:0.0";

        // Front vehicle receives the 'JOIN_PLATOON_AT_REAR' MobilityRequest and does not transition to LEADERWAITING since it is in rightmost lane
        EXPECT_EQ(plugin_front.leader_lane_change_required_, false);
        plugin_front.handle_mob_req(request);
        EXPECT_EQ(plugin_front.pm_.current_platoon_state, PlatoonState::LEADER);
        EXPECT_EQ(plugin_front.current_lane_index_, 0);
        EXPECT_EQ(plugin_front.current_lane_group_size_, 3);
        EXPECT_EQ(plugin_front.leader_lane_change_required_, true);
        EXPECT_EQ(plugin_rear.pm_.current_platoon_state, PlatoonState::LEADER);

        // Place front vehicle in lanelet 1211 so it is now in the middle lane (a suitable platooning lane)
        msg.pose.position.x = 4;
        msg.pose.position.y = 25;
        geometry_msgs::PoseStampedPtr mpt5(new geometry_msgs::PoseStamped(msg));
        plugin_front.pose_cb(mpt5);
        plugin_front.onSpin(); // Trigger state transition
        EXPECT_EQ(plugin_front.in_rightmost_lane_, false);
        EXPECT_EQ(plugin_front.current_lane_index_, 1);
        EXPECT_EQ(plugin_front.leader_lane_change_required_, false);
        EXPECT_EQ(plugin_front.pm_.current_platoon_state, PlatoonState::LEADERWAITING);

        // Rear vehicle receives ACK
        plugin_rear.pm_.current_plan.valid = true;
        cav_msgs::MobilityResponse response;
        response.is_accepted = true;
        plugin_rear.mob_resp_cb(response);
        EXPECT_EQ(plugin_rear.pm_.current_platoon_state, PlatoonState::CANDIDATEFOLLOWER);

        // Create JOIN_REQUIREMENTS MobilityOperation for front vehicle to send to rear vehicle
        plugin_front.config_.vehicleID = "Front-ID";
        cav_msgs::MobilityOperation join_requirements = plugin_front.composeMobilityOperationLeaderWaiting("JOIN_REQUIREMENTS");
        EXPECT_EQ(join_requirements.strategy_params, "JOIN_REQUIREMENTS|LANE_INDEX:1,LANE_GROUP_SIZE:3");

        // Rear vehicle receives JOIN_REQUIREMENTS MobilityOperation, which includes the target_lane_index
        plugin_rear.pm_.targetLeaderId = "Front-ID";
        EXPECT_EQ(plugin_rear.has_received_join_requirements_, false);
        plugin_rear.mob_op_cb(join_requirements);
        EXPECT_EQ(plugin_rear.has_received_join_requirements_, true);
        EXPECT_EQ(plugin_rear.cf_target_lane_index_, 1);
        EXPECT_EQ(plugin_rear.cf_lane_change_required_, true);

        // Rear vehicle enters target_lane_index and is no longer required to change lanes
        msg.pose.position.x = 4;
        msg.pose.position.y = 10;
        geometry_msgs::PoseStampedPtr mpt6(new geometry_msgs::PoseStamped(msg));
        plugin_rear.pose_cb(mpt6);
        plugin_rear.onSpin(); // Trigger state transition
        EXPECT_EQ(plugin_rear.cf_lane_change_required_, false);
    }
}


