/*
 * Copyright (C) 2022 LEIDOS.
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
 */

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>

namespace light_controlled_intersection_tactical_plugin
{
    // TODO: The package requires additional unit tests to improve unit test coverage. These unit tests will be created
    //       in a follow-on story.
    
    TEST(LCITacticalPluginTest, applyTrajectorySmoothingAlgorithm)
    {
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
        auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);
        wm->setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, wm);
        carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);
        
        
        rclcpp::NodeOptions options;
        auto lci_node = std::make_shared<light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode>(options);
        Config config;
        config.minimum_speed = 1;

        std::vector<PointSpeedPair> points_and_target_speeds;

        auto lci_tactical = LightControlledIntersectionTacticalPlugin(wm, config, lci_node->get_node_logging_interface());
    
        TrajectoryParams tp;
        tp.v1_ = 1.0;
        tp.v2_ = 2.0;
        tp.v3_ = 2.0;
        tp.x1_ = 2.0;
        tp.x2_ = 4.0;
        tp.x3_ = 5.0;

        EXPECT_THROW(lci_tactical.applyTrajectorySmoothingAlgorithm(wm, points_and_target_speeds, 0, 0, 0, 0, tp), std::invalid_argument);

        PointSpeedPair pair;
        pair.point = {1., 1.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 2.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 3.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 4.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 5.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);

        lci_tactical.applyTrajectorySmoothingAlgorithm(wm, points_and_target_speeds, 1, 4, 1, 1, tp);
        
        EXPECT_NEAR(points_and_target_speeds.front().speed, 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().speed, 2.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.front().point.y(), 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().point.y(), 5.0, 0.001);

        lci_tactical.applyTrajectorySmoothingAlgorithm(wm, points_and_target_speeds, 1, 5, 1, 1, tp);

        EXPECT_NEAR(points_and_target_speeds.front().speed, 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().speed, 2.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.front().point.y(), 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().point.y(), 5.0, 0.001);


        points_and_target_speeds = {};
        pair.point = {1., 1.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 2.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 3.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 4.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 5.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);

        tp.v1_ = 1.0;
        tp.a1_ = 0.5;
        tp.v2_ = 2.0;
        tp.a2_ = 0.5;
        tp.v3_ = 3.0;
        tp.a3_ = 0.5;
        tp.x1_ = 2.0;
        tp.x2_ = 4.0;
        tp.x3_ = 5.0;

        lci_tactical.applyTrajectorySmoothingAlgorithm(wm, points_and_target_speeds, 1, 5, 1, 1, tp);

        EXPECT_NEAR(points_and_target_speeds.front().speed, 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().speed, 2.23606, 0.001);
        EXPECT_NEAR(points_and_target_speeds.front().point.y(), 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().point.y(), 5.0, 0.001);

    }
    
    TEST(LCITacticalPluginTest, applyOptimizedTargetSpeedProfile)
    {
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
        auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);
        wm->setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, wm);
        carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);
        
        
        rclcpp::NodeOptions options;
        auto lci_node = std::make_shared<light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode>(options);
        Config config;
        config.minimum_speed = 1;

        std::vector<PointSpeedPair> points_and_target_speeds;

        auto lci_tactical = LightControlledIntersectionTacticalPlugin(wm, config, lci_node->get_node_logging_interface());
        
        carma_planning_msgs::msg::Maneuver maneuver_msg;
        maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type =
            carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector =
            carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA;
        maneuver_msg.lane_following_maneuver.start_dist = 1;
        maneuver_msg.lane_following_maneuver.start_speed = 1;
        maneuver_msg.lane_following_maneuver.end_dist = 4;
        maneuver_msg.lane_following_maneuver.end_speed = 1;
        
        TrajectoryParams tsp;
        tsp.v1_ = 1.0;
        tsp.v2_ = 2.0;
        tsp.v3_ = 2.0;
        tsp.x1_ = 2.0;
        tsp.x2_ = 4.0;
        tsp.x3_ = 5.0;

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a1_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v1_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x1_);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a2_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v2_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x2_);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a3_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v3_);

        maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);
        maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);

        PointSpeedPair pair;
        pair.point = {1., 1.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 2.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 3.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 4.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);
        pair.point = {1., 5.};
        pair.speed = 99.0;
        points_and_target_speeds.push_back(pair);

        EXPECT_THROW(lci_tactical.applyOptimizedTargetSpeedProfile(maneuver_msg, maneuver_msg.lane_following_maneuver.start_speed, points_and_target_speeds), std::invalid_argument);
        
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x3_);

        lci_tactical.applyOptimizedTargetSpeedProfile(maneuver_msg, maneuver_msg.lane_following_maneuver.start_speed, points_and_target_speeds);

        EXPECT_NEAR(points_and_target_speeds.front().speed, 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().speed, 2.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.front().point.y(), 1.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().point.y(), 5.0, 0.001);

    }
      
    TEST(LCITacticalPluginTest, createGeometryProfile)
    {
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
        auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);
        wm->setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, wm);
        carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);
        
        
        rclcpp::NodeOptions options;
        auto lci_node = std::make_shared<light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode>(options);
        Config config;
        config.minimum_speed = 1;

        auto lci_tactical = LightControlledIntersectionTacticalPlugin(wm, config, lci_node->get_node_logging_interface());
        
        carma_planning_msgs::msg::Maneuver maneuver_msg;
        maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type =
            carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector =
            carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA;
        maneuver_msg.lane_following_maneuver.start_dist = 1;
        maneuver_msg.lane_following_maneuver.start_speed = 1;
        maneuver_msg.lane_following_maneuver.end_dist = 4;
        maneuver_msg.lane_following_maneuver.end_speed = 2;
        
        TrajectoryParams tsp;
        tsp.v1_ = 1.0;
        tsp.v2_ = 2.0;
        tsp.v3_ = 2.0;
        tsp.x1_ = 2.0;
        tsp.x2_ = 4.0;
        tsp.x3_ = 5.0;

        carma_planning_msgs::msg::VehicleState state;
        state.x_pos_global = 1.0;
        state.y_pos_global = 0.0;
        state.longitudinal_vel = 1;

        carma_planning_msgs::msg::VehicleState ending_state;
        ending_state.x_pos_global = 1.0;
        ending_state.y_pos_global = 5.0;
        ending_state.longitudinal_vel = 2.0;

        DetailedTrajConfig wpg_detail_config;
        GeneralTrajConfig wpg_general_config;
        wpg_general_config.default_downsample_ratio = 1;

        EXPECT_THROW(lci_tactical.createGeometryProfile({maneuver_msg}, 1, wm, ending_state, state, wpg_general_config, wpg_detail_config), std::invalid_argument);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a1_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v1_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x1_);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a2_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v2_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x2_);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a3_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v3_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x3_);

        maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);
        maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);

        maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(1200));

        EXPECT_THROW(lci_tactical.createGeometryProfile({maneuver_msg, maneuver_msg}, 1, wm, ending_state, state, wpg_general_config, wpg_detail_config), std::invalid_argument);


        EXPECT_EQ(lci_tactical.createGeometryProfile({maneuver_msg}, 1, wm, ending_state, state, wpg_general_config, wpg_detail_config).size(), 7);
        
        auto points_and_target_speeds = lci_tactical.createGeometryProfile({maneuver_msg}, 1, wm, ending_state, state, wpg_general_config, wpg_detail_config);
        
        EXPECT_NEAR(points_and_target_speeds.front().speed, 2.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.back().speed, 2.0, 0.001);
        EXPECT_NEAR(points_and_target_speeds.front().point.y(), 0.0, 0.001);

    }

    TEST(LCITacticalPluginTest, planTrajectoryCB)
    {
        std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
        auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);
        wm->setMap(map);
        carma_wm::test::setSpeedLimit(15_mph, wm);
        carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);
        
        
        rclcpp::NodeOptions options;
        auto lci_node = std::make_shared<light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode>(options);
        Config config;
        config.minimum_speed = 1;
        config.default_downsample_ratio = 1;

        auto lci_tactical = LightControlledIntersectionTacticalPlugin(wm, config, lci_node->get_node_logging_interface());
        
        carma_planning_msgs::msg::Maneuver maneuver_msg;
        maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type =
            carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector =
            carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA | carma_planning_msgs::msg::ManeuverParameters::HAS_INT_META_DATA;
        maneuver_msg.lane_following_maneuver.start_dist = 1;
        maneuver_msg.lane_following_maneuver.start_speed = 1;
        maneuver_msg.lane_following_maneuver.end_dist = 11.0;
        maneuver_msg.lane_following_maneuver.end_speed = 2;
        
        TrajectoryParams tsp;
        tsp.v1_ = 1.0;
        tsp.v2_ = 2.0;
        tsp.v3_ = 2.0;
        tsp.x1_ = 2.0;
        tsp.x2_ = 4.0;
        tsp.x3_ = 5.0;

        carma_planning_msgs::msg::VehicleState state;
        state.x_pos_global = 1.0;
        state.y_pos_global = 0.1;
        state.longitudinal_vel = 1;

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a1_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v1_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x1_);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a2_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v2_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x2_);

        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.a3_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.v3_);
        maneuver_msg.lane_following_maneuver.parameters.float_valued_meta_data.push_back(tsp.x3_);

        maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);
        maneuver_msg.lane_following_maneuver.parameters.int_valued_meta_data.push_back(1);
        maneuver_msg.lane_following_maneuver.parameters.string_valued_meta_data.push_back("signalized");

        maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(1200));

        auto req = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
        auto resp = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Response>();
        
        req->maneuver_plan.maneuvers.push_back(maneuver_msg);
        req->maneuver_index_to_plan = 0;
        req->vehicle_state = state;

        lci_tactical.planTrajectoryCB(req, resp);
        
        EXPECT_NEAR(rclcpp::Time(resp->trajectory_plan.trajectory_points.front().target_time).seconds(), 0.0, 0.001);
        EXPECT_NEAR(rclcpp::Time(resp->trajectory_plan.trajectory_points.back().target_time).seconds(), 9.23, 0.1);
        EXPECT_NEAR(resp->trajectory_plan.trajectory_points.front().y, 0.1, 0.001);

    }

    
} // namespace light_controlled_intersection_tactical_plugin


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 