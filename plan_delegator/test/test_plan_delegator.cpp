/*
 * Copyright (C) 2022-2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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

#include <thread>
#include <chrono>
#include <stdexcept>
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_wm/WMTestLibForGuidance.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "plan_delegator.hpp"

namespace plan_delegator{


    TEST(TestPlanDelegator, UnitTestPlanDelegator) {
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);

        // Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 25;
        options.lane_width_ = 3.7;
        options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

        // Create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);

        // Set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        // Compute and print shortest path
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(1213);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);

        cmw.get()->setConfigSpeedLimit(30.0);

        pd->wm_ = cmw;

        // Test initialization
        EXPECT_EQ(pd->config_.planning_topic_prefix, "/plugins/");
        EXPECT_EQ(pd->config_.planning_topic_suffix, "/plan_trajectory");
        EXPECT_EQ(pd->config_.trajectory_planning_rate, 10.0);
        EXPECT_EQ(pd->config_.max_trajectory_duration, 6.0);
        EXPECT_EQ(pd->config_.max_traj_generation_reattempt, 10);

        // Test maneuver plan callback
        carma_planning_msgs::msg::ManeuverPlan plan;
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = maneuver.LANE_FOLLOWING;
        maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_A";
        maneuver.lane_following_maneuver.start_dist = 40;
        maneuver.lane_following_maneuver.end_dist = 50;
        maneuver.lane_following_maneuver.lane_ids.push_back("1211");

        plan.maneuvers.push_back(maneuver);
        pd->maneuverPlanCallback(std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(plan));
        EXPECT_EQ("plugin_A", GET_MANEUVER_PROPERTY(pd->latest_maneuver_plan_.maneuvers[0], parameters.planning_strategic_plugin));

        carma_planning_msgs::msg::ManeuverPlan new_plan;
        pd->maneuverPlanCallback(std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(new_plan));

        // empty plan should not be stored locally
        EXPECT_EQ("plugin_A", GET_MANEUVER_PROPERTY(pd->latest_maneuver_plan_.maneuvers[0], parameters.planning_strategic_plugin));

        // // test create service client
        EXPECT_THROW(pd->getPlannerClientByName(""), std::invalid_argument);
        pd->config_.planning_topic_prefix = "/guidance/plugins/";
        pd->config_.planning_topic_suffix = "/plan_trajectory";

        auto plugin_A = pd->getPlannerClientByName("plugin_A");
        EXPECT_EQ(0, std::string(plugin_A->get_service_name()).compare("/guidance/plugins/plugin_A/plan_trajectory"));
        EXPECT_EQ(1, pd->trajectory_planners_.size());
        auto plugin_A_copy = pd->getPlannerClientByName("plugin_A");
        EXPECT_EQ(true, plugin_A == plugin_A_copy);

        // test expired maneuver
        rclcpp::Time test_time(0, 1000);
        carma_planning_msgs::msg::Maneuver test_maneuver;
        test_maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        test_maneuver.lane_following_maneuver.end_time = test_time;
        EXPECT_EQ(false, pd->isManeuverExpired(test_maneuver, pd->get_clock()->now()));
        rclcpp::Time test_time_earlier(0, 500, pd->get_clock()->get_clock_type());
        EXPECT_EQ(false, pd->isManeuverExpired(test_maneuver, test_time_earlier));

        // test compose new plan trajectory request
        uint16_t current_maneuver_index = 0;
        carma_planning_msgs::msg::TrajectoryPlan traj_plan;
        carma_planning_msgs::msg::TrajectoryPlanPoint point_1;
        point_1.x = 0.0;
        point_1.y = 0.0;
        point_1.target_time = rclcpp::Time(0, 0, pd->get_clock()->get_clock_type());
        carma_planning_msgs::msg::TrajectoryPlanPoint point_2;
        point_2.x = 1.0;
        point_2.y = 1.0;
        point_2.target_time = rclcpp::Time(1.41421e9, pd->get_clock()->get_clock_type());
        traj_plan.trajectory_points.push_back(point_1);
        traj_plan.trajectory_points.push_back(point_2);
        auto req = pd->composePlanTrajectoryRequest(traj_plan, new_plan, current_maneuver_index);
        EXPECT_NEAR(1.0, req->vehicle_state.x_pos_global, 0.01);
        EXPECT_NEAR(1.0, req->vehicle_state.y_pos_global, 0.01);
        EXPECT_NEAR(1.0, req->vehicle_state.longitudinal_vel, 0.1);
        EXPECT_EQ(0, req->maneuver_index_to_plan);
    }

    TEST(TestPlanDelegator, TestTrajectoryGenerationFailureLimit) {
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);

        pd->guidance_engaged = true;
        pd->received_maneuver_plan_ = true;
        pd->config_.max_traj_generation_reattempt = 2;

        EXPECT_NO_THROW(pd->onTrajPlanTick());
        EXPECT_NO_THROW(pd->onTrajPlanTick());
        EXPECT_THROW(pd->onTrajPlanTick(), std::runtime_error);
    }


    TEST(TestPlanDelegator, TestPlanDelegator) {
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);
        pd->configure();
        pd->activate();

        carma_planning_msgs::msg::TrajectoryPlan res_plan;

        auto maneuver_pub = pd->create_publisher<carma_planning_msgs::msg::ManeuverPlan>("final_maneuver_plan", 5);

        carma_planning_msgs::msg::ManeuverPlan plan;
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = maneuver.LANE_FOLLOWING;
        maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_A";
        plan.maneuvers.push_back(maneuver);
        maneuver_pub->on_activate();
        maneuver_pub->publish(plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        auto num = maneuver_pub->get_subscription_count();
        EXPECT_EQ(1, num);
    }

    // These tests has been temporarily disabled to support Continuous Improvement (CI) processes.
    // Related GitHub Issue: <https://github.com/usdot-fhwa-stol/carma-platform/issues/2335>

    /**
    TEST(TestPlanDelegator, TestLaneChangeInformation){
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);
        pd->configure();
        pd->activate();

        // Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 25;
        options.lane_width_ = 3.7;
        options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

        // Create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);

        // Set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        // Compute and print shortest path
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(1213);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);

        cmw.get()->setConfigSpeedLimit(30.0);

        // Set PlanDelegator's world model object
        pd->wm_ = cmw;

        // Create pose message with vehicle placed in lanelet 1210
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose.position.x = 5.0;
        pose_msg.pose.position.y = 10.0;

        std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
        pd->poseCallback(std::move(pose_msg_ptr));

        // Create a maneuver plan with no lane change included
        carma_planning_msgs::msg::ManeuverPlan maneuver_plan;

        carma_planning_msgs::msg::Maneuver maneuver_1;
        maneuver_1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_1.lane_following_maneuver.start_dist = 0.0;
        maneuver_1.lane_following_maneuver.end_dist = 25.0;
        maneuver_1.lane_following_maneuver.lane_ids.push_back("1210");
        maneuver_plan.maneuvers.push_back(maneuver_1);

        // Verify that no upcoming lane change exists prior to PlanDelegator receiving a maneuver plan
        ASSERT_FALSE(pd->upcoming_lane_change_information_);
        ASSERT_FALSE(pd->current_lane_change_information_);

        // Trigger maneuverPlanCallback with the generated maneuver plan (no lane change included)
        std::unique_ptr<carma_planning_msgs::msg::ManeuverPlan> maneuver_plan_ptr = std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(maneuver_plan);
        pd->maneuverPlanCallback(std::move(maneuver_plan_ptr));

        // Verify that no upcoming lane change exists since PlanDelegator's maneuver plan includes no lane changes
        ASSERT_FALSE(pd->upcoming_lane_change_information_);
        ASSERT_FALSE(pd->current_lane_change_information_);

        // Create a lane change maneuver that occurs downstream of the vehicle, and add it to the maneuver plan
        carma_planning_msgs::msg::Maneuver maneuver_2;
        maneuver_2.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver_2.lane_change_maneuver.start_dist = 25.0;
        maneuver_2.lane_change_maneuver.end_dist = 50.0;
        maneuver_2.lane_change_maneuver.starting_lane_id = "1211";
        maneuver_2.lane_change_maneuver.ending_lane_id = "1221";
        maneuver_plan.maneuvers.push_back(maneuver_2);

        // Trigger maneuverPlanCallback with the generated maneuver plan (upcoming lane change included)
        std::unique_ptr<carma_planning_msgs::msg::ManeuverPlan> maneuver_plan_ptr2 = std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(maneuver_plan);
        pd->maneuverPlanCallback(std::move(maneuver_plan_ptr2));

        // Verify that an upcoming lane change exists with the correct values
        ASSERT_TRUE(pd->upcoming_lane_change_information_);
        ASSERT_EQ(pd->upcoming_lane_change_information_.get().starting_downtrack, 22.0);
        ASSERT_TRUE(pd->upcoming_lane_change_information_.get().is_right_lane_change);
        ASSERT_FALSE(pd->current_lane_change_information_);

        // Create new maneuver plan for the location where the vehicle is currently located
        carma_planning_msgs::msg::ManeuverPlan maneuver_plan2;

        maneuver_1.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver_1.lane_change_maneuver.start_dist = 0.0;
        maneuver_1.lane_change_maneuver.end_dist = 25.0;
        maneuver_1.lane_change_maneuver.starting_lane_id = "1210";
        maneuver_1.lane_change_maneuver.ending_lane_id = "1220";
        maneuver_plan2.maneuvers.push_back(maneuver_1);

        std::unique_ptr<carma_planning_msgs::msg::ManeuverPlan> maneuver_plan_ptr3 = std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(maneuver_plan2);
        pd->maneuverPlanCallback(std::move(maneuver_plan_ptr3));
        ASSERT_FALSE(pd->upcoming_lane_change_information_);
        ASSERT_TRUE(pd->current_lane_change_information_);
        ASSERT_TRUE(pd->current_lane_change_information_.get().is_right_lane_change);
    }
    */
    /*
    TEST(TestPlanDelegator, TestUpcomingLaneChangeAndTurnSignals){
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);
        pd->configure();
        pd->activate();

        // Verify that initial upcoming_lane_change_status_ indicates no upcoming lane change
        ASSERT_EQ(pd->upcoming_lane_change_status_.lane_change, carma_planning_msgs::msg::UpcomingLaneChangeStatus::NONE);

        // Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 25;
        options.lane_width_ = 3.7;
        options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

        // Create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);

        // Set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        // Compute and print shortest path
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(1213);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);

        cmw.get()->setConfigSpeedLimit(30.0);

        // Set PlanDelegator's world model object
        pd->wm_ = cmw;

        // Create pose message with vehicle placed in lanelet 1210
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose.position.x = 5.0;
        pose_msg.pose.position.y = 10.0;

        std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
        pd->poseCallback(std::move(pose_msg_ptr));

        // Create a maneuver plan with no lane change included
        carma_planning_msgs::msg::ManeuverPlan maneuver_plan;

        // Create initial lane following maneuver and add it to the maneuver plan
        carma_planning_msgs::msg::Maneuver maneuver_1;
        maneuver_1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_1.lane_following_maneuver.start_dist = 0.0;
        maneuver_1.lane_following_maneuver.end_dist = 25.0;
        maneuver_1.lane_following_maneuver.lane_ids.push_back("1210");
        maneuver_plan.maneuvers.push_back(maneuver_1);

        // Create a lane change maneuver that occurs downstream of the vehicle, and add it to the maneuver plan
        carma_planning_msgs::msg::Maneuver maneuver_2;
        maneuver_2.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver_2.lane_change_maneuver.start_dist = 25.0;
        maneuver_2.lane_change_maneuver.end_dist = 50.0;
        maneuver_2.lane_change_maneuver.starting_lane_id = "1211";
        maneuver_2.lane_change_maneuver.ending_lane_id = "1221";
        maneuver_plan.maneuvers.push_back(maneuver_2);

        // Trigger maneuverPlanCallback with the generated maneuver plan (upcoming lane change included)
        std::unique_ptr<carma_planning_msgs::msg::ManeuverPlan> maneuver_plan_ptr = std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(maneuver_plan);
        pd->maneuverPlanCallback(std::move(maneuver_plan_ptr));

        // Set vehicle speed to 0.0 so that turn signals won't be commanded, since the upcoming lane change will never occur
        pd->latest_twist_.twist.linear.x = 0.0;

        std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr2 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
        pd->poseCallback(std::move(pose_msg_ptr2));

        // Verify that an upcoming lane change exists, that an upcoming right lane change is reported, and no turn signal is commanded
        // NOTE: No turn signal is commanded for the upcoming lane change because the current vehicle speed is 0.0 m/s; it'll never reach the lane change maneuver
        ASSERT_TRUE(pd->upcoming_lane_change_information_);
        ASSERT_EQ(pd->upcoming_lane_change_status_.lane_change, carma_planning_msgs::msg::UpcomingLaneChangeStatus::RIGHT);
        ASSERT_EQ(pd->upcoming_lane_change_status_.downtrack_until_lanechange, 12.0);
        ASSERT_FALSE(pd->current_lane_change_information_);
        ASSERT_EQ(pd->latest_turn_signal_command_.r, 0);
        ASSERT_EQ(pd->latest_turn_signal_command_.l, 0);

        // Set the vehicle speed to 10.0 m/s to indicate that the upcoming lane change will occur in under config_.duration_to_signal_before_lane_change
        pd->latest_twist_.twist.linear.x = 10.0;

        std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr3 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
        pd->poseCallback(std::move(pose_msg_ptr3));

        // Verify that an upcoming lane change exists, that an upcoming right lane change is reported, and a right turn signal is commanded
        ASSERT_TRUE(pd->upcoming_lane_change_information_);
        ASSERT_EQ(pd->upcoming_lane_change_status_.lane_change, carma_planning_msgs::msg::UpcomingLaneChangeStatus::RIGHT);
        ASSERT_EQ(pd->upcoming_lane_change_status_.downtrack_until_lanechange, 12.0);
        ASSERT_FALSE(pd->current_lane_change_information_);
        ASSERT_EQ(pd->latest_turn_signal_command_.r, 1);
        ASSERT_EQ(pd->latest_turn_signal_command_.l, 0);

        // Set the vehicle pose to lanelet 1211
        pose_msg.pose.position.x = 5.0;
        pose_msg.pose.position.y = 29.0;

        std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr4 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
        pd->poseCallback(std::move(pose_msg_ptr4));

        // Remove initial lane following maneuver, so only a lane change maneuver is now included
        maneuver_plan.maneuvers.erase(maneuver_plan.maneuvers.begin());
        std::unique_ptr<carma_planning_msgs::msg::ManeuverPlan> maneuver_plan_ptr2 = std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(maneuver_plan);
        pd->maneuverPlanCallback(std::move(maneuver_plan_ptr2));

        // Update vehicle pose again (lanelet 1211) so that internal data members related to lane changes and turn signals
        pose_msg.pose.position.x = 5.0;
        pose_msg.pose.position.y = 30.0;

        std::unique_ptr<geometry_msgs::msg::PoseStamped> pose_msg_ptr5 = std::make_unique<geometry_msgs::msg::PoseStamped>(pose_msg);
        pd->poseCallback(std::move(pose_msg_ptr5));

        // Verify that no upcoming lane change exists, that no upcoming right lane change is reported, that the vehicle is currently changing lanes, and a right turn signal is commanded
        ASSERT_FALSE(pd->upcoming_lane_change_information_);
        ASSERT_EQ(pd->upcoming_lane_change_status_.lane_change, carma_planning_msgs::msg::UpcomingLaneChangeStatus::NONE);
        ASSERT_TRUE(pd->current_lane_change_information_);
        ASSERT_TRUE(pd->current_lane_change_information_.get().is_right_lane_change);
        ASSERT_EQ(pd->latest_turn_signal_command_.r, 1);
        ASSERT_EQ(pd->latest_turn_signal_command_.l, 0);
    }
*/

    struct LoopedTestMapIds
    {
        lanelet::Id lanelet_1_id;
        lanelet::Id lanelet_2_id;
        lanelet::Id lanelet_c_id;
    };

    // Builds two lanelets (L1, L2) that are each other's previous()/following() lanelet, forming a
    // 2-node routing loop, plus a third, unrelated lanelet C that never shares a boundary with
    // either. Used to force getLaneChangeInformation's following()-search to hit a routing loop
    // before finding a shared-boundary match, exercising its loop-detected fallback to a purely
    // geometric left/right estimate.
    lanelet::LaneletMapPtr buildLoopedTestMapWithUnrelatedLanelet(LoopedTestMapIds& ids, double width = 3.7, double length = 10.0)
    {
        auto p_start_left = carma_wm::test::getPoint(0.0, 0.0, 0.0);
        auto p_start_right = carma_wm::test::getPoint(width, 0.0, 0.0);
        auto p_end_left = carma_wm::test::getPoint(0.0, length, 0.0);
        auto p_end_right = carma_wm::test::getPoint(width, length, 0.0);

        // L1 and L2 share the same Point3d objects at both ends (just traversed in reverse), so
        // following(L1) == L2 and following(L2) == L1.
        lanelet::LineString3d l1_left(lanelet::utils::getId(), {p_start_left, p_end_left});
        lanelet::LineString3d l1_right(lanelet::utils::getId(), {p_start_right, p_end_right});
        lanelet::Lanelet l1 = carma_wm::test::getLanelet(l1_left, l1_right);

        lanelet::LineString3d l2_left(lanelet::utils::getId(), {p_end_left, p_start_left});
        lanelet::LineString3d l2_right(lanelet::utils::getId(), {p_end_right, p_start_right});
        lanelet::Lanelet l2 = carma_wm::test::getLanelet(l2_left, l2_right);

        // C sits two lane-widths to the right of L1/L2 and shares none of their boundary points,
        // so it can never be found as a shared-boundary match while walking the loop.
        auto c_left_0 = carma_wm::test::getPoint(2 * width, 0.0, 0.0);
        auto c_left_end = carma_wm::test::getPoint(2 * width, length, 0.0);
        auto c_right_0 = carma_wm::test::getPoint(3 * width, 0.0, 0.0);
        auto c_right_end = carma_wm::test::getPoint(3 * width, length, 0.0);
        lanelet::LineString3d c_left(lanelet::utils::getId(), {c_left_0, c_left_end});
        lanelet::LineString3d c_right(lanelet::utils::getId(), {c_right_0, c_right_end});
        lanelet::Lanelet c = carma_wm::test::getLanelet(c_left, c_right);

        ids.lanelet_1_id = l1.id();
        ids.lanelet_2_id = l2.id();
        ids.lanelet_c_id = c.id();

        lanelet::LaneletMapPtr map = lanelet::utils::createMap({l1, l2, c}, {});
        using namespace lanelet::units::literals;
        lanelet::MapConformer::ensureCompliance(map, 0_mph);
        return map;
    }

    TEST(TestPlanDelegator, TestGetLaneChangeInformation)
    {
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);

        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(3.7, 10.0);
        cmw->setMap(map);
        pd->wm_ = cmw;

        auto makeLaneChangeManeuver = [](lanelet::Id start_id, lanelet::Id end_id) {
            carma_planning_msgs::msg::Maneuver maneuver;
            maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
            maneuver.lane_change_maneuver.start_dist = 0.0;
            maneuver.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
            maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
            return maneuver;
        };

        // Case 1: starting and ending lanelets directly share a boundary -- right lane change.
        // (1200 is lane 1 segment 0; 1210 is lane 2 segment 0; both share linestring ls10.)
        {
            LaneChangeInformation info = pd->getLaneChangeInformation(makeLaneChangeManeuver(1200, 1210));
            EXPECT_TRUE(info.is_right_lane_change);
        }

        // Case 2: starting and ending lanelets directly share a boundary -- left lane change.
        {
            LaneChangeInformation info = pd->getLaneChangeInformation(makeLaneChangeManeuver(1210, 1200));
            EXPECT_FALSE(info.is_right_lane_change);
        }

        // Case 3: no direct shared boundary, but following(starting_lanelet) does share one with
        // ending_lanelet -- exercises the following()-search loop's success path.
        // 1200 (lane 1 seg 0) -> following -> 1201 (lane 1 seg 1), which shares boundary ls11 with
        // 1211 (lane 2 seg 1).
        {
            LaneChangeInformation info = pd->getLaneChangeInformation(makeLaneChangeManeuver(1200, 1211));
            EXPECT_TRUE(info.is_right_lane_change);
        }

        // Case 4: starting_lanelet has no routable successor (1203 is the last lanelet in lane 1),
        // and ending_lanelet (1221) shares no boundary with it -- exercises the no_successor
        // fallback to a purely geometric left/right estimate.
        {
            LaneChangeInformation info = pd->getLaneChangeInformation(makeLaneChangeManeuver(1203, 1221));
            // 1221 (lane 3) sits at a higher x than 1203 (lane 1) along the same direction of
            // travel, so the geometric estimate should report a right lane change.
            EXPECT_TRUE(info.is_right_lane_change);
        }

        // Case 5: starting_lanelet's following() chain forms a routing loop before any shared
        // boundary with ending_lanelet is found -- exercises the loop-detected fallback to the
        // geometric estimate.
        {
            LoopedTestMapIds ids;
            lanelet::LaneletMapPtr looped_map = buildLoopedTestMapWithUnrelatedLanelet(ids);
            std::shared_ptr<carma_wm::CARMAWorldModel> looped_cmw = std::make_shared<carma_wm::CARMAWorldModel>();
            looped_cmw->setMap(looped_map);
            pd->wm_ = looped_cmw;

            LaneChangeInformation info = pd->getLaneChangeInformation(makeLaneChangeManeuver(ids.lanelet_1_id, ids.lanelet_c_id));
            // Lanelet C sits to the right of the L1/L2 loop, so the geometric fallback should
            // report a right lane change.
            EXPECT_TRUE(info.is_right_lane_change);
        }
    }

    /**
     * Total route length should be 100m
     *
     *        |1203|1213|1223|
     *        | _  _  _  _  _|
     *        |1202|1212|1222|
     *        | _  _  _  _  _|
     *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
     *        | _  _  _  _  _|    |     = lane lines
     *        |1200|1210|1220|    - - - = Lanelet boundary
     *        |    12100     |
     *        ****************
     *           START_LINE
     */
    TEST(TestPlanDelegator, TestUpdateManeuverParameters)
    {
        rclcpp::NodeOptions node_options;
        auto pd = std::make_shared<plan_delegator::PlanDelegator>(node_options);
        pd->configure();
        pd->activate();

        // Use Guidance Lib to create map without an obstacle
        carma_wm::test::MapOptions options;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = carma_wm::test::getGuidanceTestMap(options);

        // Introduce overlapping lanelet not on the route
        lanelet::Lanelet start_lanelet = cmw->getMutableMap()->laneletLayer.get(1210);
        lanelet::Lanelet start_lanelet_overlapping = carma_wm::test::getLanelet(12100, start_lanelet.leftBound(), start_lanelet.rightBound());
        auto overlapping_llt_id = std::to_string(start_lanelet_overlapping.id());
        cmw->getMutableMap()->add(start_lanelet_overlapping);
        cmw->setMap(cmw->getMutableMap()); // re-trigger routing graph
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);
        // Set PlanDelegator's world model object
        pd->wm_ = cmw;

        // Verify that start end dist are correct and lanelet on the route is prioritized when picking lanelet
        carma_planning_msgs::msg::Maneuver maneuver_1;
        maneuver_1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        maneuver_1.lane_following_maneuver.start_dist = 25.0;
        maneuver_1.lane_following_maneuver.end_dist = 50.0;
        maneuver_1.lane_following_maneuver.lane_ids.push_back("1211");
        pd->length_to_front_bumper_ = 4.0; // 4.0 meter vehicle length
        pd->updateManeuverParameters(maneuver_1);

        EXPECT_NEAR(maneuver_1.lane_following_maneuver.start_dist, 21.0, 0.01);
        EXPECT_NEAR(maneuver_1.lane_following_maneuver.end_dist, 46.0, 0.01);
        EXPECT_EQ(maneuver_1.lane_following_maneuver.lane_ids.front(), "1210");

        // Switch route with a different lanelet to verify route prioritization
        cmw->setMap(cmw->getMutableMap()); // re-trigger routing graph
        carma_wm::test::setRouteByIds({start_lanelet_overlapping.id(), 1213}, cmw);
        pd->wm_ = cmw;
        maneuver_1.lane_following_maneuver.start_dist = 25.0;
        maneuver_1.lane_following_maneuver.end_dist = 50.0;
        maneuver_1.lane_following_maneuver.lane_ids.front() = "1211";
        pd->updateManeuverParameters(maneuver_1);

        EXPECT_NEAR(maneuver_1.lane_following_maneuver.start_dist, 21.0, 0.01);
        EXPECT_NEAR(maneuver_1.lane_following_maneuver.end_dist, 46.0, 0.01);
        EXPECT_EQ(maneuver_1.lane_following_maneuver.lane_ids.front(), overlapping_llt_id);

        // lanechange
        carma_planning_msgs::msg::Maneuver maneuver_2;
        maneuver_2.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver_2.lane_change_maneuver.start_dist = 25.0;
        maneuver_2.lane_change_maneuver.end_dist = 50.0;
        maneuver_2.lane_change_maneuver.starting_lane_id = "1211";
        maneuver_2.lane_change_maneuver.ending_lane_id = "1221";

        cmw->setMap(cmw->getMutableMap()); // re-trigger routing graph
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);
        pd->wm_ = cmw;

        pd->updateManeuverParameters(maneuver_2);
        EXPECT_NEAR(maneuver_2.lane_change_maneuver.start_dist, 21.0, 0.01);
        EXPECT_NEAR(maneuver_2.lane_change_maneuver.end_dist, 46.0, 0.01);
        EXPECT_EQ(maneuver_2.lane_change_maneuver.starting_lane_id, "1210");

        // Switch route with a different lanelet to verify route prioritization
        cmw->setMap(cmw->getMutableMap()); // re-trigger routing graph
        carma_wm::test::setRouteByIds({start_lanelet_overlapping.id(), 1211, 1212, 1213}, cmw);
        pd->wm_ = cmw;
        maneuver_2.lane_change_maneuver.start_dist = 25.0;  // reset
        maneuver_2.lane_change_maneuver.end_dist = 50.0;
        maneuver_2.lane_change_maneuver.starting_lane_id = "1211";
        maneuver_2.lane_change_maneuver.ending_lane_id = "1221";

        pd->updateManeuverParameters(maneuver_2);
        EXPECT_NEAR(maneuver_2.lane_change_maneuver.start_dist, 21.0, 0.01);
        EXPECT_NEAR(maneuver_2.lane_change_maneuver.end_dist, 46.0, 0.01);
        EXPECT_EQ(maneuver_2.lane_change_maneuver.starting_lane_id, overlapping_llt_id);

        // Verify that the non-route lanelet is being picked if no suitable lanelet is on the route
        cmw->setMap(cmw->getMutableMap()); // re-trigger routing graph
        carma_wm::test::setRouteByIds({1201, 1203}, cmw);
        pd->wm_ = cmw;

        // Lane Follow
        maneuver_1.lane_following_maneuver.start_dist = 0.0;  // Reset values
        maneuver_1.lane_following_maneuver.end_dist = 25.0;
        maneuver_1.lane_following_maneuver.lane_ids.front() = "1201";

        pd->updateManeuverParameters(maneuver_1);

        EXPECT_NEAR(maneuver_1.lane_following_maneuver.start_dist, -4.0, 0.01);
        EXPECT_NEAR(maneuver_1.lane_following_maneuver.end_dist, 21.0, 0.01);
        EXPECT_EQ(maneuver_1.lane_following_maneuver.lane_ids.front(), "1200");

        // Lanechange
        cmw->setMap(cmw->getMutableMap()); // re-trigger routing graph
        carma_wm::test::setRouteByIds({1221, 1213}, cmw);
        pd->wm_ = cmw;
        maneuver_2.lane_change_maneuver.start_dist = 0.0;  // Reset values
        maneuver_2.lane_change_maneuver.end_dist = 25.0;
        maneuver_2.lane_change_maneuver.starting_lane_id = "1221";
        maneuver_2.lane_change_maneuver.ending_lane_id = "1211";

        pd->updateManeuverParameters(maneuver_2);
        EXPECT_NEAR(maneuver_2.lane_change_maneuver.start_dist, -4.0, 0.01);
        EXPECT_NEAR(maneuver_2.lane_change_maneuver.end_dist, 21.0, 0.01);
        EXPECT_EQ(maneuver_2.lane_change_maneuver.starting_lane_id, "1220");
    }
} // namespace plan_delegator

    /*!
    * \brief Main entrypoint for unit tests
    */
    int main (int argc, char **argv) {
        ::testing::InitGoogleTest(&argc, argv);

        //Initialize ROS
        rclcpp::init(argc, argv);
        auto ret = rcutils_logging_set_logger_level(
                rclcpp::get_logger("plan_delegator").get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        bool success = RUN_ALL_TESTS();

        //shutdown ROS
        rclcpp::shutdown();

        return success;
    }
