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
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
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
        auto req = pd->composePlanTrajectoryRequest(traj_plan, current_maneuver_index);
        EXPECT_NEAR(1.0, req->vehicle_state.x_pos_global, 0.01);
        EXPECT_NEAR(1.0, req->vehicle_state.y_pos_global, 0.01);
        EXPECT_NEAR(1.0, req->vehicle_state.longitudinal_vel, 0.1);
        EXPECT_EQ(0, req->maneuver_index_to_plan);
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
