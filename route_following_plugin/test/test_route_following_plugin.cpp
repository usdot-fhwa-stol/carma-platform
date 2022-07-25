/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "route_following_plugin.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
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
#include <string>

namespace route_following_plugin
{

    TEST(RouteFollowingPluginTest, testComposeManeuverMessage)
    {
        auto rfp = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        rclcpp::Time current_time = rfp->now();
        auto msg = rfp->composeLaneFollowingManeuverMessage(1.0, 10.0, 0.9, 11.176, {2});
        EXPECT_EQ(carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING, msg.type);
        EXPECT_EQ(carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION, msg.lane_following_maneuver.parameters.negotiation_type);
        EXPECT_EQ(carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN, msg.lane_following_maneuver.parameters.presence_vector);
        EXPECT_EQ("inlanecruising_plugin", msg.lane_following_maneuver.parameters.planning_tactical_plugin);
        EXPECT_EQ("RouteFollowingPlugin", msg.lane_following_maneuver.parameters.planning_strategic_plugin);
        EXPECT_NEAR(1.0, msg.lane_following_maneuver.start_dist, 0.01);
        EXPECT_NEAR(0.9, msg.lane_following_maneuver.start_speed, 0.01);
        EXPECT_NEAR(10.0, msg.lane_following_maneuver.end_dist, 0.01);
        EXPECT_NEAR(25 / 2.237, msg.lane_following_maneuver.end_speed, 0.01);
        EXPECT_EQ(1, msg.lane_following_maneuver.lane_ids.size());
        EXPECT_EQ("2", msg.lane_following_maneuver.lane_ids[0]);
    }

    TEST(RouteFollowingPluginTest, testIdentifyLaneChange)
    {
        auto rfp = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        auto relations = lanelet::routing::LaneletRelations();
        EXPECT_TRUE(rfp->isLaneChangeNeeded(relations, 1));
        lanelet::routing::LaneletRelation relation;
        relation.relationType = lanelet::routing::RelationType::Successor;
        relations.push_back(relation);
        EXPECT_FALSE(rfp->isLaneChangeNeeded(relations, 0));
    }

    TEST(RouteFollowingPlugin, TestAssociateSpeedLimit)
    {
        //Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 25;
        options.lane_width_ = 3.7;
        options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        //create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);

        //set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        //Compute and print shortest path
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(1213);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);

        cmw.get()->setConfigSpeedLimit(30.0);

        auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker->wm_ = cmw;

        //Define current position and velocity

        lanelet::BasicPoint2d current_loc = start_lanelet.centerline2d().front();
        worker->current_loc_ = current_loc;

        //define twist
        worker->current_speed_ = 10.0;

        //Define plan for request and response
        //PlanManeuversRequest
        carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr plan_request;
        carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr plan_response;
        carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr pplan;

        carma_planning_msgs::msg::ManeuverPlan plan_req1;
        plan_req1.header;
        plan_req1.maneuver_plan_id;
        plan_req1.planning_start_time;
        plan_req1.planning_completion_time;

        rclcpp::Time current_time = worker->now();
        plan_req1.maneuvers.push_back(worker->composeLaneFollowingManeuverMessage(0, 0, 0, 11.176, {0}));
        pplan->prior_plan = plan_req1;
        plan_request = pplan;
        //PlanManeuversResponse
        carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr newplan;
        for (auto i = 0; i < plan_req1.maneuvers.size(); i++)
            newplan->new_plan.maneuvers.push_back(plan_req1.maneuvers[i]);

        plan_response = newplan;

        //RouteFollowing plan maneuver callback
        auto shortest_path = cmw->getRoute()->shortestPath();

        worker->latest_maneuver_plan_ = worker->routeCb(shortest_path);

        std::shared_ptr<rmw_request_id_t> srv_header;

        worker->plan_maneuvers_callback(srv_header,plan_request, plan_response);
        
        //check target speeds in updated response
        lanelet::Velocity limit = 30_mph;
        ASSERT_EQ(plan_response->new_plan.maneuvers[0].lane_following_maneuver.end_speed, 11.176);
        for (auto i = 1; i < plan_response->new_plan.maneuvers.size() - 1; i++)
        {
        ASSERT_EQ(plan_response->new_plan.maneuvers[i].lane_following_maneuver.end_speed, limit.value());
        }

        ASSERT_FALSE(plan_response->new_plan.maneuvers.empty());
        ASSERT_EQ(carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT, plan_response->new_plan.maneuvers.back().type);
        ASSERT_EQ(plan_response->new_plan.maneuvers.back().stop_and_wait_maneuver.start_speed, limit.value());

    }

    TEST(RouteFollowingPlugin, TestAssociateSpeedLimitusingosm)
    {
        // File to process. Path is relative to test folder
        std::string file = "../resource/map/town01_vector_map_1.osm";
        lanelet::Id start_id = 100;
        lanelet::Id end_id = 111;
        /***
         * VAVLID PATHs (consists of lanenet ids): (This is also the shortest path because certain Lanelets missing)
         * 159->160->164->136->135->137->144->121; 
         * 159->160->164->136->135->137->144->118;
         * 168->170->111
         * 159->161->168->170->111
         * 167->169->168->170->111
         * 115->146->140->139->143->167->169->168->170->111 
         * 141->139->143->167->169->168->170->111 
         * 127->146->140->139->143->167->169->168->170->111 
         * 101->100->104->167->169->168->170->111 (a counter cLock circle) 
         * **/
        // Write new map to file
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
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        //get position on map
        auto llt = map.get()->laneletLayer.get(start_id);
        lanelet::LineString3d left_bound = llt.leftBound();
        lanelet::LineString3d right_bound = llt.rightBound();
        geometry_msgs::msg::PoseStamped left;
        geometry_msgs::msg::PoseStamped right;
        for (lanelet::Point3d &p : left_bound)
        {
            left.pose.position.x = p.x();
            left.pose.position.y = p.y();
            left.pose.position.z = p.z();
        }
        for (lanelet::Point3d &p : right_bound)
        {
            right.pose.position.x = p.x();
            right.pose.position.y = p.y();
            right.pose.position.z = p.z();
        }
        //Assign start of centerline of start lanelet as current position
        lanelet::BasicPoint2d start_location;
        start_location = llt.centerline2d().back();
        worker->current_loc_ = start_location;

        //define twist
        worker->current_speed_ = 0.0;

        //Set Route
        carma_wm::test::setRouteByIds({start_id, end_id}, cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker->wm_ = cmw;
        auto shortest_path = cmw->getRoute()->shortestPath();
        //Define plan for request and response
        //PlanManeuversRequest
        carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr plan_request;
        carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr plan_response;
        carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr pplan;

        carma_planning_msgs::msg::ManeuverPlan plan_req1;
        plan_req1.header;
        plan_req1.maneuver_plan_id;
        plan_req1.planning_start_time;
        plan_req1.planning_completion_time;
        rclcpp::Time current_time = worker->now();
        plan_req1.maneuvers.push_back(worker->composeLaneFollowingManeuverMessage(0.0, 100.0, 0, 11.176, {start_id}));
        pplan->prior_plan = plan_req1;
        plan_request = pplan;
        //PlanManeuversResponse
        carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr newplan;
        for (auto i = 0; i < plan_req1.maneuvers.size(); i++)
            newplan->new_plan.maneuvers.push_back(plan_req1.maneuvers[i]);

        plan_response = newplan;
        worker->latest_maneuver_plan_ = worker->routeCb(shortest_path);

        std::shared_ptr<rmw_request_id_t> srv_header;

        worker->plan_maneuvers_callback(srv_header,plan_request, plan_response);
        //check target speeds in updated response
        lanelet::Velocity limit = 25_mph;

        for (auto i = 0; i < plan_response->new_plan.maneuvers.size() - 1; i++)
        {
          ASSERT_EQ(plan_response->new_plan.maneuvers[i].lane_following_maneuver.end_speed, limit.value());
        }

        //Test findSpeedLimit function
        auto current_lanelets = lanelet::geometry::findNearest(worker->wm_->getMap()->laneletLayer, worker->current_loc_, 10);
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;
        double speed = worker->findSpeedLimit(current_lanelet);
        if (speed < 11.176)
        {
            ASSERT_EQ(speed, 0.0);
        }
        else
            ASSERT_EQ(speed, 11.176);
    }

    TEST(RouteFollowingPlugin,testComposeLaneChangeStatus)
    {
        //Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_=25;
        options.lane_width_=3.7;
        options.speed_limit_=carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_=carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        //create the Semantic Map
        lanelet::LaneletMapPtr map=carma_wm::test::buildGuidanceTestMap(options.lane_width_,options.lane_length_);

        //set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210,1213},cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules=lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        //Compute and print shortest path
        lanelet::Lanelet start_lanelet=map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet=map->laneletLayer.get(1223);
        lanelet::Lanelet end_lanelet_1=map->laneletLayer.get(1220);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);

        cmw.get()->setConfigSpeedLimit(30.0);

        auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker->wm_=cmw;

        auto lane_change_status_msg=worker->ComposeLaneChangeStatus(start_lanelet,end_lanelet_1);

        ASSERT_EQ(lane_change_status_msg.lane_change,carma_planning_msgs::msg::UpcomingLaneChangeStatus::RIGHT);
        ASSERT_EQ(lane_change_status_msg.downtrack_until_lanechange,0);

        lane_change_status_msg=worker->ComposeLaneChangeStatus(end_lanelet_1,start_lanelet);

        ASSERT_EQ(lane_change_status_msg.lane_change,carma_planning_msgs::msg::UpcomingLaneChangeStatus::LEFT);
        ASSERT_EQ(lane_change_status_msg.downtrack_until_lanechange,0);

    }

    
    TEST(RouteFollowingPlugin, TestHelperfunctions)
    {
        auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        /*composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, int lane_id, ros::Time start_time);*/
        rclcpp::Time start_time = worker->now();
        carma_planning_msgs::msg::Maneuver maneuver = worker->composeLaneFollowingManeuverMessage(10.0, 100.0, 0.0, 100.0, {101});

        worker->setManeuverStartDist(maneuver, 50.0);
        ASSERT_EQ(maneuver.lane_following_maneuver.start_dist, 50.0);

        rclcpp::Time new_start_time = start_time + rclcpp::Duration(10.0*1e9);
        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);

        worker->updateTimeProgress(maneuvers, new_start_time);

        double start_time_change = rclcpp::Time(GET_MANEUVER_PROPERTY(maneuvers.front(), start_time)).seconds() - start_time.seconds();
        ASSERT_EQ(start_time_change, 10.0);
    }



TEST(RouteFollowingPlugin, TestReturnToShortestPath)
    {
    //Use Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 25;
        options.lane_width_ = 3.7;
        options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::NONE;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        //create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);

        //set the map with default routingGraph
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({1210, 1213}, cmw);

        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        //Compute and print shortest path
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(1210);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(1213);
        auto route = map_graph->getRoute(start_lanelet, end_lanelet);

        cmw.get()->setConfigSpeedLimit(30.0);

        auto worker = std::make_shared<RouteFollowingPlugin>(rclcpp::NodeOptions());
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        worker->wm_ = cmw;

        //RouteFollowing plan maneuver callback
        auto shortest_path = cmw->getRoute()->shortestPath();

        worker->latest_maneuver_plan_ = worker->routeCb(shortest_path);

        // If the vehicle remains on the shortest path, the next maneuver is lane following
        ASSERT_EQ(worker->latest_maneuver_plan_[0].type, carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING);

        for (auto ll:route->shortestPath())
        {
            worker->shortest_path_set_.insert(ll.id());
            std::cout<<"id added: " << ll.id() << std::endl;
        }

        lanelet::Lanelet current_lanelet = map->laneletLayer.get(1200);
        worker->returnToShortestPath(current_lanelet);

        // Since the vehicle is not on the shortest path, the first maneuver is lane change
        ASSERT_EQ(worker->latest_maneuver_plan_[0].type, carma_planning_msgs::msg::Maneuver::LANE_CHANGE);
    }
}

    /*!
    * \brief Main entrypoint for unit tests
    */
    int main (int argc, char **argv) {
        ::testing::InitGoogleTest(&argc, argv);

        //Initialize ROS
        rclcpp::init(argc, argv);
        auto ret = rcutils_logging_set_logger_level(
                rclcpp::get_logger("route_following_plugin").get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        bool success = RUN_ALL_TESTS();

        //shutdown ROS
        rclcpp::shutdown();

        return success;
    }

