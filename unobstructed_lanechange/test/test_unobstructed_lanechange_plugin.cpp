/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "unobstructed_lanechange.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include "TestHelpers.h"
#include <thread>
#include <chrono>
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
#include <string>
#include <cav_msgs/Maneuver.h>
#include <carma_utils/containers/containers.h>
#include <cav_msgs/VehicleState.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <fstream>
#include <cav_srvs/PlanTrajectory.h>
#include <sstream>
#include<ros/package.h>

namespace unobstructed_lanechange
{
    TEST(UnobstructedLaneChangePlugin,DISABLED_Testusingosm){
    // File to process. Path is relative to test folder
    std::string path = ros::package::getPath("route");
    std::string file = "/resource/map/town01_vector_map_1.osm";
    file = path.append(file);
    lanelet::Id start_id=159;
    lanelet::Id end_id=121;
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
     * * **/

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
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        //Set Route
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        //get starting position
        auto llt=map.get()->laneletLayer.get(start_id);
        lanelet::LineString3d left_bound=llt.leftBound();
        lanelet::LineString3d right_bound=llt.rightBound();
        geometry_msgs::PoseStamped left;
        geometry_msgs::PoseStamped right;
        for(lanelet::Point3d& p : left_bound)
        {
            left.pose.position.x=p.x();
            left.pose.position.y=p.y();
            left.pose.position.z=p.z();

        }
        for(lanelet::Point3d& p : right_bound)
        {
            right.pose.position.x=p.x();
            right.pose.position.y=p.y();
            right.pose.position.z=p.z();
        }
        geometry_msgs::PoseStamped curr_pos;
        curr_pos.pose.position.x=(left.pose.position.x+right.pose.position.x)/2;
        curr_pos.pose.position.y=(left.pose.position.y+right.pose.position.y)/2;
        curr_pos.pose.position.z=(left.pose.position.z+right.pose.position.z)/2;

        curr_pos.pose.orientation.x=0.0;
        curr_pos.pose.orientation.y=0.0;
        curr_pos.pose.orientation.z=0.0;
        curr_pos.pose.orientation.w=0.0;

        //Define arguments for function maneuvers_to_points
        UnobstructedLaneChangePlugin worker;
        ros::Time::init(); //Initialize ros::Time
        lanelet::BasicPoint2d veh_pos(curr_pos.pose.position.x,curr_pos.pose.position.y);
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();
        for (size_t i=0; i <shortest_path.size(); ++i)
        {
            if(shortest_path[i].id() == end_id)
            {
                lanelet::ConstLanelet ending_lanelet;
                ending_lanelet == shortest_path[i];
                ending_downtrack = cmw->routeTrackPos(shortest_path[i].centerline2d().back()).downtrack;
            }
        }
    worker.wm_ = cmw;

    //Define lane change maneuver
    cav_msgs::Maneuver maneuver;
    maneuver.type=cav_msgs::Maneuver::LANE_CHANGE;
    maneuver.lane_change_maneuver.start_dist = starting_downtrack;
    maneuver.lane_change_maneuver.end_dist = ending_downtrack;
    maneuver.lane_change_maneuver.start_speed = 30.0;
    maneuver.lane_change_maneuver.start_time = ros::Time::now();
    //calculate end_time assuming constant acceleration
    double acc = pow(maneuver.lane_change_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
    double end_time = maneuver.lane_change_maneuver.start_speed/acc;
    maneuver.lane_change_maneuver.end_speed = 30.0;
    maneuver.lane_change_maneuver.end_time = ros::Time(end_time + maneuver.lane_change_maneuver.start_time.toSec());
    maneuver.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
    maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
    
    std::vector<cav_msgs::Maneuver> maneuvers;
    maneuvers.push_back(maneuver);
    

    worker.maneuvers_to_points(maneuvers, starting_downtrack, cmw);
    EXPECT_TRUE(true);
    }

    TEST(UnobstructedLaneChangePlugin,TestusingGuidanceLib){
        //Using Guidance Lib to create map
        carma_wm::test::MapOptions options;
        options.lane_length_ = 25;
        options.lane_width_ = 3.7;
        options.speed_limit_ = carma_wm::test::MapOptions::SpeedLimit::DEFAULT;
        options.obstacle_ = carma_wm::test::MapOptions::Obstacle::DEFAULT;
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        //create the Semantic Map
        lanelet::LaneletMapPtr map = carma_wm::test::buildGuidanceTestMap(options.lane_width_, options.lane_length_);
    /**
     * This is a test library made for guidance unit tests. In general, it includes the following :
     * - Helper functions to create the world from scratch or extend the world in getGuidanceTestMap()
     * - addObstacle at a specified Cartesian or Trackpos point relative to specified lanelet Id
     * - set route by giving series of lanelet Id in the map (setRouteById)
     * - set speed of entire road (setSpeedLimit)
     * - getGuidanceTestMap gives a simple one way, 3 lane map (25mph speed limit) with one static prebaked obstacle and 
     *      4 lanelets in a lane (if 2 stripes make up one lanelet):
     *
     *        |1203|1213|1223|
     *        | _  _  _  _  _|
     *        |1202| Ob |1222|
     *        | _  _  _  _  _|
     *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
     *        | _  _  _  _  _|    |     = lane lines
     *        |1200|1210|1220|    - - - = Lanelet boundary
     *        |              |    O     = Default Obstacle
     *        ****************
     *           START_LINE
     */
        lanelet::Id start_id=1210;
        lanelet::Id end_id=1223;

        //set the map with default routingGraph
        cmw ->carma_wm::CARMAWorldModel::setMap(map);
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        lanelet::LaneletMapConstPtr const_map(map);
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

        //Compute and print the shortest path
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(start_id);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(end_id);
        auto route = map_graph->getRoute(start_lanelet,end_lanelet);

        
        cmw->carma_wm::CARMAWorldModel::setMap(map);


        //get starting position
        auto llt=map.get()->laneletLayer.get(start_id);
        lanelet::LineString3d left_bound=llt.leftBound();
        lanelet::LineString3d right_bound=llt.rightBound();
        geometry_msgs::PoseStamped left;
        geometry_msgs::PoseStamped right;
        for(lanelet::Point3d& p : left_bound)
        {
            left.pose.position.x=p.x();
            left.pose.position.y=p.y();
            left.pose.position.z=p.z();

        }
        for(lanelet::Point3d& p : right_bound)
        {
            right.pose.position.x=p.x();
            right.pose.position.y=p.y();
            right.pose.position.z=p.z();
        }
        geometry_msgs::PoseStamped curr_pos;
        curr_pos.pose.position.x=(left.pose.position.x+right.pose.position.x)/2;
        curr_pos.pose.position.y=(left.pose.position.y+right.pose.position.y)/2;
        curr_pos.pose.position.z=(left.pose.position.z+right.pose.position.z)/2;

        curr_pos.pose.orientation.x=0.0;
        curr_pos.pose.orientation.y=0.0;
        curr_pos.pose.orientation.z=0.0;
        curr_pos.pose.orientation.w=0.0;

        //Define arguments for function maneuvers_to_points
        UnobstructedLaneChangePlugin worker;
        ros::Time::init(); //Initialize ros::Time
        lanelet::BasicPoint2d veh_pos(curr_pos.pose.position.x,curr_pos.pose.position.y);
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();
        for (size_t i=0; i <shortest_path.size(); ++i)
        {
            if(shortest_path[i].id() == end_id)
            {
                lanelet::ConstLanelet ending_lanelet;
                ending_lanelet == shortest_path[i];
                ending_downtrack = cmw->routeTrackPos(shortest_path[i].centerline2d().back()).downtrack;
            }
        }
    worker.wm_ = cmw;

    //Define lane change maneuver
    cav_msgs::Maneuver maneuver;
    maneuver.type=cav_msgs::Maneuver::LANE_CHANGE;
    maneuver.lane_change_maneuver.start_dist = starting_downtrack;
    maneuver.lane_change_maneuver.end_dist = ending_downtrack;
    maneuver.lane_change_maneuver.start_speed = 25.0;
    maneuver.lane_change_maneuver.start_time = ros::Time::now();
    //calculate end_time assuming constant acceleration
    double acc = pow(maneuver.lane_change_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
    double end_time = maneuver.lane_change_maneuver.start_speed/acc;
    maneuver.lane_change_maneuver.end_speed = 30.0;
    maneuver.lane_change_maneuver.end_time = ros::Time(end_time + 10.0);
    maneuver.lane_change_maneuver.starting_lane_id = std::to_string(start_id);
    maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
    
    std::vector<cav_msgs::Maneuver> maneuvers;
    maneuvers.push_back(maneuver);
    worker.current_speed_ = maneuver.lane_change_maneuver.start_speed;
    worker.maneuvers_to_points(maneuvers, starting_downtrack, cmw);
    EXPECT_TRUE(true);

    /* Test PlanTrajectory cb */
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<cav_msgs::Maneuver> maneuvers_msg;  
        //Define lane change maneuver

        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        bool isTrajectory = worker.plan_trajectory_cb(req,resp);
        
        EXPECT_TRUE(isTrajectory);
        EXPECT_TRUE(resp.trajectory_plan.trajectory_points.size() > 2);


    }
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    return res;
}