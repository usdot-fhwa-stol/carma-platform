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

#include "stopandwait.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
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

namespace stopandwait_plugin
{
    TEST(StopandWait, TestManeuvers_to_point)
    {
        // File to process. Path is relative to test folder
        std::string file = "../resource/map/town01_vector_map_1.osm";
        //std::string file = "../resource/map/TFHRC.osm";
        lanelet::Id start_id=159;           //sample map :101
        lanelet::Id end_id=121;            //sample map = 111
        int target_lane_id = 164;       //Lane where stop is required - sample map = 167
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
        
        ros::Time::init(); //Initialize ros::Time
        lanelet::BasicPoint2d veh_pos(curr_pos.pose.position.x,curr_pos.pose.position.y);
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();
        for (size_t i=0; i <shortest_path.size(); ++i)
        {
            if(shortest_path[i].id() == target_lane_id)
            {
                lanelet::ConstLanelet ending_lanelet;
                ending_lanelet == shortest_path[i];
                ending_downtrack = cmw->routeTrackPos(shortest_path[i].centerline2d().back()).downtrack;
            }
        }
        
        //Define stop and wait maneuver
        cav_msgs::Maneuver maneuver;
        maneuver.type=cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver.stop_and_wait_maneuver.start_dist = starting_downtrack;
        maneuver.stop_and_wait_maneuver.end_dist = ending_downtrack;
        maneuver.stop_and_wait_maneuver.start_speed = 30.0;
        maneuver.stop_and_wait_maneuver.start_time = ros::Time::now();
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.stop_and_wait_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
        double end_time = maneuver.stop_and_wait_maneuver.start_speed/acc;
 
        maneuver.stop_and_wait_maneuver.end_time = ros::Time(end_time + maneuver.stop_and_wait_maneuver.start_time.toSec());
        maneuver.stop_and_wait_maneuver.starting_lane_id = start_id;
        maneuver.stop_and_wait_maneuver.ending_lane_id = target_lane_id;
        
        std::vector<cav_msgs::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        
        StopandWait sw;
        sw.wm_=cmw;
        std::vector <PointSpeedPair> points= sw.maneuvers_to_points(maneuvers, starting_downtrack, cmw);

        //Check if corresponding speed is decreasing
        auto downsampled_points = carma_utils::containers::downsample_vector(points,8);
        auto p_prev = downsampled_points[0];
        for(p : downsampled_points){
            EXPECT_TRUE(p.speed <= p_prev.speed);
            p_prev = p;
        }

        //Check Trajectory
        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory;
        cav_msgs::VehicleState state;
        state.X_pos_global=curr_pos.pose.position.x;
        state.Y_pos_global = curr_pos.pose.position.y;
        trajectory = sw.compose_trajectory_from_centerline(points,state);

    }
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    return res;
}