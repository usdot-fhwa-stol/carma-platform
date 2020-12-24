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

#include "stop_and_wait_plugin.h"
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
#include <cav_srvs/PlanTrajectory.h>
#include <sstream>
#include<ros/package.h>

namespace stop_and_wait_plugin
{
    TEST(StopandWait, TestStopandWaitPlanning)
    {
        // File to process. Path is relative to route package
        std::string path = ros::package::getPath("route");
        std::string file = "/resource/map/town01_vector_map_1.osm";
        file = path.append(file);
        lanelet::Id start_id=159;          
        lanelet::Id end_id=121;            
        int target_lane_id = 164;       

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

        //Call function
        StopandWait sw;
        sw.wm_=cmw;
        sw.current_speed_=maneuver.stop_and_wait_maneuver.start_speed;
        std::vector <PointSpeedPair> points= sw.maneuvers_to_points(maneuvers, starting_downtrack, cmw);

        //Check else condition maneuver
        cav_msgs::Maneuver maneuver_2 = maneuver;
        maneuver_2.stop_and_wait_maneuver.start_dist = ending_downtrack;
        //Less than min trajectory time
        maneuver_2.stop_and_wait_maneuver.end_time = ros::Time(3.0 + maneuver_2.stop_and_wait_maneuver.start_time.toSec()); 
        maneuvers[0] = maneuver_2;
        std::vector <PointSpeedPair> coverage_points= sw.maneuvers_to_points(maneuvers, starting_downtrack, cmw);

        //Maneuver_3- jerk req > max permittable
        cav_msgs::Maneuver maneuver_3 = maneuver;
        double jerk_req = 5.0;
        maneuver_3.stop_and_wait_maneuver.start_time = ros::Time::now();
        maneuver_3.stop_and_wait_maneuver.end_time = ros::Time(3.0 + maneuver_3.stop_and_wait_maneuver.start_time.toSec());
        maneuvers[0] = maneuver_3;
        coverage_points= sw.maneuvers_to_points(maneuvers, starting_downtrack, cmw);

        //Downsample points
        auto downsampled_points = carma_utils::containers::downsample_vector(points,8);
        //Check if corresponding speed is decreasing
        auto p_prev = downsampled_points[0];
        for(auto p : downsampled_points){
            EXPECT_TRUE(p.speed <= p_prev.speed);
            p_prev = p;
        }

        /* Test compose trajectory and helper functions*/

        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory;
        cav_msgs::VehicleState state;
        state.X_pos_global=curr_pos.pose.position.x;
        state.Y_pos_global = curr_pos.pose.position.y;
        int nearest_pt =sw.getNearestPointIndex(downsampled_points, state);
        std::vector<lanelet::BasicPoint2d> points_split;
        std::vector<double> speeds_split;
        sw.splitPointSpeedPairs(downsampled_points,&points_split,&speeds_split);
        EXPECT_TRUE(points_split.size() == speeds_split.size());
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(points_split);
        
        std::vector<double> times;
        sw.speed_to_time(downtracks,speeds_split, &times, 0.01);
        for(auto i = 0;i <times.size();i++){
            EXPECT_TRUE(times[i] >= 0);
        }

        trajectory = sw.compose_trajectory_from_centerline(downsampled_points,state);
        //Valid Trajectory has at least 2 points
        EXPECT_TRUE(trajectory.size() > 2);

        /* Test PlanTrajectory cb */
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = 30.0;
        std::vector<cav_msgs::Maneuver> maneuvers_msg;  
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = "StopandWaitPlugin";
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.stop_and_wait_maneuver.start_dist = starting_downtrack;
        maneuver_msg.stop_and_wait_maneuver.start_speed = 30.0;
        maneuver_msg.stop_and_wait_maneuver.start_time = ros::Time::now();
        
        maneuver_msg.stop_and_wait_maneuver.end_dist = ending_downtrack;
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(start_id);
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(end_id);
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.stop_and_wait_maneuver.end_time =ros::Time(end_time + maneuver.stop_and_wait_maneuver.start_time.toSec());
        maneuvers_msg.push_back(maneuver_msg);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        bool isTrajectory = sw.plan_trajectory_cb(req,resp);
        
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