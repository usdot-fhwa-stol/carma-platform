/*
 * Copyright (C) 2019-2022 LEIDOS.
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
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>
#include <carma_wm_ros2/TrafficControl.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include "cooperative_lanechange/cooperative_lanechange_node.hpp"

namespace cooperative_lanechange
{
    TEST(CooperativeLaneChangePlugin, TestTrajectorytoecef){
        rclcpp::NodeOptions options;
        auto worker = std::make_shared<cooperative_lanechange::CooperativeLaneChangePlugin>(options);

        worker->configure(); //Call configure state transition
        worker->activate();  //Call activate state transition to get not read for runtime

        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_plan;

        std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
        std_msgs::msg::String msg;
        msg.data = base_proj;
        std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>(msg);
        worker->georeference_cb(std::move(msg_ptr));  // Set projection

        carma_planning_msgs::msg::TrajectoryPlanPoint point_1;
        point_1.x = 20.0;
        point_1.y = 0.0;
        trajectory_plan.push_back(point_1);
        carma_v2x_msgs::msg::LocationECEF ecef_point_1 = worker->trajectory_point_to_ecef(point_1);

        ASSERT_NEAR(ecef_point_1.ecef_x, 637813699.0, 0.001);
        ASSERT_NEAR(ecef_point_1.ecef_y, 1999.0, 0.001);
        ASSERT_NEAR(ecef_point_1.ecef_z, 0.0, 0.001);
            

        carma_planning_msgs::msg::TrajectoryPlanPoint point_2;
        point_2.x = 19.0;
        point_2.y = 0.0;
        trajectory_plan.push_back(point_2);
        ecef_point_1 = worker->trajectory_point_to_ecef(point_2);

        carma_v2x_msgs::msg::Trajectory traj = worker->trajectory_plan_to_trajectory(trajectory_plan);
        ASSERT_NEAR(traj.location.ecef_x, 637813699.0, 0.001);
        ASSERT_NEAR(traj.location.ecef_y, 1999.0, 0.001);
        ASSERT_NEAR(traj.location.ecef_z, 0.0, 0.001);

        ASSERT_EQ(traj.offsets.size(), 1);
        ASSERT_NEAR(traj.offsets[0].offset_x, 0.0, 0.001);
        ASSERT_NEAR(traj.offsets[0].offset_y, -100.0, 0.001);
        ASSERT_NEAR(traj.offsets[0].offset_z, 0.0, 0.001);
    }

    TEST(CooperativeLaneChangePlugin,TestLaneChangefunctions)
    {
        // File to process. 
        std::string path = ament_index_cpp::get_package_share_directory("basic_autonomy_ros2");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";
        file = path.append(file);

        lanelet::Id start_id = 107;
        lanelet::Id lane_change_start_id = 106;
        lanelet::Id lag_veh_start_id = 101;
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
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        //Set Route
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);

        //get starting position
        auto llt = map.get()->laneletLayer.get(start_id);
        lanelet::BasicPoint2d curr_pose = llt.centerline2d().front();

        //Define arguments for function maneuvers_to_points
        rclcpp::NodeOptions options;
        auto worker = std::make_shared<cooperative_lanechange::CooperativeLaneChangePlugin>(options);

        worker->configure(); //Call configure state transition
        worker->activate();  //Call activate state transition to get not read for runtime

        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();

        lanelet::BasicPoint2d veh_pos=shortest_path[0].centerline2d().front();
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        ending_downtrack = cmw->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;

        worker->wm_ = cmw;

        //Define lane change maneuver
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = worker->now();
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed, 2) / (2 * (ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed / acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = rclcpp::Time(end_time) + rclcpp::Duration(10 * 1e9);
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(lane_change_start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
            
        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        worker->current_speed_ = maneuver.lane_change_maneuver.start_speed;
        carma_planning_msgs::msg::VehicleState vehicle_state;
        vehicle_state.x_pos_global = veh_pos.x();
        vehicle_state.y_pos_global = veh_pos.y();  
            
        /* Test plan lanechange */
        carma_planning_msgs::srv::PlanTrajectory::Request req;
        carma_planning_msgs::srv::PlanTrajectory::Response resp;
            
        req.maneuver_plan.planning_start_time = worker->now();
        req.maneuver_plan.planning_completion_time = rclcpp::Time(req.maneuver_plan.planning_start_time) + rclcpp::Duration(10 * 1e9);
        req.vehicle_state.x_pos_global = veh_pos.x();
        req.vehicle_state.y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers_msg;  
        //Define lane change maneuver
            
        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        auto req_ptr = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>(req);
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj_plan = worker->plan_lanechange(req_ptr);
        EXPECT_TRUE(traj_plan.size() > 2);

        carma_v2x_msgs::msg::MobilityRequest request = worker->create_mobility_request(traj_plan, maneuver);
        EXPECT_EQ(carma_v2x_msgs::msg::PlanType::CHANGE_LANE_LEFT, request.plan_type.type);
        /*Test compose trajectort and helper function*/
        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory;
    }

    TEST(CooperativeLaneChangePlugin,TestAddManeuvertoResponse){
        rclcpp::NodeOptions options;
        auto worker = std::make_shared<cooperative_lanechange::CooperativeLaneChangePlugin>(options);

        worker->configure(); //Call configure state transition
        worker->activate();  //Call activate state transition to get not read for runtime
        
        carma_planning_msgs::srv::PlanTrajectory::Request req;
        carma_planning_msgs::srv::PlanTrajectory::Response resp;

        req.maneuver_index_to_plan = 0;
        
        req.maneuver_plan.planning_start_time = worker->now();
        req.maneuver_plan.planning_completion_time = rclcpp::Time(req.maneuver_plan.planning_start_time) + rclcpp::Duration(10 * 1e9);
        
        lanelet::BasicPoint2d veh_pos(1.0,1.0);
        req.vehicle_state.x_pos_global = veh_pos.x();
        req.vehicle_state.y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = 10.0;

        std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> planned_trajectory = {};

        auto req_ptr = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>(req);
        auto resp_ptr = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Response>(resp);
        worker->add_trajectory_to_response(req_ptr, resp_ptr, planned_trajectory);
        EXPECT_EQ(10.0, resp_ptr->trajectory_plan.initial_longitudinal_velocity);
        EXPECT_EQ(0, resp_ptr->related_maneuvers.back());
    }

    TEST(CooperativeLaneChangePlugin,Testcurrentgapcb){
        // File to process. 
        std::string path = ament_index_cpp::get_package_share_directory("basic_autonomy_ros2");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";        
        file = path.append(file);

        lanelet::Id start_id = 107;
        lanelet::Id lane_change_start_id = 106;
        lanelet::Id lag_veh_start_id = 101;
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

        rclcpp::NodeOptions options;
        auto worker = std::make_shared<cooperative_lanechange::CooperativeLaneChangePlugin>(options);

        worker->configure(); //Call configure state transition
        worker->activate();  //Call activate state transition to get not read for runtime

        //Define arguments for function maneuvers_to_points

        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();

        lanelet::BasicPoint2d veh_pos=map.get()->laneletLayer.get(lane_change_start_id).centerline2d().front();
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        ending_downtrack = cmw->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;

        worker->wm_ = cmw;                
        
        //Define lane change maneuver
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = worker->now();
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed/acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = rclcpp::Time(end_time) + rclcpp::Duration(10 * 1e9);
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(lane_change_start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
        
        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        worker->current_speed_ = maneuver.lane_change_maneuver.start_speed;
        carma_planning_msgs::msg::VehicleState vehicle_state;
        vehicle_state.x_pos_global = veh_pos.x();
        vehicle_state.y_pos_global = veh_pos.y();
        
        carma_planning_msgs::srv::PlanTrajectory::Request req;
        carma_planning_msgs::srv::PlanTrajectory::Response resp;
        
        req.maneuver_plan.planning_start_time = worker->now();
        req.maneuver_plan.planning_completion_time = rclcpp::Time(req.maneuver_plan.planning_start_time) + rclcpp::Duration(10 * 1e9);
        req.vehicle_state.x_pos_global = veh_pos.x();
        req.vehicle_state.y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers_msg;  
        //Define lane change maneuver

        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
          // //Add roadway object - lag vehicle
        carma_perception_msgs::msg::ExternalObject object;

        // //Test plan trajectory cb
        object.id =1;
        object.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
        object.pose.pose.position.x =0.0;
        object.pose.pose.position.y =0.0;
        object.pose.pose.position.z =0.0;
        object.velocity.twist.linear.x = 5.0;
        
        geometry_msgs::msg::Vector3 size;
        size.x = 4;
        size.y = 2;
        size.z = 1;
        
        object.size = size;

        carma_perception_msgs::msg::PredictedState pred;
        auto pred_pose = object.pose.pose;
        pred_pose.position.y += 1;
        pred.predicted_position = pred_pose;
        pred.predicted_position_confidence = 1.0;
        
        object.predictions.push_back(pred);

        carma_perception_msgs::msg::RoadwayObstacle obstacle;
        obstacle.object = object;
        obstacle.connected_vehicle_type.type = carma_perception_msgs::msg::ConnectedVehicleType::CONNECTED;
        obstacle.lanelet_id = lag_veh_start_id;
        obstacle.cross_track = 0.0;
        obstacle.down_track = 0.0;

        std::vector<carma_perception_msgs::msg::RoadwayObstacle> obstacles;
        obstacles.push_back(obstacle);
        cmw->setRoadwayObjects(obstacles);
        cmw->setMap(map);

        EXPECT_TRUE(worker->find_current_gap(obstacle.lanelet_id,obstacle.down_track, req.vehicle_state) > 0.0);
    }

    TEST(CooperativeLaneChangePlugin,TestNoPath_roadwayobject){
        //Tests behavior when there is no path from roadway object to subject vehicle

        // File to process. 
        std::string path = ament_index_cpp::get_package_share_directory("basic_autonomy_ros2");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm"; 
        file = path.append(file);
        lanelet::Id start_id = 107;
        lanelet::Id lane_change_start_id = 106;
        lanelet::Id lag_veh_start_id = 111;     //Lag vehicle at a lanelet from which no path to lane change start lanelet of subject
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

        rclcpp::NodeOptions options;
        auto worker = std::make_shared<cooperative_lanechange::CooperativeLaneChangePlugin>(options);

        worker->configure(); //Call configure state transition
        worker->activate();  //Call activate state transition to get not read for runtime

        //Define arguments for function maneuvers_to_points

        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();

        //lanelet::BasicPoint2d veh_pos1 = map.get()->laneletLayer.get(lane_change_start_id).centerline2d()[10];
        lanelet::BasicPoint2d veh_pos= map.get()->laneletLayer.get(lane_change_start_id).centerline2d()[10];
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        ending_downtrack = cmw->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;

        worker->wm_ = cmw;          

        //Define lane change maneuver
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = worker->now();
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed/acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = rclcpp::Time(end_time) + rclcpp::Duration(10 * 1e9);
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(lane_change_start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
        
        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        worker->current_speed_ = maneuver.lane_change_maneuver.start_speed;
        carma_planning_msgs::msg::VehicleState vehicle_state;
        vehicle_state.x_pos_global = veh_pos.x();
        vehicle_state.y_pos_global = veh_pos.y();
        
        carma_planning_msgs::srv::PlanTrajectory::Request req;
        carma_planning_msgs::srv::PlanTrajectory::Response resp;
        
        req.maneuver_plan.planning_start_time = worker->now();
        req.maneuver_plan.planning_completion_time = rclcpp::Time(req.maneuver_plan.planning_start_time) + rclcpp::Duration(10 * 1e9);
        req.vehicle_state.x_pos_global = veh_pos.x();
        req.vehicle_state.y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<carma_planning_msgs::msg::Maneuver> maneuvers_msg;  
        //Define lane change maneuver

        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        //Add roadway object - lag vehicle
        carma_perception_msgs::msg::ExternalObject object;

        // //Test plan trajectory cb
        //Create a roadway object - Here we define an object from which there is no path to the subject
        object.id =1;
        object.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
        object.pose.pose.position.x =0.0;
        object.pose.pose.position.y =0.0;
        object.pose.pose.position.z =0.0;
        object.velocity.twist.linear.x = 5.0;
        
        geometry_msgs::msg::Vector3 size;
        size.x = 4;
        size.y = 2;
        size.z = 1;
        
        object.size = size;

        carma_perception_msgs::msg::PredictedState pred;
        auto pred_pose = object.pose.pose;
        pred_pose.position.y += 1;
        pred.predicted_position = pred_pose;
        pred.predicted_position_confidence = 1.0;
        
        object.predictions.push_back(pred);

        carma_perception_msgs::msg::RoadwayObstacle obstacle;
        obstacle.object = object;
        obstacle.connected_vehicle_type.type = carma_perception_msgs::msg::ConnectedVehicleType::CONNECTED;
        obstacle.lanelet_id = lag_veh_start_id;
        obstacle.cross_track = 0.0;
        obstacle.down_track = 0.0;

        std::vector<carma_perception_msgs::msg::RoadwayObstacle> obstacles;
        obstacles.push_back(obstacle);
        cmw->setRoadwayObjects(obstacles);
        cmw->setMap(map);
   
        try{
            worker->find_current_gap(obstacle.lanelet_id, obstacle.down_track, req.vehicle_state);
        }   
        catch(std::exception &ex){
            EXPECT_EQ(ex.what(), std::string("No path exists from roadway object to subject"));
        }

    }
}

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