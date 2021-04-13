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
#include "cooperative_lanechange.h"
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
#include <sstream>
#include <ros/package.h>
#include <cav_msgs/LaneChangeStatus.h>
#include <cav_msgs/VehicleState.h>
//For Unit Test
#include <cav_msgs/MobilityResponse.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <cav_msgs/LocationECEF.h>
#include <cav_msgs/Trajectory.h>


namespace cooperative_lanechange
{
    cav_msgs::LaneChangeStatus test_lanechange_status;
    void status_callback(const cav_msgs::LaneChangeStatusConstPtr& msg){
        test_lanechange_status = *msg.get();
        ROS_INFO_STREAM("Test callback...");
    }
    TEST(CooperativeLaneChangePlugin,TestMobilityResponse_cb){
        cooperative_lanechange::CooperativeLaneChangePlugin worker;
        ros::NodeHandle nh2;
        worker.lanechange_status_pub_ = nh2.advertise<cav_msgs::LaneChangeStatus>("cooperative_lane_change_status",1);
        cav_msgs::MobilityResponse mob_response_msg;
        mob_response_msg.is_accepted = true;
        worker.mobilityresponse_cb(mob_response_msg);
        
        ros::Subscriber lc_status_sub = nh2.subscribe("cooperative_lane_change_status", 5, status_callback);
        EXPECT_EQ(1, lc_status_sub.getNumPublishers());
        ros::spinOnce();
    }

    TEST(CooperativeLaneChangePlugin,Testusingosm){
    // File to process. Path is relative to unobstructed_lanechange package
        std::string path = ros::package::getPath("unobstructed_lanechange");
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

        // //Add roadway object - lag vehicle
        cav_msgs::ExternalObject object;

        auto llt_lag_veh = map.get()->laneletLayer.get(lag_veh_start_id);
        
        /*---Define position for object (based on lag_veh_start_id)---*/
        lanelet::LineString3d left_bound_lag_veh=llt_lag_veh.leftBound();
        lanelet::LineString3d lag_right_bound_lag_veh=llt_lag_veh.rightBound();
        geometry_msgs::PoseStamped left_lag_veh;
        geometry_msgs::PoseStamped right_lag_veh;
        for(lanelet::Point3d& p : left_bound_lag_veh)
        {
            left_lag_veh.pose.position.x=p.x();
            left_lag_veh.pose.position.y=p.y();
            left_lag_veh.pose.position.z=p.z();

        }
        for(lanelet::Point3d& p : lag_right_bound_lag_veh)
        {
            right_lag_veh.pose.position.x=p.x();
            right_lag_veh.pose.position.y=p.y();
            right_lag_veh.pose.position.z=p.z();
        }
        geometry_msgs::PoseStamped lag_curr_pos;
        lag_curr_pos.pose.position.x=(left.pose.position.x+right.pose.position.x)/2;
        lag_curr_pos.pose.position.y=(left.pose.position.y+right.pose.position.y)/2;
        lag_curr_pos.pose.position.z=(left.pose.position.z+right.pose.position.z)/2;

        lag_curr_pos.pose.orientation.x=0.0;
        lag_curr_pos.pose.orientation.y=0.0;
        lag_curr_pos.pose.orientation.z=0.0;
        lag_curr_pos.pose.orientation.w=0.0;

        object.id =1;
        object.object_type = cav_msgs::ExternalObject::SMALL_VEHICLE;
        object.pose.pose = lag_curr_pos.pose;
        object.velocity.twist.linear.x = 5.0;
        
        geometry_msgs::Vector3 size;
        size.x = 4;
        size.y = 2;
        size.z = 1;
        
        object.size = size;

        cav_msgs::PredictedState pred;
        auto pred_pose = object.pose.pose;
        pred_pose.position.y += 1;
        pred.predicted_position = pred_pose;
        pred.predicted_position_confidence = 1.0;
        
        object.predictions.push_back(pred);
        
        // /*-------------------Define roadway obstacle params-------------------*/
        //auto obs = cmw->toRoadwayObstacle(object);    
        cav_msgs::RoadwayObstacle obstacle;
        obstacle.object = object;
        obstacle.connected_vehicle_type.type = cav_msgs::ConnectedVehicleType::CONNECTED_AND_AUTOMATED;
        obstacle.lanelet_id = lag_veh_start_id;
        obstacle.cross_track = 0.0;
        obstacle.down_track = 0.0;

        std::vector<cav_msgs::RoadwayObstacle> obstacles;
        obstacles.push_back(obstacle);
        cmw->setRoadwayObjects(obstacles);
        cmw->setMap(map);
        
        
        //To test find current gap
        cooperative_lanechange::CooperativeLaneChangePlugin worker;
        ros::NodeHandle nh;
        worker.outgoing_mobility_request_ = nh.advertise<cav_msgs::MobilityRequest>("test_mobility_request",1);
        worker.lanechange_status_pub_ = nh.advertise<cav_msgs::LaneChangeStatus>("cooperative_lane_change_status",1);
        worker.wm_ = cmw;
        if(start_id !=lag_veh_start_id){
            EXPECT_TRUE(worker.find_current_gap(obstacle.lanelet_id,obstacle.down_track) > 0.0);
        }



        //Test plan trajectory cb
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;

        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = curr_pos.pose.position.x;
        req.vehicle_state.Y_pos_global = curr_pos.pose.position.y;
        //Define lane change maneuver
        cav_msgs::Maneuver maneuver;
        maneuver.type=cav_msgs::Maneuver::LANE_CHANGE;
        lanelet::BasicPoint2d veh_pos(curr_pos.pose.position.x,curr_pos.pose.position.y);
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();
        double ending_downtrack = cmw->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = ros::Time::now();
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed/acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = ros::Time(end_time + 10.0);
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(lane_change_start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);

        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<cav_msgs::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        worker.current_speed_ = maneuver.lane_change_maneuver.start_speed;

        req.maneuver_plan.maneuvers = maneuvers;
        worker.is_lanechange_accepted_ = true;
        bool isTrajectory = worker.plan_trajectory_cb(req,resp);
        ROS_INFO_STREAM("Trajectory plan size:"<<resp.trajectory_plan.trajectory_points.size());
        EXPECT_TRUE(isTrajectory);
        EXPECT_TRUE(resp.trajectory_plan.trajectory_points.size() > 2);
        
        //Test lane change status
        ros::Subscriber lc_status_sub = nh.subscribe("cooperative_lane_change_status", 5, status_callback);
        EXPECT_EQ(1, lc_status_sub.getNumPublishers());
        ros::spinOnce();
    }

}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_cooperative_lanechange");
    ROSCONSOLE_AUTOINIT;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
    return RUN_ALL_TESTS();
}