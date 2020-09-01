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

#include "autoware_plugin.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include "TestHelpers.h"

TEST(AutowarePluginTest, testGetWaypointsInTimeBoundary1)
{
    // compose a list of waypoints spanning 8 seconds
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 6.0;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 8.0;
    wp_3.pose.pose.position.x = 24.0;
    autoware_msgs::Waypoint wp_4;
    wp_4.twist.twist.linear.x = 8.0;
    wp_4.pose.pose.position.x = 40.0;
    autoware_msgs::Waypoint wp_5;
    wp_5.twist.twist.linear.x = 8.0;
    wp_5.pose.pose.position.x = 48.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    waypoints.push_back(wp_4);
    waypoints.push_back(wp_5);
    autoware_plugin::AutowarePlugin ap;
    std::vector<autoware_msgs::Waypoint> res = ap.get_waypoints_in_time_boundary(waypoints, 6.0);
    EXPECT_EQ(4, res.size());
    EXPECT_NEAR(2.0, res[0].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(0.0, res[0].pose.pose.position.x, 0.01);
    EXPECT_NEAR(4.0, res[1].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(6.0, res[1].pose.pose.position.x, 0.01);
    EXPECT_NEAR(8.0, res[2].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(24.0, res[2].pose.pose.position.x, 0.01);
    EXPECT_NEAR(8.0, res.back().twist.twist.linear.x, 0.01);
    EXPECT_NEAR(40.0, res.back().pose.pose.position.x, 0.01);
}

TEST(AutowarePluginTest, testGetWaypointsInTimeBoundary2)
{
    // compose a list of waypoints spaning less than 6 seconds
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 6.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    autoware_plugin::AutowarePlugin ap;
    std::vector<autoware_msgs::Waypoint> res = ap.get_waypoints_in_time_boundary(waypoints, 6.0);
    EXPECT_EQ(2, res.size());
    EXPECT_NEAR(2.0, res[0].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(0.0, res[0].pose.pose.position.x, 0.01);
    EXPECT_NEAR(4.0, res[1].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(6.0, res[1].pose.pose.position.x, 0.01);
}

TEST(AutowarePluginTest, testGetWaypointsInTimeBoundary3)
{
    // compose a list of waypoints spanning exactly 5 seconds
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 6.0;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 8.0;
    wp_3.pose.pose.position.x = 24.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    autoware_plugin::AutowarePlugin ap;
    std::vector<autoware_msgs::Waypoint> res = ap.get_waypoints_in_time_boundary(waypoints, 5.0);
    EXPECT_EQ(3, res.size());
    EXPECT_NEAR(2.0, res[0].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(0.0, res[0].pose.pose.position.x, 0.01);
    EXPECT_NEAR(4.0, res[1].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(6.0, res[1].pose.pose.position.x, 0.01);
    EXPECT_NEAR(8.0, res[2].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(24.0, res[2].pose.pose.position.x, 0.01);
}

TEST(AutowarePluginTest, testCreateUnevenTrajectory1)
{
    // compose a list of waypoints, uneven spaced
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 0.5;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 2.0;
    wp_3.pose.pose.position.x = 1.3;
    autoware_msgs::Waypoint wp_4;
    wp_4.twist.twist.linear.x = 4.0;
    wp_4.pose.pose.position.x = 1.4;
    autoware_msgs::Waypoint wp_5;
    wp_5.twist.twist.linear.x = 4.0;
    wp_5.pose.pose.position.x = 2.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    waypoints.push_back(wp_4);
    waypoints.push_back(wp_5);
    autoware_plugin::AutowarePlugin ap;
    // create pose message to indicate that the current location is on top of the starting waypoint
    ap.pose_msg_.reset(new geometry_msgs::PoseStamped());
    std::vector<cav_msgs::TrajectoryPlanPoint> traj = ap.create_uneven_trajectory_from_waypoints(waypoints);
    EXPECT_EQ(5, traj.size());
    EXPECT_NEAR(0.0, traj[0].target_time, 0.01);
    EXPECT_NEAR(0.0, traj[0].x, 0.01);
    EXPECT_NEAR(0.25, traj[1].target_time / 1e9, 0.01);
    EXPECT_NEAR(0.5, traj[1].x, 0.01);
    EXPECT_NEAR(0.45, traj[2].target_time / 1e9, 0.01);
    EXPECT_NEAR(1.3, traj[2].x, 0.01);
    EXPECT_NEAR(0.5, traj[3].target_time / 1e9, 0.01);
    EXPECT_NEAR(1.4, traj[3].x, 0.01);
    EXPECT_NEAR(0.65, traj[4].target_time / 1e9, 0.01);
    EXPECT_NEAR(2.0, traj[4].x, 0.01);
}

TEST(AutowarePluginTest, testCreateUnevenTrajectory2)
{
    // compose a list of waypoints, uneven spaced
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 0.5;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 2.0;
    wp_3.pose.pose.position.x = 1.3;
    autoware_msgs::Waypoint wp_4;
    wp_4.twist.twist.linear.x = 4.0;
    wp_4.pose.pose.position.x = 1.4;
    autoware_msgs::Waypoint wp_5;
    wp_5.twist.twist.linear.x = 4.0;
    wp_5.pose.pose.position.x = 2.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    waypoints.push_back(wp_4);
    waypoints.push_back(wp_5);
    autoware_plugin::AutowarePlugin ap;
    // create pose message to indicate that the current location is not near the starting waypoint
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1.0;
    ap.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));
    std::vector<cav_msgs::TrajectoryPlanPoint> traj = ap.create_uneven_trajectory_from_waypoints(waypoints);
    EXPECT_EQ(6, traj.size());
    EXPECT_NEAR(0.0, traj[0].target_time / 1e9, 0.01);
    EXPECT_NEAR(-1.0, traj[0].x, 0.01);
    EXPECT_NEAR(0.5, traj[1].target_time / 1e9, 0.01);
    EXPECT_NEAR(0.0, traj[1].x, 0.01);
    EXPECT_NEAR(0.75, traj[2].target_time / 1e9, 0.001);
    EXPECT_NEAR(0.5, traj[2].x, 0.01);
    EXPECT_NEAR(0.95, traj[3].target_time / 1e9, 0.001);
    EXPECT_NEAR(1.3, traj[3].x, 0.01);
    EXPECT_NEAR(1.0, traj[4].target_time / 1e9, 0.001);
    EXPECT_NEAR(1.4, traj[4].x, 0.01);
    EXPECT_NEAR(1.15, traj[5].target_time / 1e9, 0.001);
    EXPECT_NEAR(2.0, traj[5].x, 0.01);
}


TEST(AutowarePluginTest, testUpdateTrajForObject)
{

    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

    auto pl1 = carma_wm::getPoint(0, 0, 0);
    auto pl2 = carma_wm::getPoint(0, 1, 0);
    auto pl3 = carma_wm::getPoint(0, 2, 0);
    auto pr1 = carma_wm::getPoint(1, 0, 0);
    auto pr2 = carma_wm::getPoint(1, 1, 0);
    auto pr3 = carma_wm::getPoint(1, 2, 0);

    std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
    std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
    auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed);

    std::vector<lanelet::Point3d> left_2 = { pl2, pl3 };
    std::vector<lanelet::Point3d> right_2 = { pr2, pr3 };
    auto ll_2 = carma_wm::getLanelet(left_2, right_2);

    // 1. Confirm all pointers are false (done above)
    // Ensure that none of the returned pointers are valid if the map has not been set
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_FALSE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());

    // 2. Build map but do not assign
    // Create basic map and verify that the map and routing graph can be build, but the route remains false
    lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});
    // 3. Build routing graph but do not assign
    // Build routing graph from map
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    // 4. Generate route
    auto optional_route = map_graph->getRoute(ll_1, ll_2);
    ASSERT_TRUE((bool)optional_route);

    lanelet::routing::Route route = std::move(*optional_route);
    carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
    // 5. Try to set route without map and ensure it passes
    cmw->setRoute(route_ptr);
    // 6. getRoute is true but other pointers are false
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());

    cmw->setMap(map);
    // 8. All pointers exist
    ASSERT_TRUE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());
    // 9. Call setRoute again to confirm no errors
    cmw->setRoute(route_ptr);
    // 10. All pointers exist
    ASSERT_TRUE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());



    cav_msgs::RoadwayObstacleList rwol;
    cav_msgs::TrajectoryPlan tp;

    geometry_msgs::Twist veloctiy;
    __uint64_t target_time = 3;

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity.x = 0;
    linear_velocity.y = 0;

    veloctiy.linear = linear_velocity;

    
    geometry_msgs::Vector3 size;
    size.x = 4;
    size.y = 2;
    size.z = 1;

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;

    trajectory_point_1.x = 1.0;
    trajectory_point_1.y = 1.0;
    trajectory_point_1.lane_id = "1";
    trajectory_point_1.target_time = 0.0;

    trajectory_point_2.x = 1.0;
    trajectory_point_2.y = 3.0;
    trajectory_point_2.target_time = 1.0;
    trajectory_point_2.lane_id = "1";

    trajectory_point_3.x = 1.0;
    trajectory_point_3.y = 4.0;
    trajectory_point_3.target_time = 2.0;
    trajectory_point_3.lane_id = "1";

    trajectory_point_4.x = 1.0;
    trajectory_point_4.y = 6.0;
    trajectory_point_4.target_time = 3.0;
    trajectory_point_4.lane_id = "1";

    trajectory_point_5.x = 1.0;
    trajectory_point_5.y = 7.0;
    trajectory_point_5.target_time = 4.0;
    trajectory_point_5.lane_id = "1";

    tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5};

    cav_msgs::RoadwayObstacle rwo_1;

    tf2::Quaternion tf_orientation;
    tf_orientation.setRPY(0, 0, 1.5708);

    rwo_1.object.pose.pose.position.x = 1;
    rwo_1.object.pose.pose.position.y = 2;
    rwo_1.object.pose.pose.position.z = 0;

    rwo_1.object.pose.pose.orientation.x = tf_orientation.getX();
    rwo_1.object.pose.pose.orientation.y = tf_orientation.getY();
    rwo_1.object.pose.pose.orientation.z = tf_orientation.getZ();
    rwo_1.object.pose.pose.orientation.w = tf_orientation.getW();

    rwo_1.object.size.x = 1;
    rwo_1.object.size.y = 1;
    rwo_1.object.size.z = 1;

    cav_msgs::PredictedState ps_1;
    ps_1.header.stamp.nsec = 1000;

    ps_1.predicted_position.position.x = 1;
    ps_1.predicted_position.position.y = 1;
    ps_1.predicted_position.position.z = 0;

    ps_1.predicted_position.orientation.x = tf_orientation.getX();
    ps_1.predicted_position.orientation.y = tf_orientation.getY();
    ps_1.predicted_position.orientation.z = tf_orientation.getZ();
    ps_1.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_2;
    ps_2.header.stamp.nsec = 2000;

    ps_2.predicted_position.position.x = 1;
    ps_2.predicted_position.position.y = 2;
    ps_2.predicted_position.position.z = 0;

    ps_2.predicted_position.orientation.x = tf_orientation.getX();
    ps_2.predicted_position.orientation.y = tf_orientation.getY();
    ps_2.predicted_position.orientation.z = tf_orientation.getZ();
    ps_2.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_3;
    ps_3.header.stamp.nsec = 3000;

    ps_3.predicted_position.position.x = 1;
    ps_3.predicted_position.position.y = 3;
    ps_3.predicted_position.position.z = 0;

    ps_3.predicted_position.orientation.x = tf_orientation.getX();
    ps_3.predicted_position.orientation.y = tf_orientation.getY();
    ps_3.predicted_position.orientation.z = tf_orientation.getZ();
    ps_3.predicted_position.orientation.w = tf_orientation.getW();

    rwo_1.object.predictions = {ps_1,ps_2,ps_3};
    rwo_1.down_track = 2;

    std::vector<cav_msgs::RoadwayObstacle> rw_objs;

    rw_objs.push_back(rwo_1);

    cmw->setRoadwayObjects(rw_objs);

    autoware_plugin::AutowarePlugin ap;

    ap.host_vehicle_size = size;
    ap.velocity = veloctiy;

    ap.wm_ = cmw;

    ros::Time now;
    now.sec = 1.0;
    now.nsec = 0.0;

    tp.header.frame_id = "world";
    tp.header.stamp = now;
    tp.trajectory_id = 1;

    cav_msgs::TrajectoryPlan tp_new = ap.update_traj_for_object(tp);
    EXPECT_EQ(tp_new.trajectory_points.size(), 4);

}




TEST(AutowarePluginTest, PolynomialCalc)
{
    autoware_plugin::AutowarePlugin ap;

    std::vector<double> coeff;
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);
    coeff.push_back(2.0);    
    coeff.push_back(2.0);    

    double result = ap.polynomial_calc(coeff, 0);
    EXPECT_EQ(2, result);

    result = ap.polynomial_calc(coeff, 1);
    EXPECT_EQ(12, result);

    result = ap.polynomial_calc(coeff, 2);
    EXPECT_EQ(126, result);

    result = ap.polynomial_calc(coeff, 3);
    EXPECT_EQ(728, result);
}

TEST(AutowarePluginTest, MaxTrajectorySpeed)
{

    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;

    cav_msgs::TrajectoryPlanPoint point_1;
    point_1.x = 0.0;
    point_1.y = 0.0;
    point_1.target_time = 0.0;
    point_1.lane_id = "1";
    trajectory_points.push_back(point_1);

    cav_msgs::TrajectoryPlanPoint point_2;
    point_2.x = 5.0;
    point_2.y = 0.0;
    point_2.target_time = 1.0;
    point_2.lane_id = "1";
    trajectory_points.push_back(point_2);

    cav_msgs::TrajectoryPlanPoint point_3;
    point_3.x = 10.0;
    point_3.y = 0.0;
    point_3.target_time = 2.0;
    point_3.lane_id = "1";
    trajectory_points.push_back(point_3);

    cav_msgs::TrajectoryPlanPoint point_4;
    point_4.x = 15.0;
    point_4.y = 0.0;
    point_4.target_time = 3.0;
    point_4.lane_id = "1";
    trajectory_points.push_back(point_4);

    cav_msgs::TrajectoryPlanPoint point_5;
    point_5.x = 20.0;
    point_5.y = 0.0;
    point_5.target_time = 4.0;
    point_5.lane_id = "1";
    trajectory_points.push_back(point_5);

    cav_msgs::TrajectoryPlanPoint point_6;
    point_6.x = 25.0;
    point_6.y = 0.0;
    point_6.target_time = 5.0;
    point_6.lane_id = "1";
    trajectory_points.push_back(point_6);


    cav_msgs::TrajectoryPlanPoint point_7;
    point_7.x = 30.0;
    point_7.y = 0.0;
    point_7.target_time = 6.0;
    point_7.lane_id = "1";
    trajectory_points.push_back(point_7);

    autoware_plugin::AutowarePlugin ap;
    double result = ap.max_trajectory_speed(trajectory_points);
    EXPECT_EQ(5, result);

}



// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


