/*
 * Copyright (C) 2021 LEIDOS.
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
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <cav_msgs/Maneuver.h>
#include <intersection_transit_maneuvering.h>
#include <cav_srvs/PlanTrajectory.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace intersection_transit_maneuvering
{

/*Test Callback Operation*/
TEST(Intersection_Transit_Maneuvering_Test, Planning_Callback_Test)
{
    /*Create World Model Pointer and Set the Map */
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

    wm->setMap(map);

    carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

    IntersectionTransitManeuvering::IntersectionTransitManeuvering itm_node(wm,[&](auto msg),{});
    cav_msgs::Maneuver man0, man1;

    man1.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
    man1.intersection_transit_straight_maneuver.start_dist = 0.0;
    man1.intersection_transit_straight_maneuver.end_dist = 5.0;
    man1.intersection_transit_straight_maneuver.start_time = ros::Time(0.0);
    man1.intersection_transit_straight_maneuver.end_time = ros::Time(1.7701);
    man1.intersection_transit_straight_maneuver.starting_lane_id = 1200;

    cav_srvs::PlanTrajectoryRequest req;
    req.vehicle_state.X_pos_global = 1.5;
    req.vehicle_state.Y_pos_global = 5;
    req.vehicle_state.orientation = 0;
    req.vehicle_state.longitudinal_vel = 0.0;

    req.maneuver_plan.maneuvers.push_back(man1);

    cav_srvs::PlanTrajectoryResponse resp;

    itm_node.plan_trajectory_cb(req, resp);

    //Assert that the maneuver status will be updated after the callback function
    ASSERT_EQ(1, resp.maneuver_status.size());

    man0.type = cav_msgs::Maneuver::LANE_FOLLOWING;
    man0.lane_following_maneuver.start_dist = 0.0;
    man0.lane_following_maneuver.end_dist = 5.0;
    man0.lane_following_maneuver.start_time = ros::Time(0.0);
    man0.lane_following_maneuver.end_time = ros::Time(1.7701);
    man0.lane_following_maneuver.starting_lane_id = 1200;

    req.maneuver_plan.maneuvers.push_back(man0);

    //Test that the operation will throw an invalid argrument error statement due to a non-applicable maneuver type being used
    EXPECT_THROW(itm_node.plan_trajectory_cb(req, resp),std::invalid_argument());

}

TEST(Intersection_Transit_Maneuvering_Test, Convert_Maneuvers_Test)
{
    /*Create World Model Pointer and Set the Map */
    std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
    auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

    wm->setMap(map);

    carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

    IntersectionTransitManeuvering::IntersectionTransitManeuvering itm_node(wm,[&](auto msg),{});
    cav_msgs::Maneuver man1, man2, man3;

    man1.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
    man1.intersection_transit_straight_maneuver.start_dist = 0.0;
    man1.intersection_transit_straight_maneuver.end_dist = 5.0;
    man1.intersection_transit_straight_maneuver.start_time = ros::Time(0.0);
    man1.intersection_transit_straight_maneuver.end_time = ros::Time(1.7701);
    man1.intersection_transit_straight_maneuver.starting_lane_id = 1200;

    man2.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN;
    man2.intersection_transit_left_turn_maneuver.start_dist = 0.0;
    man2.intersection_transit_left_turn_maneuver.end_dist = 5.0;
    man2.intersection_transit_left_turn_maneuver.start_time = ros::Time(0.0);
    man2.intersection_transit_left_turn_maneuver.end_time = ros::Time(1.7701);
    man2.intersection_transit_left_turn_maneuver.starting_lane_id = 1200;

    man3.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN;
    man3.intersection_transit_right_turn_maneuver.start_dist = 0.0;
    man3.intersection_transit_right_turn_maneuver.end_dist = 5.0;
    man3.intersection_transit_right_turn_maneuver.start_time = ros::Time(0.0);
    man3.intersection_transit_right_turn_maneuver.end_time = ros::Time(1.7701);
    man3.intersection_transit_right_turn_maneuver.starting_lane_id = 1200;

    std::vector<cav_msgs::Maneuver> maneuvers;

    auto converted = itm_node.convert_maneuver_plan(maneuvers);
    EXPECT_THROW(converted, std::invalid_argument); //Test that the operation will throw an exception due to there being no maneuvers to convert

    maneuvers.push_back(man1);
    maneuvers.push_back(man2);
    maneuvers.push_back(man3);

    auto converted2 = itm_node.convert_maneuver_plan(maneuvers);

    for(auto i: maneuvers)
    {
        ASSERT_EQ(true, i.maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING); //Test that each maneuver has been converted to LANE_FOLLOWING
    }

}


}