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

#include "mobilitypath_publisher/mobilitypath_publisher.hpp"

TEST(Testmobilitypath_publisher, test1)
{
    carma_planning_msgs::msg::TrajectoryPlan plan;
    carma_planning_msgs::msg::TrajectoryPlanPoint p1;
    p1.x = 20.0;
    p1.y = 0.0;
    p1.target_time = rclcpp::Time(0,0);

    carma_planning_msgs::msg::TrajectoryPlanPoint p2;
    p2.x = 19.0;
    p2.y = 0.0;
    p2.target_time = rclcpp::Time(0,1);

    plan.trajectory_points.push_back(p1);
    plan.trajectory_points.push_back(p2);
    
    rclcpp::NodeOptions options;
    auto worker = std::make_shared<mobilitypath_publisher::MobilityPathPublication>(options);

    std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                        "+no_defs";
    std::unique_ptr<std_msgs::msg::String> msg_ptr = std::make_unique<std_msgs::msg::String>();
    msg_ptr->data = base_proj;
    worker->georeference_cb(move(msg_ptr));  // Set projection

    auto res = worker->mobility_path_message_generator(plan);
    ASSERT_EQ(1, res.trajectory.offsets.size());

    ASSERT_EQ(637813699.0, res.trajectory.location.ecef_x);
    ASSERT_EQ(1999.0, res.trajectory.location.ecef_y);
    ASSERT_EQ(0.0, res.trajectory.location.ecef_z);

    ASSERT_EQ(res.trajectory.offsets[0].offset_x, 0);
    ASSERT_EQ(res.trajectory.offsets[0].offset_y, -100);
    ASSERT_EQ(res.trajectory.offsets[0].offset_z, 0);
}

TEST(Testmobilitypath_publisher, test2)
{
    carma_planning_msgs::msg::TrajectoryPlan plan;
    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 1;
    point.y = 1;
    point.target_time = rclcpp::Time(0,1);
    plan.trajectory_points.push_back(point);

    rclcpp::NodeOptions options;
    auto worker = std::make_shared<mobilitypath_publisher::MobilityPathPublication>(options);

    auto res = worker->mobility_path_message_generator(plan);

    // Check values are unset as georeference was not provided
    ASSERT_EQ(0, res.trajectory.location.ecef_x);
    ASSERT_EQ(0, res.trajectory.location.ecef_y);
    ASSERT_EQ(0, res.trajectory.location.ecef_z);
    EXPECT_EQ(0, res.trajectory.offsets.size());
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