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
#include <trajectory_utils/conversions/conversions.hpp>

namespace trajectory_utils
{
namespace conversions
{
TEST(trajectory_utils_conversions_test, trajectory_to_downtrack_time)
{
  // Nominal case
  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> traj_points;
  double start_time = 0;

  std::vector<double> downtracks, times;

  carma_planning_msgs::msg::TrajectoryPlanPoint p1, p2, p3;

  double step_size = 1;
  p1.x = 0;
  p1.y = 0;
  p1.target_time = rclcpp::Time(start_time * 1e9); // Convert seconds to nanoseconds

  p2.x = step_size;
  p2.y = 0;
  p2.target_time = rclcpp::Time(1.0 * 1e9); // Convert 1 second to nanoseconds

  p3.x = step_size;
  p3.y = step_size;
  p3.target_time = rclcpp::Time(1.5 * 1e9); // Convert 1.5 seconds to nanoseconds

  traj_points = { p1, p2, p3 };

  trajectory_to_downtrack_time(traj_points, &downtracks, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_EQ(downtracks.size(), traj_points.size());

  for (size_t i = 0; i < downtracks.size(); i++)
  {
    ASSERT_EQ(downtracks[i], i * step_size);
    ASSERT_NEAR(times[i], rclcpp::Time(traj_points[i].target_time).seconds(), 0.0000000001);
  }

  // Non-Zero start time
  start_time = 0.5;

  downtracks = {};
  times = {};

  traj_points[0].target_time = rclcpp::Time(start_time * 1e9); // Convert seconds to nanoseconds

  trajectory_to_downtrack_time(traj_points, &downtracks, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_EQ(downtracks.size(), traj_points.size());

  for (size_t i = 0; i < downtracks.size(); i++)
  {
    ASSERT_EQ(downtracks[i], i * step_size);
    ASSERT_NEAR(times[i], rclcpp::Time(traj_points[i].target_time).seconds(), 0.0000000001);
  }

  // Empty input
  traj_points = {};
  downtracks = {};
  times = {};

  trajectory_to_downtrack_time(traj_points, &downtracks, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_EQ(downtracks.size(), traj_points.size());
}

TEST(trajectory_utils_conversions_test, speed_to_time)
{
  // Nominal case
  std::vector<double> downtracks = {0, 1, 2, 3 };
  std::vector<double> speeds = {1,1,1,1};
  std::vector<double> times;

  speed_to_time(downtracks, speeds, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  for (size_t i = 0; i < times.size(); i++)
  {
    ASSERT_NEAR(times[i], (double)i, 0.0000000001);
  }

  // 0 input case
  downtracks = {};
  speeds = {};
  times = {};
  ASSERT_THROW(speed_to_time(downtracks, speeds, &times), std::invalid_argument);

  // Unequal input case
  downtracks = {1, 2};
  speeds = {1};
  times = {};
  ASSERT_THROW(speed_to_time(downtracks, speeds, &times), std::invalid_argument);

  // Complex case
  downtracks = {2, 4, 7};
  speeds = {1, 3, 1};
  times = {};

  speed_to_time(downtracks, speeds, &times);

  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_NEAR(times[0], 0.0, 0.0000001);
  ASSERT_NEAR(times[1], 1.0, 0.0000001);
  ASSERT_NEAR(times[2], 2.5, 0.0000001);

}

TEST(trajectory_utils_conversions_test, time_to_speed)
{
  // Nominal case
  std::vector<double> downtracks = {0, 1, 2, 3 };
  std::vector<double> times = {1,2,3,4};
  std::vector<double> speeds;

  time_to_speed(downtracks, times, 1.0, &speeds);

  ASSERT_EQ(downtracks.size(), speeds.size());
  for (size_t i = 0; i < speeds.size(); i++)
  {
    ASSERT_NEAR(speeds[i], 1.0, 0.0000000001);
  }

  // 0 input case
  downtracks = {};
  speeds = {};
  times = {};
  ASSERT_THROW(time_to_speed(downtracks, times, 1.0, &speeds), std::invalid_argument);

  // Unequal input case
  downtracks = {1, 2};
  speeds = {1};
  times = {};
  ASSERT_THROW(time_to_speed(downtracks, times, 1.0, &speeds), std::invalid_argument);

  // Complex case
  downtracks = {2, 4, 7};
  times = {0.0, 1.0, 2.5};
  speeds = {};
  time_to_speed(downtracks, times, 1.0, &speeds);
  
  ASSERT_EQ(downtracks.size(), times.size());
  ASSERT_NEAR(speeds[0], 1.0, 0.0000001);
  ASSERT_NEAR(speeds[1], 3.0, 0.0000001);
  ASSERT_NEAR(speeds[2], 1.0, 0.0000001);

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

}  // namespace conversions
}  // namespace trajectory_utils