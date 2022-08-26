/*
 * Copyright (C) 2022 LEIDOS.
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

#include <yield_plugin/yield_plugin.hpp>
#include <yield_plugin/yield_plugin_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm_ros2/CARMAWorldModel.hpp>
#include <math.h>
#include <boost/property_tree/json_parser.hpp>
#include <carma_wm_ros2/WMTestLibForGuidance.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm_ros2/MapConformer.hpp>
#include <unsupported/Eigen/Splines>
#include <tf2/LinearMath/Vector3.h>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <carma_perception_msgs/msg/predicted_state.hpp>

using namespace yield_plugin;


TEST(YieldPluginTest, test_polynomial_calc)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  std::vector<double> coeff;
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);    
  coeff.push_back(2.0);

  double result = plugin.polynomial_calc(coeff, 0);
  EXPECT_EQ(2, result);

  result = plugin.polynomial_calc(coeff, 1);
  EXPECT_EQ(12, result);
  
  result = plugin.polynomial_calc(coeff, 2);
  EXPECT_EQ(126, result);

  result = plugin.polynomial_calc(coeff, 3);
  EXPECT_EQ(728, result);
}

TEST(YieldPluginTest, test_polynomial_calc_derivative)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  std::vector<double> coeff;
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);
  coeff.push_back(2.0);    
  coeff.push_back(2.0);

  double result = plugin.polynomial_calc_d(coeff, 0);
  EXPECT_EQ(2, result);

  result = plugin.polynomial_calc_d(coeff, 1);
  EXPECT_EQ(30, result);
  
  result = plugin.polynomial_calc_d(coeff, 2);
  EXPECT_EQ(258, result);

  result = plugin.polynomial_calc_d(coeff, 3);
  EXPECT_EQ(1094, result);
}

TEST(YieldPluginTest, MaxTrajectorySpeed)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points;

  rclcpp::Time startTime(1.0);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_1;
  point_1.x = 0.0;
  point_1.y = 0.0;
  point_1.target_time = startTime;
  point_1.lane_id = "1";
  trajectory_points.push_back(point_1);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_2;
  point_2.x = 5.0;
  point_2.y = 0.0;
  point_2.target_time = startTime + rclcpp::Duration(1*1e9);
  point_2.lane_id = "1";
  trajectory_points.push_back(point_2);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_3;
  point_3.x = 10.0;
  point_3.y = 0.0;
  point_3.target_time = startTime + rclcpp::Duration(2*1e9);
  point_3.lane_id = "1";
  trajectory_points.push_back(point_3);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_4;
  point_4.x = 15.0;
  point_4.y = 0.0;
  point_4.target_time = startTime + rclcpp::Duration(3*1e9);
  point_4.lane_id = "1";
  trajectory_points.push_back(point_4);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_5;
  point_5.x = 20.0;
  point_5.y = 0.0;
  point_5.target_time = startTime + rclcpp::Duration(4*1e9);
  point_5.lane_id = "1";
  trajectory_points.push_back(point_5);

  carma_planning_msgs::msg::TrajectoryPlanPoint point_6;
  point_6.x = 25.0;
  point_6.y = 0.0;
  point_6.target_time = startTime + rclcpp::Duration(5*1e9);
  point_6.lane_id = "1";
  trajectory_points.push_back(point_6);


 carma_planning_msgs::msg::TrajectoryPlanPoint point_7;
  point_7.x = 40.0;
  point_7.y = 0.0;
  point_7.target_time = startTime + rclcpp::Duration(6*1e9);
  point_7.lane_id = "1";
  trajectory_points.push_back(point_7);

  double result = plugin.max_trajectory_speed(trajectory_points);
  EXPECT_EQ(5, result);

}

TEST(YieldPluginTest, test_update_traj)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(100,100);
  
  wm->setMap(map);
  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);
  
  YieldPluginConfig config;
  config.vehicle_length = 4;
  config.vehicle_width = 2;
  config.vehicle_height = 1;
  
  // std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  

  carma_perception_msgs::msg::RoadwayObstacleList rwol;
  carma_planning_msgs::msg::TrajectoryPlan tp;
  
  rclcpp::Time startTime(1.0);

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 10.0;
  trajectory_point_2.y = 20.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 30.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);

  trajectory_point_4.x = 10.0;
  trajectory_point_4.y = 40.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 10.0;
  trajectory_point_5.y = 50.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 10.0;
  trajectory_point_6.y = 60.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 10.0;
  trajectory_point_7.y = 70.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);

  tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};

  carma_perception_msgs::msg::RoadwayObstacle rwo_1;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  rwo_1.object.pose.pose.position.x = 60;
  rwo_1.object.pose.pose.position.y = 50;
  rwo_1.object.pose.pose.position.z = 0;

  rwo_1.object.pose.pose.orientation.x = tf_orientation.getX();
  rwo_1.object.pose.pose.orientation.y = tf_orientation.getY();
  rwo_1.object.pose.pose.orientation.z = tf_orientation.getZ();
  rwo_1.object.pose.pose.orientation.w = tf_orientation.getW();

  rwo_1.object.size.x = 1;
  rwo_1.object.size.y = 1;
  rwo_1.object.size.z = 1;

  carma_perception_msgs::msg::PredictedState ps_1;
  ps_1.header.stamp.nanosec = 1000;

  ps_1.predicted_position.position.x = 10;
  ps_1.predicted_position.position.y = 10;
  ps_1.predicted_position.position.z = 0;

  ps_1.predicted_position.orientation.x = tf_orientation.getX();
  ps_1.predicted_position.orientation.y = tf_orientation.getY();
  ps_1.predicted_position.orientation.z = tf_orientation.getZ();
  ps_1.predicted_position.orientation.w = tf_orientation.getW();

  carma_perception_msgs::msg::PredictedState ps_2;
  ps_2.header.stamp.nanosec = 2000;

  ps_2.predicted_position.position.x = 10;
  ps_2.predicted_position.position.y = 20;
  ps_2.predicted_position.position.z = 0;

  ps_2.predicted_position.orientation.x = tf_orientation.getX();
  ps_2.predicted_position.orientation.y = tf_orientation.getY();
  ps_2.predicted_position.orientation.z = tf_orientation.getZ();
  ps_2.predicted_position.orientation.w = tf_orientation.getW();

  carma_perception_msgs::msg::PredictedState ps_3;
  ps_3.header.stamp.nanosec = 3000;

  ps_3.predicted_position.position.x = 10;
  ps_3.predicted_position.position.y = 30;
  ps_3.predicted_position.position.z = 0;

  ps_3.predicted_position.orientation.x = tf_orientation.getX();
  ps_3.predicted_position.orientation.y = tf_orientation.getY();
  ps_3.predicted_position.orientation.z = tf_orientation.getZ();
  ps_3.predicted_position.orientation.w = tf_orientation.getW();

  rwo_1.object.predictions = {ps_1,ps_2,ps_3};
  rwo_1.object.velocity.twist.linear.x = 5.0;

  rwol.roadway_obstacles = {rwo_1};


  std::vector<carma_perception_msgs::msg::RoadwayObstacle> rw_objs;

  rw_objs.push_back(rwo_1);

  wm->setRoadwayObjects(rw_objs);

  carma_planning_msgs::msg::TrajectoryPlan tp_new = plugin.update_traj_for_object(tp, 10.0);

  for (size_t i = 1; i < tp_new.trajectory_points.size(); i++) {
   std::cout << tp_new.trajectory_points[i].x<< tp_new.trajectory_points[i].y << std::endl;
  }

  EXPECT_EQ(7, tp.trajectory_points.size());

}

TEST(YieldPluginTest, test_update_traj2)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 5.0;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);
  
  trajectory_point_4.x = 15.0;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 20.0;
  trajectory_point_5.y = 1.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 25.0;
  trajectory_point_6.y = 1.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 30.0;
  trajectory_point_7.y = 1.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);
   
  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};


  double initial_pos = 0.0;
  double goal_pos = 35.0;
  double current_speed_ = 10.0;
  double goal_velocity = 5.0;
  double initial_accel = 0.0;
  double goal_accel = 0.0;
  double initial_time = 0.0;
  double tp = 5;

  std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos, 
                                                                                              goal_pos, 
                                                                                              current_speed_, 
                                                                                              goal_velocity, 
                                                                                              initial_accel,
                                                                                              goal_accel, 
                                                                                              initial_time, 
                                                                                              tp);

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> new_trajectory_points;
  new_trajectory_points.push_back(original_tp.trajectory_points[0]);

  std::vector<double> original_traj_downtracks = plugin.get_relative_downtracks(original_tp);

  std::vector<double> new_speeds;
  std::vector<double> distance_travelled;


  for(size_t i = 1; i < original_tp.trajectory_points.size() ; i++ )
  {            
      double traj_target_time = i * tp / original_tp.trajectory_points.size();
      double dt_dist = plugin.polynomial_calc(values, traj_target_time);
      distance_travelled.push_back(dt_dist);
      double dv = plugin.polynomial_calc_d(values, traj_target_time);
      new_speeds.push_back(dv);
      carma_planning_msgs::msg::TrajectoryPlanPoint new_tpp;
      new_tpp.x = original_tp.trajectory_points[i].x;
      new_tpp.y = original_tp.trajectory_points[i].y;
      new_tpp.target_time = rclcpp::Time(new_trajectory_points[0].target_time) + rclcpp::Duration((original_traj_downtracks[i]/dv)*1e9);
      new_trajectory_points.push_back(new_tpp);
  }

  // speeds are decreasing 
  EXPECT_EQ(6, new_speeds.size());
  EXPECT_TRUE(new_speeds[5] <= new_speeds[4]);
  EXPECT_TRUE(new_speeds[3] <= new_speeds[1]);


  EXPECT_EQ(6, distance_travelled.size());
  // did not exceed goal position
  EXPECT_TRUE(distance_travelled[5] < goal_pos );
}


TEST(YieldPluginTest, test_update_traj_stop)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 5.0;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);
  
  trajectory_point_4.x = 15.0;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 20.0;
  trajectory_point_5.y = 1.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 25.0;
  trajectory_point_6.y = 1.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 30.0;
  trajectory_point_7.y = 1.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);
   
  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};



  // When the lead vehicle is stopped

  double initial_pos = 0.0;
  double goal_pos = 15.0;
  double current_speed_ = 10.0;
  double goal_velocity = 0.0;
  double initial_accel = 0.0;
  double goal_accel = 0.0;
  double initial_time = 0.0;
  double tp = 5;

  std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos, 
                                                                                              goal_pos, 
                                                                                              current_speed_, 
                                                                                              goal_velocity, 
                                                                                              initial_accel,
                                                                                              goal_accel, 
                                                                                              initial_time, 
                                                                                              tp);

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> new_trajectory_points;
  new_trajectory_points.push_back(original_tp.trajectory_points[0]);
  std::vector<double> original_traj_downtracks = plugin.get_relative_downtracks(original_tp);

  std::vector<double> new_speeds;
  std::vector<double> distance_travelled;
  for(size_t i = 1; i < original_tp.trajectory_points.size() ; i++ )
  {            
      double traj_target_time = i * tp / original_tp.trajectory_points.size();
      double dt_dist = plugin.polynomial_calc(values, traj_target_time);
      double dv = plugin.polynomial_calc_d(values, traj_target_time);
      carma_planning_msgs::msg::TrajectoryPlanPoint new_tpp;
      
      if (dv >= 1.0)
        {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("yield_plugin"),"target speed is positive");
          if (dv >= current_speed_){
            dv = current_speed_;
          }
          // trajectory point is copied to move all the available information, then its target time is updated
          new_tpp = original_tp.trajectory_points[i];
          new_tpp.target_time = rclcpp::Time(new_trajectory_points[i-1].target_time) + rclcpp::Duration((original_traj_downtracks[i]/dv)*1e9);
          new_trajectory_points.push_back(new_tpp);
        }
        else
        {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("yield_plugin"),"target speed is zero");
          new_tpp = new_trajectory_points[i-1];
          new_tpp.target_time = rclcpp::Time(new_trajectory_points[0].target_time) + rclcpp::Duration(traj_target_time*1e9);
          new_trajectory_points.push_back(new_tpp);
        }
      new_speeds.push_back(dv);
  }

  
  EXPECT_EQ(original_tp.trajectory_points.size(), new_trajectory_points.size());
  // Trajectory point location same as previous point
  EXPECT_EQ(new_trajectory_points[5].x, new_trajectory_points[4].x);
  // Trajectory point time is greater than previous point
  EXPECT_TRUE(rclcpp::Time(new_trajectory_points[5].target_time) > rclcpp::Time(new_trajectory_points[4].target_time));
}

TEST(YieldPluginTest, jmt_traj)
{
  YieldPluginConfig config;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 5.0;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 10.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);
  
  trajectory_point_4.x = 15.0;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  trajectory_point_5.x = 20.0;
  trajectory_point_5.y = 1.0;
  trajectory_point_5.target_time = rclcpp::Time(4,0);

  trajectory_point_6.x = 25.0;
  trajectory_point_6.y = 1.0;
  trajectory_point_6.target_time = rclcpp::Time(5,0);

  trajectory_point_7.x = 30.0;
  trajectory_point_7.y = 1.0;
  trajectory_point_7.target_time = rclcpp::Time(6,0);
   
  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};


  // When the lead vehicle is stopped

  double initial_pos = 0.0;
  double goal_pos = 35.0;
  double initial_velocity = 10.0;
  double goal_velocity = 5.0;
  double initial_accel = 0.0;
  double goal_accel = 0.0;
  double initial_time = 0.0;
  double tp = 5;

  carma_planning_msgs::msg::TrajectoryPlan jmt_traj = plugin.generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, tp);

  EXPECT_EQ(jmt_traj.trajectory_points.size(), original_tp.trajectory_points.size());
  EXPECT_LE(rclcpp::Time(jmt_traj.trajectory_points[2].target_time), rclcpp::Time(original_tp.trajectory_points[2].target_time));

}

TEST(YieldPluginTest, min_digital_gap)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  auto map = carma_wm::test::buildGuidanceTestMap(3,3);

  auto ll_1200 = map->laneletLayer.get(1200);

  double min_gap = 12;

  auto regulatory_element = std::make_shared<lanelet::DigitalMinimumGap>(lanelet::DigitalMinimumGap::buildData(lanelet::utils::getId(), min_gap, {ll_1200}, { },
                                                     { lanelet::Participants::VehicleCar }));
  ll_1200.addRegulatoryElement(regulatory_element);
  map->add(regulatory_element);
  wm->setMap(map);

  YieldPluginConfig config;
  auto nh = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());

  YieldPlugin plugin(nh,wm, config,[&](auto msg) {}, [&](auto msg) {});

  carma_planning_msgs::msg::TrajectoryPlan original_tp;

  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_1;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_2;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_3;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_4;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_5;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_6;
  carma_planning_msgs::msg::TrajectoryPlanPoint trajectory_point_7;

  trajectory_point_1.x = 1.0;
  trajectory_point_1.y = 1.0;
  trajectory_point_1.target_time = rclcpp::Time(0);

  trajectory_point_2.x = 1.5;
  trajectory_point_2.y = 1.0;
  trajectory_point_2.target_time = rclcpp::Time(1,0);

  trajectory_point_3.x = 2.0;
  trajectory_point_3.y = 1.0;
  trajectory_point_3.target_time = rclcpp::Time(2,0);
  
  trajectory_point_4.x = 2.5;
  trajectory_point_4.y = 1.0;
  trajectory_point_4.target_time = rclcpp::Time(3,0);

  original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4};

  double gap = plugin.check_traj_for_digital_min_gap(original_tp);

  EXPECT_EQ(gap, min_gap);
    
}





