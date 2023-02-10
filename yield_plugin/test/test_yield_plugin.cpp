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
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

namespace std_ph = std::placeholders;

class YieldNode :  public carma_ros2_utils::CarmaLifecycleNode
{
public:

    explicit YieldNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
    {}

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &){ return CallbackReturn::SUCCESS; }

    void callback(const carma_v2x_msgs::msg::MobilityResponse::UniquePtr msg)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("yield_plugin"),"Test mob_resp callback..");
    }

    void status_callback(const carma_planning_msgs::msg::LaneChangeStatus::UniquePtr msg)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("yield_plugin"),"Test lc callback..");
    }
    
};

TEST(YieldPlugin, UnitTestYield)
{
    // Note: Comment out the transform lookup before ros test. Since there is no map, it will break the tests   
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
    original_tp.trajectory_id = "test";

    carma_planning_msgs::msg::ManeuverPlan plan;
    carma_planning_msgs::msg::Maneuver maneuver;
    maneuver.type = maneuver.LANE_FOLLOWING;
    maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "InLane_Cruising";
    plan.maneuvers.push_back(maneuver);

    std::shared_ptr<carma_planning_msgs::srv::PlanTrajectory::Request> traj_srv = std::make_shared<carma_planning_msgs::srv::PlanTrajectory::Request>();
    traj_srv->initial_trajectory_plan = original_tp;
    traj_srv->vehicle_state.x_pos_global= 1;
    traj_srv->vehicle_state.y_pos_global = 1;
    traj_srv->vehicle_state.longitudinal_vel = 11;
    traj_srv->maneuver_plan = plan;
       
    double res = 0;
    std::string id = "test";

    auto nh1 = std::make_shared<YieldNode>(rclcpp::NodeOptions());
    auto nh2 = std::make_shared<yield_plugin::YieldPluginNode>(rclcpp::NodeOptions());
    nh2->configure();
    nh2->activate();

    auto yield_client= nh1->create_client<carma_planning_msgs::srv::PlanTrajectory>("yield_plugin/plan_trajectory");
  
    auto mob_resp_sub = nh1->create_subscription<carma_v2x_msgs::msg::MobilityResponse>("outgoing_mobility_response", 5, std::bind(&YieldNode::callback,nh1.get(),std_ph::_1));  
    auto lc_status_sub = nh1->create_subscription<carma_planning_msgs::msg::LaneChangeStatus>("cooperative_lane_change_status", 5,std::bind(&YieldNode::status_callback,nh1.get(),std_ph::_1));
    
    auto mob_req_pub = nh1->create_publisher<carma_v2x_msgs::msg::MobilityRequest>("incoming_mobility_request", 5);
    auto bsm_pub = nh1->create_publisher<carma_v2x_msgs::msg::BSM>("bsm_outbound", 5);
    carma_v2x_msgs::msg::MobilityRequest req1;
    req1.plan_type.type = 0;

    nh1->configure();
    nh1->activate();
    mob_req_pub->publish(req1);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh1->get_node_base_interface());
    executor.add_node(nh2->get_node_base_interface());

    // Spin executor for 2 seconds
    auto end_time = std::chrono::system_clock::now() + std::chrono::seconds(2);
    while(std::chrono::system_clock::now() < end_time){
      executor.spin_once();
    }
    
    EXPECT_EQ(1, mob_req_pub->get_subscription_count()); 
    
    EXPECT_EQ(1, bsm_pub->get_subscription_count());

    EXPECT_EQ(1, mob_resp_sub->get_publisher_count()); 

    EXPECT_EQ(1, lc_status_sub->get_publisher_count()); 
  
    auto traj_resp = yield_client->async_send_request(traj_srv);

    auto future_status = traj_resp.wait_for(std::chrono::milliseconds(100));

    if (future_status == std::future_status::ready)
    {
      RCLCPP_ERROR(rclcpp::get_logger("yield_plugin"),"Service called");
      res = traj_resp.get()->trajectory_plan.trajectory_points.size();
      id =  traj_resp.get()->trajectory_plan.trajectory_id;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("yield_plugin"),"Failed to call service");
      res=0;
    }

    EXPECT_EQ(original_tp.trajectory_id, id);
}

// Run all the tests
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  //Initialize ROS
  rclcpp::init(argc, argv);
  auto ret = rcutils_logging_set_logger_level("yield_plugin", RCUTILS_LOG_SEVERITY_DEBUG);

  bool success = RUN_ALL_TESTS();

  //shutdown ROS
  rclcpp::shutdown();

  return success;
} 

