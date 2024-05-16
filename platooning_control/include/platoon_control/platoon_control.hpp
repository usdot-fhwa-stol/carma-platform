/*
 * Copyright (C) 2024 LEIDOS.
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_msgs/msg/control_command.hpp>
#include <carma_planning_msgs/msg/platooning_info.hpp>

#include <carma_guidance_plugins/control_plugin.hpp>
#include "platoon_control/platoon_control_config.hpp"
#include "platoon_control/platoon_control_worker.hpp"
#include <pure_pursuit/pure_pursuit.hpp>
#include <basic_autonomy/basic_autonomy.hpp>
#include <gtest/gtest_prod.h>

namespace pure_pursuit = autoware::motion::control::pure_pursuit;
namespace platoon_control
{

  /**
    * \brief This class includes node-level logic for Platooning Control such as its publishers, subscribers, and their callback functions.
    * Platooning Control is used for generating control commands to maintain the gap in platoon as well as generating longitudinal and lateral control commands to follow the trajectory.
	*/
  class PlatoonControlPlugin : public carma_guidance_plugins::ControlPlugin
  {

  public:
    /**
     * \brief PlatoonControlPlugin constructor
     */
    explicit PlatoonControlPlugin(const rclcpp::NodeOptions& options);

    /**
			* \brief generate control signal by calculating speed and steering commands.
			* \param point0 start point of control window
			* \param point_end end point of control wondow
			*/
		autoware_msgs::msg::ControlCommandStamped generate_control_signals(const carma_planning_msgs::msg::TrajectoryPlanPoint& first_trajectory_point, const carma_planning_msgs::msg::TrajectoryPlanPoint& lookahead_point, const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::TwistStamped& current_twist);

    /**
			* \brief Compose twist message from linear and angular velocity commands.
			* \param linear_vel linear velocity in m/s
			* \param angular_vel angular velocity in rad/s
			* \return twist message
			*/
		geometry_msgs::msg::TwistStamped compose_twist_cmd(double linear_vel, double angular_vel);

    motion::motion_common::State convert_state(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::TwistStamped& twist);

      /**
			* \brief find the point correspoding to the lookahead distance
			* \param trajectory_plan trajectory plan
			* \return trajectory point
			*/
		carma_planning_msgs::msg::TrajectoryPlanPoint get_lookahead_trajectory_point(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan, const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::TwistStamped& current_twist);

    double trajectory_speed_ = 0.0;

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
    * \brief callback function for trajectory plan
    * \param msg trajectory plan msg
    */
    void current_trajectory_callback(const carma_planning_msgs::msg::TrajectoryPlan::UniquePtr tp);

    /**
			* \brief Compose control message from speed and steering commands.
			* \param linear_vel linear velocity in m/s
			* \param steering_angle steering angle in rad
			* \return control command
			*/
		autoware_msgs::msg::ControlCommandStamped compose_ctrl_cmd(double linear_vel, double steering_angle);

    /**
    * \brief Returns availability of plugin. Always true
    */
    bool get_availability() override;

    /**
    * \brief Returns version id of plugn.
    */
    std::string get_version_id() override;

    ////
    // Overrides
    ////

    autoware_msgs::msg::ControlCommandStamped generate_command() override;

    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */
    carma_ros2_utils::CallbackReturn on_configure_plugin() override;

    std::shared_ptr<pure_pursuit::PurePursuit> pp_;


  private:

    // Node configuration
    PlatooningControlPluginConfig config_;

    // platoon control worker object
    PlatoonControlWorker pcw_;

    // Variables
    PlatoonLeaderInfo platoon_leader_;
    long prev_input_time_ms_ = 0;				//timestamp of the previous trajectory plan input received
    long consecutive_input_counter_ = 0;	//num inputs seen without a timeout

    /**
    * \brief callback function for platoon info
    * \param msg platoon info msg
    */
    void platoon_info_cb(const carma_planning_msgs::msg::PlatooningInfo::SharedPtr msg);

    /**
    * \brief calculate average speed of a set of trajectory points
    * \param trajectory_points set of trajectory points
    * \return trajectory speed
    */
    double get_trajectory_speed(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_points);


    // Subscribers
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> trajectory_plan_sub_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::PlatooningInfo> platoon_info_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::PlatooningInfo> platoon_info_pub_;

    // Unit Test Accessors
    FRIEND_TEST(PlatoonControlPluginTest, test_platoon_info_cb);
    FRIEND_TEST(PlatoonControlPluginTest, test_get_trajectory_speed);
    FRIEND_TEST(PlatoonControlPluginTest, test_generate_controls);
    FRIEND_TEST(PlatoonControlPluginTest, test_current_trajectory_callback);

  };

} // platoon_control
