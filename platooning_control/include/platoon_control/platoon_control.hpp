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
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_msgs/msg/control_command.hpp>
#include <carma_planning_msgs/msg/platooning_info.hpp>

#include <carma_guidance_plugins/control_plugin.hpp>
#include "platoon_control/platoon_control_config.hpp"
#include "platoon_control/platoon_control_worker.hpp"

namespace platoon_control
{

  /**
    * \brief This class includes logic for Platoon control. It includes publishers and subscribers and their callback functions
	*/
  class PlatoonControlPlugin : public carma_guidance_plugins::ControlPlugin
  {

  private:

    // Node configuration
    PlatooningControlPluginConfig config_;

    // platoon control worker object
    PlatoonControlWorker pcw_;

    // Variables
    PlatoonLeaderInfo platoon_leader_;
    long prev_input_time_ = 0;				//timestamp of the previous trajectory plan input received
    long consecutive_input_counter_ = 0;	//num inputs seen without a timeout

    /**
    * \brief callback function for platoon info
    * \param msg platoon info msg
    */
    void platoonInfo_cb(const carma_planning_msgs::msg::PlatooningInfo::SharedPtr msg);

    /**
    * \brief calculate average speed of a set of trajectory points
    * \param trajectory_points set of trajectory points
    * \return trajectory speed
    */
    double getTrajectorySpeed(std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points);


    // Subscribers
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> trajectory_plan_sub_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::PlatooningInfo> platoon_info_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<geometry_msgs::msg::TwistStamped> twist_pub_;
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::PlatooningInfo> platoon_info_pub_;


  public:
    /**
     * \brief PlatoonControlPlugin constructor
     */
    explicit PlatoonControlPlugin(const rclcpp::NodeOptions &);

    /**
     * \brief Example callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
			* \brief generate control signal by calculating speed and steering commands.
			* \param point0 start point of control window
			* \param point_end end point of control wondow
			*/
		autoware_msgs::msg::ControlCommandStamped generateControlSignals(const carma_planning_msgs::msg::TrajectoryPlanPoint& point0, const carma_planning_msgs::msg::TrajectoryPlanPoint& point_end);

    /**
			* \brief Compose twist message from linear and angular velocity commands.
			* \param linear_vel linear velocity in m/s
			* \param angular_vel angular velocity in rad/s
			* \return twist message
			*/
			geometry_msgs::msg::TwistStamped composeTwistCmd(double linear_vel, double angular_vel);


      /**
			* \brief find the point correspoding to the lookahead distance
			* \param trajectory_plan trajectory plan
			* \return trajectory point
			*/
			carma_planning_msgs::msg::TrajectoryPlanPoint getLookaheadTrajectoryPoint(carma_planning_msgs::msg::TrajectoryPlan trajectory_plan);

      double trajectory_speed_ = 0.0;

      carma_planning_msgs::msg::TrajectoryPlan latest_trajectory_;

    ////
    // Overrides
    ////

    autoware_msgs::msg::ControlCommandStamped generate_command() override;

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
		autoware_msgs::msg::ControlCommandStamped composeCtrlCmd(double linear_vel, double steering_angle);


    bool get_availability() override;

    std::string get_version_id() override;

    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */
    carma_ros2_utils::CallbackReturn on_configure_plugin() override;

  };

} // platoon_control
