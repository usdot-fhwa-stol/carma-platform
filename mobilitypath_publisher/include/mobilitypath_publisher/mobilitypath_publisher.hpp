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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/shared_ptr.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <bsm_helper/bsm_helper.h>
#include <std_msgs/msg/string.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_v2x_msgs/msg/mobility_path.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "mobilitypath_publisher/mobilitypath_publisher_config.hpp"

namespace mobilitypath_publisher
{

  /**
   * \class MobilityPathPublication
   * \brief The class responsible for publishing MobilityPath messages based on the latest
   * trajectory plan 
   * 
   */
  class MobilityPathPublication : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> traj_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> bsm_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;

    bool guidance_engaged_ = false;
    
    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityPath> path_pub_;

    // Timer for publishing MobilityPath message
    rclcpp::TimerBase::SharedPtr path_pub_timer_;

    // Node configuration
    Config config_;

    // The most recently published trajectory plan
    carma_planning_msgs::msg::TrajectoryPlan latest_trajectory_;

    // The MobilityPath message generated from the most recently published trajectory plan
    carma_v2x_msgs::msg::MobilityPath latest_mobility_path_;

    // The BSMCoreData from the most recently published BSM message 
    carma_v2x_msgs::msg::BSMCoreData bsm_core_;

    // Map projection string, which defines the lat/lon -> map conversion
    std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

    // Recipient's static ID (Empty string indicates a broadcast message)
    std::string recipient_id = "";

    // Sender's dynamic ID which is its BSM id in hex string
    std::string sender_bsm_id = "FFFF";

    /**
    * \brief Spin callback, which will be called frequently based on the configured spin rate
    */
    bool spin_callback();

    /**
     * \brief Callback for the trajectory plan subscriber, which stores the latest trajectory plan locally
     * and stores its corresponding MobilityPath locally
     * \param msg Latest trajectory plan message
     */
    void trajectory_cb(const carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg);

    /**
     * \brief Callback for the BSM subscriber, which stores the BSM's BSMCoreData locally
     * \param msg Latest BSM message
     */
    void bsm_cb(const carma_v2x_msgs::msg::BSM::UniquePtr msg);

    /**
     * \brief Callback for the Guidance State
     * \param msg Latest GuidanceState message
     */
    void guidance_state_cb(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg);

    /**
     * \brief Generates a MobilityHeader to be used for a published MobilityPath message
     * \param time Time in milliseconds
     * \return A MobilityHeader 
     */
    carma_v2x_msgs::msg::MobilityHeader compose_mobility_header(uint64_t time);

    /**
     * \brief Converts a Trajectory Plan to a (Mobility) Trajectory
     * \param traj_points The TrajectoryPlan to be converted into a (Mobility) Trajectory
     * \return A (Mobility) Trajectory
     */
    carma_v2x_msgs::msg::Trajectory trajectory_plan_to_trajectory(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points) const;
    
    /**
     * \brief Converts Trajectory Plan Point to ECEF (accepts meters and outputs in cm)
     * \param traj_points The Trajectory Plan Point to be converted to ECEF
     * \return The Trajectory Plan Point in ECEF
     */
    carma_v2x_msgs::msg::LocationECEF trajectory_point_to_ECEF(const carma_planning_msgs::msg::TrajectoryPlanPoint& traj_point) const;

  public:
    /**
     * \brief MobilityPathPublication constructor 
     */
    explicit MobilityPathPublication(const rclcpp::NodeOptions &);

    /**
     * \brief Function callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Generates a MobilityPath message from a TrajectoryPlan
     * \param trajectory_plan A TrajectoryPlan
     * \return A MobilityPath message
     */
    carma_v2x_msgs::msg::MobilityPath mobility_path_message_generator(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan);

    /**
    * \brief Callback for map projection string to define lat/lon -> map conversion
    * \brief msg The proj string defining the projection.
    */ 
    void georeference_cb(const std_msgs::msg::String::UniquePtr msg);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  };

} // mobilitypath_publisher
