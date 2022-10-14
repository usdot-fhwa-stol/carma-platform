/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include <functional>
// ROS
#include <rclcpp/rclcpp.hpp>
// msgs
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <autoware_msgs/msg/lane.hpp>
#include "pure_pursuit_wrapper_config.hpp"
#include <algorithm>
#include <trajectory_utils/trajectory_utils.hpp>
#include <carma_guidance_plugins/control_plugin.hpp>
#include <pure_pursuit/pure_pursuit.hpp>
#include <gtest/gtest_prod.h>
#include <basic_autonomy_ros2/basic_autonomy.hpp>

namespace pure_pursuit_wrapper {

using WaypointPub = std::function<void(autoware_msgs::msg::Lane)>;
using PluginDiscoveryPub = std::function<void(carma_planning_msgs::msg::Plugin)>;
namespace pure_pursuit = autoware::motion::control::pure_pursuit;

class PurePursuitWrapperNode : public carma_guidance_plugins::ControlPlugin 
{
    public:
    /**
     * @brief Constructor
     */
    explicit PurePursuitWrapperNode(const rclcpp::NodeOptions& options);
    
    autoware_msgs::msg::ControlCommandStamped generate_command() override;

    /**
     * \brief Example callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */ 
    carma_ros2_utils::CallbackReturn on_configure_plugin() override;

    bool get_availability() override;

    std::string get_version_id() override;

    /**
     * \brief Drops any points that sequentially have same target_time and return new trajectory_points in order to avoid divide by zero situation
     * \param traj_points Velocity profile to shift. The first point should be the current vehicle speed
     * 
     * NOTE: This function assumes the target_time will not go backwards. In other words, it only removes "sequential" points that have same target_time
     * \return A new trajectory without any repeated time_stamps
     */ 
    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> remove_repeated_timestamps(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points);

    //CONVERSIONS

    motion::motion_common::State convert_state(geometry_msgs::msg::PoseStamped pose, geometry_msgs::msg::TwistStamped twist);
    autoware_msgs::msg::ControlCommandStamped convert_cmd(motion::motion_common::Command cmd);

    private:

    PurePursuitWrapperConfig config_;

    std::shared_ptr<pure_pursuit::PurePursuit> pp_;

    std::shared_ptr<pure_pursuit::PurePursuit> get_pure_pursuit_worker()
    {
        return pp_;
    }

    // Unit Test Accessors
    FRIEND_TEST(PurePursuitTest, sanity_check);

};

}  // namespace pure_pursuit_wrapper