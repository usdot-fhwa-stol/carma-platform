/*
 * Copyright (C) 2023 LEIDOS.
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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <vector>
#include <map>

#include <carma_msgs/msg/light_bar_cda_type.hpp>
#include <carma_msgs/msg/light_bar_indicator.hpp>
#include <carma_msgs/msg/light_bar_indicator_controllers.hpp>
#include <carma_driver_msgs/msg/light_bar_status.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>

#include <carma_msgs/srv/request_indicator_control.hpp>
#include <carma_msgs/srv/release_indicator_control.hpp>
#include <carma_driver_msgs/srv/set_light_bar_indicator.hpp>
#include <carma_driver_msgs/srv/set_lights.hpp>

#include "lightbar_manager/lightbar_manager_worker.hpp"

namespace lightbar_manager
{

/**
 * \brief Config struct
 */
struct Config
{
    double spin_rate_hz = 10.0;
    bool normal_operation = true;   // if false, other plugins are able to take control over the lightbar status
    std::vector<std::string> lightbar_priorities = {}; // Tsesters are for unit testing. Keep it there.
    std::vector<std::string> lightbar_cda_table = {}; // Keys for lightbar_cda_to_ind_table, 1-to-1 with lightbar_ind_table
    std::vector<std::string> lightbar_ind_table = {}; // Values for lightbar_cda_to_ind_table, 1-to-1 with lightbar_cda_table

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "LightBarManager::Config { " << std::endl
           << "spin_rate_hz: " << c.spin_rate_hz << std::endl
           << "lightbar_priorities.size(): " << c.lightbar_priorities.size() << std::endl
           << "lightbar_cda_table.size(): " << c.lightbar_cda_table.size() << std::endl
           << "lightbar_ind_table.size(): " << c.lightbar_ind_table.size() << std::endl
           << "}" << std::endl;
      return output;
    }

};

class LightBarManager : public carma_ros2_utils::CarmaLifecycleNode
{
    public:

        /**
         * \class LightBarManager
         * \brief The class responsible for managing light bar status based on the guidance status
         * 
         */
        explicit LightBarManager(const rclcpp::NodeOptions &);
        
        /*!
        * \brief Get ptr to lightbar_manager_worker (for ease of unit testing)
        * \return LightBarManagerWorker
        */
        std::shared_ptr<LightBarManagerWorker> getWorker();
 
        /*!
        * \brief Miscellaneous function that forces the state to disengaged and turn off all indicators.
        * Used in special demo cases as well as when carma is disengaged
        */
        void turnOffAll();

        /*!
        * \brief Try to turn the given indicator ON or OFF (comm with hardware) upon the given component's request
        * \return Returns the status code whether if the request was successful or not
        */
        int setIndicator(LightBarIndicator ind, IndicatorStatus ind_status, const std::string& requester_name);

        /*!
        * \brief Callback function for turning signal
        * \return 
        */
        void turnSignalCallback(automotive_platform_msgs::msg::TurnSignalCommand::UniquePtr msg_ptr);

        ////
        // Overrides
        ////
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

    private:
        
        Config config_;

        /*!
        * \brief Turn signal callback function helper
        * \return 
        */
        void processTurnSignal(const automotive_platform_msgs::msg::TurnSignalCommand& msg);

        // Node Data
        std::string node_name_ = "lightbar_manager";
        double spin_rate_ = 10.0;
        std::map<lightbar_manager::LightBarIndicator, std::string> prev_owners_before_turn_;

        // spin callback function
        bool spinCallBack();

        // Message/service callbacks
        bool requestControlCallBack(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_msgs::srv::RequestIndicatorControl::Request> req,
                                std::shared_ptr<carma_msgs::srv::RequestIndicatorControl::Response> resp);
        bool releaseControlCallBack(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_msgs::srv::ReleaseIndicatorControl::Request> req,
                                std::shared_ptr<carma_msgs::srv::ReleaseIndicatorControl::Response> resp);
        bool setIndicatorCallBack(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_driver_msgs::srv::SetLightBarIndicator::Request> req,
                                std::shared_ptr<carma_driver_msgs::srv::SetLightBarIndicator::Response> resp);
        void stateChangeCallBack(carma_planning_msgs::msg::GuidanceState::UniquePtr msg_ptr);

        // Service servers/clients
        carma_ros2_utils::ServicePtr<carma_msgs::srv::RequestIndicatorControl> request_control_server_;
        carma_ros2_utils::ServicePtr<carma_msgs::srv::ReleaseIndicatorControl> release_control_server_;
        carma_ros2_utils::ServicePtr<carma_driver_msgs::srv::SetLightBarIndicator> set_indicator_server_;
        carma_ros2_utils::ClientPtr<carma_driver_msgs::srv::SetLights> lightbar_driver_client_;

        // Publishers
        carma_ros2_utils::PubPtr<carma_msgs::msg::LightBarIndicatorControllers> indicator_control_publisher_;

        // Subscribers
        carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_subscriber_;
        carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::TurnSignalCommand> turn_signal_subscriber_;

        // LightBarManager Worker
        std::shared_ptr<LightBarManagerWorker> lbm_;

        rclcpp::TimerBase::SharedPtr pub_timer_;

        FRIEND_TEST(LightBarManagerNodeTest, testSetIndicator);
        FRIEND_TEST(LightBarManagerNodeTest, testTurnOffAll);
        FRIEND_TEST(LightBarManagerNodeTest, testTurnSignalCallback);

}; //class LightBarManagerNode
} // namespace lightbar_manager

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(lightbar_manager::LightBarManager)