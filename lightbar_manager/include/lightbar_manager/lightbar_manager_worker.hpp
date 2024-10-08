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
#include <gtest/gtest.h>
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

#include "lightbar_manager/lightbar_manager_sm.hpp"
#define INDICATOR_COUNT 8
#define CDA_MSG_TYPE_COUNT 4

namespace lightbar_manager
{

// forward declaration
class LightBarManagerStateMachine;

class LightBarManagerWorker
{
    public:

        /*!
        * \brief Default constructor for LightBarManager
        */
        LightBarManagerWorker();

        /*!
        * \brief Get current state of the LightBarStateMachine
        * \return current state of the lightbar state machine
        */
        LightBarState getCurrentState();

        /*!
        * \brief Transition to the next state of the LightBarStateMachine
        * Not used at the moment as state machine fully handles everything
        */
        void next(const LightBarEvent& event);

        /*!
        * \brief This function relays the state change msg to the state maching. 
        * It triggers the transitioning to the next state in LightBarStateMachine based on the guidance state change. 
        */
        void handleStateChange(const carma_planning_msgs::msg::GuidanceState& msg);

        /*!
        * \brief This function checks if the turn signal should be changed on the lightbar
        * \return size one vector of turn signal, empty if no change is required
        */
        std::vector<lightbar_manager::LightBarIndicator> handleTurnSignal(const automotive_platform_msgs::msg::TurnSignalCommand& msg);

        /*!
        * \brief Releases the specified owner plugin or component's control of the given indicator list.
        * This function handles successful transitioning of next ownership when that happens.
        */
        void releaseControl(std::vector<LightBarIndicator> ind_list, std::string owner_name);

        /*!
        * \brief Requests the control of the given list of indicators to the requester.
        * This function handles successful transitioning of next ownership and illogical requests such as mutually exclusive indicators
        * \return Returns the list of the indicators that were denied for the requester
        */
        std::vector<LightBarIndicator> requestControl(std::vector<LightBarIndicator> ind_list, std::string requester_name);
        
        /*!
        * \brief Try to turn the given indicator ON or OFF (locally) upon the given component's request
        * \return Returns the changed vector of indicator status to be set by the driver client
        */
        std::vector<IndicatorStatus> setIndicator(LightBarIndicator ind, IndicatorStatus ind_status, std::string requester_name);

        /*!
        * \brief Helper functions that translates an indicator to its corresponding CDA msg type  
        * The mapping between indicators and CDA types are configured through ROS params.
        * \return Returns the indicator corresponding to the msg type.
        */
        LightBarIndicator getIndicatorFromCDAType(LightBarCDAType cda_type);

        /*!
        * \brief Helper functions that translates a CDA msg type to its corresponding indicator  
        * The mapping between indicators and CDA types are configured through ROS params.
        * \return Returns the CDA msg type that the indicator represents
        */
        LightBarCDAType getCDATypeFromIndicator(LightBarIndicator indicator);

        /*!
        * \brief Helper function that initializes CDAType to Indicator Mapping (updates internal copy)
        * \return return the mapping for debug purposes. 
        */
        std::map<LightBarCDAType, LightBarIndicator> setIndicatorCDAMap(const std::vector<std::string>& lightbar_cda_table, const std::vector<std::string>& lightbar_ind_table);

        /*!
        * \brief Helper function that gets all current owners of the indicator.
        * \return return the mapping of indicators to their owners
        */
        std::map<LightBarIndicator,std::string> getIndicatorControllers();
       
        /*!
        * \brief Helper function that initializes supporting Indicators and their owner mapping as empty strings
        */
        void setIndicatorControllers();

        /*!
        * \brief Helper function that resets Indicators and their owner mapping
        */
        void setIndicatorControllers(std::map<LightBarIndicator, std::string> ind_ctrl_map);

        /*!
        * \brief Helper function that checks if the first input component has higher priority than the second.
        * \return true if the requester has higher priority than the controller
        */
        bool hasHigherPriority(std::string requester, std::string controller);

        /*!
        * \brief Helper function that handles control lost/gained event of a component. 
        * This function registers/removes controller's name to all mutually inexclusive indicators.
        * e.g. If registering a component as controller of YELLOW_ARROW_OUT, it will be registered for YELLOW_ARROW_LEFT/RIGHT/FLASH too. 
        */
        void handleControlChange(LightBarIndicator indicator, std::string controller, IndicatorControlEvent event);

        /*!
        * \brief Helper function that translates IndicatorStatus vector into LightBarStatus.msg, a lightbar driver compatible msg.
        * \return light bar status
        */
        carma_driver_msgs::msg::LightBarStatus getLightBarStatusMsg(std::vector<IndicatorStatus> indicators);

        /*!
        * \brief Helper function that translates LightBarIndicator vector into LightBarIndicator.msg vector.
        * \return LightBarIndicator.msg vector
        */
        std::vector<carma_msgs::msg::LightBarIndicator> getMsg(std::vector<LightBarIndicator> indicators);

        /*!
        * \brief Helper function that translates LightBarIndicator vector into LightBarIndicator.msg vector.
        * \return LightBarCDAType.msg vector
        */
        std::vector<carma_msgs::msg::LightBarCDAType> getMsg(std::vector<LightBarCDAType> cda_types);

        /*!
        * \brief Helper function that translates mapping of indicators to their owners into Msg
        * \return LightBarIndicatorControlllers.msg
        */
        carma_msgs::msg::LightBarIndicatorControllers getMsg(std::map<LightBarIndicator, std::string> ind_ctrl_map);

        // Priorities of components/plugins read from ROSParamter
        std::vector<std::string> control_priorities;

        // LightBarStatus local copy in LightBarIndicator representation
        std::vector<IndicatorStatus> light_status;

    private:
        

        // LightBarManager state machine
        LightBarManagerStateMachine lbsm_;

        // Indicators and their owners' mapping
        std::map<LightBarIndicator, std::string> ind_ctrl_map_;

        // Indicators and their corresponding CDA msg type mapping
        std::map<LightBarCDAType, LightBarIndicator> cda_ind_map_;

        // Current turn signal
        uint8_t current_turn_signal_ = automotive_platform_msgs::msg::TurnSignalCommand::NONE;

        // Helper maps that convert string into enum representations when reading from ROSParameter
        std::map<std::string, LightBarCDAType> cda_type_dict_ = {
            {"TypeA",TYPE_A},
            {"TypeB",TYPE_B},
            {"TypeC",TYPE_C},
            {"TypeD",TYPE_D}};
            
        std::map<std::string, LightBarIndicator> ind_dict = {
            {"GREEN_SOLID", GREEN_SOLID},
            {"GREEN_FLASH", GREEN_FLASH},
            {"YELLOW_SIDES", YELLOW_SIDES},
            {"YELLOW_DIM", YELLOW_DIM},
            {"YELLOW_FLASH", YELLOW_FLASH},
            {"YELLOW_ARROW_LEFT", YELLOW_ARROW_LEFT},
            {"YELLOW_ARROW_RIGHT", YELLOW_ARROW_RIGHT},
            {"YELLOW_ARROW_OUT", YELLOW_ARROW_OUT}};
        
        FRIEND_TEST(LightBarManagerNodeTest, testSetIndicator);
        FRIEND_TEST(LightBarManagerNodeTest, testTurnOffAll);
        FRIEND_TEST(LightBarManagerNodeTest, testTurnSignalCallback);
}; //class LightBarManagerWorker
} // namespace lightbar_manager
