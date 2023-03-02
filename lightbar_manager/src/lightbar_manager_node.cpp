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


#include "lightbar_manager/lightbar_manager_node.hpp"
#include <algorithm>

namespace lightbar_manager
{

namespace std_ph = std::placeholders;

LightBarManager::LightBarManager(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options)
{
    // Create initial config
    config_ = Config();
    config_.spin_rate_hz = declare_parameter<double>("spin_rate_hz", config_.spin_rate_hz);
    config_.normal_operation = declare_parameter<bool>("normal_operation", config_.normal_operation);
    declare_parameter("lightbar_cda_table");
    declare_parameter("lightbar_ind_table");
    declare_parameter("lightbar_priorities");
    lbm_ = std::make_shared<LightBarManagerWorker>();
}

carma_ros2_utils::CallbackReturn LightBarManager::handle_on_configure(const rclcpp_lifecycle::State &)
{
    // Reset config
    config_ = Config();
    
    RCLCPP_INFO_STREAM(rclcpp::get_logger("lightbar_manager"),"Initalizing lightbar manager node...");
    
    // Load the spin rate param to determine how fast to process messages
    // Default rate 10.0 Hz
    get_parameter<double>("spin_rate_hz", config_.spin_rate_hz);

    request_control_server_= create_service<carma_msgs::srv::RequestIndicatorControl>("request_control", std::bind(&LightBarManager::requestControlCallBack, this, std_ph::_1, std_ph::_2, std_ph::_3));
    release_control_server_= create_service<carma_msgs::srv::ReleaseIndicatorControl>("release_control", std::bind(&LightBarManager::releaseControlCallBack, this, std_ph::_1, std_ph::_2, std_ph::_3));
    set_indicator_server_= create_service<carma_driver_msgs::srv::SetLightBarIndicator>("set_indicator", std::bind(&LightBarManager::setIndicatorCallBack, this, std_ph::_1, std_ph::_2, std_ph::_3));
    indicator_control_publisher_ = create_publisher<carma_msgs::msg::LightBarIndicatorControllers>("indicator_control", 5);
    guidance_state_subscriber_ =  create_subscription<carma_planning_msgs::msg::GuidanceState>("state", 5, std::bind(&LightBarManager::stateChangeCallBack, this, std_ph::_1));
    turn_signal_subscriber_ =  create_subscription<automotive_platform_msgs::msg::TurnSignalCommand>("turn_signal_command", 5, std::bind(&LightBarManager::turnSignalCallback, this, std_ph::_1));
    lightbar_driver_client_ = create_client<carma_driver_msgs::srv::SetLights>("set_lights");

    // Load Conversion table, CDAType to Indicator mapping
 
    rclcpp::Parameter lightbar_cda_table_param = get_parameter("lightbar_cda_table");
    if (lightbar_cda_table_param.get_type() !=  rclcpp::ParameterType::PARAMETER_NOT_SET)
        config_.lightbar_cda_table = lightbar_cda_table_param.as_string_array();

    rclcpp::Parameter lightbar_ind_table_param = get_parameter("lightbar_ind_table");
    if (lightbar_ind_table_param.get_type() !=  rclcpp::ParameterType::PARAMETER_NOT_SET)
        config_.lightbar_ind_table = lightbar_ind_table_param.as_string_array();
    
    if (config_.lightbar_cda_table.size() != config_.lightbar_ind_table.size())
    {
        throw std::invalid_argument("Size of lightbar_cda_table is not same as that of lightbar_ind_table");
    }
    
    lbm_->setIndicatorCDAMap(config_.lightbar_cda_table, config_.lightbar_ind_table);

    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    lbm_->setIndicatorControllers();

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        lbm_->light_status.push_back(OFF);

    rclcpp::Parameter lightbar_priorities_param = get_parameter("lightbar_priorities");
    if (lightbar_priorities_param.get_type() !=  rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
        config_.lightbar_priorities = lightbar_priorities_param.as_string_array();
        lbm_->control_priorities = config_.lightbar_priorities;
    }
        
    // Take control of green light
    get_parameter<bool>("normal_operation", config_.normal_operation);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("lightbar_manager"), "Loaded params: " << config_);

    std::vector<LightBarIndicator> denied_list, greens = {GREEN_SOLID, GREEN_FLASH};

    denied_list = lbm_->requestControl(greens, node_name_);
    if (denied_list.size() != 0)
    {
        if (config_.normal_operation)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("lightbar_manager"),"In fuction " << __FUNCTION__ << ", LightBarManager was not able to take control of all green indicators."
                << ".\n Please check priority list rosparameter, and ensure " << node_name_ << " has the highest priority."
                << ".\n If this is intended, pass false to normal_operation argument. Exiting...");
            throw INVALID_LIGHTBAR_MANAGER_PRIORITY();
        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("lightbar_manager"),"In fuction " << __FUNCTION__ << ", LightBarManager was not able to take control of all green indicators."
                << ".\n Resuming...");   
        }
    }
    return CallbackReturn::SUCCESS;
}
    
carma_ros2_utils::CallbackReturn LightBarManager::handle_on_activate(const rclcpp_lifecycle::State &)
{
    pub_timer_ = create_timer(get_clock(), 
            std::chrono::milliseconds((int)(1 / config_.spin_rate_hz * 1000)),
            std::bind(&LightBarManager::spinCallBack, this));
    return CallbackReturn::SUCCESS;
}

void LightBarManager::turnOffAll()
{
    // All components lose controls and turn the indicators off by giving lightbarmanar the control
    std::vector<LightBarIndicator> all_indicators;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("lightbar_manager"),"LightBarManager was commanded to turn off all indicators!");

    for (std::pair <LightBarIndicator, std::string> element : lbm_->getIndicatorControllers())
        all_indicators.push_back(element.first);
    
    // Reset all controls
    lbm_->setIndicatorControllers();
    
    // Give lightbar manager the control which is guaranteed to succeed
    lbm_->requestControl(all_indicators, node_name_);
    int response_code = 0;
    for (auto indicator : all_indicators)
    {
        response_code = setIndicator(indicator, OFF, node_name_);
        if (response_code != 0)
            RCLCPP_WARN_STREAM(rclcpp::get_logger("lightbar_manager"),"In Function " << __FUNCTION__ << ": LightBarManager was not able to turn off indicator ID:" 
                << indicator << ". Response code: " << response_code);
    }

    // Once forced them off, release controls in case carma is still running so that lightbar_manager does not hog the control
    all_indicators.erase(std::remove(all_indicators.begin(), all_indicators.end(), GREEN_FLASH), all_indicators.end());
    all_indicators.erase(std::remove(all_indicators.begin(), all_indicators.end(), GREEN_SOLID), all_indicators.end());
    lbm_->releaseControl(all_indicators, node_name_);
}

bool LightBarManager::requestControlCallBack(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_msgs::srv::RequestIndicatorControl::Request> req,
                                std::shared_ptr<carma_msgs::srv::RequestIndicatorControl::Response> resp)
{
    std::vector<LightBarIndicator> ind_list, controlled_ind_list;
    std::vector<LightBarCDAType> controlled_cda_type_list;
    
    // Use CDAType if the field is not empty in the request
    if (req->cda_list.size() != 0)
    {   
        for (auto cda_type : req->cda_list) 
        {
            // return false if invalid cda_type number
            if (static_cast<uint8_t>(cda_type.type) >= INDICATOR_COUNT)
                return false; 
            ind_list.push_back(lbm_->getIndicatorFromCDAType(static_cast<LightBarCDAType>(cda_type.type)));
        }
    }
    else
    {
        for (auto indicator : req->ind_list) 
        {
            // return false if invalid indicator number
            if (static_cast<uint8_t>(indicator.indicator) >= INDICATOR_COUNT)
                return false;
            ind_list.push_back(static_cast<LightBarIndicator>(indicator.indicator));
        }
            
    }
    
    controlled_ind_list = lbm_->requestControl(ind_list, req->requester_name);
    
    for (auto indicator : controlled_ind_list)
    {
        try
        {
            // Some indicators do not have mapped CDAType
            controlled_cda_type_list.push_back(lbm_->getCDATypeFromIndicator(indicator));
        }
        catch(INDICATOR_NOT_MAPPED)
        {
            // 4 out of 8 does not have mapping, so skip
            continue;
        }
    }
        
    // Modify the response
    resp->cda_list = lbm_->getMsg(controlled_cda_type_list);
    resp->ind_list = lbm_->getMsg(controlled_ind_list);
    return true;
}

bool LightBarManager::releaseControlCallBack(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<carma_msgs::srv::ReleaseIndicatorControl::Request> req,
                                std::shared_ptr<carma_msgs::srv::ReleaseIndicatorControl::Response> resp)
{
    std::vector<LightBarIndicator> ind_list;

    // Use CDAType if the field is not empty in the request
    if (req->cda_list.size() != 0)
    {   
        for (auto cda_type : req->cda_list) 
        {
            // return false if invalid indicator number
            if (static_cast<uint8_t>(cda_type.type) >= INDICATOR_COUNT)
                return false; 
            ind_list.push_back(lbm_->getIndicatorFromCDAType(static_cast<LightBarCDAType>(cda_type.type)));
        }
    }
    else
    {
        for (auto indicator : req->ind_list)
        {
            // return false if invalid cda_type number
            if (static_cast<uint8_t>(indicator.indicator) >= INDICATOR_COUNT)
                return false; 
            ind_list.push_back(static_cast<LightBarIndicator>(indicator.indicator));
        }
    }
    lbm_->releaseControl(ind_list, req->requester_name);
    return true;
}

bool LightBarManager::spinCallBack()
{
    indicator_control_publisher_->publish(lbm_->getMsg(lbm_->getIndicatorControllers()));
    return true;
}

void LightBarManager::stateChangeCallBack(carma_planning_msgs::msg::GuidanceState::UniquePtr msg_ptr)
{
    // Relay the msg to state machine
    LightBarState prev_lightbar_state = lbm_->getCurrentState();
    lbm_->handleStateChange(*msg_ptr);

    // Change green lights depending on states, no need to check if its current owner, as it will always be for green lights
    LightBarState curr_lightbar_state = lbm_->getCurrentState();
    if (curr_lightbar_state != prev_lightbar_state)
    {
        switch(curr_lightbar_state)
        {
            case DISENGAGED:
                turnOffAll();
                break;
            case ACTIVE:
                setIndicator(GREEN_FLASH, ON, node_name_);
                break;
            case ENGAGED:
                setIndicator(GREEN_SOLID, ON, node_name_);
                break;
            default:
                break;
        }
    }
}

void LightBarManager::turnSignalCallback(automotive_platform_msgs::msg::TurnSignalCommand::UniquePtr msg_ptr)
{
    return processTurnSignal(*msg_ptr);
}

void LightBarManager::processTurnSignal(const automotive_platform_msgs::msg::TurnSignalCommand& msg)
{
    // if not automated
    if (msg.mode != 1)
    {
        return;
    }

    // check if left or right signal should be controlled, or none at all
    std::vector<lightbar_manager::LightBarIndicator> changed_turn_signal = lbm_->handleTurnSignal(msg);


    if (changed_turn_signal.empty())
    {
        return; //no need to do anything if it is same turn signal changed
    }
    
    lightbar_manager::IndicatorStatus indicator_status;
    // check if we should turn off or on given any indicator
    if (msg.turn_signal == automotive_platform_msgs::msg::TurnSignalCommand::NONE)
    {
        indicator_status = lightbar_manager::IndicatorStatus::OFF;
    }
    else 
    {
        indicator_status = lightbar_manager::IndicatorStatus::ON;
        prev_owners_before_turn_ = lbm_->getIndicatorControllers(); // save the owner if new turn is starting

    }
    if (lbm_->requestControl(changed_turn_signal, node_name_).empty())
    {
        int response_code = 0;
        response_code = setIndicator(changed_turn_signal[0], indicator_status, node_name_);
        if (response_code != 0)
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("lightbar_manager"),"In Function " << __FUNCTION__ << ": LightBarManager was not able to set light of indicator ID:" 
                << changed_turn_signal[0] << ". Response code: " << response_code);
    }
    else
    {
        std::string turn_string = msg.turn_signal == automotive_platform_msgs::msg::TurnSignalCommand::LEFT ? "left" : "right";
        RCLCPP_WARN_STREAM(rclcpp::get_logger("lightbar_manager"),"Lightbar was not able to take control of lightbar to indicate " << turn_string << "turn!");
        return;
    }

    // release control if it is turning off and put back the previous owners
    if (indicator_status == lightbar_manager::IndicatorStatus::OFF)
    {
        lbm_->releaseControl(changed_turn_signal, node_name_);
        // we need to force set the <indicator, owner> mapping
        // as series of requestControl may not be able to exactly achieve previous combination
        lbm_->setIndicatorControllers(prev_owners_before_turn_);
        // reset as the turn is finished
        prev_owners_before_turn_.clear();
    }
}

std::shared_ptr<LightBarManagerWorker> LightBarManager::getWorker()
{
    return lbm_;
}

int LightBarManager::setIndicator(LightBarIndicator ind, IndicatorStatus ind_status, const std::string& requester_name)
{
    // Handle the msg/service translation
    int response_code = 0;
    if (static_cast<uint8_t>(ind) >= INDICATOR_COUNT || static_cast<uint8_t>(ind_status) > 1)
    {
        // Invalid indicator ID or ind_status
        response_code = 1;
        return response_code;
    }

    // Check if the requester has control of this light
    std::string current_controller = lbm_->getIndicatorControllers()[ind];
    if (requester_name == "" || current_controller != requester_name) 
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("lightbar_manager"),requester_name << " failed to set the LightBarIndicator ID" << ind 
            << " as this was already controlled by " << current_controller);
        response_code = 1;
        return response_code;
    }

    std::vector<IndicatorStatus> light_status_proposed = lbm_->setIndicator(ind, ind_status, requester_name);
    carma_driver_msgs::msg::LightBarStatus msg = lbm_->getLightBarStatusMsg(light_status_proposed);
    auto srv = std::make_shared<carma_driver_msgs::srv::SetLights::Request>();
    srv->set_state = msg;
    
    auto resp = lightbar_driver_client_->async_send_request(srv);

    auto future_status = resp.wait_for(std::chrono::milliseconds(100));
    // Try to send the request
    if (future_status == std::future_status::ready)
    {
        // if successful, update the local copy.
        lbm_->light_status = light_status_proposed;
        response_code =  0;
    }
    else
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("lightbar_manager"),"In function: " << __FUNCTION__ << ": Failed to set lights. ROSservice call to the driver failed. ");
        response_code =  2;
    }
    return response_code;

}

bool LightBarManager::setIndicatorCallBack(const std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<carma_driver_msgs::srv::SetLightBarIndicator::Request> req,
                        std::shared_ptr<carma_driver_msgs::srv::SetLightBarIndicator::Response> resp)
{
    LightBarIndicator indicator;
    int response_code = 0;
    if (req->cda_type.type != NULL)
        indicator = lbm_->getIndicatorFromCDAType(static_cast<LightBarCDAType>(req->cda_type.type));
    else
        indicator = static_cast<LightBarIndicator>(req->indicator.indicator);

    response_code = setIndicator(indicator, static_cast<IndicatorStatus>(req->state), req->requester_name);
    resp->status_code = response_code;
    if (response_code != 0)
        return false;

    return true;
}

} // namespace lightbar_manager


