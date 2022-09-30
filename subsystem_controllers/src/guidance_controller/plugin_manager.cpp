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

#include <boost/algorithm/string.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "subsystem_controllers/guidance_controller/plugin_manager.h"

using std_msec = std::chrono::milliseconds;

namespace subsystem_controllers
{

    PluginManager::PluginManager(const std::vector<std::string>& required_plugins,
                          const std::vector<std::string>& auto_activated_plugins, 
                          const std::vector<std::string>& ros2_initial_plugins,
                          std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> plugin_lifecycle_mgr,
                          GetParentNodeStateFunc get_parent_state_func,
                          ServiceNamesAndTypesFunc get_service_names_and_types_func,
                          std::chrono::nanoseconds service_timeout, std::chrono::nanoseconds call_timeout)
        : required_plugins_(required_plugins.begin(), required_plugins.end()),
            auto_activated_plugins_(auto_activated_plugins.begin(), auto_activated_plugins.end()),
            ros2_initial_plugins_(ros2_initial_plugins.begin(), ros2_initial_plugins.end()),
            plugin_lifecycle_mgr_(plugin_lifecycle_mgr), get_parent_state_func_(get_parent_state_func),
            get_service_names_and_types_func_(get_service_names_and_types_func),
            service_timeout_(service_timeout), call_timeout_(call_timeout)
    {
        if (!plugin_lifecycle_mgr)
            throw std::invalid_argument("Input plugin_lifecycle_mgr to PluginManager constructor cannot be null");
        
        // For all required and auto activated plugins add unknown entries but with 
        // user_requested_activation set to true.
        // This will be used later to determine how to transition the plugin specified by that entry
        
        for (const auto& p : required_plugins_) {
            bool is_ros1 = ros2_initial_plugins_.find(p) == ros2_initial_plugins_.end();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("subsystem_controllers"), "Added: " << p << ", as is_ros1:" << is_ros1);
            
            Entry e(false, false, p, carma_planning_msgs::msg::Plugin::UNKNOWN, "", true, is_ros1);
            em_.update_entry(e);
            if (!is_ros1)
                plugin_lifecycle_mgr_->add_managed_node(p);
        }

        for (const auto& p : auto_activated_plugins_) {
            bool is_ros1 = ros2_initial_plugins_.find(p) == ros2_initial_plugins_.end();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("subsystem_controllers"), "Added: " << p << ", as is_ros1:" << is_ros1);

            Entry e(false, false, p, carma_planning_msgs::msg::Plugin::UNKNOWN, "", true, is_ros1);
            em_.update_entry(e);
            if (!is_ros1)
                plugin_lifecycle_mgr_->add_managed_node(p);
        }
    }

    bool PluginManager::is_ros2_lifecycle_node(const std::string& node)
    {
        // Determine if this plugin is a ROS1 or ROS2 plugin
        std::vector<std::string> name_parts;
        boost::split(name_parts, node, boost::is_any_of("/"));

        if (name_parts.empty()) {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("subsystem_controllers"), "Invalid name for plugin: " << node << " Plugin may not function in system.");
          return false;
        }

        std::string base_name = name_parts.back();
        name_parts.pop_back();
        std::string namespace_joined = boost::algorithm::join(name_parts, "/");

        std::map<std::string, std::vector<std::string>> services_and_types;
        try {
            services_and_types = get_service_names_and_types_func_(base_name, namespace_joined);
        } 
        catch (const std::runtime_error& e) {
            return false; // Seems this method can throw an exception if not a ros2 node
        }
        


        // Next we check if both services are available with the correct type
        // Short variable names used here to make conditional more readable
        const std::string cs_srv = node + "/change_state";
        const std::string gs_srv = node + "/get_state";

        if (services_and_types.find(cs_srv) != services_and_types.end() 
          && services_and_types.find(gs_srv) != services_and_types.end()
          && std::find(services_and_types.at(cs_srv).begin(), services_and_types.at(cs_srv).end(), "lifecycle_msgs/srv/ChangeState") != services_and_types.at(cs_srv).end()
          && std::find(services_and_types.at(gs_srv).begin(), services_and_types.at(gs_srv).end(), "lifecycle_msgs/srv/GetState") != services_and_types.at(gs_srv).end())
        {

          return true;

        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("subsystem_controllers"), "Detected non-ros2 lifecycle plugin " << node);
        return false;
    }

    void PluginManager::add_plugin(const Entry& plugin)
    {

        // If this is a ros1 node we will still track it but we will not attempt to manage its state machine
        if (!is_ros2_lifecycle_node(plugin.name_)) {

            Entry ros1_plugin = plugin;

            ros1_plugin.is_ros1_ = true;
          
            em_.update_entry(ros1_plugin);

          return;
        }

        plugin_lifecycle_mgr_->add_managed_node(plugin.name_); 

        em_.update_entry(plugin);

        Entry deactivated_entry = plugin;

        auto parent_node_state = get_parent_state_func_();

        // If this node is not in the active or inactive states then move the plugin to unconfigured
        if (parent_node_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
            && parent_node_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
        {

            deactivated_entry.active_ = false;

            // Move plugin to match this nodes state
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(parent_node_state, plugin.name_, service_timeout_, call_timeout_);

            if(result_state != parent_node_state) 
            {                

                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Newly discovered required plugin " + plugin.name_ + " could not be brought to controller node state.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to transition newly discovered non-required plugin: " 
                    << plugin.name_ << " Marking as deactivated and unavailable!"); 
            
                deactivated_entry.available_ = false;

            }

            em_.update_entry(deactivated_entry);
            return;
        }
        
        
        // If the node is active or inactive then, 
        // when adding a plugin, it should be brought to the inactive state
        // We do not need transition it beyond inactive in this function as that will be managed by the plugin activation process via UI or parameters
        auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name_, service_timeout_, call_timeout_);

        if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
        {
            // If this plugin was required then trigger exception
            if (required_plugins_.find(plugin.name_) != required_plugins_.end())
            {
                throw std::runtime_error("Newly discovered required plugin " + plugin.name_ + " could not be brought to inactive state.");
            }

            // If this plugin was not required log an error and mark it is unavailable and deactivated               
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to configure newly discovered non-required plugin: " 
                << plugin.name_ << " Marking as deactivated and unavailable!"); 

            deactivated_entry.available_ = false;
        }

        deactivated_entry.active_ = false;
        em_.update_entry(deactivated_entry);

    }

    bool PluginManager::configure()
    {
        bool full_success = true;
        // Bring all known plugins to the inactive state
        for (auto plugin : em_.get_entries())
        {
            if (plugin.is_ros1_) // We do not manage lifecycle of ros1 nodes
                continue;

            
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name_, service_timeout_, call_timeout_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be configured.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to configure newly discovered non-required plugin: " 
                    << plugin.name_ << " Marking as deactivated and unavailable!"); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active_ = false;
                deactivated_entry.available_ = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);

                full_success = false;
            }

        }

        return full_success;

    }
    
    bool PluginManager::activate()
    {
        bool full_success = true;
        // Bring all required or auto activated plugins to the active state
        for (auto plugin : em_.get_entries())
        {
            if (plugin.is_ros1_) // We do not manage lifecycle of ros1 nodes
                continue;

            // If this is not a plugin slated for activation then continue and leave up to user to activate manually later
            if (!plugin.user_requested_activation_)
                continue;

            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, plugin.name_, service_timeout_, call_timeout_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be activated.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to activate newly discovered non-required plugin: " 
                    << plugin.name_ << " Marking as deactivated and unavailable!"); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active_ = false;
                deactivated_entry.available_ = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);

                full_success = false;
            }

            // If this was an auto activated plugin and not required then we only activate is once
            if (auto_activated_plugins_.find(plugin.name_) != auto_activated_plugins_.end())
            {
                plugin.user_requested_activation_ = false;
            }

            plugin.active_ = true; // Mark plugin as active
            
            em_.update_entry(plugin);

        }

        return full_success;
    }
    
    bool PluginManager::deactivate()
    {
        bool full_success = true;
        // Bring all required or auto activated plugins to the active state
        for (auto plugin : em_.get_entries())
        {

            if (plugin.is_ros1_) // We do not manage lifecycle of ros1 nodes
                continue;
            
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name_, service_timeout_, call_timeout_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be deactivated.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to deactivate non-required plugin: " 
                    << plugin.name_ << " Marking as deactivated and unavailable!"); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active_ = false;
                deactivated_entry.available_ = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);

                full_success = false;
            }

        }

        return full_success;
    }
    
    bool PluginManager::cleanup()
    {
        bool full_success = true;
        // Bring all required or auto activated plugins to the unconfigured state
        for (auto plugin : em_.get_entries())
        {

            if (plugin.is_ros1_) // We do not manage lifecycle of ros1 nodes
                continue;
            
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, plugin.name_, service_timeout_, call_timeout_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be cleaned up.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("subsystem_controllers"), "Failed to cleanup non-required plugin: " 
                    << plugin.name_ << " Marking as deactivated and unavailable!"); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active_ = false;
                deactivated_entry.available_ = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);
                
                full_success = false;
            }

        }

        return full_success;
    }
    
    bool PluginManager::shutdown()
    {
        std::vector<std::string> all_names = em_.get_entry_names();
        std::vector<std::string> ros2_names;
        ros2_names.reserve(all_names.size());

        std::copy_if(all_names.begin(), all_names.end(),
                 std::back_inserter(ros2_names),
                 [this](const std::string& n) { return !em_.get_entry_by_name(n)->is_ros1_; });

        return plugin_lifecycle_mgr_->shutdown(service_timeout_, call_timeout_, false, ros2_names).empty();
    }

    void PluginManager::get_registered_plugins(SrvHeader, carma_planning_msgs::srv::PluginList::Request::SharedPtr, carma_planning_msgs::srv::PluginList::Response::SharedPtr res)
    {
        // convert to plugin list
        for(const auto& plugin :  em_.get_entries())
        {
            carma_planning_msgs::msg::Plugin msg;
            msg.activated = plugin.active_;
            msg.available = plugin.available_;
            msg.name = plugin.name_;
            msg.type = plugin.type_;
            msg.capability = plugin.capability_;
            res->plugins.push_back(msg);
        }
    }

    void PluginManager::get_active_plugins(SrvHeader, carma_planning_msgs::srv::PluginList::Request::SharedPtr, carma_planning_msgs::srv::PluginList::Response::SharedPtr res)
    {
        // convert to plugin list
        for(const auto& plugin : em_.get_entries())
        {
            if(plugin.active_)
            {
                carma_planning_msgs::msg::Plugin msg;
                msg.activated = plugin.active_;
                msg.available = plugin.available_;
                msg.name = plugin.name_;
                msg.type = plugin.type_;
                msg.capability = plugin.capability_;
                res->plugins.push_back(msg);
            }
        }
    }

    void PluginManager::activate_plugin(SrvHeader, carma_planning_msgs::srv::PluginActivation::Request::SharedPtr req, carma_planning_msgs::srv::PluginActivation::Response::SharedPtr res)
    {
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(req->plugin_name);
        
        if(!requested_plugin) // If not a known plugin then obviously not activated. Though really it would be better to have an indication of name failure in the message
        {
            res->newstate = false;
            return;
        }

        if (requested_plugin->is_ros1_)
        {
            res->newstate = true;
            return;
        }

        bool activated = false;
        if (req->activated)
        {
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, requested_plugin->name_, service_timeout_, call_timeout_);

            activated = (result_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        } else {
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, requested_plugin->name_, service_timeout_, call_timeout_);

            activated = (result_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        }

        Entry updated_entry(requested_plugin->available_, activated, requested_plugin->name_, requested_plugin->type_, requested_plugin->capability_, true, requested_plugin->is_ros1_); // Mark as user activated
        em_.update_entry(updated_entry);

        res->newstate = activated;
    }

    void PluginManager::update_plugin_status(carma_planning_msgs::msg::Plugin::UniquePtr msg)
    {
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(msg->name);

        if (!requested_plugin) // This is a new plugin so we need to add it
        {
            Entry plugin(msg->available, msg->activated, msg->name, msg->type, msg->capability, false, false); //is_ros1 flag is updated appropriately in add_plugin
            add_plugin(plugin);
            return;
        }

        Entry plugin(msg->available, msg->activated, msg->name, msg->type, msg->capability, requested_plugin->user_requested_activation_, requested_plugin->is_ros1_);
        
        em_.update_entry(plugin);
    }

    bool PluginManager::matching_capability(const std::vector<std::string>& base_capability_levels, const std::vector<std::string>& compared_capability_levels)
    {
        for (size_t i=0; i < base_capability_levels.size() &&  i < compared_capability_levels.size(); i++)
        {
            if (compared_capability_levels[i].compare(base_capability_levels[i]) != 0)
                return false;
        }

        return true;
    }

    void PluginManager::get_control_plugins_by_capability(SrvHeader, carma_planning_msgs::srv::GetPluginApi::Request::SharedPtr req, carma_planning_msgs::srv::GetPluginApi::Response::SharedPtr res)
    {
        std::vector<std::string> req_capability_levels;
        boost::split(req_capability_levels, req->capability, boost::is_any_of("/"));

        for(const auto& plugin : em_.get_entries())
        {

            std::vector<std::string> plugin_capability_levels; 
            boost::split(plugin_capability_levels, plugin.capability_, boost::is_any_of("/"));

            if(plugin.type_ == carma_planning_msgs::msg::Plugin::CONTROL && 
                (req->capability.size() == 0 || (matching_capability(plugin_capability_levels, req_capability_levels) && plugin.active_ && plugin.available_)))
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("guidance_controller"), "discovered control plugin: " << plugin.name_);
                res->plan_service.push_back(plugin.name_ + control_trajectory_suffix_);
            }
            else
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("guidance_controller"), "not valid control plugin: " << plugin.name_);
            } 
        }
    }

    void PluginManager::get_tactical_plugins_by_capability(SrvHeader, carma_planning_msgs::srv::GetPluginApi::Request::SharedPtr req, carma_planning_msgs::srv::GetPluginApi::Response::SharedPtr res)
    {
        std::vector<std::string> req_capability_levels;
        boost::split(req_capability_levels, req->capability, boost::is_any_of("/"));

        for(const auto& plugin : em_.get_entries())
        {

            std::vector<std::string> plugin_capability_levels;
            boost::split(plugin_capability_levels, plugin.capability_, boost::is_any_of("/"));

            if(plugin.type_ == carma_planning_msgs::msg::Plugin::TACTICAL && 
                (req->capability.size() == 0 || (matching_capability(plugin_capability_levels, req_capability_levels) && plugin.active_ && plugin.available_)))
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("guidance_controller"), "discovered tactical plugin: " << plugin.name_);
                res->plan_service.push_back(plugin.name_ + plan_trajectory_suffix_);
            }
            else
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("guidance_controller"), "not valid tactical plugin: " << plugin.name_);
            } 
        }
    }

    void PluginManager::get_strategic_plugins_by_capability(SrvHeader, carma_planning_msgs::srv::GetPluginApi::Request::SharedPtr req, carma_planning_msgs::srv::GetPluginApi::Response::SharedPtr res)
    {
        std::vector<std::string> req_capability_levels;
        boost::split(req_capability_levels, req->capability, boost::is_any_of("/"));

        for(const auto& plugin : em_.get_entries())
        {

            std::vector<std::string> plugin_capability_levels; 
            boost::split(plugin_capability_levels, plugin.capability_, boost::is_any_of("/"));

            if(plugin.type_ == carma_planning_msgs::msg::Plugin::STRATEGIC && 
                (req->capability.size() == 0 || (matching_capability(plugin_capability_levels, req_capability_levels) && plugin.active_ && plugin.available_)))
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("guidance_controller"), "discovered strategic plugin: " << plugin.name_);
                res->plan_service.push_back(plugin.name_ + plan_maneuvers_suffix_);
            }
            else
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("guidance_controller"), "not valid strategic plugin: " << plugin.name_);
            } 
        }
    }

}
