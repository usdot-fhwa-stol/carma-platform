#pragma once

/*
 * Copyright (C) 2019-2022LEIDOS.
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

#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/get_plugin_api.hpp>
#include <carma_planning_msgs/srv/plugin_list.hpp>
#include <carma_planning_msgs/srv/plugin_activation.hpp>
#include <ros2_lifecycle_manager/lifecycle_manager_interface.hpp>
#include <unordered_set>
#include <functional>
#include <vector>
#include <memory>
#include "entry_manager.h"
#include "entry.h"


namespace subsystem_controllers
{

    using GetParentNodeStateFunc = std::function<uint8_t()>;

    /**
     * \brief The PluginManager serves as a component to manage CARMA Guidance Plugins via their ros2 lifecycle interfaces
     */ 
    class PluginManager
    {
        public:

            // TODO do we need a system alert callback in this manager as well

            /**
             * \brief Constructor for PluginManager
             * 
             * \param required_plugins The set of plugins which will be treated as required. A failure in these plugins will result in an exception
             * \param auto_activated_plugins The set of plugins which will be automatically activated at first system activation but not treated specially after that.
             * \param plugin_lifecycle_mgr A fully initialized lifecycle manager which will be used trigger plugin transitions
             * \param get_parent_state_func A callback which will allow this object to access the parent process lifecycle state
             */
            PluginManager(const std::vector<std::string>& required_plugins,
                          const std::vector<std::string>& auto_activated_plugins,
                          std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> plugin_lifecycle_mgr,
                          GetParentNodeStateFunc get_parent_state_func);

            /**
             * Below are the state transition methods which will cause this manager to trigger the corresponding 
             * state transitions in the managed plugins. 
             */ 
            void configure(); 
            void activate(); 
            void deactivate(); 
            void cleanup(); 
            void shutdown(); 

            /**
             * \brief Update the status of a certain plugin
             * 
             * \param msg A plugin status message
             */
            void update_plugin_status(carma_planning_msgs::msg::Plugin::UniquePtr msg);

            /**
             * \brief Returns the list of known plugins
             * 
             * \param req The req details
             * \param[out] res The response containing the list of known plugins
             */ 
            void get_registered_plugins(carma_planning_msgs::srv::PluginList::Request::SharedPtr req, carma_planning_msgs::srv::PluginList::Response::SharedPtr res);

            /**
             * \brief Get the list of currently active plugins
             * 
             * \param req The req details
             * \param[out] res The response containing the list of active plugins
             */ 
            void get_active_plugins(carma_planning_msgs::srv::PluginList::Request::SharedPtr req, carma_planning_msgs::srv::PluginList::Response::SharedPtr res);
            
            /**
             * \brief Activate the specified plugin
             * 
             * \param req The req details containing the plugin to activate
             * \param[out] res The response containing the success flag
             */ 
            void activate_plugin(carma_planning_msgs::srv::PluginActivation::Request::SharedPtr req, carma_planning_msgs::srv::PluginActivation::Response::SharedPtr res);

            /**
             * \brief Get strategic plugins by capability
             * 
             * \param req The req which identifies which capability is required
             * \param res The res which identifies the strategic plugins with the requested capability
             */
            void get_strategic_plugins_by_capability(carma_planning_msgs::srv::GetPluginApi::Request::SharedPtr req, carma_planning_msgs::srv::GetPluginApi::Response::SharedPtr res);

            /**
             * \brief Get tactical plugins by capability
             * 
             * \param req The req which identifies which capability is required
             * \param res The res which identifies the tactical plugins with the requested capability
             */
            void get_tactical_plugins_by_capability(carma_planning_msgs::srv::GetPluginApi::Request::SharedPtr req, carma_planning_msgs::srv::GetPluginApi::Response::SharedPtr res);

            /**
             * \brief Get control plugins by capability
             * 
             * \param req The req which identifies which capability is required
             * \param res The res which identifies the control plugins with the requested capability
             */
            void get_control_plugins_by_capability(carma_planning_msgs::srv::GetPluginApi::Request::SharedPtr req, carma_planning_msgs::srv::GetPluginApi::Response::SharedPtr res);

        protected:

            /**
             * \brief Add the specified entry to our plugin management
             *        This function will attempt to move the newly detected plugin to the required state
             *        based on this nodes own state
             * 
             * \param plugin The entry representing the plugin to add 
             * 
             * \throws std::runtime_error if this was a required plugin and it could not be transitioned as needed
             */ 
            void add_plugin(const Entry& plugin);

            /**
             * \brief Returns true if the provided base capability hierarchy can achieve the requested capability hierarchy
             *        A capability hierarchy is described as a list of strings where the first string is the most generic description
             *        of the capability while the last string the is the most detailed description.
             * 
             * For example the capability "tactical_plan/plan_trajectory" would be described as ["tactical_plan", "plan_trajectory"]
             * if the user wanted to match this to "tactical_plan/plan_trajectory/platooning_trajectory" then the compared_capability_levels
             * would be ["tactical_plan", "plan_trajectory", "compared_capability_levels"]
             * Matching those two inputs as ( ["tactical_plan", "plan_trajectory"], ["tactical_plan", "plan_trajectory", "compared_capability_levels"])
             * would be false because the compared_capability_levels is more detailed than the base. 
             * By contrast, matching ( ["tactical_plan", "plan_trajectory", "compared_capability_levels"], ["tactical_plan", "plan_trajectory"] ) 
             * would be true since the base is more generic then the request
             * 
             * // TODO check with Kyle on this because it maybe should be the reverse since a more specific capability may require specific meta data. 
             * 
             * \param base_capability_levels The base hierarchy to check for compatability
             * \param compared_capability_levels The compared_capability_levels which are being checked against the base
             * 
             * \return True if base_capability_levels supports compared_capability_levels
             */ 
            bool matching_capability(const std::vector<std::string>& base_capability_levels, const std::vector<std::string>& compared_capability_levels);

            //! Set of required plugins a failure of which necessitates system shutdown
            std::unordered_set<std::string> required_plugins_;

            //! Set of use specified auto activated plugins which will automatically started without need for user input
            // These will only be activated once, if the user later deactivates them then that behavior will be preserved
            std::unordered_set<std::string> auto_activated_plugins_;

            //! Lifecycle Manager which will track the plugin nodes and call their lifecycle services on request
            std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> plugin_lifecycle_mgr_;

            //! Callback to retrieve the lifecycle state of the parent process 
            GetParentNodeStateFunc get_parent_state_func_;

    };
}