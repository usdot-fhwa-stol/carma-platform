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

#pragma once

#include <gtest/gtest_prod.h>
#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_guidance_plugins
{

  /**
   * \brief PluginBaseNode provides default functionality for all carma guidance plugins.
   *        This includes basic state machine management (largely delegated to ROS2 lifecycle behavior), required interfaces, and plugin discovery
   * 
   * Extending classes must implement the on_configure_plugin method to load parameters, and my override the other state transitions methods on_<state>_plugin if desired.
   * Additionally, extending classes must implement the methods such as get_plugin_name() which are used to populate the plugin discovery message.
   * 
   * NOTE: At the moment, this class and all extending classes are setup to support only single threading.
   * 
   */
  class PluginBaseNode : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Publishers
    //! Publisher for plugin discovery. This publisher will begin publishing 
    //immediately on node construction regardless of lifecycle state. This is meant to allow for fast plugin activation on startup.
    rclcpp::Publisher<carma_planning_msgs::msg::Plugin>::SharedPtr plugin_discovery_pub_;

    // Timers
    //! Timer to trigger publication of the plugin discovery message at a fixed frequency 
    rclcpp::TimerBase::SharedPtr discovery_timer_;

    // WorldModel listener
    // This variable is intentionally private, so that is can be lazily initialized 
    // when the extending class calls get_world_model_listener(); or get_world_model();
    std::shared_ptr<carma_wm::WMListener> wm_listener_;

    // World Model populated by the listener at runtime
    // This variable is intentionally private, so that is can be lazily initialized 
    // when the extending class calls get_world_model_listener(); or get_world_model();
    carma_wm::WorldModelConstPtr wm_;

    /**
     * \brief Callback for the plugin discovery timer which will publish the plugin discovery message
     */
    void discovery_timer_callback();

    /**
     * \brief Helper function for lazy initialization of wm_listener_. If already initialized method returns (ie. not a reset)
     */ 
    void lazy_wm_initialization();

  public:
    /**
     * \brief PluginBaseNode constructor 
     */
    explicit PluginBaseNode(const rclcpp::NodeOptions &);

    //! Virtual destructor for safe deletion
    virtual ~PluginBaseNode() = default;

    /**
     * \brief Method to return the default world model listener provided as a convience by this base class
     *        If this method or get_world_model() are not called then the world model remains uninitialized and 
     *        will not create unnecessary subscriptions. 
     * 
     * \return Pointer to an initialized world model listener
     */ 
    virtual std::shared_ptr<carma_wm::WMListener> get_world_model_listener() final;

    /**
     * \brief Method to return the default world model provided as a convience by this base class
     *        If this method or get_world_model_listener() are not called then the world model remains uninitialized and 
     *        will not create unnecessary subscriptions. 
     * 
     * \return Pointer to an initialized world model. Returned instance is that same as get_world_model_listener()->getWorldModel();
     */ 
    virtual carma_wm::WorldModelConstPtr get_world_model() final;

    /**
     * \brief Returns the activation status of this plugin.
     *        The plugins API callbacks will only be triggered when this method returns true. 
     * 
     * \return True if plugin is in the ACTIVE state. False otherwise.
     */ 
    virtual bool get_activation_status() final;


    /**
     * \brief Returns the type of this plugin according to the carma_planning_msgs::Plugin type enum.
     *        Extending classes for the specific type should override this method. 
     * 
     * \return The extending class type or UNKOWN if the class or no parent class has implement this method. 
     */ 
    virtual uint8_t get_type();

    /**
     * \brief Get the availability status of this plugin based on the current operating environment.
     *        Method must be overriden by extending classes. 
     * 
     * \return This method should return true if the plugin's current understanding of the world
     *         means it would be capable of planning or executing its capability.
     */ 
    virtual bool get_availability() = 0;

    /**
     * \brief Get the capability string representing this plugins capabilities
     *        Method must be overriden by extending classes.
     *        Expectation is that abstract plugin type parent classes will provide a default implementation.
     * 
     * \return Forward slash separated string describing the plugin's capabilities per the plugin capabilites API
     */ 
    virtual std::string get_capability() = 0;

    /**
     * \brief Returns the name of this plugin.
     *        This name may be automatically prepended to plugin API service or topic names. 
     * 
     * \return Name of this plugin
     */ 
    virtual std::string get_plugin_name() = 0;

    /**
     * \brief Returns the version id of this plugin.
     * 
     * \return The version id represented as a string
     */ 
    virtual std::string get_version_id() = 0;

    /**
     * \brief Method which is triggered when this plugin is moved from the UNCONFIGURED to INACTIVE states. 
     *        This method should be used to load parameters and is required to be implemented. 
     * 
     * \return SUCCESS, FAILURE, or ERROR. Transition to INACTIVE will only complete on SUCCESS. 
     */  
    virtual carma_ros2_utils::CallbackReturn on_configure_plugin() = 0;

    /**
     * \brief Method which is triggered when this plugin is moved from the INACTIVE to ACTIVE states. 
     *        This method should be used to prepare for future callbacks for plugin's capabilites. 
     * 
     * \return SUCCESS, FAILURE, or ERROR. Transition to ACTIVE will only complete on SUCCESS. 
     */ 
    virtual carma_ros2_utils::CallbackReturn on_activate_plugin();

    /**
     * \brief Method which is triggered when this plugin is moved from the ACTIVE to INACTIVE states. 
     *        This method should be used to disable any functionality which should cease execution when plugin is inactive.
     * 
     * \return SUCCESS, FAILURE, or ERROR. Transition to INACTIVE will only complete on SUCCESS. 
     */  
    virtual carma_ros2_utils::CallbackReturn on_deactivate_plugin();

    /**
     * \brief Method which is triggered when this plugin is moved from the INACTIVE to UNCONFIGURED states. 
     *        This method should be used to fully reset the plugin such that a future call to on_configure_plugin would leave the plugin
     *        in a fresh state as though just launched.
     * 
     * \return SUCCESS, FAILURE, or ERROR. Transition to UNCONFIGURED will only complete on SUCCESS. 
     */
    virtual carma_ros2_utils::CallbackReturn on_cleanup_plugin();

    /**
     * \brief Method which is triggered when this plugin is moved from any state to FINALIZED
     *        This method should be used to generate any shutdown logs or final cleanup.
     * 
     * \return SUCCESS, FAILURE, or ERROR. On ERROR, may temporarily go to on_error_plugin() before calling finalized.
     */
    virtual carma_ros2_utils::CallbackReturn on_shutdown_plugin();

    /**
     * \brief Method which is triggered when an unhandled exception occurs in this plugin
     *        This method should be used to cleanup such that the plugin could be moved to UNCONFIGURED state if possible.
     * 
     * \param exception_string The exception description
     * 
     * \return SUCCESS, FAILURE, or ERROR. On SUCCESS plugin moves to UNCONFIGURED state else FINALIZED. Default behavior is to move to FINALIZED.
     */
    virtual carma_ros2_utils::CallbackReturn on_error_plugin(const std::string &exception_string);

    ////
    // Overrides
    ////
    // For simplicity of managing the plugin state machine all lifecycle callbacks are implemented as final by the core extending classes (strategic, tactical, control)
    // Plugins will use the plugin specific callbacks via the template pattern 
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &) override;
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &) override; 
    carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &, const std::string &exception_string) override;
    
    // Unit Test Accessors
    FRIEND_TEST(carma_guidance_plugins_test, connections_test);

  };

} // carma_guidance_plugins
