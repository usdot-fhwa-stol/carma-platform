/*
 * Copyright (C) 2021 LEIDOS.
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

#ifndef ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_
#define ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <chrono>

namespace ros2_lifecycle_manager
{

  using std_nanosec = std::chrono::nanoseconds;

  /**
   * \brief This interface defines a C++ API to be used for lifecycle management which will manage rclcpp::LifecycleNode(s)
   *  
   * The user should use set_managed_nodes to provide a list of nodes to manage.
   * The user can then walk the nodes through their lifecycle using the provided 
  *  configure, cleanup, activate, deactivate, and shutdown methods. 
   * 
   */
  class LifecycleManagerInterface
  {
  public:

    /**
     * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
     *
     */
    virtual ~LifecycleManagerInterface(){};

    /**
     * \brief Set the nodes which will be managed by the implementing object. 
     * 
     * \param nodes The list of Fully Qualified Names for nodes to manage.
     *              The order of these nodes will be obeyed in the transition methods if the ordered argument is used.
     */ 
    virtual void set_managed_nodes(const std::vector<std::string> &nodes) = 0;

    /**
     * \brief Returns the list of managed node's Fully Qualified Names
     */ 
    virtual std::vector<std::string> get_managed_nodes() = 0;

    /**
     * \brief Returns the Lifecycle state of the provided node
     */ 
    virtual uint8_t get_managed_node_state(const std::string &node) = 0;

    /**
     * \brief Send the Configure transition to all managed nodes. Returns true if all nodes transitioned successfully.
     * 
     * \param connection_timeout The length of time in nanoseconds to wait for connection to be established with EACH node. 
     * \param call_timeout The length of time in nanoseconds to wait for successfull transition execution for EACH node. 
     * \param ordered If true then the nodes will be transitioned in order of the list provided by set_managed_nodes. 
     *                If false the nodes will all be triggered at once.
     * 
     * \return True if all nodes successfully transitioned. False otherwise. 
     */
    virtual bool configure(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) = 0;

    //! \brief Same as configure() except for Cleanup transition
    virtual bool cleanup(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) = 0;

    //! \brief Same as configure() except for Activate transition
    virtual bool activate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) = 0;

    //! \brief Same as configure() except for Deactivate transition
    virtual bool deactivate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) = 0;

    //! \brief Same as configure() except for Shutdown transition
    virtual bool shutdown(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) = 0;
  };

} // namespace ros2_lifecycle_manager

#endif // ROS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_INTERFACE_HPP_
