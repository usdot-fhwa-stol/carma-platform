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

#ifndef ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_MANAGER_HPP_
#define ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_MANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_manager_interface.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

namespace ros2_lifecycle_manager
{
  using ChangeStateClient = rclcpp::Client<lifecycle_msgs::srv::ChangeState>;
  using GetStateClient = rclcpp::Client<lifecycle_msgs::srv::GetState>;

  // Struct combining the node name and relevant service clients together
  struct ManagedNode
  {

    /**
     * \brief Constructor
     * \param node_name The fully qualified node name
     * \param change_state_client The service client to use for accessing the change state service
     * \param get_state_client The service client to use for accessing the get state service
     */ 
    ManagedNode(const std::string &node_name, std::shared_ptr<ChangeStateClient> change_state_client, std::shared_ptr<GetStateClient> get_state_client) : node_name(node_name), change_state_client(change_state_client), get_state_client(get_state_client){};

    //! The fully qualified name of this node
    std::string node_name;
    //! The service client to use for the change_state service
    std::shared_ptr<ChangeStateClient> change_state_client;
    //! The service client to use for the get_state service
    std::shared_ptr<GetStateClient> get_state_client;

  };

  /**
   * \brief Implementation of the LifecycleManagerInterface using ROS2 service calls
   * NOTE: This class will use a separate Reentrant service client for service calls. 
   *       Therefore for the logic to execute correctly a MultiThreadedExecutor may be required. 
   */
  class Ros2LifecycleManager : public LifecycleManagerInterface
  {

  public:
    using ChangeStateSharedFutureWithRequest = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFutureWithRequest;

    /**
     * \brief Constructor
     * 
     * \param node_logging The base logging interface
     * \param node_services The base service providing interface
     */ 
    Ros2LifecycleManager(
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
      rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services
    );

    ~Ros2LifecycleManager() = default;

    ////
    // Overrides
    ////
    void set_managed_nodes(const std::vector<std::string> &nodes) override;
    std::vector<std::string> get_managed_nodes() override;
    uint8_t get_managed_node_state(const std::string &node) override;
    bool configure(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) override;
    bool cleanup(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) override;
    bool activate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) override;
    bool deactivate(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) override;
    bool shutdown(const std_nanosec &connection_timeout, const std_nanosec &call_timeout, bool ordered = true) override;

  protected:
    /**
     * \brief Method to call the relevant services for all managed nodes to execute the provided transition.
     * 
     * \param connection_timeout The length of time in nanoseconds to wait for connection to be established with EACH node. 
     * \param call_timeout The length of time in nanoseconds to wait for successfull transition execution for EACH node. 
     * \param ordered If true then the nodes will be transitioned in order of the list provided by set_managed_nodes. 
     *                If false the nodes will all be triggered at once.
     * 
     * \return True if all nodes successfully transitioned. False otherwise. 
     */
    bool transition_multiplex(uint8_t transition, bool ordered, const std_nanosec &connection_timeout, const std_nanosec &call_timeout);

    /**
     * \brief Helper function to wait on the provided service client for the provided period of time.
     *        Log a warning if not available.
     * 
     * \tparam T The message type of the client
     * \param client The client to wait on
     * \param timeout The timeout in nanoseconds to wait on the client
     * 
     * \return True if the service is available within the provided timeout. False otherwise.
     */ 
    template <class T>
    bool waitForService(std::shared_ptr<rclcpp::Client<T>> client, const std_nanosec &timeout)
    {
      if (!client->wait_for_service(timeout))
      {
        RCLCPP_ERROR(
            node_logging_->get_logger(),
            "Service %s is not available.",
            client->get_service_name());
        return false;
      }

      return true;
    }

    /**
     * \brief Helper function to create a service client of the provided type using the node interfaces provided in the constructor
     * 
     * \param service_name The name of the service
     * \return A pointer to the initialized service
     */ 
    template <class ServiceT>
    typename rclcpp::Client<ServiceT>::SharedPtr
    create_client(const std::string service_name) {
      return rclcpp::create_client<ServiceT>(
        node_base_,
        node_graph_,
        node_services_,
        service_name,
        rmw_qos_profile_services_default,
        service_callback_group_); // Use the interface specific callbackgroup
    }

    /**
     * \brief Helper function to wait for the provided future and return true if the future has the indicator for successful transition.
     * 
     * \param futrue The future to wait on
     * \param timeout The timeout in nanoseconds to wait
     * 
     * \return True if the future returned and had the correct value. False otherwise.
     */ 
    bool wait_on_change_state_future(const ChangeStateSharedFutureWithRequest &future,
                                   const std_nanosec &timeout);

    //! The service base name used for ROS2 Lifecycle Node's change state operation
    const std::string change_state_topic_ = "/change_state";
    //! The service base name used for ROS2 Lifecycle Node's get state operation
    const std::string get_state_topic_ = "/get_state";
    //! The list of managed nodes
    std::vector<ManagedNode> managed_nodes_;
    //! The list of managed node fully qualified names
    std::vector<std::string> managed_node_names_;
    //! HashMap of node names with index for fast access
    std::unordered_map<std::string, size_t> node_map_;
    //! The required node interfaces
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_;
    //! Reentrant callback group to use with service calls. Setup this way so that this classes functions 
    //  can be called from topic callbacks
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  };

} // namespace ros2_lifecycle_manager

#endif // ROS2_LIFECYCLE_MANAGER__ROS2_LIFECYCLE_MANAGER_HPP_
