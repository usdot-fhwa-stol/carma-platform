/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <string>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include "traffic_incident_parser_worker.hpp"

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace traffic_incident_parser
{

  /**
   * \class TrafficIncidentParserNode
   * \brief The class responsible for processing incoming MobilityOperation messages with strategy 
   *        "carma3/Incident_Use_Case"
   * 
   */
  class TrafficIncidentParserNode : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<std_msgs::msg::String> projection_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::TrafficControlMessage> traffic_control_msg_pub_;

    // World Model Listener; must be declared before traffic_parser_worker_ for proper initialization
    std::shared_ptr<carma_wm::WMListener> wm_listener_;
    
    // TrafficIncidentParserWorker class object
    std::shared_ptr<TrafficIncidentParserWorker> traffic_parser_worker_;

  public:

    /**
     * \brief TrafficIncidentParserNode constructor 
     */
    explicit TrafficIncidentParserNode(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    /**
     * \brief Publish traffic control message 
     */
    void publishTrafficControlMessage(const carma_v2x_msgs::msg::TrafficControlMessage& traffic_control_msg) const;
  };

} // traffic_incident_parser
