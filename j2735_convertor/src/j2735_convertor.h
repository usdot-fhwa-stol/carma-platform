#pragma once
/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <j2735_msgs/BSM.h>
#include <j2735_msgs/SPAT.h>
#include <j2735_msgs/MapData.h>
#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/SPAT.h>
#include <cav_msgs/MapData.h>
#include "bsm_convertor.h"
#include "map_convertor.h"
#include "spat_convertor.h"

/**
 * @class J2735Convertor
 * @brief Is the class responsible for conversion of j2735_msgs to cav_msgs
 * 
 * The J2735Convertor is a ROS Node which subscribes to topics containing messages from the j2735_msgs package.
 * The j2735_msgs are then converted to cav_msgs types including any necessary unit conversions. 
 * 
 * Each j2735 topic has its own thread for processing. This will help prevent larger message types like Map from blocking
 * small high frequency message types like BSMs.
 * 
 * Other subscribed topics like system_alert are handled on the default global queue
 *
 * When an internal exception is triggered the node will first broadcast a FATAL message to the system_alert topic before shutting itself down. 
 * This node will also shut itself down on recieve of a SHUTDOWN message from system_alert
 */
class J2735Convertor 
{
private:
    // Shutdown flags and mutex
    std::mutex shutdown_mutex_;
    bool shutting_down_ = false;
    // Members used in ROS behavior
    int default_spin_rate_ = 10;
    ros::Publisher converted_bsm_pub_, converted_spat_pub_, converted_map_pub_, system_alert_pub_, outbound_j2735_bsm_pub_;
    ros::Subscriber j2735_bsm_sub_, j2735_spat_sub_, j2735_map_sub_, system_alert_sub_, outbound_bsm_sub_;
    std::shared_ptr<ros::NodeHandle> default_nh_;
    std::shared_ptr<ros::NodeHandle> bsm_nh_;
    std::shared_ptr<ros::NodeHandle> spat_nh_;
    std::shared_ptr<ros::NodeHandle> map_nh_;
    ros::CallbackQueue bsm_queue_;
    ros::CallbackQueue spat_queue_;
    ros::CallbackQueue map_queue_;
public:
  /**
   * @brief Constructor
   * @param argc - command line argument count
   * @param argv - command line arguments
   */
  J2735Convertor(int argc, char** argv) {
    ros::init(argc,argv,"j2735_convertor_node"); // Initialize ROS connection
  };

  /**
   * @brief Execution function which will start the ROS subscriptions and publications. Exits on node shutdown.
   */
  int run();

private:

  /**
   * @brief Initializes the subscribers and publishers for this node
   */
  void initialize();

  /**
   * @brief Converts cav_msgs::BSM messages to j2735_msgs::BSM and publishes the converted messages
   * 
   * @param message The message to convert
   */
  void BsmHandler(const cav_msgs::BSMConstPtr& message);

  /**
   * @brief Converts j2735_msgs::BSM messages to cav_msgs::BSM and publishes the converted messages
   * 
   * @param message The message to convert
   */
  void j2735BsmHandler(const j2735_msgs::BSMConstPtr& message);

  /**
   * @brief Converts j2735_msgs::SPAT messages to cav_msgs::SPAT and publishes the converted messages
   * 
   * @param message The message to convert
   */
  void j2735SpatHandler(const j2735_msgs::SPATConstPtr& message);

  /**
   * @brief Converts j2735_msgs::MapData messages to cav_msgs::MapData and publishes the converted messages
   * 
   * @param message The message to convert
   */
  void j2735MapHandler(const j2735_msgs::MapDataConstPtr& message);

  /**
   * @brief Handles incoming SystemAlert messages
   * 
   * @param message The message to handle
   * 
   * Handles incoming SystemAlert messages and will shutdown this node if that message was of type SHUTDOWN
   */
  void systemAlertHandler(const cav_msgs::SystemAlertConstPtr& message);

  /**
   * @brief Handles caught exceptions which have reached the top level of this node
   * 
   * @param message The exception to handle
   * 
   * If an exception reaches the top level of this node it should be passed to this function.
   * The function will try to log the exception and publish a FATAL message to system_alert before shutting itself down.
   */
  void handleException(const std::exception& e);

  /**
   * @brief Shutsdown this node
   */
  void shutdown();
};