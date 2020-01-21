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

/**
 * CPP File containing J2735Convertor method definitions
 */

#include "j2735_convertor.h"

int J2735Convertor::run() {
  // Initialize Node
  try {
    initialize();
  }
  catch(const std::exception& e) {
    ROS_ERROR_STREAM("Failed to initialize node with exception: " << e.what());
  }

  // Attach bsm, spat, map to processing to unique threads
  ros::AsyncSpinner bsm_spinner(1, &bsm_queue_);
  bsm_spinner.start();

  ros::AsyncSpinner spat_spinner(1, &spat_queue_);
  spat_spinner.start();

  ros::AsyncSpinner map_spinner(1, &map_queue_);
  map_spinner.start();

  // Continuosly process callbacks for default_nh_ using the GlobalCallbackQueue
  ros::Rate r(default_spin_rate_);
  while (ros::ok() && !shutting_down_)
  {
    ros::spinOnce();
    r.sleep();
  }
  // Request ros node shutdown before exit
  ros::shutdown();

  return 0; 
}

void J2735Convertor::initialize() {
  // Setup node handles here if needed
  default_nh_.reset(new ros::NodeHandle());
  bsm_nh_.reset(new ros::NodeHandle());
  spat_nh_.reset(new ros::NodeHandle());
  map_nh_.reset(new ros::NodeHandle());
  // Set Callback Queues for Node Handles
  bsm_nh_->setCallbackQueue(&bsm_queue_);
  spat_nh_->setCallbackQueue(&spat_queue_);
  map_nh_->setCallbackQueue(&map_queue_);

  // J2735 BSM Subscriber
  j2735_bsm_sub_ = bsm_nh_->subscribe("incoming_j2735_bsm", 100, &J2735Convertor::j2735BsmHandler, this);

  // BSM Publisher
  converted_bsm_pub_ = bsm_nh_->advertise<cav_msgs::BSM>("incoming_bsm", 100);

  // Outgoing J2735 BSM Subscriber
  outbound_bsm_sub_ = bsm_nh_->subscribe("outgoing_bsm", 1, &J2735Convertor::BsmHandler, this); // Queue size of 1 as we should never publish outdated BSMs

  // BSM Publisher
  outbound_j2735_bsm_pub_ = bsm_nh_->advertise<j2735_msgs::BSM>("outgoing_j2735_bsm", 1); // Queue size of 1 as we should never publish outdated BSMs

  // J2735 SPAT Subscriber
  j2735_spat_sub_ = spat_nh_->subscribe("incoming_j2735_spat", 100, &J2735Convertor::j2735SpatHandler, this);

  // SPAT Publisher TODO think about queue sizes
  converted_spat_pub_ = spat_nh_->advertise<cav_msgs::SPAT>("incoming_spat", 100);

  // J2735 MAP Subscriber
  j2735_map_sub_ = map_nh_->subscribe("incoming_j2735_map", 50, &J2735Convertor::j2735MapHandler, this);

  // MAP Publisher TODO think about queue sizes
  converted_map_pub_ = map_nh_->advertise<cav_msgs::MapData>("incoming_map", 50);

  // SystemAlert Subscriber
  system_alert_sub_ = default_nh_->subscribe("system_alert", 10, &J2735Convertor::systemAlertHandler, this);

  // SystemAlert Publisher
  system_alert_pub_ = default_nh_->advertise<cav_msgs::SystemAlert>("system_alert", 10, true);
}

void J2735Convertor::BsmHandler(const cav_msgs::BSMConstPtr& message) {
  try {
    j2735_msgs::BSM j2735_msg;
    BSMConvertor::convert(*message, j2735_msg); // Convert message
    outbound_j2735_bsm_pub_.publish(j2735_msg); // Publish converted message
  }
  catch(const std::exception& e) {
    handleException(e);
  }
}

void J2735Convertor::j2735BsmHandler(const j2735_msgs::BSMConstPtr& message) {
  try {
    cav_msgs::BSM converted_msg;
    BSMConvertor::convert(*message, converted_msg); // Convert message
    converted_bsm_pub_.publish(converted_msg); // Publish converted message
  }
  catch(const std::exception& e) {
    handleException(e);
  }
}

void J2735Convertor::j2735SpatHandler(const j2735_msgs::SPATConstPtr& message) {
  try {
    cav_msgs::SPAT converted_msg;
    SPATConvertor::convert(*message, converted_msg); // Convert message
    converted_spat_pub_.publish(converted_msg); // Publish converted message
  }
  catch(const std::exception& e) {
    handleException(e);
  }
}

void J2735Convertor::j2735MapHandler(const j2735_msgs::MapDataConstPtr& message) {
  try {
    cav_msgs::MapData converted_msg;
    MapConvertor::convert(*message, converted_msg); // Convert message
    converted_map_pub_.publish(converted_msg); // Publish converted message
  }
  catch(const std::exception& e) {
    handleException(e);
  }
}

void J2735Convertor::systemAlertHandler(const cav_msgs::SystemAlertConstPtr& message) {
  try {
    ROS_INFO_STREAM("Received SystemAlert message of type: " << message->type);
    switch(message->type) {
      case cav_msgs::SystemAlert::SHUTDOWN: 
        shutdown(); // Shutdown this node when a SHUTDOWN request is received 
        break;
    }
  }
  catch(const std::exception& e) {
    handleException(e);
  }
}

void J2735Convertor::handleException(const std::exception& e) {
  // Create system alert message
  cav_msgs::SystemAlert alert_msg;
  alert_msg.type = cav_msgs::SystemAlert::FATAL;
  alert_msg.description = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
 
  ROS_ERROR_STREAM(alert_msg.description); // Log exception

  system_alert_pub_.publish(alert_msg); // Notify the rest of the system

  ros::Duration(0.05).sleep(); // Leave a small amount of time for the alert to be published
  shutdown(); // Shutdown this node
}

void J2735Convertor::shutdown() {
  std::lock_guard<std::mutex> lock(shutdown_mutex_);
  ROS_WARN_STREAM("Node shutting down");
  shutting_down_ = true;
}
