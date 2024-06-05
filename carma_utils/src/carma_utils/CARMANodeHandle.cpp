/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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
 * CPP File containing CARMANodeHandle method definitions
 */

#include <sstream>
#include "carma_utils/CARMANodeHandle.h"
#include <ros/callback_queue.h>

namespace ros {
  // Initialize default static values

   std::mutex CARMANodeHandle::shutdown_mutex_;
   std::mutex CARMANodeHandle::system_alert_mutex_;
   std::mutex CARMANodeHandle::exception_mutex_;
   std::mutex CARMANodeHandle::spin_mutex_;
   bool CARMANodeHandle::shutting_down_ = false;
   bool CARMANodeHandle::allow_node_shutdown_ = true;
   std::string CARMANodeHandle::system_alert_topic_ = "/system_alert";
   boost::optional<double> CARMANodeHandle::spin_rate_;

   std::mutex CARMANodeHandle::static_pub_sub_mutex_;
   volatile bool CARMANodeHandle::common_pub_sub_set_ = false;
   Subscriber CARMANodeHandle::system_alert_sub_;
   Publisher CARMANodeHandle::system_alert_pub_;

   CARMANodeHandle::SystemAlertCB CARMANodeHandle::system_alert_cb_;

   CARMANodeHandle::ShutdownCB CARMANodeHandle::shutdown_cb_;

   CARMANodeHandle::ExceptionCB CARMANodeHandle::exception_cb_;

   CARMANodeHandle::SpinCB CARMANodeHandle::spin_cb_;

  CARMANodeHandle::CARMANodeHandle( const NodeHandle & 	rhs	) 
    : NodeHandle(rhs) 
  {
    initPubSub();
  }

  CARMANodeHandle::CARMANodeHandle(	const std::string & ns, const M_string & 	remappings)
    : NodeHandle(ns, remappings) 
  {
    initPubSub();
  }

  CARMANodeHandle::CARMANodeHandle(	const CARMANodeHandle & rhs	) 
    : NodeHandle(rhs)
  {
    initPubSub();
  }

  CARMANodeHandle::CARMANodeHandle(	const CARMANodeHandle & parent, const std::string & ns )
    : NodeHandle(parent, ns)
  {
    initPubSub();
  }

  CARMANodeHandle::CARMANodeHandle	(	const CARMANodeHandle & parent,
    const std::string & 	ns,
    const M_string & 	remappings)
    : NodeHandle(parent, ns, remappings)
  {
    initPubSub();
  }

  void CARMANodeHandle::initPubSub() {
    std::lock_guard<std::mutex> lock(static_pub_sub_mutex_);
    if (!common_pub_sub_set_) {
      system_alert_sub_ = this->subscribe(system_alert_topic_, 5, &CARMANodeHandle::systemAlertHandler);
      system_alert_pub_ = this->advertise<cav_msgs::SystemAlert>(system_alert_topic_, 5, true);
      common_pub_sub_set_ = true;
    }
  }

  void CARMANodeHandle::systemAlertHandler(const cav_msgs::SystemAlertConstPtr& msg) {
    ROS_INFO_STREAM("Received SystemAlert message of type: " << msg->type);
    
    std::lock_guard<std::mutex> lock(system_alert_mutex_);
    if (validFunctionPtr(system_alert_cb_)) {
      system_alert_cb_(msg); // Allow user to react to system alert message
    }

    // Take action if shutdown message
    switch(msg->type) {
      case cav_msgs::SystemAlert::SHUTDOWN: 
        shutdown(); // Shutdown this node when a SHUTDOWN request is received
        break;
    }
  }

  void CARMANodeHandle::publishSystemAlert(const cav_msgs::SystemAlert& msg) {
    system_alert_pub_.publish(msg);
  }
  
  void CARMANodeHandle::setSystemAlertCallback(SystemAlertCB cb) {
    std::lock_guard<std::mutex> lock(system_alert_mutex_);
    validateCallback(cb);
    system_alert_cb_ = cb;
  }

  void CARMANodeHandle::setShutdownCallback(ShutdownCB cb) {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    validateCallback(cb);
    shutdown_cb_ = cb;
  }

  void CARMANodeHandle::setExceptionCallback(ExceptionCB cb) { 
    std::lock_guard<std::mutex> lock(exception_mutex_);
    validateCallback(cb);
    exception_cb_ = cb;
  }

  void CARMANodeHandle::setSpinCallback(SpinCB cb) {
    std::lock_guard<std::mutex> lock(spin_mutex_);
    if (!spin_rate_) {
      handleException(std::invalid_argument("Tried to set a spin callback however the spin_rate is not set. "));
      return;
    }
    validateCallback(cb);
    spin_cb_ = cb;
  }

  void CARMANodeHandle::allowNodeShutdown(const bool allow_shutdown) {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    if (allow_shutdown == false) {
      ROS_WARN("CARMANodeHandle::allowNodeShutdown(false) has been called. This is risky in production code, but allowable in testing.");
    }
    allow_node_shutdown_ = allow_shutdown;
  }

  void CARMANodeHandle::resetShutdownStatus() {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    if (allow_node_shutdown_ == true) {
      ROS_WARN("CARMANodeHandle::resetShutdownStatus() has been called while allowNodeShutdown(true). This is not allowed and no action is taken.");
      return;
    }
    ROS_WARN("CARMANodeHandle::resetShutdownStatus() has been called. This is risky in production code, but allowable in testing.");
    shutting_down_ = false;
  }

  void CARMANodeHandle::setSpinRate(double hz) {
    std::lock_guard<std::mutex> lock(spin_mutex_);
    if (hz <= 0.0) {
      ROS_DEBUG("Unsetting spin rate as provided with 0 or negative frequency.");
      spin_rate_ = boost::none;
      return;
    }
    spin_rate_ = hz;
  }

  void CARMANodeHandle::spin() {
    ROS_INFO("Entered Spin: Waiting for any other calls to spin to complete");
    std::lock_guard<std::mutex> lock(spin_mutex_);
    ROS_INFO("Obtained lock. Starting spin");

    if (spin_rate_) {
      ROS_INFO_STREAM("Using rate controlled spin at frequency: " << spin_rate_.get());
      // Continuosly process callbacks for default_nh_ using the GlobalCallbackQueue
      ros::Rate r(*spin_rate_);
      while (ros::ok() && !shutting_down_)
      {
        ros::spinOnce();
        try {
          if (validFunctionPtr(spin_cb_) && !spin_cb_()) {
            cav_msgs::SystemAlert alert_msg;
            alert_msg.type = cav_msgs::SystemAlert::WARNING;
            alert_msg.description = "Node: " + ros::this_node::getName() + " cleanly shutting down using CARMANodeHandle after spin callback returned false";
            ROS_WARN_STREAM(alert_msg.description); // Log notice

            system_alert_pub_.publish(alert_msg); // Notify the rest of the system
            shutdown();
          }
        }
        catch(const std::exception& e) {
          handleException(e);
        }
        r.sleep();
      }
    } else {

      ROS_INFO("Using event driven spin");
      while (ros::ok() && !shutting_down_) // Default spin implementation with added exception handling based off implementation shown here http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning and https://github.com/ros/ros_comm/blob/25af588f9971c3dee7f5a79f8ce143ad0c4644df/clients/roscpp/src/libros/spinner.cpp#L156
      {
        try {
          ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        }
        catch(const std::exception& e) {
          handleException(e);
        }
      }

    }
    
    { // Empty scope for applying lock_guard for shutdown
      std::lock_guard<std::mutex> lock(shutdown_mutex_);
      // Request ros node shutdown before exit
      if (allow_node_shutdown_) {
        ros::shutdown();
      } else {
        ROS_WARN("Node shutdown attempted but CARMANodeHandle::allowNodeShutdown(false) has been called");
      }
    }
  }

  void CARMANodeHandle::handleException(const std::exception& e) {
    std::lock_guard<std::mutex> lock(exception_mutex_);
    // Create system alert message
    cav_msgs::SystemAlert alert_msg;
    alert_msg.type = cav_msgs::SystemAlert::FATAL;
    alert_msg.description = "Uncaught Exception in " + ros::this_node::getName() + " exception: " + e.what();
  
    ROS_ERROR_STREAM(alert_msg.description); // Log exception

    system_alert_pub_.publish(alert_msg); // Notify the rest of the system

    if (validFunctionPtr(exception_cb_)) {
      exception_cb_(e); // Allow user to react to exception
    }

    ros::Duration(0.05).sleep(); // Leave a small amount of time for the alert to be published
    shutdown(); // Shutdown this node
  }

  void CARMANodeHandle::shutdown() {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    if (validFunctionPtr(shutdown_cb_)) {
      shutdown_cb_(); // Allow user to react to shutdown request
    }
    
    ROS_WARN_STREAM("Node shutting down");
    shutting_down_ = true;
  }

  bool CARMANodeHandle::isRestrictedTopic(const std::string& topic) {
    return system_alert_topic_ == topic;
  }

  void CARMANodeHandle::checkSubscriptionInput(const std::string& topic) {
    if (isRestrictedTopic(topic) && common_pub_sub_set_) {
      ROS_WARN_STREAM("User requested subscription to existing CARMANodeHandle reserved topic: " << topic);
    }
  }

  void CARMANodeHandle::checkPublisherInput(const std::string& topic) {
    if (isRestrictedTopic(topic) && common_pub_sub_set_) {
      ROS_WARN_STREAM("User requested publisher to existing CARMANodeHandle reserved topic: " << topic);
    }
  }

  void CARMANodeHandle::checkServiceInput(const std::string& service) {
    if (isRestrictedTopic(service) && common_pub_sub_set_) {
      ROS_WARN_STREAM("User requested creation of service server for existing CARMANodeHandle reserved service/topic name: " << service);
    }
  }

  /////
  // OVERRIDES
  /////
  Subscriber CARMANodeHandle::subscribe(SubscribeOptions &ops) {
    ROS_WARN("subscribe(SubscribeOptions) called from CARMANodeHandle. This overload does not support exception handling. The user must implement their own");

    return NodeHandle::subscribe(ops);
  }

  Publisher CARMANodeHandle::advertise (AdvertiseOptions &ops) {
    checkPublisherInput(ops.topic);
    // Check if callbacks are set and if not set them to empty lambdas to prevent need for empty checks throughout code
    if (!validFunctionPtr(ops.connect_cb)) {
      ops.connect_cb = [](const SingleSubscriberPublisher& ssp) -> void {};
    }
    if (!validFunctionPtr(ops.disconnect_cb)) {
      ops.disconnect_cb = [](const SingleSubscriberPublisher& ssp) -> void {};
    }

    auto connect_func = callbackWrapper<const SingleSubscriberPublisher&>(ops.connect_cb);
    auto disconnect_func = callbackWrapper<const SingleSubscriberPublisher&>(ops.disconnect_cb);

    AdvertiseOptions carma_ops(ops.topic, ops.queue_size, ops.md5sum,
      ops.datatype, ops.message_definition, connect_func, disconnect_func
    );

    return NodeHandle::advertise(carma_ops);
  }

  ServiceServer CARMANodeHandle::advertiseService (AdvertiseServiceOptions &ops) {

    ROS_WARN("advertiseService(AdvertiseServiceOptions) called from CARMANodeHandle. This overload does not support exception handling. The user must implement their own");
    return NodeHandle::advertiseService(ops);
  }

  SteadyTimer CARMANodeHandle::createSteadyTimer (WallDuration period, const SteadyTimerCallback &callback, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const SteadyTimerEvent &>(callback);
    return NodeHandle::createSteadyTimer(period, func, oneshot, autostart);
  }
  
  SteadyTimer CARMANodeHandle::createSteadyTimer (SteadyTimerOptions &ops) {

    auto func = callbackWrapper<const SteadyTimerEvent &>(ops.callback);
    SteadyTimerOptions carma_ops(ops.period, func, ops.callback_queue, ops.oneshot, ops.autostart);

    return NodeHandle::createSteadyTimer(carma_ops);
  }

  Timer CARMANodeHandle::createTimer (Duration period, const TimerCallback &callback, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const TimerEvent &>(boost::bind(callback, _1));
    return NodeHandle::createTimer(period, func, oneshot, autostart);
  }

  Timer CARMANodeHandle::createTimer (TimerOptions &ops) {

    auto func = callbackWrapper<const TimerEvent &>(ops.callback);

    TimerOptions carma_ops (ops.period, 
      func,
      ops.callback_queue, ops.oneshot, ops.autostart
    );

    carma_ops.tracked_object = ops.tracked_object;

    return NodeHandle::createTimer(carma_ops);
  }

  WallTimer CARMANodeHandle::createWallTimer (WallDuration period, const WallTimerCallback &callback, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const WallTimerEvent &>(boost::bind(callback, _1));
    return NodeHandle::createWallTimer(period, func, oneshot, autostart);
  }

  WallTimer CARMANodeHandle::createWallTimer (WallTimerOptions &ops) {

    auto func = callbackWrapper<const WallTimerEvent &>(ops.callback);

    WallTimerOptions carma_ops (ops.period, 
      func,
      ops.callback_queue, ops.oneshot, ops.autostart
    );

    carma_ops.tracked_object = ops.tracked_object;

    return NodeHandle::createWallTimer(carma_ops);
  }
}
