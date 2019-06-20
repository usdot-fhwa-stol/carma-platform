/*
 * Copyright (C) 2018-2019 LEIDOS.
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


namespace ros {
  // Initialize default static values

   std::mutex CARMANodeHandle::shutdown_mutex_;
   std::mutex CARMANodeHandle::system_alert_mutex_;
   std::mutex CARMANodeHandle::exception_mutex_;
   std::mutex CARMANodeHandle::spin_mutex_;
   bool CARMANodeHandle::shutting_down_ = false;
   bool CARMANodeHandle::allow_node_shutdown_ = true;
   std::string CARMANodeHandle::system_alert_topic_ = "system_alert";
   double CARMANodeHandle::default_spin_rate_ = 20.0;

   std::mutex CARMANodeHandle::static_pub_sub_mutex_;
   volatile bool CARMANodeHandle::common_pub_sub_set_ = false;
   Subscriber CARMANodeHandle::system_alert_sub_;
   Publisher CARMANodeHandle::system_alert_pub_;

   CARMANodeHandle::SystemAlertCB CARMANodeHandle::system_alert_cb_;

   CARMANodeHandle::ShutdownCB CARMANodeHandle::shutdown_cb_;

   CARMANodeHandle::ExceptionCB CARMANodeHandle::exception_cb_;

   CARMANodeHandle::SpinCB CARMANodeHandle::spin_cb_;

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

  void CARMANodeHandle::setSpinCallback(SpinCB cb) {
    std::lock_guard<std::mutex> lock(spin_mutex_);
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
    default_spin_rate_ = hz;
  }

  void CARMANodeHandle::spin() {
    ROS_INFO("Entered Spin: Waiting for any other calls to spin to complete");
    std::lock_guard<std::mutex> lock(spin_mutex_);
    ROS_INFO("Obtained lock. Starting spin");
    // Continuosly process callbacks for default_nh_ using the GlobalCallbackQueue
    ros::Rate r(default_spin_rate_);
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

  template<class C>
  bool CARMANodeHandle::validFunctionPtr(const C& cb) {
    return !(!cb);
  }

  template<class C>
  void CARMANodeHandle::validateCallback(const C& cb) { 
    if (!validFunctionPtr(cb)) {
      std::ostringstream msg;
      msg << "Invalid callback used in CARMANodeHandle: Callback does not point to callable object";
      throw std::invalid_argument(msg.str());
    }
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

  template<class C>
  boost::function< void(C)> CARMANodeHandle::callbackWrapper(const boost::function<void(C)>& callback) {
    validateCallback(callback);

    boost::function< void(C)> wrappedFunc = 
    [callback] (const C& msg) -> void {
      try {
        callback(msg);
      } catch(const std::exception& e) {
        handleException(e);
      }
    };

    return wrappedFunc;
  }

  template<class C, class R>
  boost::function< bool(C, R)> CARMANodeHandle::serviceCallbackWrapper(const boost::function<bool(C, R)>& callback) {
    validateCallback(callback);

    boost::function< bool(C, R)> wrappedFunc = 
    [callback] (C req, R res) -> bool {
      try {
        callback(req, res);
      } catch(const std::exception& e) {
        handleException(e);
      }
    };

    return wrappedFunc;
  }

  template<class E>
  boost::function< bool(E)> CARMANodeHandle::serviceEventCallbackWrapper(const boost::function<bool(E)>& callback) {
    validateCallback(callback);

    boost::function< bool(E)> wrappedFunc = 
    [callback] (E event) -> bool {
      try {
        callback(event);
      } catch(const std::exception& e) {
        handleException(e);
      }
    };

    return wrappedFunc;
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints) {

    checkSubscriptionInput(topic);

    auto func = callbackWrapper<M>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, T *obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<M>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), T *obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<const boost::shared_ptr< M const > &>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, T *obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<const boost::shared_ptr< M const > &>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<M>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<M>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<const boost::shared_ptr< M const > &>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M , class T >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<const boost::shared_ptr< M const > &>(boost::bind(fp, obj, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(M), const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<M>(boost::bind(fp, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< M const > &), const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<const boost::shared_ptr< M const > &>(boost::bind(fp, _1));
    return NodeHandle::subscribe<M>(topic, queue_size, func, VoidConstPtr(), transport_hints);
  }

  template<class M >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, const boost::function< void(const boost::shared_ptr< M const > &)> &callback, const VoidConstPtr &tracked_object, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<const boost::shared_ptr< M const > &>(callback);
    return NodeHandle::subscribe<M>(topic, queue_size, func, tracked_object, transport_hints);
  }

  template<class M , class C >
  Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, const boost::function< void(C)> &callback, const VoidConstPtr &tracked_object, const TransportHints &transport_hints) {
    
    checkSubscriptionInput(topic);

    auto func = callbackWrapper<C>(callback);
    return NodeHandle::subscribe<M>(topic, queue_size, func, tracked_object, transport_hints);
  }

  Subscriber CARMANodeHandle::subscribe(SubscribeOptions &ops) {
    ROS_WARN("subscribe(SubscribeOptions) called from CARMANodeHandle. This overload does not support exception handling. The user must implement their own");

    return NodeHandle::subscribe(ops);
  }

  template<class M >
  Publisher CARMANodeHandle::advertise (const std::string &topic, uint32_t queue_size, bool latch) {
    checkPublisherInput(topic);
    return NodeHandle::advertise<M>(topic, queue_size, latch);
  }
  
  template<class M >
  Publisher CARMANodeHandle::advertise (const std::string &topic, uint32_t queue_size, const SubscriberStatusCallback &connect_cb, const SubscriberStatusCallback &disconnect_cb, const VoidConstPtr &tracked_object, bool latch) {
    checkPublisherInput(topic);
    // Check if callbacks are set and if not set them to empty lambdas to prevent need for empty checks throughout code
    SubscriberStatusCallback new_connect_cb = connect_cb;
    SubscriberStatusCallback new_disconnect_cb = disconnect_cb;
    if (!validFunctionPtr(connect_cb)) {
      new_connect_cb = [](const SingleSubscriberPublisher& ssp) -> void {};
    }
    if (!validFunctionPtr(disconnect_cb)) {
      new_disconnect_cb = [](const SingleSubscriberPublisher& ssp) -> void {};
    }

    auto connect_func = callbackWrapper<const SingleSubscriberPublisher&>(new_connect_cb);
    auto disconnect_func = callbackWrapper<const SingleSubscriberPublisher&>(new_disconnect_cb);
    return NodeHandle::advertise<M>(topic, queue_size, connect_func, disconnect_func, tracked_object, latch);
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

  template<class T , class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, bool(T::*srv_func)(MReq &, MRes &), T *obj) {
    checkServiceInput(service);

    auto func = serviceCallbackWrapper<MReq&, MRes&>(boost::bind(srv_func, obj, _1, _2));
    return NodeHandle::advertiseService(service, func, VoidConstPtr());
  }
  
  template<class T , class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, bool(T::*srv_func)(ServiceEvent< MReq, MRes > &), T *obj) {
    checkServiceInput(service);

    auto func = serviceEventCallbackWrapper<ServiceEvent< MReq, MRes > &>(boost::bind(srv_func, obj, _1));
    return NodeHandle::advertiseService(service, func, VoidConstPtr());
  }
  
  template<class T , class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, bool(T::*srv_func)(MReq &, MRes &), const boost::shared_ptr< T > &obj) {
    checkServiceInput(service);

    auto func = serviceCallbackWrapper<MReq&, MRes&>(boost::bind(srv_func, obj, _1, _2));
    return NodeHandle::advertiseService(service, func, VoidConstPtr());
  }
  
  template<class T , class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, bool(T::*srv_func)(ServiceEvent< MReq, MRes > &), const boost::shared_ptr< T > &obj) {
    checkServiceInput(service);

    auto func = serviceEventCallbackWrapper<ServiceEvent< MReq, MRes > &>(boost::bind(srv_func, obj, _1));
    return NodeHandle::advertiseService(service, func, VoidConstPtr());
  }
  
  template<class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, bool(*srv_func)(MReq &, MRes &)) {
    checkServiceInput(service);

    auto func = serviceCallbackWrapper<MReq&, MRes&>(boost::bind(srv_func, _1, _2));
    return NodeHandle::advertiseService(service, func, VoidConstPtr());
  }
  
  template<class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, bool(*srv_func)(ServiceEvent< MReq, MRes > &)) {
    checkServiceInput(service);

    auto func = serviceEventCallbackWrapper<ServiceEvent< MReq, MRes > &>(boost::bind(srv_func, _1));
    return NodeHandle::advertiseService(service, func, VoidConstPtr());
  }
  
  template<class MReq , class MRes >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, const boost::function< bool(MReq &, MRes &)> &callback, const VoidConstPtr &tracked_object) {
    checkServiceInput(service);

    auto func = serviceCallbackWrapper<MReq&, MRes&>(callback);
    return NodeHandle::advertiseService(service, func, tracked_object);
  }
  
  template<class S >
  ServiceServer CARMANodeHandle::advertiseService (const std::string &service, const boost::function< bool(S &)> &callback, const VoidConstPtr &tracked_object) {
    checkServiceInput(service);

    auto func = serviceEventCallbackWrapper<S &>(callback);
    return NodeHandle::advertiseService(service, func, tracked_object);
  }
  
  ServiceServer CARMANodeHandle::advertiseService (AdvertiseServiceOptions &ops) {

    ROS_WARN("advertiseService(AdvertiseServiceOptions) called from CARMANodeHandle. This overload does not support exception handling. The user must implement their own");
    return NodeHandle::advertiseService(ops);
  }

  template<class T >
  SteadyTimer CARMANodeHandle::createSteadyTimer (WallDuration period, void(T::*callback)(const SteadyTimerEvent &), T *obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const SteadyTimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createSteadyTimer(period, func, oneshot, autostart);
  }
  
  template<class T >
  SteadyTimer CARMANodeHandle::createSteadyTimer (WallDuration period, void(T::*callback)(const SteadyTimerEvent &), const boost::shared_ptr< T > &obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const SteadyTimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createSteadyTimer(period, func, oneshot, autostart);
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

  template<class Handler , class Obj >
  Timer CARMANodeHandle::createTimer (Rate r, Handler h, Obj o, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const TimerEvent&>(h);

    return NodeHandle::createTimer (r.expectedCycleTime(), 
      func,
      oneshot, autostart);
  }

  template<class T >
  Timer CARMANodeHandle::createTimer (Duration period, void(T::*callback)(const TimerEvent &) const, T *obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const TimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createTimer(period, func, oneshot, autostart);
  }

  template<class T >
  Timer CARMANodeHandle::createTimer (Duration period, void(T::*callback)(const TimerEvent &), T *obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const TimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createTimer(period, func, oneshot, autostart);
  }

  template<class T >
  Timer CARMANodeHandle::createTimer (Duration period, void(T::*callback)(const TimerEvent &), const boost::shared_ptr< T > &obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const TimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createTimer(period, func, oneshot, autostart);
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

  template<class T >
  WallTimer CARMANodeHandle::createWallTimer (WallDuration period, void(T::*callback)(const WallTimerEvent &), T *obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const WallTimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createWallTimer(period, func, oneshot, autostart);
  }

  template<class T >
  WallTimer CARMANodeHandle::createWallTimer (WallDuration period, void(T::*callback)(const WallTimerEvent &), const boost::shared_ptr< T > &obj, bool oneshot, bool autostart) {

    auto func = callbackWrapper<const WallTimerEvent &>(boost::bind(callback, obj, _1));
    return NodeHandle::createWallTimer(period, func, oneshot, autostart);
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