#ifdef CARMA_UTILS_CARMA_NODE_HANDLE_H
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

namespace ros {

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
        return callback(req, res);
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
        return callback(event);
      } catch(const std::exception& e) {
        handleException(e);
      }
    };

    return wrappedFunc;
  }

  template<class M , class T >
  inline Subscriber CARMANodeHandle::subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints) {

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
}

#endif // CARMA_UTILS_CARMA_NODE_HANDLE_H
