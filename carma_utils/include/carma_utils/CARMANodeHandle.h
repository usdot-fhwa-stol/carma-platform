#ifndef CARMA_UTILS_CARMA_NODE_HANDLE_H
#define CARMA_UTILS_CARMA_NODE_HANDLE_H
/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <string>
#include <vector>
#include <stdexcept>
#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <cav_msgs/SystemAlert.h>
#include <boost/optional.hpp>

namespace ros {
  /**
   * @class CARMANodeHandle
   * @brief Extends ros::NodeHandle to provide default exception handling and shutdown behavior
   * 
   * Every function which uses callbacks (subscribers, publishers, services, timers) is wrapped in a try catch block
   * In the event an exception makes it to this class it will result in a SystemAlert message being published and the node shutting down.
   * 
   * The exception handling and system alert publication is handled statically and is shared between instances. 
   * Therefore the user should create a single default CARMANodeHandle() to ensure the system_alert publisher is properly namespaced.
   * 
   * The user can add their own exception, alert, and shutdown handling using the set callback functions.
   * 
   * This class also provides a spin() function which should be used in place of custom while loops as it will provide exception handling and shutdown behavior. 
   * A spin callback can be set to perform custom operations during spin.
   * This class is thread safe
   */

  class CARMANodeHandle: public NodeHandle
  {
    private:
      // Callback typedefs
      using SystemAlertCB = std::function<void(const cav_msgs::SystemAlertConstPtr&)>;
      using ShutdownCB = std::function<void()>;
      using ExceptionCB = std::function<void(const std::exception&)>;
      using SpinCB = std::function<bool()>;
      // Callback setting mutex
      static std::mutex shutdown_mutex_;
      static std::mutex system_alert_mutex_;
      static std::mutex exception_mutex_;
      static std::mutex spin_mutex_;
      static std::mutex static_pub_sub_mutex_;
      static bool shutting_down_; // Shutdown flag
      static std::string system_alert_topic_;
      static boost::optional<double> spin_rate_; // Rate in Hz of spin loop. If this is not set then system will use event driven spin
      // System alert pub/sub
      static volatile bool common_pub_sub_set_;
      static Subscriber system_alert_sub_;
      static Publisher system_alert_pub_;

      // Callbacks
      static SystemAlertCB system_alert_cb_;
      static ShutdownCB shutdown_cb_;
      static ExceptionCB exception_cb_;
      static SpinCB spin_cb_;

      static bool allow_node_shutdown_; // Flag controlling if handle can call ros::shutdown

      /**
       * @brief Wrapper for pub/sub callbacks which provides exception handling
       * 
       * @param callback A callable which will be used as the callback to wrap
       * @tparam C The argument type for the callback
       * 
       * @return A boost function which wraps callback in exception handling logic
       */ 
      template<class C> 
      boost::function< void(C)> callbackWrapper(const boost::function<void(C)>& callback);
      
      /**
       * @brief Wrapper for service callbacks which provides exception handling
       * @param callback A callable which will be used as the callback to wrap
       * @tparam C The argument type for the callback
       * 
       * @return A boost function which wraps callback in exception handling logic
       */ 
      template<class C, class R>
      boost::function< bool(C, R)> serviceCallbackWrapper(const boost::function<bool(C, R)>& callback);
      /**
       * @brief Wrapper for service callbacks that use the ServiceEvent<> interface which provides exception handling
       * @param callback A callable which will be used as the callback to wrap
       * @tparam C The argument type for the callback
       * 
       * @return A boost function which wraps callback in exception handling logic
       */ 
      template<class E>
      boost::function< bool(E)> serviceEventCallbackWrapper(const boost::function<bool(E)>& callback);
      /**
       * @brief Handles incoming SystemAlert messages
       * 
       * @param message The message to handle
       * 
       * Handles incoming SystemAlert messages and will shutdown this node if that message was of type SHUTDOWN
       */
      static void systemAlertHandler(const cav_msgs::SystemAlertConstPtr& message);

      /**
      * @brief Shutsdown this node
      */
      static void shutdown();

      /**
       * Helper function checks if the topic name is one already reserved for use by this class
       */ 
      static bool isRestrictedTopic(const std::string& topic);
      /**
       * Helper function checks the input topic for subscribers
       */ 
      static void checkSubscriptionInput(const std::string& topic);
      /**
       * Helper function checks the input topic for publishers
       */ 
      static void checkPublisherInput(const std::string& topic);
      /**
       * Helper function checks the input topic for services
       */ 
      static void checkServiceInput(const std::string& service);
      /**
       * Helper function validates if a callback can be called
       */ 
      template<class C>
      static void validateCallback(const C& cb);
      /**
       * Helper function validates if a function pointer can be called. Intend for use with boost::function objs
       */ 
      template<class C>
      static bool validFunctionPtr(const C& cb);

      /**
       * Initializes the system alert publisher and subscriber
       */ 
      void initPubSub();

    public:
      /**
       * @brief Handles caught exceptions which have reached the top level of this node
       * 
       * @param message The exception to handle
       * 
       * If an exception reaches the top level of this node it should be passed to this function.
       * The function will try to log the exception and publish a FATAL message to system_alert before shutting itself down.
       */
      static void handleException(const std::exception& e);

      /**
       * @brief Set the system alert callback
       * 
       * Set a callback to be triggered on receipt of a SystemAlert message.
       * This callback will be triggered before default alert handling.
       * The default behavior is to shut this node down if a SHUTDOWN message is received
       * 
       * @param cb Callback function
       */ 
      static void setSystemAlertCallback(SystemAlertCB cb);
      /**
       * @brief Set the shutdown callback
       * 
       * Set a callback to be triggered when shutdown is called
       * This callback will be triggered before the node attempts to shutdown
       */ 
      static void setShutdownCallback(ShutdownCB cb);
      /**
       * @brief Set the exception callback
       * 
       * Set a callback to be triggered when an exception makes it to this node without being caught
       * A system alert message of type FATAL will be sent before this callback is triggered
       * After the callback is triggered the node will attempt to shut itself down
       * 
       * @param cb Callback function
       */ 
      static void setExceptionCallback(ExceptionCB cb);
      /**
       * @brief Set the spin callback
       * 
       * Set a callback to be triggered during each loop of spin()
       * This callback will be triggered after each call to spinOnce()
       * If the callback function returns false the node will attempt to shutdown 
       * 
       * @throw std::invalid_argument if the spin rate has not been set with setSpinRate(). This exception is caught internally but will cause the node to shutdown.
       * 
       * @param cb Callback function
       */ 
      static void setSpinCallback(SpinCB cb);
      /**
       * @brief Exception safe replacement for ros::spin(). 
       * 
       * Equivalent to ros::spin() if setSpinRate() has not been called. 
       * If it has been called then, this loops continuously at the rate set by setSpinRate();
       * On rate controlled loop ros::spinOnce() is called then the spinCallback is triggered
       * This loop will exit and attempt to shutdown the node if the spin callback returns false
       * 
       */ 
      static void spin();
      /**
       * @brief Flag determining if the ros::shutdown can be called from CARMANodeHandle functions. NOTE: Not intended for production.
       * 
       * This function is not intended for production use and will log a warning if it is called. It is safe to use in testing.
       * 
       * @param allow_shutdown Set to false to prevent CARMANodeHandle functions from being able to call ros::shutdown
       * 
       */ 
      static void allowNodeShutdown(const bool allow_shutdown);
      /**
       * @brief Sets the spin rate of the spin() function
       * 
       * @param hz The rate in Hz at which the spin() function will call ros::spinOnce();
       *           If set to 0.0 or a negative value it is equivalent to unsetting the rate so spin() will behave like ros::spin().
       * 
       */ 
      static void setSpinRate(double hz);
      /**
       * @brief Resets the shutdown status of CARMANodeHandle. NOTE: Not intended for production.
       * 
       * This function is not intended for production use and will log a warning if it is called. It is safe to use in testing.
       * 
       */ 
      static void resetShutdownStatus();
      /**
       * @brief Publish a SystemAlert message
       * 
       * @param msg The message to publish
       */ 
      void publishSystemAlert(const cav_msgs::SystemAlert& msg);

      //////
      // OVERRIDES
      //   The following methods are overrides of NodeHandle
      //////
      CARMANodeHandle	(	const std::string & 	ns = std::string(),
        const M_string & 	remappings = M_string() 
      );	

      CARMANodeHandle	(	const NodeHandle & 	rhs	);

      CARMANodeHandle	(	const CARMANodeHandle & 	rhs	);

      CARMANodeHandle	(	const CARMANodeHandle & 	parent,
        const std::string & 	ns 
      );
      
      CARMANodeHandle	(	const CARMANodeHandle & 	parent,
        const std::string & 	ns,
        const M_string & 	remappings 
      );

      ~CARMANodeHandle() {};

      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, T *obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(M) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &), const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M , class T >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(T::*fp)(const boost::shared_ptr< M const > &) const, const boost::shared_ptr< T > &obj, const TransportHints &transport_hints=TransportHints());
      
      template<class M >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(*fp)(M), const TransportHints &transport_hints=TransportHints());
      
      template<class M >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, void(*fp)(const boost::shared_ptr< M const > &), const TransportHints &transport_hints=TransportHints());
      
      template<class M >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, const boost::function< void(const boost::shared_ptr< M const > &)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr(), const TransportHints &transport_hints=TransportHints());
      
      template<class M , class C >
      Subscriber 	subscribe (const std::string &topic, uint32_t queue_size, const boost::function< void(C)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr(), const TransportHints &transport_hints=TransportHints());
      
      // NOTE: This function does not support CARMANodeHandle exception handling as there is no way to extract the callback from SubscribeOptions
      Subscriber subscribe (SubscribeOptions &ops); 

      template<class M >
      Publisher advertise (const std::string &topic, uint32_t queue_size, bool latch=false);
      
      template<class M >
      Publisher advertise (const std::string &topic, uint32_t queue_size, const SubscriberStatusCallback &connect_cb, const SubscriberStatusCallback &disconnect_cb=SubscriberStatusCallback(), const VoidConstPtr &tracked_object=VoidConstPtr(), bool latch=false);

      Publisher advertise (AdvertiseOptions &ops);

      template<class T , class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, bool(T::*srv_func)(MReq &, MRes &), T *obj);
      
      template<class T , class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, bool(T::*srv_func)(ServiceEvent< MReq, MRes > &), T *obj);
      
      template<class T , class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, bool(T::*srv_func)(MReq &, MRes &), const boost::shared_ptr< T > &obj);
      
      template<class T , class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, bool(T::*srv_func)(ServiceEvent< MReq, MRes > &), const boost::shared_ptr< T > &obj);
      
      template<class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, bool(*srv_func)(MReq &, MRes &));
      
      template<class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, bool(*srv_func)(ServiceEvent< MReq, MRes > &));
      
      template<class MReq , class MRes >
      ServiceServer 	advertiseService (const std::string &service, const boost::function< bool(MReq &, MRes &)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr());
      
      template<class S >
      ServiceServer 	advertiseService (const std::string &service, const boost::function< bool(S &)> &callback, const VoidConstPtr &tracked_object=VoidConstPtr());
      
      // NOTE: This function does not support CARMANodeHandle exception handling as there is no way to extract the callback from AdvertiseServiceOptions
      ServiceServer 	advertiseService (AdvertiseServiceOptions &ops);

      template<class T >
      SteadyTimer 	createSteadyTimer (WallDuration period, void(T::*callback)(const SteadyTimerEvent &), T *obj, bool oneshot=false, bool autostart=true);
      
      template<class T >
      SteadyTimer 	createSteadyTimer (WallDuration period, void(T::*callback)(const SteadyTimerEvent &), const boost::shared_ptr< T > &obj, bool oneshot=false, bool autostart=true);
      
      SteadyTimer 	createSteadyTimer (WallDuration period, const SteadyTimerCallback &callback, bool oneshot=false, bool autostart=true);
      
      SteadyTimer 	createSteadyTimer (SteadyTimerOptions &ops);

      template<class Handler , class Obj >
      Timer 	createTimer (Rate r, Handler h, Obj o, bool oneshot=false, bool autostart=true);

      template<class T >
      Timer 	createTimer (Duration period, void(T::*callback)(const TimerEvent &) const, T *obj, bool oneshot=false, bool autostart=true);

      template<class T >
      Timer 	createTimer (Duration period, void(T::*callback)(const TimerEvent &), T *obj, bool oneshot=false, bool autostart=true);

      template<class T >
      Timer 	createTimer (Duration period, void(T::*callback)(const TimerEvent &), const boost::shared_ptr< T > &obj, bool oneshot=false, bool autostart=true);

      Timer 	createTimer (Duration period, const TimerCallback &callback, bool oneshot=false, bool autostart=true);

      Timer 	createTimer (TimerOptions &ops);

      template<class T >
      WallTimer 	createWallTimer (WallDuration period, void(T::*callback)(const WallTimerEvent &), T *obj, bool oneshot=false, bool autostart=true);

      template<class T >
      WallTimer 	createWallTimer (WallDuration period, void(T::*callback)(const WallTimerEvent &), const boost::shared_ptr< T > &obj, bool oneshot=false, bool autostart=true);

      WallTimer 	createWallTimer (WallDuration period, const WallTimerCallback &callback, bool oneshot=false, bool autostart=true);

      WallTimer 	createWallTimer (WallTimerOptions &ops);
  };
}

// Template functions cannot be linked unless the implementation is provided
// Therefore include implementation to allow for template functions
#include "internal/CARMANodeHandle.tpp"

#endif // CARMA_UTILS_CARMA_NODE_HANDLE_H