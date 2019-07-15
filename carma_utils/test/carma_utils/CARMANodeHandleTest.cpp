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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <cav_msgs/SystemAlert.h>
#include "carma_utils/CARMANodeHandle.h"
#include "CallRecorder.h"

using namespace ros;

class A {
  public:

    std::string unique_obj_name_ = "";
    std::string unique_test_name_ = "";
    //// 
    // Subscription Callbacks
    ////
    void cb_1(const std_msgs::Bool& msg) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_1");
    }
    void cb_2(const std_msgs::Bool& msg) const {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_2");
    }
    void cb_3(const std_msgs::BoolConstPtr& msg) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_3");
    }
    void cb_4(const std_msgs::BoolConstPtr& msg) const {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_4");
    }
    void cb_5(const std_msgs::Bool& msg) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_5");
    }
    void cb_6(const std_msgs::Bool& msg) const {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_6");
    }
    void cb_7(const std_msgs::BoolConstPtr& msg) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_7");
    }
    void cb_8(const std_msgs::BoolConstPtr& msg) const {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_8");
    }
    //// 
    // Service Callbacks
    ////
    bool cb_s1(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_s1");
      return true;
    }
    bool cb_s2(ServiceEvent<std_srvs::SetBool::Request, std_srvs::SetBool::Response>& event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_s2");
      return true;
    }
    bool cb_s3(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_s3");
      return true;
    }
    bool cb_s4(ServiceEvent<std_srvs::SetBool::Request, std_srvs::SetBool::Response>& event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_s4");
      return true;
    }
    //// 
    // SteadyTimer Callbacks
    ////
    void cb_st1(const SteadyTimerEvent & event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_st1");
    }
    void cb_st2(const SteadyTimerEvent & event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_st2");
    }
    ////
    // Timer Callbacks
    ////
    void cb_t2(const TimerEvent & event) const {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_t2");
    }
    void cb_t3(const TimerEvent & event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_t3");
    }
    void cb_t4(const TimerEvent & event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_t4");
    }
    ////
    // WallTimer Callbacks
    ////
    void cb_wt1(const WallTimerEvent & event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_wt1");
    }
    void cb_wt2(const WallTimerEvent & event) {
      CallRecorder::markCalled(unique_test_name_, "A:" + unique_obj_name_ + ":cb_wt2");
    }
};
////
// Subscription Free Callbacks
////
void cb_9(const std_msgs::Bool& msg) {
  CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::cb_9");
}
void cb_10(const std_msgs::BoolConstPtr& msg) {
  CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::cb_10");
}
void cb_11(const std_msgs::BoolConstPtr& msg) {
  CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::cb_11");
}
void cb_12(const std_msgs::Bool& msg) {
  CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::cb_12");
}
void cb_13(const std_msgs::BoolConstPtr& msg) {
  CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::cb_13");
}
void cb_14(const std_msgs::BoolConstPtr& msg) {
  CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::cb_14");
}
////
// Service Free Callbacks
////
bool cb_s5(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  CallRecorder::markCalled("testCARMANodeHandleServices","::cb_s5");
  return true;
}
bool cb_s6(ServiceEvent<std_srvs::SetBool::Request, std_srvs::SetBool::Response>& event) {
  CallRecorder::markCalled("testCARMANodeHandleServices","::cb_s6");
  return true;
}
bool cb_s7(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  CallRecorder::markCalled("testCARMANodeHandleServices","::cb_s7");
  return true;
}
bool cb_s8(ServiceEvent<std_srvs::SetBool::Request, std_srvs::SetBool::Response>& event) {
  CallRecorder::markCalled("testCARMANodeHandleServices","::cb_s8");
  return true;
}
bool cb_s9(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  CallRecorder::markCalled("testCARMANodeHandleServices","::cb_s9");
  return true;
}
////
// SteadyTimer Free Callbacks
////
void cb_st3(const SteadyTimerEvent & event) {
  CallRecorder::markCalled("testCARMANodeHandleSteadyTimers","::cb_st3");
}
void cb_st4(const SteadyTimerEvent & event) {
  CallRecorder::markCalled("testCARMANodeHandleSteadyTimers","::cb_st4");
}
////
// Timer Free Callbacks
////
void cb_t5(const TimerEvent & event) {
  CallRecorder::markCalled("testCARMANodeHandleTimers","::cb_t5");
}
void cb_t6(const TimerEvent & event) {
  CallRecorder::markCalled("testCARMANodeHandleTimers","::cb_t6");
}
////
// WallTimer Free Callbacks
////
void cb_wt3(const WallTimerEvent & event) {
  CallRecorder::markCalled("testCARMANodeHandleWallTimers","::cb_wt3");
}
void cb_wt4(const WallTimerEvent & event) {
  CallRecorder::markCalled("testCARMANodeHandleWallTimers","::cb_wt4");
}

// Functor object used in testCARMANodeHandleTimers
class TimerFunctor {
  public:
    std::string unique_obj_name_ = "";
    std::string unique_test_name_ = "";
  void operator () (const TimerEvent & event) { 
    CallRecorder::markCalled(unique_test_name_, "TimerFunctor:" + unique_obj_name_ + ":cb_t1");
  } 
};

/**
 * Helper function will wait until the requested function has been called the requested number of times
 */ 
void awaitFunctionCall(const std::string& testKey, const std::string& functionKey, unsigned int callCount, double timeout_sec) {
  Duration timeout(timeout_sec);
  Time startTime = Time::now(); // Wait for connection
  Rate r(5);
  while (ros::ok() && CallRecorder::callCount(testKey, functionKey) < callCount && (Time::now() - startTime) < timeout) {
    ros::spinOnce();
    r.sleep();
  }
}

TEST(CARMANodeHandleTests, testCARMANodeHandleConstructor)
{
  CARMANodeHandle cnh;

  cnh.setExceptionCallback([](const std::exception& exp) -> void {});

  cnh.setShutdownCallback([]() -> void {});

  cnh.setSystemAlertCallback([](const cav_msgs::SystemAlertConstPtr& msg) -> void {});

  cnh.setSpinCallback([]() -> bool {return true;});

  CARMANodeHandle cnh2(cnh);

  CARMANodeHandle cnh3(cnh, "h");

  M_string remappings;

  remappings["gh"]="fh";

  CARMANodeHandle	cnh4(cnh,
    "g",
    remappings 
  );
}

TEST(CARMANodeHandleTests, testCARMANodeHandleSubscriber)
{
  CallRecorder::clearHistory("testCARMANodeHandleSubscriber");
  CARMANodeHandle cnh;

  A a;
  a.unique_obj_name_ = "a";
  a.unique_test_name_ = "testCARMANodeHandleSubscriber";
  boost::shared_ptr<A> aPtr = boost::make_shared<A>(a);

  Subscriber s1 = cnh.subscribe ("t", 1, &A::cb_1, &a);
  
  Subscriber s2 = cnh.subscribe ("t", 1, &A::cb_2, &a);
  
  Subscriber s3 = cnh.subscribe ("t", 1, &A::cb_3, &a);

  Subscriber s4 = cnh.subscribe ("t", 1, &A::cb_4, &a);

  Subscriber s5 = cnh.subscribe ("t", 1, &A::cb_5, aPtr);

  Subscriber s6 = cnh.subscribe ("t", 1, &A::cb_6, aPtr);

  Subscriber s7 = cnh.subscribe ("t", 1, &A::cb_7, aPtr);

  Subscriber s8 = cnh.subscribe ("t", 1, &A::cb_8, aPtr);

  Subscriber s9 = cnh.subscribe ("t", 1, cb_9);

  Subscriber s10 = cnh.subscribe ("t", 1, cb_10);
  
  Subscriber s11 = cnh.subscribe<std_msgs::Bool>("t", 1, cb_11, VoidConstPtr());

  Subscriber s12 = cnh.subscribe<std_msgs::Bool, const std_msgs::Bool&>("t", 1, boost::bind(cb_12, _1));
  
  // NOTE: This SubscribeOptions check is just for validating the warning is printed manually
  SubscribeOptions subOps;
  subOps.init<std_msgs::Bool>("t", 1, boost::bind(cb_13, _1));
  Subscriber s13 = cnh.subscribe(subOps); 

  NodeHandle nh;
  Publisher t_pub = nh.advertise<std_msgs::Bool>("t", 1);
  std_msgs::Bool msg;
  t_pub.publish(msg);

  awaitFunctionCall("testCARMANodeHandleSubscriber", "A:a:cb_1", 1, 4);
  
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_2"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_4"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_5"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_6"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_7"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "A:a:cb_8"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::cb_9"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::cb_10"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::cb_11"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::cb_12"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::cb_13"), 1);

  Publisher p1 = cnh.advertise<std_msgs::Bool>("p", 1);

  Publisher p2 = cnh.advertise<std_msgs::Bool>("p", 1,
    [](const SingleSubscriberPublisher& ssp) -> void {CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::lambda-p1");},
    [](const SingleSubscriberPublisher& ssp) -> void {CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::lambda-p2");}
  );

  AdvertiseOptions adv_ops;
  adv_ops.init<std_msgs::Bool>("p", 1, 
    [](const SingleSubscriberPublisher& ssp) -> void {CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::lambda-p3");},
    [](const SingleSubscriberPublisher& ssp) -> void {CallRecorder::markCalled("testCARMANodeHandleSubscriber", "::lambda-p4");}
  );
  Publisher p3 = cnh.advertise(adv_ops);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p1"), 0);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p2"), 0);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p3"), 0);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p4"), 0);

  Subscriber s14 = nh.subscribe ("p", 10, cb_14);

  awaitFunctionCall("testCARMANodeHandleSubscriber", "::lambda-p1", 1, 4);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p2"), 0);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p4"), 0);

  std_msgs::Bool boolMsg;
  p1.publish(boolMsg);
  p2.publish(boolMsg);
  p3.publish(boolMsg);

  awaitFunctionCall("testCARMANodeHandleSubscriber", "::cb_14", 3, 8);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::cb_14"), 3);

  s14.shutdown(); // Unsubscribe

  awaitFunctionCall("testCARMANodeHandleSubscriber", "::lambda-p2", 1, 4);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p2"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSubscriber", "::lambda-p4"), 1);

}

TEST(CARMANodeHandleTests, testCARMANodeHandleServices)
{
  CallRecorder::clearHistory("testCARMANodeHandleServices");

  A a;
  a.unique_obj_name_ = "a";
  a.unique_test_name_ = "testCARMANodeHandleServices";
  boost::shared_ptr<A> aPtr = boost::make_shared<A>(a);

  CARMANodeHandle srv_cnh;
  ros::CallbackQueue srv_queue;
  srv_cnh.setCallbackQueue(&srv_queue);
  ros::AsyncSpinner service_spinner(1, &srv_queue);
  service_spinner.start();

  ServiceServer ss1 = srv_cnh.advertiseService ("s1", &A::cb_s1, &a);
  
  ServiceServer ss2 = srv_cnh.advertiseService ("s2", &A::cb_s2, &a);
  
  ServiceServer ss3 = srv_cnh.advertiseService ("s3", &A::cb_s3, aPtr);
  
  ServiceServer ss4 = srv_cnh.advertiseService ("s4", &A::cb_s4, aPtr);
  
  ServiceServer ss5 = srv_cnh.advertiseService ("s5", cb_s5);
  
  ServiceServer ss6 = srv_cnh.advertiseService ("s6", cb_s6);
  
  ServiceServer ss7 = srv_cnh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("s7", cb_s7, VoidConstPtr());
  
  ServiceServer ss8 = srv_cnh.advertiseService<ServiceEvent<std_srvs::SetBool::Request, std_srvs::SetBool::Response>>("s8", cb_s8, VoidConstPtr());
  
  // NOTE: This SubscribeOptions check is just for validating the warning is printed manually
  AdvertiseServiceOptions srv_ops;
  srv_ops.init<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("s9", cb_s9); 
  ServiceServer ss9 = srv_cnh.advertiseService (srv_ops);

  ros::Duration srv_timeout(4);
  ros::service::waitForService("s1", srv_timeout);
  ros::service::waitForService("s2", srv_timeout);
  ros::service::waitForService("s3", srv_timeout);
  ros::service::waitForService("s4", srv_timeout);
  ros::service::waitForService("s5", srv_timeout);
  ros::service::waitForService("s6", srv_timeout);
  ros::service::waitForService("s7", srv_timeout);
  ros::service::waitForService("s8", srv_timeout);
  ros::service::waitForService("s9", srv_timeout);

  std_srvs::SetBool srv_msg;
  ros::service::call("s1", srv_msg);
  ros::service::call("s2", srv_msg);
  ros::service::call("s3", srv_msg);
  ros::service::call("s4", srv_msg);
  ros::service::call("s5", srv_msg);
  ros::service::call("s6", srv_msg);
  ros::service::call("s7", srv_msg);
  ros::service::call("s8", srv_msg);
  ros::service::call("s9", srv_msg);
  // Service calls should block so no need to wait for callbacks here
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "A:a:cb_s1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "A:a:cb_s2"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "A:a:cb_s3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "A:a:cb_s4"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "::cb_s5"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "::cb_s6"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "::cb_s7"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "::cb_s8"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleServices", "::cb_s9"), 1);

  service_spinner.stop();
}

TEST(CARMANodeHandleTests, testCARMANodeHandleSteadyTimers)
{
  CallRecorder::clearHistory("testCARMANodeHandleSteadyTimers");

  A a;
  a.unique_obj_name_ = "a";
  a.unique_test_name_ = "testCARMANodeHandleSteadyTimers";
  boost::shared_ptr<A> aPtr = boost::make_shared<A>(a);

  CARMANodeHandle st_cnh;

  ros::WallDuration timerLength(1); // 1sec
  SteadyTimer st1 = st_cnh.createSteadyTimer (timerLength, &A::cb_st1, &a, true);

  SteadyTimer st2 = st_cnh.createSteadyTimer (timerLength, &A::cb_st2, aPtr, true);

  SteadyTimer st3 = st_cnh.createSteadyTimer (timerLength, boost::bind(cb_st3, _1), true);

  SteadyTimerOptions st_ops(timerLength, boost::bind(cb_st4, _1), NULL, true);
  SteadyTimer st4 = st_cnh.createSteadyTimer (st_ops);

  awaitFunctionCall("testCARMANodeHandleSteadyTimers", "A:a:cb_st1", 1, 4);
  awaitFunctionCall("testCARMANodeHandleSteadyTimers", "A:a:cb_st2", 1, 4);
  awaitFunctionCall("testCARMANodeHandleSteadyTimers", "::cb_st3", 1, 4);
  awaitFunctionCall("testCARMANodeHandleSteadyTimers", "::cb_st4", 1, 4);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSteadyTimers", "A:a:cb_st1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSteadyTimers", "A:a:cb_st2"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSteadyTimers", "::cb_st3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSteadyTimers", "::cb_st4"), 1);
}

TEST(CARMANodeHandleTests, testCARMANodeHandleTimers)
{
  CallRecorder::clearHistory("testCARMANodeHandleTimers");

  A a;
  a.unique_obj_name_ = "a";
  a.unique_test_name_ = "testCARMANodeHandleTimers";
  boost::shared_ptr<A> aPtr = boost::make_shared<A>(a);

  CARMANodeHandle t_cnh;

  TimerFunctor tf;
  tf.unique_obj_name_ = "tf";
  tf.unique_test_name_ = "testCARMANodeHandleTimers";

  ros::Rate tR(1);
  ros::Duration timerTimeout(1);

  Timer t1 = t_cnh.createTimer (tR, tf, tf, true);

  Timer t2 = t_cnh.createTimer (timerTimeout, &A::cb_t2, &a, true);

  Timer t3 = t_cnh.createTimer (timerTimeout, &A::cb_t3, &a, true);

  Timer t4 = t_cnh.createTimer (timerTimeout, &A::cb_t4, aPtr, true);

  Timer t5 = t_cnh.createTimer (timerTimeout, boost::bind(cb_t5, _1), true);

  TimerOptions t_ops(timerTimeout, boost::bind(cb_t6, _1), NULL, true);
  
  Timer t6 = t_cnh.createTimer (t_ops);

  awaitFunctionCall("testCARMANodeHandleTimers", "TimerFunctor:tf:cb_t1", 1, 4);
  awaitFunctionCall("testCARMANodeHandleTimers", "A:a:cb_t2", 1, 4);
  awaitFunctionCall("testCARMANodeHandleTimers", "A:a:cb_t3", 1, 4);
  awaitFunctionCall("testCARMANodeHandleTimers", "A:a:cb_t4", 1, 4);
  awaitFunctionCall("testCARMANodeHandleTimers", "::cb_t5", 1, 4);
  awaitFunctionCall("testCARMANodeHandleTimers", "::cb_t6", 1, 4);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleTimers", "TimerFunctor:tf:cb_t1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleTimers", "A:a:cb_t2"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleTimers", "A:a:cb_t3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleTimers", "A:a:cb_t4"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleTimers", "::cb_t5"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleTimers", "::cb_t6"), 1);
}

TEST(CARMANodeHandleTests, testCARMANodeHandleWallTimers)
{
  CallRecorder::clearHistory("testCARMANodeHandleWallTimers");

  A a;
  a.unique_obj_name_ = "a";
  a.unique_test_name_ = "testCARMANodeHandleWallTimers";
  boost::shared_ptr<A> aPtr = boost::make_shared<A>(a);

  CARMANodeHandle wt_cnh;
  ros::WallDuration wallTimeout(1);
  WallTimer wt1 = wt_cnh.createWallTimer (wallTimeout, &A::cb_wt1, &a, true);

  WallTimer wt2 = wt_cnh.createWallTimer (wallTimeout, &A::cb_wt2, aPtr, true);

  WallTimer wt3 = wt_cnh.createWallTimer (wallTimeout, boost::bind(cb_wt3, _1), true);

  ros::WallTimerOptions wt_ops(wallTimeout, boost::bind(cb_wt4, _1), NULL, true);
  WallTimer wt4 = wt_cnh.createWallTimer (wt_ops);

  awaitFunctionCall("testCARMANodeHandleWallTimers", "A:a:cb_wt1", 1, 4);
  awaitFunctionCall("testCARMANodeHandleWallTimers", "A:a:cb_wt2", 1, 4);
  awaitFunctionCall("testCARMANodeHandleWallTimers", "::cb_wt3", 1, 4);
  awaitFunctionCall("testCARMANodeHandleWallTimers", "::cb_wt4", 1, 4);

  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleWallTimers", "A:a:cb_wt1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleWallTimers", "A:a:cb_wt2"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleWallTimers", "::cb_wt3"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleWallTimers", "::cb_wt4"), 1);
}

TEST (CARMANodeHandleTests, testCARMANodeHandleSystemAlert) {
  CallRecorder::clearHistory("testCARMANodeHandleSystemAlert");
  CARMANodeHandle cnh;

  cnh.setSystemAlertCallback([](const cav_msgs::SystemAlertConstPtr& msg) -> void {
    CallRecorder::markCalled("testCARMANodeHandleSystemAlert","::alertLambda1");
  });

  cav_msgs::SystemAlert msg;
  cnh.publishSystemAlert(msg);
  awaitFunctionCall("testCARMANodeHandleSystemAlert", "::alertLambda1", 1, 4);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSystemAlert", "::alertLambda1"), 1);
}

TEST (CARMANodeHandleTests, testCARMANodeHandleExceptionAndShutdown) {
  CallRecorder::clearHistory("testCARMANodeHandleExceptionAndShutdown");
  CARMANodeHandle cnh;

  cnh.setSystemAlertCallback([](const cav_msgs::SystemAlertConstPtr& msg) -> void {
    CallRecorder::markCalled("testCARMANodeHandleExceptionAndShutdown","::alertLambda1");
    throw std::invalid_argument("This is not a real exception and is coming from a unit test");
  });

  cnh.setExceptionCallback([](const std::exception& e) -> void {
    CallRecorder::markCalled("testCARMANodeHandleExceptionAndShutdown","::exceptionLambda1");
  });

  cnh.setShutdownCallback([]() -> void {
    CallRecorder::markCalled("testCARMANodeHandleExceptionAndShutdown","::shutdownLambda1");
  });

  // Disable ros::shutdown to allow for testing
  CARMANodeHandle::allowNodeShutdown(false);

  cav_msgs::SystemAlert msg;
  cnh.publishSystemAlert(msg);
  awaitFunctionCall("testCARMANodeHandleExceptionAndShutdown", "::exceptionLambda1", 1, 4);
  awaitFunctionCall("testCARMANodeHandleExceptionAndShutdown", "::shutdownLambda1", 1, 4);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleExceptionAndShutdown", "::exceptionLambda1"), 1);
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleExceptionAndShutdown", "::shutdownLambda1"), 1);
}

TEST (CARMANodeHandleTests, testCARMANodeHandleSpin) {
  CallRecorder::clearHistory("testCARMANodeHandleSpin");
  CARMANodeHandle cnh;

  cnh.setExceptionCallback([](const std::exception& exp) -> void {});

  cnh.setShutdownCallback([]() -> void {});

  cnh.setSystemAlertCallback([](const cav_msgs::SystemAlertConstPtr& msg) -> void {});

  cnh.setSpinCallback([]() -> bool {
    static int callCount = 0;
    CallRecorder::markCalled("testCARMANodeHandleSpin","::spinLambda1");
    callCount++;
    if (callCount >= 4) {
      return false;
    }
    return true;
  });

  // Disable ros::shutdown to allow for testing
  CARMANodeHandle::allowNodeShutdown(false);
  CARMANodeHandle::resetShutdownStatus();

  cnh.spin(); // Blocks until spin callback returns false
  ASSERT_EQ(CallRecorder::callCount("testCARMANodeHandleSpin", "::spinLambda1"), 4);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carma_node_tests", ros::init_options::AnonymousName);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}