/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <carma_wm_ctrl_ros2/GeofenceSchedule.hpp>
#include <carma_wm_ctrl_ros2/Geofence.hpp>
#include <carma_wm_ctrl_ros2/GeofenceScheduler.hpp>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include <carma_ros2_utils/testing/TestHelpers.hpp>
#include <carma_ros2_utils/timers/testing/TestTimer.hpp>
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>
#include <rclcpp/time_source.hpp>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>


using carma_ros2_utils::timers::testing::TestTimer;
using carma_ros2_utils::timers::testing::TestTimerFactory;
using namespace std::chrono_literals;

namespace carma_wm_ctrl
{
TEST(GeofenceScheduler, Constructor)
{
  
  
  std::cerr << "we got here1" << std::endl;
  auto options = rclcpp::NodeOptions();
  auto clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  auto dummy= std::make_shared<carma_ros2_utils::CarmaLifecycleNode>(options);
  rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue( true ) );
  
  auto useSimTime = dummy->get_parameter( "use_sim_time" ).as_bool();

  std::cerr<<"use_sime_time: " << useSimTime << std::endl;

  dummy->configure();
  dummy->set_parameter( simTime );
  dummy->activate();
  useSimTime = dummy->get_parameter( "use_sim_time" ).as_bool();

  std::cerr<<"use_sime_time: " << useSimTime << std::endl;

  rclcpp::PublisherOptions intra_proc_disabled; 
  intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; 
  auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepAll());
  pub_qos_transient_local.transient_local();

  auto clock_publisher = dummy->create_publisher<builtin_interfaces::msg::Time>("/clock", pub_qos_transient_local, intra_proc_disabled);
  auto clock_subscriber = dummy->create_subscription<builtin_interfaces::msg::Time>("/clock", 1,
                                                              std::bind(&carma_ros2_utils::CarmaLifecycleNode::misheel_callback, dummy, std::placeholders::_1));
    
  clock_publisher->on_activate();
  std::cerr << "pub: " << clock_subscriber->get_publisher_count() << std::endl;
  std::cerr << "sub: " << clock_publisher->get_subscription_count() << std::endl;

  rclcpp::spin_some(dummy->get_node_base_interface());

  std::cerr << "pub: " << clock_subscriber->get_publisher_count() << std::endl;
  std::cerr << "sub: " << clock_publisher->get_subscription_count() << std::endl;


  //auto time_source = rclcpp::TimeSource();
  
  //dummy->get_node_time_source_interface()->make_shared<rclcpp::TimeSource>();

  //time_source.attachClock(dummy->get_clock());

  auto timer = std::make_unique<carma_ros2_utils::timers::testing::TestTimerFactory>();
  timer->setClockInterface(dummy->get_node_clock_interface());
  
  std::cerr << "Time now: " << std::to_string(dummy->now().seconds()) << std::endl;

  std::chrono::duration sleep = std::chrono::milliseconds(1000);
  rclcpp::sleep_for(sleep, NULL);

  //rclcpp::executors::MultiThreadedExecutor executor;
  //executor.add_node(dummy->get_node_base_interface());
  
  std::cerr << "Time now: " << std::to_string(dummy->now().seconds()) << std::endl;
  
  rclcpp::spin_some(dummy->get_node_base_interface());

  std::cerr << "Time now: " << std::to_string(dummy->now().seconds()) << std::endl;

  clock_publisher->publish(builtin_interfaces::msg::Time(rclcpp::Time(5,0)));

  std::cerr << "Time now: " << std::to_string(dummy->now().seconds()) << std::endl;

  rclcpp::spin_some(dummy->get_node_base_interface());
  

  std::cerr << "Time now: " << std::to_string(dummy->now().seconds()) << std::endl;

  if (dummy->get_clock()->ros_time_is_active())
    std::cerr<<"HEEEEEY ROS TIME ISACTIVE " << std::endl;
  else
    std::cerr<<"ITWAS NOT ACTIVE" << std::endl; 

  
  GeofenceScheduler scheduler(std::move(timer));  // Create scheduler with test timers. Having this
                                                                      // check helps verify that the timers do not crash
                                                                      // on destruction
  
  std::cerr << "we got here" << std::endl;
  
  throw std::invalid_argument("intentional");
}

TEST(GeofenceScheduler, addGeofence)
{
  // Test adding then evaulate if the calls to active and inactive are done correctly
  // Finally test cleaing the timers
  auto gf_ptr = std::make_shared<Geofence>();

  std::cerr << "we got here1" << std::endl;

 

  std::cerr << "we got here" << std::endl;
  boost::uuids::uuid first_id = boost::uuids::random_generator()(); 
  std::size_t first_id_hashed = boost::hash<boost::uuids::uuid>()(first_id);
  gf_ptr->id_ = first_id;

  /*
  gf_ptr->schedules.push_back(
      GeofenceSchedule(rclcpp::Time(1e10),  // Schedule between 1 and 8
                       rclcpp::Time(8e10),
                       rclcpp::Duration(2e10),    // Starts at 2
                       rclcpp::Duration(3.5e10),  // Ends at by 5.5
                       rclcpp::Duration(0),    // repetition start 0 offset, so still start at 2
                       rclcpp::Duration(1e10),    // Duration of 1 and interval of 2 so active durations are (2-3 and 4-5)
                       rclcpp::Duration(2e10)));
  rclcpp::Time::setNow(rclcpp::Time(0));  // Set current time

  GeofenceScheduler scheduler(std::make_unique<TestTimerFactory>());  // Create scheduler
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<uint32_t> inactive_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  std::atomic<std::size_t> last_inactive_gf(0);
  scheduler.onGeofenceActive([&](std::shared_ptr<Geofence> gf_ptr) {
    active_call_count.store(active_call_count.load() + 1);
    // atomic is not working for boost::uuids::uuid, so hash it
    last_active_gf.store(boost::hash<boost::uuids::uuid>()(gf_ptr->id_));
  });

  scheduler.onGeofenceInactive([&](std::shared_ptr<Geofence> gf_ptr) {
    inactive_call_count.store(inactive_call_count.load() + 1);
    // atomic is not working for boost::uuids::uuid, so hash it
    last_inactive_gf.store(boost::hash<boost::uuids::uuid>()(gf_ptr->id_));
  });

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  scheduler.addGeofence(gf_ptr);

  rclcpp::Time::setNow(rclcpp::Time(1.0e10));  // Set current time

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  rclcpp::Time::setNow(rclcpp::Time(2.1e10));  // Set current time

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(100, first_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  rclcpp::Time::setNow(rclcpp::Time(3.1e10));  // Set current time

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(100., first_id_hashed, last_inactive_gf));
  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());

  rclcpp::Time::setNow(rclcpp::Time(3.5e10));  // Set current time

  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());
  ASSERT_EQ(first_id_hashed, last_inactive_gf.load());

  rclcpp::Time::setNow(rclcpp::Time(4.2e10));  // Set current time

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, 2, active_call_count));
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());

  rclcpp::Time::setNow(rclcpp::Time(5.5e10));  // Set current time

  ASSERT_TRUE(carma_ros2_utils::testing::waitForEqOrTimeout(10.0, 2, inactive_call_count));
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());

  rclcpp::Time::setNow(rclcpp::Time(9.5e10));  // Set current time

  ASSERT_EQ(2, inactive_call_count.load());
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());
  ASSERT_EQ(first_id_hashed, last_inactive_gf.load());

  // Basic check that expired geofence is not added
  boost::uuids::uuid second_id = boost::uuids::random_generator()();
  gf_ptr->id_ = second_id;
  scheduler.addGeofence(gf_ptr);

  rclcpp::Time::setNow(rclcpp::Time(11.0e10));  // Set current time

  carma_ros2_utils::testing::waitForEqOrTimeout(30.0, 100, inactive_call_count);  // Let some time pass just in case
  ASSERT_EQ(2, inactive_call_count.load());
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());
  ASSERT_EQ(first_id_hashed, last_inactive_gf.load());
  */
}

}  // namespace carma_wm_ctrl