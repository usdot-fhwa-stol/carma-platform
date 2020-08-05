#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "cpp_mock_drivers/ROSComms.h"
#include <iostream>
#include <boost/shared_ptr.hpp>
// #include 


#include <type_traits>
#include <typeinfo>
#ifndef _MSC_VER
#   include <cxxabi.h>
#endif
#include <memory>
#include <string>
#include <cstdlib>


// decltype printing
template <class T>
std::string
type_name()
{
    typedef typename std::remove_reference<T>::type TR;
    std::unique_ptr<char, void(*)(void*)> own
           (
#ifndef _MSC_VER
                abi::__cxa_demangle(typeid(TR).name(), nullptr,
                                           nullptr, nullptr),
#else
                nullptr,
#endif
                std::free
           );
    std::string r = own != nullptr ? own.get() : typeid(TR).name();
    if (std::is_const<TR>::value)
        r += " const";
    if (std::is_volatile<TR>::value)
        r += " volatile";
    if (std::is_lvalue_reference<T>::value)
        r += "&";
    else if (std::is_rvalue_reference<T>::value)
        r += "&&";
    return r;
}



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

class TestCB {
    public:
        void test_call(const std_msgs::String::ConstPtr& msg)
            {
            ROS_INFO("I heard: [%s]", msg->data.c_str());
            }
};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */


  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  TestCB tcb;

  std::function<void(const std_msgs::String::ConstPtr&)> fcnPtr = std::bind(&TestCB::test_call, &tcb, std::placeholders::_1);

  CommTypes ct = sub;
  mock_drivers::ROSComms<const std_msgs::String::ConstPtr&> test_comms(fcnPtr, ct, false, 10, "ooga_booga");
  mock_drivers::ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&> test_comms2(fcnPtr, ct, false, 10, "unga_bunga");

  boost::shared_ptr<mock_drivers::ROSComms<const std_msgs::String::ConstPtr&>> test_ptr(&test_comms);

  if (test_comms.getCommType() == CommTypes::sub){
    ROS_INFO_STREAM("neat");
  }

  // vector<mock_drivers::IComms *> v;
  // v.push_back(&test_comms);
  // v.push_back(&test_comms2);

  // vector<shared_ptr<mock_drivers::IComms>> vs;
  // vs.push_back(shared_ptr<mock_drivers::IComms>(&test_comms));
  // vs.push_back(shared_ptr<mock_drivers::IComms>(&test_comms2));

  // vector<boost::any> v_any;
  // v_any.push_back(&test_comms);
  // v_any.push_back(&test_comms2);

  // for (auto i = vs.begin(); i != vs.cend(); ++i){
  //   string str = (*i)->getTopic();
  //   // auto e = static_cast<mock_drivers::ROSComms*>(*i)->getMessageType();
  //   // auto e = (*i)->

  //   ROS_INFO_STREAM(str);
  //   ROS_INFO_STREAM(type_name<decltype(*i)>());
  //   // ROS_INFO_STREAM(type_name<decltype(static_cast<mock_drivers::IComms*>(*i))>());
  //   // ros::Subscriber sub = n.subscribe(str, 1000, &mock_drivers::ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&>::callback, &test_comms);
  // }  

  ros::Subscriber sub = n.subscribe(test_ptr->getTopic(), test_ptr->getQueueSize(), &mock_drivers::ROSComms<decltype(test_ptr->getTemplateType())>::callback, test_ptr);
  // ros::Subscriber sub = n.subscribe("ooga_booga", 1000, &mock_drivers::ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&>::callback, &test_comms);



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}