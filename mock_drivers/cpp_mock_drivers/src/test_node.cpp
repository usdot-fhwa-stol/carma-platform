#include <ros/ros.h>
#include "std_msgs/String.h"

#include "ROSComms.h"
#include "comm_types.h"
#include "MockDriverNode.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void test_cb(){
}

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
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //-------------------------
  // ROSComms::ROSComms r_c(fptr cbf, CommTypes ct, bool latch, M M_type, int qs, string t);
  CommTypes ct = sub;
  int egg = 2;
  // ROSComms<int> test_comms = ROSComms<typename int>(test_cb, ct, false, egg, 10, "ooga_booga");

  mock_drivers::MockDriverNode mdn(egg);

  // mock_drivers::ROSComms<std_msgs::String> test_comms(test_cb, ct, false, 10, "ooga_booga");

  std::cout << "test message" << endl;

  //-------------------------

  // std::cout << test_comms.getCallback() << endl;

  // ros::Publisher chatter_pub = n.advertise<decltype(test_comms.getMessageType())>(test_comms.getTopic(), 1000);

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("ooga_booga", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    
    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}