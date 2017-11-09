/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <socketcan_bridge/topic_to_socketcan.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/dummy.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <list>

class frameCollector
{
  public:
    std::list<can::Frame> frames;

    frameCollector() {}

    void frameCallback(const can::Frame& f)
    {
      frames.push_back(f);
    }
};

TEST(TopicToSocketCANTest, checkCorrectData)
{
  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  boost::shared_ptr<can::DummyInterface> driver_ = boost::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(&nh, &nh_param, driver_);
  to_socketcan_bridge.setup();

  // init the driver to test stateListener (not checked automatically).
  driver_->init("string_not_used", true);

  // register for messages on received_messages.
  ros::Publisher publisher_ = nh.advertise<can_msgs::Frame>("sent_messages", 10);

  // create a frame collector.
  frameCollector frame_collector_;

  //  driver->createMsgListener(&frameCallback);
  can::CommInterface::FrameListener::Ptr frame_listener_ = driver_->createMsgListener(
            can::CommInterface::FrameDelegate(&frame_collector_, &frameCollector::frameCallback));

  // create a message
  can_msgs::Frame msg;
  msg.is_extended = true;
  msg.is_rtr = false;
  msg.is_error = false;
  msg.id = 0x1337;
  msg.dlc = 8;
  for (uint8_t i=0; i < msg.dlc; i++)
  {
    msg.data[i] = i;
  }

  msg.header.frame_id = "0";  // "0" for no frame.
  msg.header.stamp = ros::Time::now();

  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  can::Frame received;
  received = frame_collector_.frames.back();
  EXPECT_EQ(received.id, msg.id);
  EXPECT_EQ(received.dlc, msg.dlc);
  EXPECT_EQ(received.is_extended, msg.is_extended);
  EXPECT_EQ(received.is_rtr, msg.is_rtr);
  EXPECT_EQ(received.is_error, msg.is_error);
  EXPECT_EQ(received.data, msg.data);
}

TEST(TopicToSocketCANTest, checkInvalidFrameHandling)
{
  // - tries to send a non-extended frame with an id larger than 11 bits.
  //   that should not be sent.
  // - verifies that sending one larger than 11 bits actually works.
  // - tries sending a message with a dlc > 8 bytes, this should not be sent.
  //   sending with 8 bytes is verified by the checkCorrectData testcase.

  ros::NodeHandle nh(""), nh_param("~");

  // create the dummy interface
  boost::shared_ptr<can::DummyInterface> driver_ = boost::make_shared<can::DummyInterface>(true);

  // start the to topic bridge.
  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(&nh, &nh_param, driver_);
  to_socketcan_bridge.setup();

  // register for messages on received_messages.
  ros::Publisher publisher_ = nh.advertise<can_msgs::Frame>("sent_messages", 10);

  // create a frame collector.
  frameCollector frame_collector_;

  //  add callback to the dummy interface.
  can::CommInterface::FrameListener::Ptr frame_listener_ = driver_->createMsgListener(
          can::CommInterface::FrameDelegate(&frame_collector_, &frameCollector::frameCallback));

  // create a message
  can_msgs::Frame msg;
  msg.is_extended = false;
  msg.id = (1<<11)+1;  // this is an illegal CAN packet... should not be sent.
  msg.header.frame_id = "0";  // "0" for no frame.
  msg.header.stamp = ros::Time::now();

  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);

  // give some time for the interface some time to process the message
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(frame_collector_.frames.size(), 0);

  msg.is_extended = true;
  msg.id = (1<<11)+1;  // now it should be alright.
  // send the can_frame::Frame message to the sent_messages topic.
  publisher_.publish(msg);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(frame_collector_.frames.size(), 1);

  // wipe the frame queue.
  frame_collector_.frames.clear();


  // finally, check if frames with a dlc > 8 are discarded.
  msg.dlc = 10;
  publisher_.publish(msg);
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(frame_collector_.frames.size(), 0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_to_topic");
  ros::WallDuration(1.0).sleep();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
