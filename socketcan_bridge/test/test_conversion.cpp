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
#include <socketcan_bridge/socketcan_to_topic.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>

// Bring in gtest
#include <gtest/gtest.h>

// test whether the content of a conversion from a SocketCAN frame
// to a ROS message correctly maintains the data.
TEST(ConversionTest, socketCANToTopicStandard)
{
  can::Frame f;
  can_msgs::Frame m;
  f.id = 127;
  f.dlc = 8;
  f.is_error = false;
  f.is_rtr = false;
  f.is_extended = false;
  for (uint8_t i = 0; i < f.dlc; ++i)
  {
    f.data[i] = i;
  }
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(127, m.id);
  EXPECT_EQ(8, m.dlc);
  EXPECT_EQ(false, m.is_error);
  EXPECT_EQ(false, m.is_rtr);
  EXPECT_EQ(false, m.is_extended);

  for (uint8_t i=0; i < 8; i++)
  {
    EXPECT_EQ(i, m.data[i]);
  }
}

// test all three flags seperately.
TEST(ConversionTest, socketCANToTopicFlags)
{
  can::Frame f;
  can_msgs::Frame m;

  f.is_error = true;
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(true, m.is_error);
  f.is_error = false;

  f.is_rtr = true;
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(true, m.is_rtr);
  f.is_rtr = false;

  f.is_extended = true;
  socketcan_bridge::convertSocketCANToMessage(f, m);
  EXPECT_EQ(true, m.is_extended);
  f.is_extended = false;
}

// idem, but the other way around.
TEST(ConversionTest, topicToSocketCANStandard)
{
  can::Frame f;
  can_msgs::Frame m;
  m.id = 127;
  m.dlc = 8;
  m.is_error = false;
  m.is_rtr = false;
  m.is_extended = false;
  for (uint8_t i = 0; i < m.dlc; ++i)
  {
    m.data[i] = i;
  }
  socketcan_bridge::convertMessageToSocketCAN(m, f);
  EXPECT_EQ(127, f.id);
  EXPECT_EQ(8, f.dlc);
  EXPECT_EQ(false, f.is_error);
  EXPECT_EQ(false, f.is_rtr);
  EXPECT_EQ(false, f.is_extended);


  for (uint8_t i=0; i < 8; i++)
  {
    EXPECT_EQ(i, f.data[i]);
  }
}

TEST(ConversionTest, topicToSocketCANFlags)
{
  can::Frame f;
  can_msgs::Frame m;

  m.is_error = true;
  socketcan_bridge::convertMessageToSocketCAN(m, f);
  EXPECT_EQ(true, f.is_error);
  m.is_error = false;

  m.is_rtr = true;
  socketcan_bridge::convertMessageToSocketCAN(m, f);
  EXPECT_EQ(true, f.is_rtr);
  m.is_rtr = false;

  m.is_extended = true;
  socketcan_bridge::convertMessageToSocketCAN(m, f);
  EXPECT_EQ(true, f.is_extended);
  m.is_extended = false;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
